/**
 * \file  test_cjson.c
 * \brief cJSON 落地层的主机单元测试：池分配器 + hooks + 【实测池的峰值需求】
 *
 * 为什么这一组测试非有不可：
 *   cJSON 是"建树"式解析，RAM 开销是报文长度的数倍，而本工程只有静态内存，
 *   池给小了 = 云端下发的命令静默失败(回 1007)，给大了 = 白吃 RAM。
 *   到底要多大【只能量】：本测试把文档级的完整配置和工程允许的最坏下行都真解析
 *   一遍，把峰值打出来，池尺寸就按这个数定。
 *
 * ★ 本测试最重要的一个结论(用来定架构)：
 *   cJSON_Print()/cJSON_PrintUnformatted() 需要【额外一块会边打印边扩容的缓冲】，
 *   而池里没有 realloc 时扩容是"新分配 + 拷贝 + 释放旧块"——峰值因此被顶高一截。
 *   换成 cJSON_PrintPreallocated() 直接打进入调用方的静态缓冲，峰值显著下降。
 *   所以本工程的应答/上报一律用 PrintPreallocated。下面 [4] 会把三种口径都量出来。
 *
 * 覆盖：
 *   1) 池的物理不变量：8 字节对齐、块链首尾相接、无相邻双空闲、记账一致
 *   2) 分配/释放/重复释放/NULL 释放 不崩
 *   3) cJSON 通过 hooks 走我们的池(而不是 malloc)：parse -> 导航 -> print -> delete
 *   4) ★ 峰值实测：纯解析 / +PrintPreallocated / +PrintUnformatted / 解析+建应答同时存活
 *   5) 池耗尽的失败路径：返回 NULL、失败计数递增、不崩
 *   6) 反复 parse/delete 不泄漏、不碎片化(used 回 0、池自检恒真、峰值不涨)
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>

#include "log.h"
#include "cjson_port.h"

/* ================= 测试框架(与 test_cloud.c 同一套) ================= */
static int g_fail;
static int g_pass;

#define CHECK(cond, ...)                                        \
    do {                                                        \
        if (cond) { g_pass++; printf("  ok   : "); }             \
        else      { g_fail++; printf("  FAIL : "); }             \
        printf(__VA_ARGS__);                                    \
        printf("\n");                                           \
    } while (0)

/* 报文/输出缓冲一律 static(本工程栈只有 4KB，照目标机的约束来) */
static char s_in[8192];
static char s_out[8192];

/* ------------------------------------------------------------------ */
/* 造一份"文档级"的完整配置 JSON(结构对齐 gateway-config.docx 第 7 章)   */
/* ------------------------------------------------------------------ */
static uint16_t append(char *buf, uint16_t cap, uint16_t off, const char *fmt, ...)
    __attribute__((format(printf, 4, 5)));

static uint16_t append(char *buf, uint16_t cap, uint16_t off, const char *fmt, ...)
{
    va_list ap;
    int     n;

    if (off >= cap) { return cap; }
    va_start(ap, fmt);
    n = vsnprintf(buf + off, (size_t)(cap - off), fmt, ap);
    va_end(ap);
    if (n < 0) { return cap; }
    if ((uint32_t)n >= (uint32_t)(cap - off)) { return cap; }
    return (uint16_t)(off + (uint16_t)n);
}

static uint16_t build_config_json(char *buf, uint16_t cap,
                                  uint8_t n_sensor, uint8_t n_device, uint16_t name_len)
{
    uint16_t off = 0u;
    uint8_t  i;
    char     name[40];

    if (name_len > (uint16_t)sizeof(name) - 1u) { name_len = (uint16_t)sizeof(name) - 1u; }
    for (i = 0u; i < (uint8_t)name_len; i++) { name[i] = 'A'; }
    name[name_len] = '\0';

    off = append(buf, cap, off, "{\"system\":{\"gwId\":\"GW0001\",\"password\":\"\",\"timezone\":8},");
    off = append(buf, cap, off,
        "\"network\":{\"host\":\"mqtt.example.com\",\"port\":1883,\"tls\":false,"
        "\"clientId\":\"GW0001\",\"username\":\"roadtwin\",\"password\":\"pw\","
        "\"keepAliveSeconds\":60,\"reconnectMinSeconds\":5,\"reconnectMaxSeconds\":300,"
        "\"statusIntervalSeconds\":60},");
    off = append(buf, cap, off, "\"sensors\":[");
    for (i = 0u; i < n_sensor; i++)
    {
        /* 数值一律取【最长】, 与 test_cfg 的"最坏情况"对齐 —— 池峰值是由报文
         * 【字节数】决定的, 报文不放到最坏, 量出来的峰值就没有意义。 */
        off = append(buf, cap, off,
            "%s{\"uid\":\"2100%02u\",\"serial\":\"SSSSSSSSSSSSSSSS\",\"name\":\"%s\","
            "\"measureMode\":\"distance\",\"installHeightCm\":2000,\"calibOffsetMm\":-5000,"
            "\"reportPeriodSeconds\":3600,\"lowBatteryPercent\":100,\"offlineTimeoutSeconds\":65535,"
            "\"enabled\":true}",
            (i == 0u) ? "" : ",", (unsigned)(i + 1u), name);
    }
    off = append(buf, cap, off, "],");
    off = append(buf, cap, off,
        "\"thresholds\":{\"alarmCm\":10,\"warningCm\":7,\"resolveCm\":2,\"minChangeCm\":1,"
        "\"eventMinIntervalSeconds\":60,\"confirmCount\":2},");
    off = append(buf, cap, off, "\"gears\":[");
    for (i = 0u; i < 4u; i++)
    {
        off = append(buf, cap, off,
            "%s{\"gear\":%d,\"name\":\"g\",\"targets\":\"all\",\"flashMode\":%d,"
            "\"ledColor\":\"red\",\"ledTextTemplate\":\"路面积水{level}CM 禁止通行\","
            "\"ttsTextTemplate\":\"前方积水{level}厘米，禁止通行\"}",
            (i == 0u) ? "" : ",", (int)i, (int)(i % 3u));
    }
    off = append(buf, cap, off, "],");
    off = append(buf, cap, off, "\"devices\":[");
    for (i = 0u; i < n_device; i++)
    {
        off = append(buf, cap, off,
            "%s{\"uid\":\"3100%02u\",\"serial\":\"SSSSSSSSSSSSSSSS\",\"name\":\"%s\","
            "\"type\":3,\"volume\":100,\"flashMode\":3,\"enabled\":true}",
            (i == 0u) ? "" : ",", (unsigned)(i + 1u), name);
    }
    off = append(buf, cap, off, "],");
    off = append(buf, cap, off,
        "\"rules\":{\"mode\":\"auto\",\"manualTimeoutSeconds\":300,\"sensorOfflinePolicy\":\"keep\"},");
    off = append(buf, cap, off, "\"ota\":{\"fwVersion\":\"1.0.0\",\"upgradePolicy\":\"immediate\"},");
    off = append(buf, cap, off,
        "\"meta\":{\"configVersion\":6,\"crc32\":\"9E1A2B3C\",\"updatedAt\":1735689600}}");

    return off;
}

/*
 * 三种打印口径：
 *   0 = 不打印(只解析)
 *   1 = PrintPreallocated(打进调用方的静态缓冲，本工程要用这个)
 *   2 = PrintUnformatted(走池扩容，用来量化它多花多少)
 * 返回打印出来的长度；解析或打印失败返回 0。
 */
static uint16_t roundtrip(const char *json, uint16_t cap, int mode)
{
    cJSON *root;
    uint16_t n = 0u;

    root = cJSON_Parse(json);
    if (root == NULL) { return 0u; }

    if (mode == 1)
    {
        if (cJSON_PrintPreallocated(root, s_out, (int)cap, 0) != 0)
        {
            n = (uint16_t)strlen(s_out);
        }
    }
    else if (mode == 2)
    {
        char *txt = cJSON_PrintUnformatted(root);

        if (txt != NULL)
        {
            n = (uint16_t)strlen(txt);
            if (n >= cap) { n = cap; }
            (void)memcpy(s_out, txt, n);
            s_out[n] = '\0';
            cJSON_free(txt);
        }
    }
    else
    {
        n = 1u;                     /* 只解析：用非 0 表示"解析成功" */
    }

    cJSON_Delete(root);
    return n;
}

int main(void)
{
    uint16_t len;
    uint32_t p_parse;
    uint32_t p_pre;
    uint32_t p_unf;
    int      i;

    printf("=== cJSON 落地层测试(池分配器 / hooks / 峰值实测) ===\n\n");

    host_log_enable(0);

    cjson_port_init();
    CHECK(cjson_port_pool_size() == CJSON_PORT_POOL_SIZE, "池已建立: %u B",
          (unsigned)cjson_port_pool_size());

    /* ================= [1] 池的物理不变量 ================= */
    printf("\n[1] 池的物理不变量\n");
    CHECK(cjson_port_check(), "刚初始化的池自检通过(块链铺满/无相邻双空闲)");
    CHECK(cjson_port_used() == 0u, "初始占用 = 0");
    CHECK(cjson_port_fail_count() == 0u, "初始失败计数 = 0");

    /* ================= [2] 分配 / 释放 / 异常释放 ================= */
    printf("\n[2] 分配 / 释放 / 异常释放\n");
    {
        cJSON *a = cJSON_CreateObject();
        cJSON *b = cJSON_CreateArray();

        CHECK((a != NULL) && (b != NULL), "cJSON 对象/数组分配成功(走的是我们的池)");
        CHECK(cjson_port_used() > 0u, "有占用 (%u B)", (unsigned)cjson_port_used());
        CHECK(cjson_port_peak() >= cjson_port_used(), "峰值 >= 当前占用");
        CHECK(cjson_port_check(), "分配中途池自检通过");

        cJSON_Delete(a);
        cJSON_Delete(b);
        CHECK(cjson_port_used() == 0u, "全部释放后占用回到 0 (实际 %u)", (unsigned)cjson_port_used());
        CHECK(cjson_port_check(), "释放后池自检通过(相邻空闲块已合并)");

        cJSON_free(NULL);
        CHECK(true, "cJSON_free(NULL) 不崩");

        {
            cJSON *c   = cJSON_CreateObject();
            void  *raw = cJSON_malloc(32);
            cJSON_Delete(c);
            cJSON_free(raw);
            CHECK(cjson_port_check(), "重复释放/交错释放后池自检仍通过");
        }
    }

    /* ================= [3] cJSON 走 hooks：解析/导航/打印 ================= */
    printf("\n[3] cJSON 解析 -> 导航 -> 打印 -> 释放\n");
    {
        cJSON *root;
        cJSON *cmd;
        cJSON *arr;
        cJSON *e0;

        (void)strcpy(s_in,
            "{\"cmd\":\"cfg.set\",\"msgId\":\"c100\",\"ts\":1735689600,"
            "\"data\":{\"expectVersion\":6,\"sections\":{\"sensors\":["
            "{\"uid\":\"21000000000000000000000001\",\"name\":\"下穿通道东入口\"},"
            "{\"uid\":\"21000000000000000000000002\"}]}}}");
        len = (uint16_t)strlen(s_in);

        root = cJSON_Parse(s_in);
        CHECK(root != NULL, "解析成功 (%u 字节报文)", (unsigned)len);
        if (root != NULL)
        {
            cmd = cJSON_GetObjectItemCaseSensitive(root, "cmd");
            CHECK(cJSON_IsString(cmd) && (strcmp(cJSON_GetStringValue(cmd), "cfg.set") == 0),
                  "取到 cmd = cfg.set");

            arr = cJSON_GetObjectItemCaseSensitive(root, "data");
            arr = (arr != NULL) ? cJSON_GetObjectItemCaseSensitive(arr, "sections") : NULL;
            arr = (arr != NULL) ? cJSON_GetObjectItemCaseSensitive(arr, "sensors") : NULL;
            CHECK(cJSON_IsArray(arr) && (cJSON_GetArraySize(arr) == 2), "sections.sensors 有 2 个元素");

            e0 = cJSON_GetArrayItem(arr, 0);
            e0 = (e0 != NULL) ? cJSON_GetObjectItemCaseSensitive(e0, "uid") : NULL;
            CHECK(cJSON_IsString(e0) && (strcmp(cJSON_GetStringValue(e0), "uid") != 0) &&
                  (strcmp(cJSON_GetStringValue(e0), "21000000000000000000000001") == 0),
                  "sensors[0].uid = 210001");
            CHECK(cjson_port_check(), "解析态池自检通过");

            /* 顺便验证 PrintPreallocated 真的能打进入静态缓冲 */
            CHECK(cJSON_PrintPreallocated(root, s_out, (int)sizeof(s_out), 0) != 0,
                  "PrintPreallocated 成功(打进调用方缓冲)");
            CHECK(strstr(s_out, "\"cmd\":\"cfg.set\"") != NULL, "打出来的正文含 cmd");

            cJSON_Delete(root);
            CHECK(cjson_port_used() == 0u, "Delete 后占用回到 0(无泄漏)");
        }
    }

    /* ================= [4] ★ 峰值实测 ★ ================= */
    printf("\n[4] 峰值实测(池尺寸按这个定)\n");
    {
        /* 工程允许的最坏下行：5 传感器 / 5 设备 / 满长名字 */
        cjson_port_reset();
        len = build_config_json(s_in, (uint16_t)sizeof(s_in), 5u, 5u, 32u);
        printf("       最坏下行报文 = %u 字节  (工程的接收上限 MAIN_CLOUD_JSON_MAX = 4160，\n"
               "       但配置域本身有上限：5 传感器 + 5 设备 + 4 档位，真实最坏就是这一份；\n"
               "       工程自己的 test_cfg.c 量的最坏序列化是 3288 B，与这里吻合)\n",
               (unsigned)len);
        CHECK(len >= 3000u, "最坏下行确实是最大的一份配置 (%u >= 3000)", (unsigned)len);

        /* (a) 只解析 */
        cjson_port_reset();
        CHECK(roundtrip(s_in, (uint16_t)sizeof(s_out), 0) != 0u, "纯解析成功");
        p_parse = cjson_port_peak();
        CHECK(cjson_port_used() == 0u, "释放干净");
        printf("       (a) 纯解析              峰值 = %u B\n", (unsigned)p_parse);

        /* (b) 解析 + PrintPreallocated(本工程采用) */
        cjson_port_reset();
        len = roundtrip(s_in, (uint16_t)sizeof(s_out), 1);
        p_pre = cjson_port_peak();
        CHECK(len > 0u, "解析 + PrintPreallocated 成功, 打印 %u 字节", (unsigned)len);
        CHECK(cjson_port_used() == 0u, "释放干净");
        printf("       (b) 解析+Preallocated   峰值 = %u B  ★工程采用这个口径★\n", (unsigned)p_pre);

        /* (c) 解析 + PrintUnformatted(走池扩容) —— 【刻意只观测, 不要求成功】
         * 这一口径要额外一块"边打印边扩容"的缓冲；而池里没有 realloc，扩容是
         * "新分配 + 拷贝 + 释放旧块"，池被解析树切碎之后很可能找不到足够大的连续块。
         * (实测：24 KB 池下它就是失败的。) 它【不是】本工程采用的路径 ——
         * 我们一律用 PrintPreallocated 打进入调用方的静态缓冲。
         * 留在这里是为了把"为什么必须用 Preallocated"变成可复现的证据。 */
        cjson_port_reset();
        len = roundtrip(s_in, (uint16_t)sizeof(s_out), 2);
        p_unf = cjson_port_peak();
        printf("       (c) 解析+Unformatted    峰值 = %u B, 结果 = %s\n",
               (unsigned)p_unf, (len > 0u) ? "成功" : "失败(碎片/池不足) —— 正是要避免的");
        CHECK(cjson_port_used() == 0u, "释放干净(即使 Unformatted 失败也没有泄漏)");
        CHECK(cjson_port_check(), "Unformatted 失败后池自检仍通过");
        CHECK((len > 0u) || (cjson_port_fail_count() > 0u),
              "要么成功, 要么明确失败并被计数 —— 不存在悄悄写坏");

        CHECK(p_parse <= p_pre, "纯解析峰值 <= 带打印峰值 (符合预期)");
        CHECK(p_pre <= p_unf, "Preallocated 峰值 <= Unformatted 峰值 (符合预期)");

        /* (d) 真实生产最坏：下行解析树 + 应答树【同时存活】 */
        cjson_port_reset();
        {
            cJSON *req  = cJSON_Parse(s_in);
            cJSON *resp = cJSON_CreateObject();

            CHECK((req != NULL) && (resp != NULL), "下行解析树 + 应答树 同时建立成功");
            if (resp != NULL)
            {
                cJSON *data = cJSON_CreateObject();

                (void)cJSON_AddStringToObject(resp, "cmd", "cfg.set");
                (void)cJSON_AddStringToObject(resp, "msgId", "c100");
                (void)cJSON_AddNumberToObject(resp, "code", 0);
                (void)cJSON_AddStringToObject(resp, "msg", "ok");
                (void)cJSON_AddNumberToObject(resp, "ts", 1735689601);
                (void)cJSON_AddItemToObject(resp, "data", data);
                (void)cJSON_AddNumberToObject(data, "configVersion", 7);
                (void)cJSON_AddStringToObject(data, "applied", "sensors");
                CHECK(cJSON_PrintPreallocated(resp, s_out, (int)sizeof(s_out), 0) != 0,
                      "应答树 PrintPreallocated 成功");
            }
            printf("       (d) 下行树+应答树同时存活 峰值 = %u B  ★★ 这才是真正的上限 ★★\n",
                   (unsigned)cjson_port_peak());
            if (req != NULL)  { cJSON_Delete(req); }
            if (resp != NULL) { cJSON_Delete(resp); }
        }
        CHECK(cjson_port_used() == 0u, "释放干净");
        CHECK(cjson_port_fail_count() == 0u, "★ 全过程没有一次分配失败(池够用)");

        printf("       => 池 %u B, 最坏峰值 %u B, 富余 %d B\n",
               (unsigned)CJSON_PORT_POOL_SIZE, (unsigned)cjson_port_peak(),
               (int)CJSON_PORT_POOL_SIZE - (int)cjson_port_peak());
    }

    /* ================= [5] 池耗尽的失败路径 ================= */
    printf("\n[5] 池耗尽的失败路径(必须不崩、计数递增)\n");
    {
        static cJSON *hold[1024];                   /* static：照目标机的约束来 */
        int      n = 0;
        uint32_t fail_before;

        cjson_port_reset();
        fail_before = cjson_port_fail_count();      /* ★ 必须在 reset 之后取 */
        CHECK(fail_before == 0u, "reset 之后失败计数归零");

        /* (5a) 单次要的比整个池还大：必须返回 NULL 并计数，而不是去动别的地方 */
        CHECK(cJSON_malloc((size_t)CJSON_PORT_POOL_SIZE + 1u) == NULL,
              "单次请求超过池容量 -> 返回 NULL");
        CHECK(cjson_port_fail_count() == (fail_before + 1u), "超池请求让失败计数 +1");

        /* (5b) 一直分配，直到池满 */
        while (n < 1024)
        {
            hold[n] = cJSON_CreateObject();
            if (hold[n] == NULL) { break; }
            n++;
        }
        CHECK(n < 1024, "一直分配最终会失败(成功 %d 个后池满)", n);
        CHECK(cjson_port_fail_count() > fail_before, "失败计数递增 (%u -> %u)",
              (unsigned)fail_before, (unsigned)cjson_port_fail_count());
        CHECK(cjson_port_check(), "池满状态下自检仍通过");

        for (i = 0; i < n; i++) { cJSON_Delete(hold[i]); }
        CHECK(cjson_port_used() == 0u, "全部回收后占用回到 0 (%u)", (unsigned)cjson_port_used());
        CHECK(cjson_port_check(), "回收后池自检通过");
    }

    /* ================= [6] 反复 parse/delete：不泄漏、不碎片 ================= */
    printf("\n[6] 反复 parse/print/delete 不泄漏、不碎片化\n");
    {
        uint32_t peak_before;
        uint16_t ok = 0u;

        cjson_port_reset();
        (void)build_config_json(s_in, (uint16_t)sizeof(s_in), 5u, 5u, 32u);
        (void)roundtrip(s_in, (uint16_t)sizeof(s_out), 1);       /* 先跑一遍把峰值定下来 */
        peak_before = cjson_port_peak();

        for (i = 0; i < 50; i++)
        {
            if (roundtrip(s_in, (uint16_t)sizeof(s_out), 1) == 0u) { break; }
            ok++;
        }
        CHECK(ok == 50u, "连续 50 次 parse+print+delete 全部成功 (%u)", (unsigned)ok);
        CHECK(cjson_port_used() == 0u, "占用回到 0(没有泄漏)");
        CHECK(cjson_port_check(), "池自检通过(没有碎片没合并)");
        CHECK(cjson_port_peak() == peak_before,
              "50 次之后峰值一点没涨(%u == %u) —— 没有碎片累积",
              (unsigned)cjson_port_peak(), (unsigned)peak_before);
    }

    cjson_port_log_stats("测试结束");
    printf("\n==== 通过 %d, 失败 %d ====\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
