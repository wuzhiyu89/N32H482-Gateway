/**
 * \file  test_cfg.c
 * \brief config_store 的【主机单元测试】—— 任务 P2-2 / P2-4
 *
 * 为什么必须在 PC 上测:
 *   配置系统里最容易错的三件事都跟"板子跑不跑"无关, 只跟逻辑有关:
 *     1) 序列化/解析是不是【互为逆运算】(漏一个字段、截断一个字符串,
 *        在板子上只表现为"某项设置没生效", 极难定位);
 *     2) CRC32 覆盖边界对不对(文档要求覆盖"全量配置不含 meta");
 *     3) 双区回退: 一区坏掉能不能用另一区, 两区都坏能不能回退出厂默认。
 *   这三件事用确定输入在 PC 上一次跑完, 比烧板子后"改一项看一眼"快得多。
 *
 * P2-4 追加覆盖:
 *   4) config_store_merge_sections(): 逐域独立的 applied/rejected —— 尤其是
 *      "只带 1 个传感器/设备/档位时, 其余条目必须原样保留"(静默丢数据是这里
 *      最危险的失效方式, 见 [9][11]);
 *   5) gears 按 gear 字段定位(不是数组下标, 见 [12]);
 *   6) config_store_write_members(): cfg.query 应答 data 的写出与回读(见 [13])。
 *
 * 存储端口(config_store.c 里的 cfg_port_*)在本构建下是 RAM 映像,
 * 所以下面可以直接破坏某个 bank 的字节来模拟 Flash 损坏。
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "log.h"
#include "cjson_port.h"      /* v33-2: 写配置已迁到 cJSON, 测试前要先装 hooks */
#include "config_store.h"

/* ================= 测试框架 ================= */
static int g_fail;
static int g_pass;

#define CHECK(cond, ...)                                        \
    do {                                                        \
        if (cond) { g_pass++; printf("  ok   : "); }             \
        else      { g_fail++; printf("  FAIL : "); }             \
        printf(__VA_ARGS__);                                    \
        printf("\n");                                           \
    } while (0)

/* 比较两个配置(忽略 meta.crc32 —— 它是派生值, 由序列化自动回填) */
static int cfg_same(const config_store_t *a, const config_store_t *b)
{
    config_store_t x = *a;
    config_store_t y = *b;

    x.meta.crc32 = 0u;
    y.meta.crc32 = 0u;

    return (memcmp(&x, &y, sizeof(x)) == 0) ? 1 : 0;
}

static char g_json[CFG_JSON_MAX];

/* ================================================================= */
static void test_factory_default(void)
{
    config_store_t c;

    printf("\n[1] 出厂默认\n");

    config_store_factory_default(&c);

    CHECK(config_store_validate(&c) == CFG_OK, "出厂默认本身合法");
    /* ★2026-10-08: 出厂 gwId 由 "GW0001" 改成 "11000000000000000000000000"(甲方平台的设备 ID 规则:
     * 网关号段 1xxxxx) —— 这个值同时用作 MQTT 用户名与主题里的 {gwId}。 */
    CHECK(strcmp(c.system.gw_id, "11000000000000000000000000") == 0, "gwId 默认 11000000000000000000000000(= 设备 ID)");
    CHECK(c.system.timezone == 8, "timezone 默认 +8");
    CHECK(c.network.port == 1883, "port 默认 1883");
    CHECK(c.network.keep_alive_s == 60, "keepAlive 默认 60");
    CHECK(c.network.status_interval_s == 30,
          "★ statusInterval 默认 30(2026-09-30 用户口径: 保活 30s)");
    CHECK(c.thresholds.alarm_cm == 10 && c.thresholds.warning_cm == 7 &&
          c.thresholds.resolve_cm == 2, "三级阈值默认 10/7/2");
    CHECK(c.thresholds.confirm_count == 1, "confirmCount 默认 1(不启用)");
    CHECK(c.rules.mode == CFG_RULE_AUTO, "rules.mode 默认 auto");
    CHECK(c.rules.manual_timeout_s == 300, "manualTimeout 默认 300");
    CHECK(c.rules.sensor_offline_policy == CFG_OFFLINE_KEEP, "sensorOfflinePolicy 默认 keep");
    CHECK(c.ota.upgrade_policy == CFG_OTA_IMMEDIATE, "upgradePolicy 默认 immediate");
    CHECK(c.meta.config_version == 0, "出厂 configVersion 归零");
    CHECK(c.sensor_count == 0 && c.device_count == 0, "sensors[]/devices[] 清空");

    /* 文档 3.5 的档位默认表(★2026-10-08 按新字段重写★) */
    CHECK(c.gears[0].led_color[0] == CFG_COLOR_OFF && c.gears[0].sound_mode == 0, "档位0 关屏");
    /* ★sound_mode 语义变了: 3 = 快闪(原来 2 = 闪光+语音; 语音现在看 tts 是否为空)★ */
    CHECK(c.gears[3].led_color[0] == CFG_COLOR_RED && c.gears[3].sound_mode == 3, "档位3 红光+快闪");
    CHECK(c.gears[3].led_color[1] == CFG_COLOR_RED, "档位3 两个半屏都是红色");
    /* ★单个 led_text 拆成 led_text1 / led_text2★ */
    CHECK(strcmp(c.gears[3].led_text1, "路面积水{level}CM") == 0, "档位3 文本1 含 {level}");
    CHECK(strcmp(c.gears[3].led_text2, "禁止通行") == 0, "档位3 文本2");
    CHECK(strcmp(c.gears[3].tts_text, "前方积水{level}厘米，禁止通行") == 0, "档位3 语音文本");
    CHECK(c.gears[1].targets_all == 1 && c.gears[1].target_count == 0, "档位 targets 默认 all");
}

/* ================================================================= */
static void test_roundtrip(void)
{
    config_store_t a, b;
    uint16_t       n;

    printf("\n[2] 序列化 <-> 解析 互为逆运算\n");

    config_store_factory_default(&a);

    n = config_store_to_json(&a, g_json, (uint16_t)sizeof(g_json));
    CHECK(n > 0u && n < CFG_JSON_MAX, "默认配置序列化成功 n=%u", (unsigned)n);
    if (n == 0u)
    {
        /* 序列化失败时 g_json 是空串, 而空串解析出来恰好等于出厂默认,
         * 会让后面的往返用例【假通过】。这里直接跳过, 让失败保持诚实。 */
        printf("  !! 序列化失败, 本组后续用例跳过(否则空缓冲会造成假通过)\n");
        return;
    }
    CHECK(strstr(g_json, "\"system\"") != NULL, "含 system 域");
    CHECK(strstr(g_json, "\"meta\"") != NULL, "含 meta 域");
    CHECK(strstr(g_json, "\"crc32\":\"") != NULL, "crc32 是 HEX 字符串(文档形状)");

    CHECK(config_store_from_json(&b, g_json) == CFG_OK, "解析自己的输出成功");
    CHECK(cfg_same(&a, &b), "往返后配置逐字段一致");

    /* 带 meta.crc32 的必须自校验通过 */
    CHECK(b.meta.crc32 == config_store_calc_crc(&a), "回填的 crc32 与重算一致");

    /* 改坏 crc32 字符串 -> 必须被检出 */
    {
        char *p = strstr(g_json, "\"crc32\":\"");
        CHECK(p != NULL, "找到 crc32 字段用于破坏");
        if (p != NULL)
        {
            char save = p[11];
            p[11] = (save == '0') ? '1' : '0';      /* 只改一个 HEX 字符 */
            CHECK(config_store_from_json(&b, g_json) == CFG_ERR_CRC,
                  "crc32 被改一个字符 -> CFG_ERR_CRC");
            p[11] = save;
        }
    }

    /* 改坏正文里的一个值 -> crc32 也必须对不上 */
    {
        char *p = strstr(g_json, "\"alarmCm\":10");
        CHECK(p != NULL, "找到 alarmCm 用于破坏");
        if (p != NULL)
        {
            p[12] = '9';                            /* 10 -> 19, 合法但内容变了 */
            CHECK(config_store_from_json(&b, g_json) == CFG_ERR_CRC,
                  "正文被改 -> CFG_ERR_CRC(CRC 覆盖到了正文)");
            p[12] = '0';
        }
    }

    /* 空对象 -> 全部回落出厂默认 */
    CHECK(config_store_from_json(&b, "{}") == CFG_OK, "空 JSON {} 可解析");
    CHECK(cfg_same(&a, &b), "空 JSON 结果 == 出厂默认");
}

/* ================================================================= */
static void test_full_config(void)
{
    static const char *FULL =
        "{\"system\":{\"gwId\":\"GW0001\",\"password\":\"pw\",\"timezone\":8},"
        "\"network\":{\"host\":\"mqtt.example.com\",\"port\":1883,\"tls\":false,"
        "\"clientId\":\"GW0001\",\"username\":\"roadtwin\",\"password\":\"sec\","
        "\"keepAliveSeconds\":60,\"reconnectMinSeconds\":5,\"reconnectMaxSeconds\":300,"
        "\"statusIntervalSeconds\":60},"
        "\"sensors\":["
        "{\"uid\":\"21000000000000000000000001\",\"serial\":\"WL20260001\",\"name\":\"A\",\"measureMode\":\"level\","
        "\"installHeightCm\":0,\"calibOffsetMm\":0,\"reportPeriodSeconds\":60,"
        "\"lowBatteryPercent\":0,\"offlineTimeoutSeconds\":90,\"enabled\":true},"
        "{\"uid\":\"21000000000000000000000002\",\"serial\":\"WL20260002\",\"name\":\"B\",\"measureMode\":\"distance\","
        "\"installHeightCm\":300,\"calibOffsetMm\":15,\"reportPeriodSeconds\":60,"
        "\"lowBatteryPercent\":20,\"offlineTimeoutSeconds\":90,\"enabled\":true}],"
        "\"thresholds\":{\"alarmCm\":10,\"warningCm\":7,\"resolveCm\":2,\"minChangeCm\":1,"
        "\"eventMinIntervalSeconds\":60,\"confirmCount\":2},"
        "\"gears\":["
        /* ★2026-10-08: 字段按 §1.9 改名/改形★
         *   ledColor          单值 -> "第一个半屏,第二个半屏"
         *   ledTextTemplate   单段 -> ledTextTemplate1 + ledTextTemplate2
         *   ledShowMode / ledTextShowMode  新增
         *   soundMode         0~3(3 = 快闪) */
        "{\"gear\":0,\"name\":\"解除\",\"targets\":\"all\",\"soundMode\":0,"
        "\"ledShowMode\":0,\"ledTextShowMode\":\"0,0\",\"ledColor\":\"off,off\","
        "\"ledTextTemplate1\":\"\",\"ledTextTemplate2\":\"\",\"ttsTextTemplate\":\"\"},"
        "{\"gear\":1,\"name\":\"正常\",\"targets\":\"all\",\"soundMode\":1,"
        "\"ledShowMode\":0,\"ledTextShowMode\":\"2,2\",\"ledColor\":\"green,green\","
        "\"ledTextTemplate1\":\"路面积水{level}CM\",\"ledTextTemplate2\":\"缓慢通行\","
        "\"ttsTextTemplate\":\"\"},"
        "{\"gear\":2,\"name\":\"预警\",\"targets\":[\"31000000000000000000000001\",\"31000000000000000000000002\"],\"soundMode\":2,"
        "\"ledShowMode\":1,\"ledTextShowMode\":\"1,2\",\"ledColor\":\"yellow,red\","
        "\"ledTextTemplate1\":\"路面积水{level}CM\",\"ledTextTemplate2\":\"谨慎通行\","
        "\"ttsTextTemplate\":\"\"},"
        "{\"gear\":3,\"name\":\"告警\",\"targets\":[\"31000000000000000000000001\"],\"soundMode\":3,"
        "\"ledShowMode\":0,\"ledTextShowMode\":\"1,0\",\"ledColor\":\"red,yellow\","
        "\"ledTextTemplate1\":\"路面积水{level}CM\",\"ledTextTemplate2\":\"禁止通行\","
        "\"ttsTextTemplate\":\"前方积水{level}厘米，禁止通行\"}],"
        "\"devices\":["
        "{\"uid\":\"31000000000000000000000001\",\"serial\":\"LK20260001\",\"name\":\"D1\",\"type\":3,\"volume\":60,"
        "\"flashMode\":1,\"enabled\":true},"
        "{\"uid\":\"31000000000000000000000002\",\"serial\":\"LK20260002\",\"name\":\"D2\",\"type\":1,\"volume\":50,"
        "\"flashMode\":1,\"enabled\":true}],"
        "\"rules\":{\"mode\":\"auto\",\"manualTimeoutSeconds\":300,"
        "\"sensorOfflinePolicy\":\"keep\"},"
        "\"ota\":{\"fwVersion\":\"1.0.0\",\"upgradePolicy\":\"immediate\"}}";

    config_store_t c, b;
    uint16_t       n;

    printf("\n[3] 文档第 7 章那种满配\n");

    {
        static config_store_t dc;
        static const char *const S[] = {
            "{\"system\":{\"gwId\":\"GW0001\"}}",
            "{\"network\":{\"port\":1883}}",
            "{\"sensors\":[{\"uid\":\"21000000000000000000000001\",\"reportPeriodSeconds\":60}]}",
            "{\"devices\":[{\"uid\":\"31000000000000000000000001\",\"type\":3,\"volume\":60}]}",
            "{\"gears\":[{\"gear\":3,\"soundMode\":3,\"targets\":[\"31000000000000000000000001\"]}]}",
            "{\"gears\":[{\"gear\":1,\"soundMode\":1,\"targets\":[]}]}",
            "{\"gears\":[{\"gear\":1,\"soundMode\":1,\"targets\":\"all\"}]}",
            "{\"thresholds\":{\"alarmCm\":20,\"warningCm\":10,\"resolveCm\":5}}",
            "{\"rules\":{\"mode\":\"auto\"}}"
        };
        /* ★决定性验证: 直接问 cfg_uid_valid() 自己 */
        {
            static const char probe[] = "21000000000000000000000001";
            printf("  [diag] CFG_UID_HEX_LEN=%d  CFG_UID_BYTES=%d  probe_len=%d  valid=%d\n",
                   (int)CFG_UID_HEX_LEN, (int)CFG_UID_BYTES,
                   (int)(sizeof(probe) - 1u), (int)cfg_uid_valid(probe));
        }
        unsigned si;
        for (si = 0u; si < (unsigned)(sizeof(S)/sizeof(S[0])); si++)
        {
            printf("  [diag] sec[%u] -> %d\n", si, (int)config_store_from_json(&dc, S[si]));
        }
    }
    CHECK(config_store_from_json(&c, FULL) == CFG_OK, "文档示例可解析");
    CHECK(c.sensor_count == 2, "传感器 2 个");
    CHECK(c.device_count == 2, "设备 2 个");
    CHECK(c.sensors[1].measure_mode == CFG_MEASURE_DISTANCE, "sensor[1] distance 模式");
    CHECK(c.sensors[1].install_height_cm == 300, "sensor[1] 安装高度 300");
    CHECK(c.sensors[1].calib_offset_mm == 15, "sensor[1] 校准偏移 15");
    CHECK(c.sensors[1].low_battery_pct == 20, "sensor[1] 低电阈值 20%%");
    CHECK(strcmp(c.sensors[0].uid, "21000000000000000000000001") == 0, "sensor[0] uid=210001");
    CHECK(c.thresholds.confirm_count == 2, "confirmCount=2");
    CHECK(c.devices[0].type == CFG_DEVTYPE_ALLINONE, "设备 type=3 一体机");
    CHECK(c.devices[0].volume == 60, "设备音量 60");

    /* targets 两种形态: "all" 与 [6,7] */
    CHECK(c.gears[1].targets_all == 1, "档位1 targets=all");
    CHECK(c.gears[2].targets_all == 0 && c.gears[2].target_count == 2, "档位2 targets=[6,7] 两个");
    CHECK(strcmp(c.gears[2].target_uid[0], "31000000000000000000000001") == 0 &&
          strcmp(c.gears[2].target_uid[1], "31000000000000000000000002") == 0, "档位2 目标 UID 310001,310002");
    CHECK(c.gears[3].target_count == 1 &&
          strcmp(c.gears[3].target_uid[0], "31000000000000000000000001") == 0, "档位3 targets=[310001]");

    /* 往返 */
    n = config_store_to_json(&c, g_json, (uint16_t)sizeof(g_json));
    CHECK(n > 0u, "满配序列化 n=%u", (unsigned)n);
    CHECK(config_store_from_json(&b, g_json) == CFG_OK, "满配往返解析成功");
    CHECK(cfg_same(&c, &b), "满配往返逐字段一致(含 targets 数组)");

    /* 再满一点: 5 传感器 + 5 设备 + 满长文本 + 最长数值, 确认不溢出 */
    {
        config_store_t big;
        int  k;

        config_store_factory_default(&big);
        big.sensor_count = CFG_SENSOR_MAX;      /* 液位传感器上限 5(文档 1~5, 见 config_store.h) */
        big.device_count = CFG_DEVICE_MAX;      /* 联动设备上限 5 */

        for (k = 0; k < CFG_SENSOR_MAX; k++)
        {
            cfg_sensor_t *s = &big.sensors[k];

            /* 前缀守约定: 液位传感器 21xxxx */
            (void)snprintf(s->uid, CFG_UID_STR_MAX, "210000000000000000000000%02u", (unsigned)(k + 1));
            s->measure_mode      = CFG_MEASURE_DISTANCE;
            s->install_height_cm = 2000u;       /* 最长 */
            s->calib_offset_mm   = -5000;       /* 带负号, 最长 */
            s->report_period_s   = 3600u;
            s->low_battery_pct   = 100u;
            s->offline_timeout_s = 65535u;      /* > 3600, 满足校验 */
            s->enabled           = 1u;
            memset(s->serial, 'S', CFG_SERIAL_MAX);
            s->serial[CFG_SERIAL_MAX] = '\0';
            memset(s->name, 'N', CFG_NAME_MAX);
            s->name[CFG_NAME_MAX] = '\0';
        }

        for (k = 0; k < CFG_DEVICE_MAX; k++)
        {
            cfg_device_t *d = &big.devices[k];

            /* 前缀守约定: 联动设备 31xxxx */
            (void)snprintf(d->uid, CFG_UID_STR_MAX, "310000000000000000000000%02u", (unsigned)(k + 1));
            d->type       = CFG_DEVTYPE_ALLINONE;
            d->volume     = 100u;
            d->flash_mode = 3u;
            d->enabled    = 1u;
            memset(d->serial, 'D', CFG_SERIAL_MAX);
            d->serial[CFG_SERIAL_MAX] = '\0';
            memset(d->name, 'M', CFG_NAME_MAX);
            d->name[CFG_NAME_MAX] = '\0';
        }

        /* 档位文本也填满, 目标列表也给满 */
        for (k = 0; k < CFG_GEAR_COUNT; k++)
        {
            int j;

            big.gears[k].gear        = (uint8_t)k;
            big.gears[k].targets_all = 0u;
            big.gears[k].target_count = CFG_DEVICE_MAX;
            for (j = 0; j < CFG_DEVICE_MAX; j++)
            {
                (void)snprintf(big.gears[k].target_uid[j], CFG_UID_STR_MAX,
                               "310000000000000000000000%02u", (unsigned)(j + 1));
            }
            memset(big.gears[k].name, 'G', CFG_NAME_MAX);
            big.gears[k].name[CFG_NAME_MAX] = '\0';
            memset(big.gears[k].led_text1, 'L', CFG_LEDTEXT_MAX);
            big.gears[k].led_text1[CFG_LEDTEXT_MAX] = '\0';
            memset(big.gears[k].led_text2, 'M', CFG_LEDTEXT_MAX);
            big.gears[k].led_text2[CFG_LEDTEXT_MAX] = '\0';
            memset(big.gears[k].tts_text, 'T', CFG_TTSTEXT_MAX);
            big.gears[k].tts_text[CFG_TTSTEXT_MAX] = '\0';
        }

        CHECK(config_store_validate(&big) == CFG_OK, "最坏情况配置本身合法");

        n = config_store_to_json(&big, g_json, (uint16_t)sizeof(g_json));
        CHECK(n > 0u, "5 传感器 + 5 设备 + 满长文本: 序列化成功 n=%u", (unsigned)n);
        CHECK(n < CFG_JSON_MAX, "最坏情况 %u < 上限 %u(单页 8KB 装得下)",
              (unsigned)n, (unsigned)CFG_JSON_MAX);
        CHECK(config_store_from_json(&b, g_json) == CFG_OK, "最坏情况往返解析成功");
        CHECK(cfg_same(&big, &b), "最坏情况往返逐字段一致");
    }
}

/* ================================================================= */
static void test_defaults_for_partial_elements(void)
{
    config_store_t c;

    printf("\n[4] 数组元素缺省字段 -> 取文档默认值\n");

    /* 传感器只给 uid, 其余全缺 */
    CHECK(config_store_from_json(&c,
        "{\"sensors\":[{\"uid\":\"21000000000000000000000001\"}]}") == CFG_OK, "只给 uid 的传感器可解析");
    CHECK(c.sensor_count == 1, "1 个传感器");
    CHECK(c.sensors[0].report_period_s == 60, "reportPeriodSeconds 缺省 -> 60");
    CHECK(c.sensors[0].offline_timeout_s == 90, "offlineTimeoutSeconds 缺省 -> 90");
    CHECK(c.sensors[0].enabled == 1, "enabled 缺省 -> true");
    CHECK(c.sensors[0].measure_mode == CFG_MEASURE_LEVEL, "measureMode 缺省 -> level");
    CHECK(config_store_validate(&c) == CFG_OK,
          "文档默认 60/90 能通过校验(见下条说明)");

    /* 设备只给 uid+type, 其余缺 */
    CHECK(config_store_from_json(&c,
        "{\"devices\":[{\"uid\":\"31000000000000000000000009\",\"type\":1}]}") == CFG_OK, "只给 uid+type 的设备可解析");
    CHECK(c.devices[0].volume == 50, "volume 缺省 -> 50");
    CHECK(c.devices[0].flash_mode == 1, "flashMode 缺省 -> 1");
    CHECK(c.devices[0].enabled == 1, "enabled 缺省 -> true");

    /* ---- UID 格式本身 ---- */
    CHECK(config_store_from_json(&c, "{\"sensors\":[{\"uid\":\"11000\"}]}") == CFG_ERR_RANGE,
          "uid 只有 5 个字符 -> CFG_ERR_RANGE");
    CHECK(config_store_from_json(&c, "{\"sensors\":[{\"uid\":\"ZZZZZZ\"}]}") == CFG_ERR_RANGE,
          "uid 含非 hex 字符 -> CFG_ERR_RANGE");
    CHECK(config_store_from_json(&c, "{\"sensors\":[{\"uid\":\"000000\"}]}") == CFG_ERR_RANGE,
          "uid 全 0(未绑定/擦除态) -> CFG_ERR_RANGE");
    CHECK(config_store_from_json(&c, "{\"sensors\":[{\"name\":\"无地址\"}]}") == CFG_ERR_RANGE,
          "完全不给 uid -> CFG_ERR_RANGE(无法定位改哪台)");
    CHECK(config_store_from_json(&c, "{\"sensors\":[{\"uid\":\"1100000000000000000000000a\"}]}") == CFG_OK,
          "小写 hex 也被接受");
}

/* ================================================================= */
static void test_merge_incremental(void)
{
    config_store_t c, orig;
    uint8_t        mask = 0u;

    printf("\n[5] 增量合并(cfg.set 按域)\n");

    config_store_factory_default(&c);
    c.thresholds.confirm_count = 3;
    c.system.timezone          = 9;
    orig = c;

    /* 只发一个域 */
    CHECK(config_store_merge_json(&c, "{\"thresholds\":{\"alarmCm\":20}}", &mask) == CFG_OK,
          "只改 thresholds 成功");
    CHECK((mask & CFG_CH_THRESHOLDS) != 0u, "changed_mask 标出 thresholds");
    CHECK((mask & CFG_CH_SYSTEM) == 0u, "changed_mask 未标 system");
    CHECK(c.thresholds.alarm_cm == 20, "alarmCm 已改为 20");
    CHECK(c.thresholds.warning_cm == 7, "同域未提及的 warningCm 保持 7");
    CHECK(c.thresholds.confirm_count == 3, "同域未提及的 confirmCount 保持 3");
    CHECK(c.system.timezone == 9, "其它域(timezone=9)未被影响");

    /* 域内没提到的字段必须保持, 这正是"增量修改"的语义 */
    CHECK(c.network.port == 1883, "network 域完全未被触碰");

    /* 空对象 = 什么都不改 */
    mask = 0xFFu;
    CHECK(config_store_merge_json(&c, "{}", &mask) == CFG_OK, "空对象合并成功");
    CHECK(mask == 0u, "空对象 changed_mask 为 0");
    CHECK(cfg_same(&c, &orig) == 0, "改了 alarmCm 所以与原始不同(预期)");
    CHECK(c.thresholds.alarm_cm == 20 && c.system.timezone == 9, "空对象未改动任何值");
}

/* ================================================================= */
static void test_validation(void)
{
    config_store_t c;

    printf("\n[6] 取值越界必须被拒(不能静默截断)\n");

    config_store_factory_default(&c);
    /* gwId 上限 26 字节(2026-10-18: 改成 26 位 hex UID) */
    CHECK(config_store_apply_json(&c,
        "{\"system\":{\"gwId\":\"123456789012345678901234567\"}}") == CFG_ERR_RANGE,
        "gwId 27 字节 -> CFG_ERR_RANGE (2026-10-18: 上限 26)");
    CHECK(strcmp(c.system.gw_id, "11000000000000000000000000") == 0, "被拒后原值未被破坏");

    CHECK(config_store_apply_json(&c, "{\"system\":{\"timezone\":99}}") == CFG_ERR_RANGE,
          "timezone=99 -> CFG_ERR_RANGE");
    CHECK(config_store_apply_json(&c, "{\"system\":{\"timezone\":14}}") == CFG_OK,
          "timezone=14 合法(边界内)");

    /* alarm 必须 > warning > resolve */
    CHECK(config_store_apply_json(&c, "{\"thresholds\":{\"alarmCm\":5}}") == CFG_ERR_RANGE,
          "alarmCm=5 <= warningCm=7 -> CFG_ERR_RANGE");
    CHECK(c.thresholds.alarm_cm == 10, "被拒后 alarmCm 仍是 10(事务性)");

    CHECK(config_store_apply_json(&c, "{\"thresholds\":{\"warningCm\":2}}") == CFG_ERR_RANGE,
          "warningCm=2 <= resolveCm=2 -> CFG_ERR_RANGE");

    CHECK(config_store_apply_json(&c, "{\"thresholds\":{\"confirmCount\":0}}") == CFG_ERR_RANGE,
          "confirmCount=0 -> CFG_ERR_RANGE(文档 1~10)");
    CHECK(config_store_apply_json(&c, "{\"thresholds\":{\"confirmCount\":11}}") == CFG_ERR_RANGE,
          "confirmCount=11 -> CFG_ERR_RANGE");

    /* 非法枚举 */
    CHECK(config_store_apply_json(&c,
        "{\"sensors\":[{\"uid\":\"21000000000000000000000001\",\"measureMode\":\"bogus\"}]}") == CFG_ERR_RANGE,
        "measureMode=bogus -> CFG_ERR_RANGE");
    CHECK(config_store_apply_json(&c, "{\"rules\":{\"mode\":\"auto2\"}}") == CFG_ERR_RANGE,
          "rules.mode=auto2 -> CFG_ERR_RANGE");

    /* 数组超长 */
    CHECK(config_store_apply_json(&c,
        "{\"sensors\":[{\"uid\":\"21000000000000000000000001\"},{\"uid\":\"21000000000000000000000002\"},{\"uid\":\"21000000000000000000000003\"},"
        "{\"uid\":\"31000000000000000000000004\"},{\"uid\":\"21000000000000000000000005\"},{\"uid\":\"21000000000000000000000004\"}]}")
        == CFG_ERR_RANGE, "6 个传感器 -> CFG_ERR_RANGE(文档 1~5)");
    CHECK(c.sensor_count == 0, "被拒后传感器列表未被破坏");

    /* UID 冲突: 传感器之间 */
    CHECK(config_store_apply_json(&c,
        "{\"sensors\":[{\"uid\":\"21000000000000000000000003\"},{\"uid\":\"21000000000000000000000003\"}]}") == CFG_ERR_RANGE,
        "两个传感器同 UID -> CFG_ERR_RANGE");

    /* UID 冲突: 传感器与设备同一编号空间 */
    CHECK(config_store_apply_json(&c,
        "{\"sensors\":[{\"uid\":\"31000000000000000000000004\"}],\"devices\":[{\"uid\":\"31000000000000000000000004\",\"type\":1}]}")
        == CFG_ERR_RANGE, "传感器与设备同 UID -> CFG_ERR_RANGE");

    /* ★2026-10-05 用户口径: offlineTimeoutSeconds 与 reportPeriodSeconds 的大小校验
     *   已【彻底去掉】—— 平台只下发一个字段(上报周期), 不该因为另一个字段没跟着改
     *   就被整条打回。而且该字段在本工程里没有任何消费者(失联判定用的是
     *   3 × reportPeriodSeconds), 拦它没有意义。 */
    CHECK(config_store_apply_json(&c,
        "{\"sensors\":[{\"uid\":\"21000000000000000000000001\",\"reportPeriodSeconds\":60,\"offlineTimeoutSeconds\":60}]}")
        == CFG_OK, "offlineTimeout == reportPeriod -> 现在【接受】");
    CHECK(config_store_apply_json(&c,
        "{\"sensors\":[{\"uid\":\"21000000000000000000000001\",\"reportPeriodSeconds\":3600,\"offlineTimeoutSeconds\":5}]}")
        == CFG_OK, "offlineTimeout 远小于 reportPeriod -> 也【接受】(该字段无消费者)");
    CHECK(config_store_apply_json(&c,
        "{\"sensors\":[{\"uid\":\"21000000000000000000000001\",\"reportPeriodSeconds\":60,\"offlineTimeoutSeconds\":90}]}")
        == CFG_OK, "文档默认 60/90 被接受");

    /* reportPeriodSeconds 自身的范围仍然要拦(文档 5~3600) */
    CHECK(config_store_apply_json(&c,
        "{\"sensors\":[{\"uid\":\"21000000000000000000000001\",\"reportPeriodSeconds\":4}]}")
        == CFG_ERR_RANGE, "reportPeriodSeconds = 4 -> 仍 CFG_ERR_RANGE(文档 5~3600)");
    CHECK(config_store_apply_json(&c,
        "{\"sensors\":[{\"uid\":\"21000000000000000000000001\",\"reportPeriodSeconds\":3601}]}")
        == CFG_ERR_RANGE, "reportPeriodSeconds = 3601 -> 仍 CFG_ERR_RANGE");

    /* 网络域 */
    CHECK(config_store_apply_json(&c, "{\"network\":{\"port\":0}}") == CFG_ERR_RANGE,
          "port=0 -> CFG_ERR_RANGE");
    CHECK(config_store_apply_json(&c, "{\"network\":{\"keepAliveSeconds\":10}}") == CFG_ERR_RANGE,
          "keepAlive=10 -> CFG_ERR_RANGE(文档 30~300)");
    CHECK(config_store_apply_json(&c,
        "{\"network\":{\"reconnectMinSeconds\":100,\"reconnectMaxSeconds\":50}}")
        == CFG_ERR_RANGE, "reconnectMax < reconnectMin -> CFG_ERR_RANGE");
    CHECK(config_store_apply_json(&c, "{\"network\":{\"statusIntervalSeconds\":5}}")
        == CFG_ERR_RANGE, "statusInterval=5 -> CFG_ERR_RANGE(文档 10~600)");
}

/* ================================================================= */
static void test_dual_bank(void)
{
    config_store_t a, b;
    uint8_t       *p;

    printf("\n[7] 双区 + 掉电安全回退\n");

    /* 先确保两区是空的(未烧写 = 全 0xFF) */
    config_store_test_wipe(0);
    config_store_test_wipe(1);
    CHECK(config_store_load(&b) == CFG_ERR_NOBANK, "空板: 两区都无效 -> NOBANK");
    CHECK(strcmp(b.system.gw_id, "11000000000000000000000000") == 0, "空板回退出厂默认");

    /* 存一份有辨识度的配置 */
    config_store_factory_default(&a);
    a.meta.config_version = 7u;
    a.meta.updated_at     = 1735689600u;
    a.system.timezone     = 9;
    a.thresholds.alarm_cm = 25;
    a.sensor_count        = 1u;
    (void)strcpy(a.sensors[0].uid, "21000000000000000000000003");
    /* 数组元素必须带齐必填字段: reportPeriodSeconds 文档要求 5~3600,
     * 留 0 的话连 config_store_validate 都过不去(save 会直接拒绝)。 */
    a.sensors[0].measure_mode      = CFG_MEASURE_LEVEL;
    a.sensors[0].report_period_s   = 60u;
    a.sensors[0].offline_timeout_s = 90u;
    a.sensors[0].enabled           = 1u;
    (void)strcpy(a.sensors[0].name, "东入口");

    CHECK(config_store_save(&a) == CFG_OK, "保存成功(先写 B 再写 A)");

    CHECK(config_store_load(&b) == CFG_OK, "读回成功");
    CHECK(cfg_same(&a, &b), "读回逐字段一致");
    CHECK(b.meta.config_version == 7u, "configVersion=7 已恢复");
    CHECK(b.meta.updated_at == 1735689600u, "updatedAt 已恢复");
    CHECK(b.system.timezone == 9u, "timezone 已恢复");
    CHECK(strcmp(b.sensors[0].name, "东入口") == 0, "UTF-8 名称往返正确");

    /* --- 破坏主区 A 的正文 -> 必须回退到 B ---
     * 头部 24 字节之后是 JSON, 改其中一个字节就会让 CRC 对不上。 */
    p = config_store_test_bank(0);
    p[40] ^= 0xFFu;
    CHECK(config_store_load(&b) == CFG_OK, "主区 A 正文损坏 -> 仍能载入(回退 B)");
    CHECK(cfg_same(&a, &b), "回退 B 后数据依然正确");
    CHECK(b.meta.config_version == 7u, "回退 B 后 configVersion 仍为 7");

    /* --- 破坏主区 A 的头部自校验 --- */
    config_store_save(&a);                       /* 先把两区都修好 */
    p = config_store_test_bank(0);
    p[0] ^= 0xFFu;                               /* magic 被破坏 */
    CHECK(config_store_load(&b) == CFG_OK, "主区 A magic 损坏 -> 仍能载入(回退 B)");
    CHECK(cfg_same(&a, &b), "回退 B 后数据依然正确");

    /* --- 两区都坏 -> 回退出厂默认 --- */
    config_store_test_wipe(0);
    config_store_test_wipe(1);
    CHECK(config_store_load(&b) == CFG_ERR_NOBANK, "两区都坏 -> NOBANK");
    CHECK(strcmp(b.system.gw_id, "11000000000000000000000000") == 0, "两区都坏 -> 出厂默认");
    CHECK(b.meta.config_version == 0u, "两区都坏 -> configVersion 归零");

    /* --- 恢复出厂 --- */
    config_store_factory_default(&a);
    CHECK(config_store_save(&a) == CFG_OK, "出厂默认可保存");
    CHECK(config_store_load(&b) == CFG_OK, "出厂默认可读回");
    CHECK(cfg_same(&a, &b), "出厂默认往返一致");
}

/* ================================================================= */
static void test_version_and_reset(void)
{
    config_store_t c;

    printf("\n[8] configVersion 与恢复出厂\n");

    config_store_factory_default(&c);
    CHECK(c.meta.config_version == 0u, "初始 0");

    config_store_touch(&c, 1000u);
    CHECK(c.meta.config_version == 1u, "touch 后 +1");
    CHECK(c.meta.updated_at == 1000u, "touch 记录时间");

    config_store_touch(&c, 2000u);
    CHECK(c.meta.config_version == 2u, "再 touch 再 +1");

    CHECK(config_store_reset(&c, 3000u) == CFG_OK, "恢复出厂成功");
    CHECK(c.meta.config_version == 0u, "恢复出厂后 configVersion 归零(文档第 6 章)");
    CHECK(c.system.timezone == 8, "恢复出厂后 timezone 回到 +8");
    CHECK(c.sensor_count == 0u, "恢复出厂后 sensors[] 清空");
}

/* ================================================================= */
static void test_merge_sections_core(void)
{
    /* 【本任务的核心验收点】
     * "增量合并"最危险的失效方式不是报错, 而是【静默丢数据】: 平台只想改 1 号
     * 传感器的名字, 结果整份数组被替换, 2 号传感器从配置里消失 —— 云上看着
     * "设置成功", 现场却少了一路水位监测。这条用例就是把这个场景钉死。 */
    static const char *TWO_SENSORS =
        "{\"sensors\":["
        "{\"uid\":\"21000000000000000000000001\",\"name\":\"一号\",\"measureMode\":\"distance\","
        "\"installHeightCm\":300,\"calibOffsetMm\":15,\"reportPeriodSeconds\":120,"
        "\"lowBatteryPercent\":30,\"offlineTimeoutSeconds\":200,\"enabled\":false},"
        "{\"uid\":\"21000000000000000000000002\",\"name\":\"二号\",\"measureMode\":\"level\","
        "\"reportPeriodSeconds\":60,\"lowBatteryPercent\":10,"
        "\"offlineTimeoutSeconds\":90,\"enabled\":true}]}";

    config_store_t       c;
    cfg_sections_result_t res;

    printf("\n[9] 逐域增量合并 config_store_merge_sections (核心: 不许静默丢数据)\n");

    /* ---- 准备: 一份含 2 个液位传感器的配置 ---- */
    config_store_factory_default(&c);
    CHECK(config_store_from_json(&c, TWO_SENSORS) == CFG_OK, "先铺 2 个传感器(210001/210002)");
    CHECK(c.sensor_count == 2u, "初始 sensor_count = 2");

    /* ---- 只带 210001 的 name ---- */
    CHECK(config_store_merge_sections(&c,
              "{\"sensors\":[{\"uid\":\"21000000000000000000000001\",\"name\":\"改过的名字\"}]}", &res) == CFG_OK,
          "只改 210001 的 name -> CFG_OK");
    CHECK(res.applied == CFG_CH_SENSORS && res.rejected == 0u, "sensors 计入 applied, 无 rejected");

    CHECK(c.sensor_count == 2u, "★ sensor_count 仍是 2 (210002 没有被静默删掉)");
    CHECK(strcmp(c.sensors[1].uid, "21000000000000000000000002") == 0, "★ 210002 仍在列表里");
    CHECK(strcmp(c.sensors[0].uid, "21000000000000000000000001") == 0, "210001 仍在 0 号槽");
    CHECK(strcmp(c.sensors[0].name, "改过的名字") == 0, "210001 的 name 已改");

    /* 【这一组才是关键】未出现在来包里的字段必须保持原值, 不能被铺回默认值 */
    CHECK(c.sensors[0].measure_mode == CFG_MEASURE_DISTANCE, "210001 measureMode 未被重置(仍 distance)");
    CHECK(c.sensors[0].install_height_cm == 300u, "210001 installHeightCm 未被重置(仍 300)");
    CHECK(c.sensors[0].calib_offset_mm == 15, "210001 calibOffsetMm 未被重置(仍 15)");
    CHECK(c.sensors[0].report_period_s == 120u, "210001 reportPeriodSeconds 未被重置(仍 120, 不是默认 60)");
    CHECK(c.sensors[0].low_battery_pct == 30u, "210001 lowBatteryPercent 未被重置(仍 30)");
    CHECK(c.sensors[0].offline_timeout_s == 200u, "210001 offlineTimeoutSeconds 未被重置(仍 200, 不是默认 90)");
    CHECK(c.sensors[0].enabled == 0u, "210001 enabled 未被重置(仍 false)");

    /* 210002 一个字节都不许动 */
    CHECK(strcmp(c.sensors[1].name, "二号") == 0 &&
          c.sensors[1].report_period_s == 60u &&
          c.sensors[1].offline_timeout_s == 90u &&
          c.sensors[1].low_battery_pct == 10u &&
          c.sensors[1].enabled == 1u,
          "★ 210002 的所有字段保持原值");

    /* ---- 未命中的 uid -> 追加(先铺文档默认值) ---- */
    CHECK(config_store_merge_sections(&c,
              "{\"sensors\":[{\"uid\":\"21000000000000000000000003\"}]}", &res) == CFG_OK, "追加一台新传感器 -> OK");
    CHECK(c.sensor_count == 3u && strcmp(c.sensors[2].uid, "21000000000000000000000003") == 0, "210003 已追加到末尾");
    CHECK(c.sensors[2].report_period_s == 60u && c.sensors[2].offline_timeout_s == 90u &&
          c.sensors[2].enabled == 1u,
          "追加的传感器先铺文档默认值(60/90/true)");

    /* ---- 容量满: 追加到上限(5)之后再加必须【整域拒绝】, 且 *c 不被改坏 ---- */
    /* 上限从 3 改成 5 之后, 上面那台 210003 只是第 3 台, 还不到上限 —— 先补满 */
    CHECK(config_store_merge_sections(&c,
              "{\"sensors\":[{\"uid\":\"21000000000000000000000004\"}]}", &res) == CFG_OK, "追加第 4 台 -> OK");
    CHECK(c.sensor_count == 4u, "第 4 台已追加");
    CHECK(config_store_merge_sections(&c,
              "{\"sensors\":[{\"uid\":\"21000000000000000000000005\"}]}", &res) == CFG_OK, "追加第 5 台 -> OK");
    CHECK(c.sensor_count == 5u, "第 5 台已追加(到上限)");

    CHECK(config_store_merge_sections(&c,
              "{\"sensors\":[{\"uid\":\"21000000000000000000000006\"}]}", &res) == CFG_OK, "超上限不崩");
    CHECK(res.rejected == CFG_CH_SENSORS && res.applied == 0u,
          "sensor_count 已到上限 5 -> sensors 域被拒");
    CHECK(c.sensor_count == 5u, "被拒后 sensor_count 仍是 5");
    CHECK(strcmp(c.sensors[0].name, "改过的名字") == 0, "被拒后已有的改动仍在");
}

/* ================================================================= */
static void test_merge_sections_per_domain(void)
{
    config_store_t       c;
    cfg_sections_result_t res;

    printf("\n[10] 逐域 applied/rejected 位 (合法域生效 / 非法域被拒, *c 不被污染)\n");

    /* ---- 一个域合法 + 一个域非法(解析阶段就被拒) ---- */
    config_store_factory_default(&c);
    c.thresholds.alarm_cm = 10u;

    CHECK(config_store_merge_sections(&c,
              "{\"thresholds\":{\"alarmCm\":20},\"system\":{\"timezone\":99}}", &res) == CFG_OK,
          "混合下发 -> 仍返回 CFG_OK(域级失败写在 res 里)");
    CHECK(res.applied == CFG_CH_THRESHOLDS, "thresholds 生效(applied)");
    CHECK(res.rejected == CFG_CH_SYSTEM, "system 被拒(timezone=99 越界)");
    CHECK(c.thresholds.alarm_cm == 20u, "合法域的改动已提交回 *c");
    CHECK(c.system.timezone == 8, "★ 非法域没有污染 *c(时区仍是出厂默认 +8)");

    /* ---- 域内字段语法都合法, 但【整体校验】不过: 同样整域拒绝 ---- */
    CHECK(config_store_merge_sections(&c,
              "{\"thresholds\":{\"alarmCm\":5},\"rules\":{\"manualTimeoutSeconds\":77}}", &res)
          == CFG_OK, "alarmCm=5 会破坏 alarm>warning 的约束");
    CHECK(res.applied == CFG_CH_RULES, "rules 生效");
    CHECK(res.rejected == CFG_CH_THRESHOLDS, "★ thresholds 因校验失败被整域拒(rejected)");
    CHECK(c.thresholds.alarm_cm == 20u, "被拒的域一个字段都没落进 *c");
    CHECK(c.rules.manual_timeout_s == 77u, "同一个包里的合法域照常生效");

    /* ---- 未出现的域: 既不算 applied 也不算 rejected ---- */
    CHECK((res.applied & (CFG_CH_NETWORK | CFG_CH_SENSORS | CFG_CH_OTA)) == 0u &&
          (res.rejected & (CFG_CH_NETWORK | CFG_CH_SENSORS | CFG_CH_OTA)) == 0u,
          "来包里没有的域既不算 applied 也不算 rejected");

    /* ---- meta 必须被忽略(文档 sections 列表里没有它, 它是系统维护字段) ---- */
    config_store_factory_default(&c);
    c.meta.config_version = 5u;
    CHECK(config_store_merge_sections(&c,
              "{\"meta\":{\"configVersion\":999,\"crc32\":\"DEADBEEF\"}}", &res) == CFG_OK,
          "带 meta 的下发不报错(忽略而不是拒绝)");
    CHECK(res.applied == 0u && res.rejected == 0u, "★ meta 既不算 applied 也不算 rejected");
    CHECK(c.meta.config_version == 5u, "★ 平台改不动 meta.configVersion");

    /* ---- 全部域都失败: 仍然 CFG_OK ---- */
    CHECK(config_store_merge_sections(&c,
              "{\"system\":{\"timezone\":99},\"network\":{\"port\":0}}", &res) == CFG_OK,
          "全部域被拒也返回 CFG_OK(由 res 判断)");
    CHECK(res.applied == 0u && res.rejected == (CFG_CH_SYSTEM | CFG_CH_NETWORK),
          "两个域都进 rejected");

    /* ---- 参数非法才 CFG_ERR_PARAM ---- */
    CHECK(config_store_merge_sections(NULL, "{}", &res) == CFG_ERR_PARAM, "c=NULL -> CFG_ERR_PARAM");
    CHECK(config_store_merge_sections(&c, NULL, &res) == CFG_ERR_PARAM, "json=NULL -> CFG_ERR_PARAM");
    CHECK(config_store_merge_sections(&c, "{}", NULL) == CFG_OK, "res=NULL 也安全");
    CHECK(config_store_merge_sections(&c, "{}", &res) == CFG_OK &&
          res.applied == 0u && res.rejected == 0u, "空域对象 -> 什么都不改");
}

/* ================================================================= */
static void test_merge_sections_devices(void)
{
    static const char *TWO_DEVICES =
        "{\"devices\":["
        "{\"uid\":\"31000000000000000000000001\",\"name\":\"D1\",\"type\":3,\"volume\":60,\"flashMode\":2,"
        "\"enabled\":true},"
        "{\"uid\":\"31000000000000000000000002\",\"name\":\"D2\",\"type\":1,\"volume\":80,\"flashMode\":3,"
        "\"enabled\":false}]}";

    config_store_t       c;
    cfg_sections_result_t res;

    printf("\n[11] devices 的 uid 增量合并\n");

    config_store_factory_default(&c);
    CHECK(config_store_from_json(&c, TWO_DEVICES) == CFG_OK, "先铺 2 个联动设备");
    CHECK(c.device_count == 2u, "初始 device_count = 2");

    CHECK(config_store_merge_sections(&c,
              "{\"devices\":[{\"uid\":\"31000000000000000000000002\",\"name\":\"改过的设备\"}]}", &res) == CFG_OK,
          "只改 310002 的 name");
    CHECK(res.applied == CFG_CH_DEVICES && res.rejected == 0u, "devices 计入 applied");
    CHECK(c.device_count == 2u, "★ device_count 仍是 2 (310001 没被删掉)");
    CHECK(strcmp(c.devices[0].uid, "31000000000000000000000001") == 0 && c.devices[0].volume == 60u &&
          c.devices[0].flash_mode == 2u, "310001 原样未动");
    CHECK(strcmp(c.devices[1].name, "改过的设备") == 0, "310002 的 name 已改");
    CHECK(c.devices[1].volume == 80u && c.devices[1].flash_mode == 3u &&
          c.devices[1].enabled == 0u && c.devices[1].type == CFG_DEVTYPE_SOUND_LIGHT,
          "★ 310002 的其它字段保持原值(volume 仍 80, 没被铺成默认 50)");

    /* 未命中 -> 追加(先铺 volume=50/flashMode=1/enabled=1) */
    CHECK(config_store_merge_sections(&c,
              "{\"devices\":[{\"uid\":\"31000000000000000000000003\",\"type\":2}]}", &res) == CFG_OK, "追加 310003");
    CHECK(c.device_count == 3u && strcmp(c.devices[2].uid, "31000000000000000000000003") == 0, "310003 已追加");
    CHECK(c.devices[2].volume == 50u && c.devices[2].flash_mode == 1u &&
          c.devices[2].enabled == 1u, "追加的设备先铺默认值(50/1/true)");

    /* 超上限 -> 整域拒绝且 *c 不被污染 */
    CHECK(config_store_merge_sections(&c,
              "{\"devices\":[{\"uid\":\"31000000000000000000000004\",\"type\":1},{\"uid\":\"31000000000000000000000005\",\"type\":1},"
              "{\"uid\":\"31000000000000000000000006\",\"type\":1}]}", &res) == CFG_OK, "超出 CFG_DEVICE_MAX 不崩");
    CHECK(res.rejected == CFG_CH_DEVICES && res.applied == 0u, "超上限 -> devices 域被拒");
    CHECK(c.device_count == 3u, "被拒后 device_count 仍是 3(没有半截追加)");
}

/* ================================================================= */
static void test_parse_gears_by_gear_field(void)
{
    config_store_t       c;
    cfg_sections_result_t res;

    printf("\n[12] gears 按 gear 字段定位槽位(不是数组下标)\n");

    /* 【原来的隐患】来包只带 {"gear":2} 而数组下标是 0:
     * 若按下标写就会改到 0 号档 —— 平台改的是"预警档", 实际改的是"解除档"。 */
    config_store_factory_default(&c);
    CHECK(config_store_merge_sections(&c,
              "{\"gears\":[{\"gear\":2,\"name\":\"新预警\"}]}", &res) == CFG_OK,
          "只带 1 个 gear 对象");
    CHECK(res.applied == CFG_CH_GEARS, "gears 域 applied");
    CHECK(strcmp(c.gears[2].name, "新预警") == 0, "★ 改到了 2 号档(而不是 0 号)");
    CHECK(strcmp(c.gears[0].name, "解除") == 0, "0 号档没有被误改");
    CHECK(c.gears[2].led_color[0] == CFG_COLOR_YELLOW && c.gears[2].sound_mode == 2u,
          "2 号档的其它字段保持原值(没被 0 号档的默认值覆盖)");
    CHECK(c.gears[0].gear == 0u && c.gears[1].gear == 1u &&
          c.gears[2].gear == 2u && c.gears[3].gear == 3u,
          "槽位不变量 gears[i].gear == i 仍成立");
    CHECK(config_store_validate(&c) == CFG_OK, "按 gear 合并后整体仍合法");

    /* gear 字段缺失 -> 无法定位 -> 该域被拒(CFG_ERR_RANGE) */
    CHECK(config_store_merge_sections(&c,
              "{\"gears\":[{\"name\":\"没有档位号\"}]}", &res) == CFG_OK, "缺 gear 不崩");
    CHECK(res.rejected == CFG_CH_GEARS && res.applied == 0u, "★ 缺 gear -> gears 域被拒");
    CHECK(strcmp(c.gears[2].name, "新预警") == 0, "被拒后档位内容没变");

    /* 越界 gear 同样被拒 */
    CHECK(config_store_merge_sections(&c, "{\"gears\":[{\"gear\":7}]}", &res) == CFG_OK &&
          res.rejected == CFG_CH_GEARS, "gear=7 越界 -> 被拒");

    /* 全量加载走同一个函数, 所以也必须按 gear 定位 */
    CHECK(config_store_from_json(&c, "{\"gears\":[{\"gear\":3,\"name\":\"全量告警\"}]}") == CFG_OK,
          "全量解析同一形状");
    CHECK(strcmp(c.gears[3].name, "全量告警") == 0, "★ 全量解析也按 gear 字段写 3 号档");
    CHECK(strcmp(c.gears[0].name, "解除") == 0, "全量解析未误改 0 号档");
    CHECK(config_store_from_json(&c, "{\"gears\":[{\"name\":\"x\"}]}") == CFG_ERR_RANGE,
          "全量解析缺 gear -> CFG_ERR_RANGE");
}

/* ================================================================= */
/* cfg.query 用的样例配置: 刻意与出厂默认不同, 免得用例被默认值蒙过去 */
static void make_sample_config(config_store_t *c)
{
    config_store_factory_default(c);

    c->meta.config_version = 12u;
    c->meta.updated_at     = 1735689600u;
    c->system.timezone     = 9;
    (void)strcpy(c->system.gw_id, "GW0009");

    c->sensor_count = 2u;
    (void)strcpy(c->sensors[0].uid, "21000000000000000000000001");
    c->sensors[0].measure_mode      = CFG_MEASURE_LEVEL;
    c->sensors[0].report_period_s   = 60u;
    c->sensors[0].offline_timeout_s = 90u;
    c->sensors[0].enabled           = 1u;
    (void)strcpy(c->sensors[0].name, "东入口");

    (void)strcpy(c->sensors[1].uid, "21000000000000000000000002");
    c->sensors[1].measure_mode      = CFG_MEASURE_DISTANCE;
    c->sensors[1].install_height_cm = 300u;
    c->sensors[1].calib_offset_mm   = 15;
    c->sensors[1].report_period_s   = 120u;
    c->sensors[1].low_battery_pct   = 20u;
    c->sensors[1].offline_timeout_s = 240u;
    c->sensors[1].enabled           = 1u;
    (void)strcpy(c->sensors[1].name, "西出口");

    c->device_count = 1u;
    (void)strcpy(c->devices[0].uid, "31000000000000000000000001");
    c->devices[0].type       = CFG_DEVTYPE_ALLINONE;
    c->devices[0].volume     = 60u;
    c->devices[0].flash_mode = 2u;
    c->devices[0].enabled    = 1u;
    (void)strcpy(c->devices[0].name, "东门一体机");

    c->gears[1].targets_all   = 0u;
    c->gears[1].target_count  = 1u;
    (void)strcpy(c->gears[1].target_uid[0], "31000000000000000000000001");
}

static void test_write_members(void)
{
    config_store_t c;
    config_store_t b;
    cJSON         *root;
    static char    buf[CFG_JSON_MAX];
    uint16_t       n;

    printf("\n[13] config_store_write_members (cfg.query 的应答 data)\n");

    make_sample_config(&c);
    CHECK(config_store_validate(&c) == CFG_OK, "样例配置本身合法");

    root = cJSON_CreateObject();
    CHECK(root != NULL, "建好 data 对象");
    CHECK(config_store_write_members(root, &c) == true, "写入成功");
    CHECK(cJSON_PrintPreallocated(root, buf, (int)sizeof(buf), 0) != 0,
          "打印成功(PrintPreallocated 直接打进静态缓冲)");
    CHECK(buf[0] == '{' && buf[strlen(buf) - 1u] == '}', "输出是一个完整的 JSON 对象");

    CHECK(strstr(buf, "\"system\"") != NULL && strstr(buf, "\"network\"") != NULL &&
          strstr(buf, "\"sensors\"") != NULL && strstr(buf, "\"thresholds\"") != NULL &&
          strstr(buf, "\"gears\"") != NULL && strstr(buf, "\"devices\"") != NULL &&
          strstr(buf, "\"rules\"") != NULL && strstr(buf, "\"ota\"") != NULL &&
          strstr(buf, "\"meta\"") != NULL,
          "★ 9 个域齐全(含 meta)");
    CHECK(strstr(buf, "\"crc32\":\"") != NULL, "meta.crc32 是 HEX 字符串");
    CHECK(strstr(buf, "\"configVersion\":12") != NULL, "meta.configVersion 写出");

    /* 关键: 写出来的就是一份【完整的、能存 Flash 的配置 JSON】 */
    CHECK(config_store_from_json(&b, buf) == CFG_OK,
          "★ 能被 config_store_from_json 读回(meta.crc32 自校验通过)");
    CHECK(cfg_same(&c, &b), "读回后逐字段一致");
    CHECK(b.meta.crc32 == config_store_calc_crc(&c), "crc32 与重算一致");

    /* 与存 Flash 用的序列化器必须逐字节一致 —— 否则"云上看到"和"Flash 存的"会漂 */
    n = config_store_to_json(&c, g_json, (uint16_t)sizeof(g_json));
    CHECK((n > 0u) && (n == strlen(buf)) && (strcmp(buf, g_json) == 0),
          "★ 与 config_store_to_json() 的输出逐字节一致");

    /* 参数保护 */
    CHECK(!config_store_write_members(NULL, &c), "obj=NULL -> false");
    CHECK(!config_store_write_members(root, NULL), "c=NULL -> false");

    cJSON_Delete(root);
}

/* ================================================================= */
/* ★★2026-10-14 新增: [14] 重绑设备(同 UID 换新设备) ★★
 * ----------------------------------------------------------------------------
 * 对应用户场景: 设备坏了换新, 重新烧录后回到同一默认地址
 *   ⇒ 允许同 UID 再绑一次, 但条目数不变。
 *
 * ★★ 本用例的若干条断言是【防护性】的 ★★ —— 它们存在的唯一目的, 是让将来
 *    图省事的实现者立刻看到红色:
 *      · 若用 config_store_bind_device() 做"覆盖" -> 它对已存在 UID 直接返回
 *        CFG_ERR_RANGE, serial/name 【静默不更新】-> 断言会红;
 *      · 若用 unbind + bind "先删再建" -> 会清掉 gears[].target_uid 里指向该
 *        UID 的条目 -> "档位关联仍在"那条断言会红。
 *    这两个陷阱都会造成现场极难查的故障(换了设备云端还是老序列号;
 *    或"能绑定、能上报, 就是不动灯")。 */
static void test_rebind_device(void)
{
    config_store_t *c;
    config_store_t  before;
    cfg_result_t    r;

    printf("\n[14] ★重绑: 同 UID 只更新元数据, 【不新增】条目\n");

    /* 干净起点 + 用绑定接口建两台(走真实路径, 别手工拼结构) */
    config_store_factory_default(&before);
    (void)config_store_save(&before);
    (void)memcpy(config_store_get_mut(), &before, sizeof(before));

    r = config_store_bind_device("21000000000000000000000001", "SN-OLD-001", "西入口液位", true,
                                 CFG_DEVTYPE_SOUND_LIGHT, 1000u);
    CHECK(r == CFG_OK, "先绑 210001 (r=%d)", (int)r);
    r = config_store_bind_device("21000000000000000000000002", "SN-002", "东入口液位", true,
                                 CFG_DEVTYPE_SOUND_LIGHT, 1000u);
    CHECK(r == CFG_OK, "再绑 210002 (r=%d)", (int)r);

    c = config_store_get_mut();
    CHECK(c->sensor_count == 2u, "准备: sensor_count=2 (实际 %u)", (unsigned)c->sensor_count);

    /* 给 210001 设一个"非默认"的上报周期 —— 重绑【必须】保住它 */
    c->sensors[0].report_period_s = 180u;
    c->sensors[0].enabled         = 1u;
    c->sensors[1].report_period_s = 60u;

    /* ---- ① 正常重绑: 覆盖 serial/name ---- */
    r = config_store_rebind_device("21000000000000000000000001", "SN-NEW-999", "西入口液位(新)", 0u, 2000u);
    CHECK(r == CFG_OK, "★ 重绑已存在的 UID -> CFG_OK (r=%d)", (int)r);

    c = config_store_get_mut();
    CHECK(c->sensor_count == 2u, "★★ 条目数【不变】(仍是 2, 实际 %u)", (unsigned)c->sensor_count);
    CHECK(strcmp(c->sensors[0].serial, "SN-NEW-999") == 0,
          "★ serial 被覆盖成新值 (实际 %s)", c->sensors[0].serial);
    CHECK(strcmp(c->sensors[0].name, "西入口液位(新)") == 0,
          "★ name 被覆盖成新值 (实际 %s)", c->sensors[0].name);

    /* ---- ② ★★ 运行参数【必须】保住(用户口径②) ★★ ---- */
    CHECK(c->sensors[0].report_period_s == 180u,
          "★★ reportPeriodSeconds 保住了 180(【不是】被重置成默认 60) 实际 %u",
          (unsigned)c->sensors[0].report_period_s);

    /* ---- ③ 其它设备完全不受影响 ---- */
    CHECK(strcmp(c->sensors[1].serial, "SN-002") == 0, "★ 另一台的 serial 没被动");
    CHECK(c->sensors[1].report_period_s == 60u, "★ 另一台的周期没被动");

    /* ---- ④ 版本递增(平台的 expectVersion 乐观锁靠它) ---- */
    CHECK(config_store_version() != 2000u, "版本不是拿 now 直接当版本(是自增的)");

    /* ---- ⑤ 未找到的 UID -> CFG_ERR_RANGE(与 unbind 同口径) ---- */
    r = config_store_rebind_device("21000000000000000000000099", "X", "Y", 0u, 3000u);
    CHECK(r == CFG_ERR_RANGE, "★ 重绑不存在的 UID -> CFG_ERR_RANGE (实际 %d)", (int)r);
    CHECK(config_store_get()->sensor_count == 2u, "失败的重绑不改变条目数");

    /* ---- ⑥ 参数防护 ---- */
    CHECK(config_store_rebind_device(NULL, "s", "n", 0u, 0u) == CFG_ERR_PARAM,
          "uid=NULL -> CFG_ERR_PARAM");
    CHECK(config_store_rebind_device("ZZZZ", "s", "n", 0u, 0u) == CFG_ERR_PARAM,
          "uid 非法 -> CFG_ERR_PARAM");
    CHECK(config_store_rebind_device("21000000000000000000000001", NULL, NULL, 0u, 4000u) == CFG_OK,
          "serial/name 传 NULL 当空串(与 bind 同口径)");
    CHECK(strcmp(config_store_get()->sensors[0].serial, "") == 0, "NULL -> 空串");

    /* ---- ⑦ ★★ 设备侧重绑要连 type 一起更新 ★★ ---- */
    (void)config_store_bind_device("31000000000000000000000001", "SN-ALM", "声光1", false,
                                   CFG_DEVTYPE_SOUND_LIGHT, 5000u);
    r = config_store_rebind_device("31000000000000000000000001", "SN-ALM2", "声光1(新)", 2u, 6000u);
    CHECK(r == CFG_OK, "重绑声光设备 -> CFG_OK");
    c = config_store_get_mut();
    CHECK(c->device_count == 1u, "★ 设备条目数不变(仍是 1)");
    CHECK(c->devices[0].type == 2u, "★ 设备 type 被更新成 2 (实际 %u)", (unsigned)c->devices[0].type);
    CHECK(strcmp(c->devices[0].name, "声光1(新)") == 0, "★ 设备 name 被覆盖");
}

/* ================================================================= */
int main(void)
{
    printf("=== config_store 主机单测 (P2-2 / P2-4) ===\n");

    cjson_port_init();      /* v33-2: 写配置已改用 cJSON, 必须先装静态池 hooks */

    /* === 临时诊断(2026-10-18): 一条 cfg.set 全回 1002(版本冲突) 的根因定位 === */
    printf("  [diag] version=%u validate=%d sensor0_uid_len=%u\n",
           (unsigned)config_store_version(),
           (int)config_store_validate(config_store_get()),
           (unsigned)strlen(config_store_get()->sensors[0].uid));

    test_factory_default();
    test_roundtrip();
    test_full_config();
    test_defaults_for_partial_elements();
    test_merge_incremental();
    test_validation();
    test_dual_bank();
    test_version_and_reset();
    test_merge_sections_core();
    test_merge_sections_per_domain();
    test_merge_sections_devices();
    test_parse_gears_by_gear_field();
    test_write_members();
    test_rebind_device();

    printf("\n==== 通过 %d, 失败 %d ====\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
