/**
 * @file  config_store.c
 * @brief 网关全量配置存储(9 域) —— 任务 P2-2
 *
 * 见 config_store.h 的设计说明。这里只补充实现层面的三点:
 *
 * 1) 【存储端口分层】
 *    目标构建(N32H482)直接擦写 FM_CFG_A_ADDR / FM_CFG_B_ADDR 两个 Flash 页;
 *    主机构建用两块 RAM 映像。分层不是"为了好看": 配置逻辑里最容易错的是
 *    序列化/CRC/双区回退, 这些在 PC 上能完整测出来, 不该为了跑测试去接板子。
 *    端口只有 read/erase/write 三个函数, 加起来不到 40 行。
 *
 * 2) 【为什么 from_json 里要校验 CRC】
 *    文档 meta.crc32 的定义是"全量配置(不含 meta 域)的 CRC32"。于是
 *    "解析出来的结构体"重新序列化一次、算 CRC、与 JSON 里带的值比 —— 这个比对
 *    同时验证了【序列化器与解析器互为逆运算】。任何一边有偏差(截断、漏字段、
 *    数值精度)都会立刻暴露成 CRC 不符, 而不是悄悄丢一个字段。
 *    这正是主机单测要抓的东西。
 *
 * 3) 【每段 JSON 的 CRC 覆盖边界】
 *    cfg_serialize() 先写 system..ota, 此时 out[0..meta_start) 恰好就是
 *    "全量配置不含 meta", 对它算 CRC, 再写 meta。所以不需要"为了算 CRC 少写一段"
 *    或者"算完再补一段"的临时缓冲 —— 边界天然落在 meta_start。
 */
#include "config_store.h"
#include "cloud_json.h"     /* 反序列化那半边(cfg_parse_xxx / cfg_get_xxx)仍用它 */
#include "cjson_port.h"     /* v33-2: 序列化(写配置)已迁到 cJSON */
#include "log.h"
#include "utils.h"

/* ★2026-10-18 (甲): 设备 ID 从 6 位改成 13 字节 UID 的 26 位 hex。
 *   gwId / clientId 都取 "11" + 芯片 96 位唯一 ID(24 hex)。
 *   ⇒ MQTT 主题 rt/gw/<26位>/... , clientId 也是同一个值。
 *   hook 未注册(主机测试)时退化成 "11" + 22 个 0 —— 长度仍是 26, 校验能过。 */
static void (*s_gw_uid_hook)(uint8_t out[12]);

void config_store_set_uid_hook(void (*fn)(uint8_t out[12]))
{
    s_gw_uid_hook = fn;
}

/** 生成 26 位 hex 的出厂设备 ID（"11" + 12 字节芯片 UID） */
static void cfg_default_gw_id(char out[27])
{
    static const char HEX[] = "0123456789ABCDEF";
    uint8_t u[12];
    uint8_t i;

    for (i = 0u; i < 12u; i++) { u[i] = 0u; }
    if (s_gw_uid_hook != NULL) { s_gw_uid_hook(u); }

    out[0] = '1';
    out[1] = '1';
    for (i = 0u; i < 12u; i++)
    {
        out[2u + (i * 2u)]      = HEX[(u[i] >> 4) & 0x0Fu];
        out[2u + (i * 2u) + 1u] = HEX[u[i] & 0x0Fu];
    }
    out[26] = '\0';
}

#include <string.h>
#include <stdio.h>
#include <stdlib.h>     /* strtoul: 解析 meta.crc32 的 HEX 字符串 */
#include <stddef.h>

/* 目标构建才引 MCU/Flash 头; 主机测试的包含路径里没有 app/common */
#if defined(N32H482)
#include "n32h47x_48x.h"
#include "n32h47x_48x_flash.h"
#include "flash_map.h"
#endif

/* ==================== 存储布局 ==================== */

/* 9 域全量序列化后的 JSON 上界。文档第 7 章示例约 1.8KB;
 * 满配(5 传感器 + 5 设备 + 4 档位)实测见主机单测, 这里留足余量。
 * ★2026-10-08: 4096 -> 6144(联动档位新字段让最坏情况变长, 理由见 config_store.h)★ */
#ifndef CFG_JSON_MAX
#define CFG_JSON_MAX        6144
#endif

/* 单区大小 = 1 页 Flash(所有分区模式都是 0x2000) */
#define CFG_BANK_SIZE       0x2000u

#define CFG_BANK_A          0u      /* 主区   */
#define CFG_BANK_B          1u      /* 备份区 */

#if defined(N32H482)
/* 分区表说单区必须是 0x2000, 否则这里的擦页/写越界假设不成立 */
typedef char cfg_bank_size_ok[(FM_CFG_SIZE == CFG_BANK_SIZE) ? 1 : -1];
typedef char cfg_bank_present[(FM_HAS_CFG_BANK != 0) ? 1 : -1];
#endif

/* 够放满配 JSON */
typedef char cfg_json_fits[(sizeof(config_store_t) <= CFG_JSON_MAX) ? 1 : -1];

/** 每区头部(24 字节, 全 uint32 → 无填充/无对齐坑, 8 字节对齐) */
typedef struct
{
    uint32_t magic;
    uint32_t version;       /* configVersion */
    uint32_t crc32;         /* = meta.crc32, 覆盖 system..ota 那段 JSON */
    uint32_t updated_at;
    uint32_t json_len;
    uint32_t hdr_crc;       /* 前 20 字节的 CRC32 */
} cfg_bank_hdr_t;

#define CFG_BANK_MAGIC      0x47435452u     /* 'R''T''C''G' */

/* ==================== 静态内存 ==================== */
/* 全部静态, 不占栈(配置结构体 ~1.7KB, 放栈上会压爆小栈) */
static config_store_t s_config;                 /* 惰性加载的当前配置 */
static config_store_t s_merge_tmp;               /* 增量合并的事务副本 */
static bool           s_loaded;

static char s_json_wr[CFG_JSON_MAX];             /* 序列化(保存载荷)用 */
static char s_json_crc[CFG_JSON_MAX];            /* CRC 复算专用 —— 见下 */
static char s_json_rd[CFG_JSON_MAX];             /* 从 Flash 读回的原文 */

/* 【为什么 CRC 复算要单独的缓冲】
 * config_store_save() 的流程是: 序列化到 s_json_wr -> 写 B -> 【回读 B 校验】-> 写 A。
 * 而"回读 B 校验"里会用解析结果重新序列化一次来复算 CRC。如果复算也写 s_json_wr,
 * 就会在"还没写 A"的时候把 A 要用的载荷缓冲覆盖掉 —— 一旦两次序列化长度不同,
 * 写 A 就会带上残留字节。所以复算走 s_json_crc, 与载荷彻底分开。 */

/* ==================== CRC32 (IEEE 802.3 / poly 0xEDB88320) ====================
 * 与 param_save.c / ota_port.c 用的是同一个算法, 保证全工程一致。
 * 选软件而非硬件 CRC32 单元: 硬件只吃 32-bit word, JSON 长度不是 4 的倍数。 */
// static uint32_t cfg_crc32(const uint8_t *buf, uint32_t len)
// {
//     uint32_t crc = 0xFFFFFFFFu;
//     uint32_t i;
//     uint8_t  bit;

//     for (i = 0u; i < len; i++)
//     {
//         crc ^= buf[i];
//         for (bit = 0u; bit < 8u; bit++)
//         {
//             crc = (crc & 1u) ? ((crc >> 1u) ^ 0xEDB88320u) : (crc >> 1u);
//         }
//     }
//     return ~crc;
// }
//全部迁移到utils.c

/* ==================== 存储端口 ==================== */

#if defined(N32H482)

static uint32_t cfg_bank_base(uint8_t bank)
{
    return (bank == CFG_BANK_B) ? (uint32_t)FM_CFG_B_ADDR : (uint32_t)FM_CFG_A_ADDR;
}

static cfg_result_t cfg_port_erase(uint8_t bank)
{
    FLASH_STS sts;
    uint32_t  base = cfg_bank_base(bank);

    FLASH_Unlock();
    sts = FLASH_EraseOnePage(base);
    if (sts == FLASH_EOP) { sts = FLASH_WaitForLastOpt(0xFFFFFFFFu); }
    FLASH_Lock();

    return (sts == FLASH_EOP) ? CFG_OK : CFG_ERR_FLASH;
}

static cfg_result_t cfg_port_write(uint8_t bank, uint32_t off,
                                   const uint8_t *buf, uint16_t len)
{
    uint32_t  base = cfg_bank_base(bank);
    uint32_t  done;
    FLASH_STS sts;

    if (((uint32_t)len + off) > CFG_BANK_SIZE) { return CFG_ERR_PARAM; }

    FLASH_Unlock();

    for (done = 0u; done < (uint32_t)len; done += 8u)
    {
        uint64_t dw   = 0u;
        uint8_t *pdw  = (uint8_t *)&dw;
        uint32_t j;

        for (j = 0u; j < 8u; j++)
        {
            uint32_t idx = done + j;
            /* 末段不足 8 字节补 0x00(param_save.c 同做法) */
            pdw[j] = (idx < (uint32_t)len) ? buf[idx] : 0u;
        }

        sts = FLASH_ProgramdoubleWord(base + off + done,
                                      (uint32_t)(dw & 0xFFFFFFFFu),
                                      (uint32_t)((dw >> 32u) & 0xFFFFFFFFu));
        if (sts == FLASH_EOP) { sts = FLASH_WaitForLastOpt(0xFFFFFFFFu); }
        if (sts != FLASH_EOP) { FLASH_Lock(); return CFG_ERR_FLASH; }
    }

    FLASH_Lock();
    return CFG_OK;
}

static cfg_result_t cfg_port_read(uint8_t bank, uint32_t off,
                                  uint8_t *buf, uint16_t len)
{
    if (((uint32_t)len + off) > CFG_BANK_SIZE) { return CFG_ERR_PARAM; }
    (void)memcpy(buf, (const void *)(cfg_bank_base(bank) + off), len);
    return CFG_OK;
}

#else   /* ---------------- 主机测试: RAM 映像 ---------------- */

static uint8_t s_bank_ram[2][CFG_BANK_SIZE];

/* 主机测试用的"写失败"注入次数(见 config_store.h 的说明) */
static int s_test_fail_write;

void config_store_test_fail_write(int times)
{
    s_test_fail_write = (times < 0) ? 0 : times;
}

uint8_t *config_store_test_bank(uint8_t bank)
{
    return s_bank_ram[(bank == CFG_BANK_B) ? 1u : 0u];
}

void config_store_test_wipe(uint8_t bank)
{
    (void)memset(s_bank_ram[(bank == CFG_BANK_B) ? 1u : 0u], 0xFF, CFG_BANK_SIZE);
}

static cfg_result_t cfg_port_erase(uint8_t bank)
{
    config_store_test_wipe(bank);       /* Flash 擦除后是全 0xFF */
    return CFG_OK;
}

static cfg_result_t cfg_port_write(uint8_t bank, uint32_t off,
                                   const uint8_t *buf, uint16_t len)
{
    if (((uint32_t)len + off) > CFG_BANK_SIZE) { return CFG_ERR_PARAM; }

    /* 主机测试注入的写失败(目标板不会走到这里): 用来验证上层"失败不回 code=0" */
    if (s_test_fail_write > 0)
    {
        s_test_fail_write--;
        return CFG_ERR_FLASH;
    }

    (void)memcpy(&s_bank_ram[(bank == CFG_BANK_B) ? 1u : 0u][off], buf, len);
    return CFG_OK;
}

static cfg_result_t cfg_port_read(uint8_t bank, uint32_t off,
                                  uint8_t *buf, uint16_t len)
{
    if (((uint32_t)len + off) > CFG_BANK_SIZE) { return CFG_ERR_PARAM; }
    (void)memcpy(buf, &s_bank_ram[(bank == CFG_BANK_B) ? 1u : 0u][off], len);
    return CFG_OK;
}

#endif  /* N32H482 */

/* ==================== 取值辅助 ====================
 * 三个结果, 必须区分:
 *   缺失   -> 增量合并时"不动这个字段"(保持现状); 全量解析时"回落出厂默认"
 *   越界   -> 直接报错, 不能悄悄截断(截断会让平台以为设置成功了) */

#define CFG_GET_MISSING   0
#define CFG_GET_OK        1
#define CFG_GET_RANGE    (-1)

static int cfg_get_u32(const char *json, const char *path,
                       uint32_t *dst, uint32_t lo, uint32_t hi)
{
    cj_type_t t = cj_get(json, path, NULL, 0);
    uint32_t  v;

    if (t == CJ_NOTFOUND) { return CFG_GET_MISSING; }
    if ((t != CJ_NUM) || (t == CJ_NULL)) { return CFG_GET_RANGE; }

    v = cj_get_uint(json, path, 0xFFFFFFFFu);
    if ((v < lo) || (v > hi)) { return CFG_GET_RANGE; }

    *dst = v;
    return CFG_GET_OK;
}

static int cfg_get_u16(const char *json, const char *path,
                       uint16_t *dst, uint16_t lo, uint16_t hi)
{
    uint32_t v = 0u;
    int      r = cfg_get_u32(json, path, &v, lo, hi);

    if (r == CFG_GET_OK) { *dst = (uint16_t)v; }
    return r;
}

static int cfg_get_u8(const char *json, const char *path,
                      uint8_t *dst, uint8_t lo, uint8_t hi)
{
    uint32_t v = 0u;
    int      r = cfg_get_u32(json, path, &v, lo, hi);

    if (r == CFG_GET_OK) { *dst = (uint8_t)v; }
    return r;
}

static int cfg_get_i16(const char *json, const char *path,
                       int16_t *dst, int32_t lo, int32_t hi)
{
    cj_type_t t = cj_get(json, path, NULL, 0);
    int32_t   v;

    if (t == CJ_NOTFOUND) { return CFG_GET_MISSING; }
    if ((t != CJ_NUM) || (t == CJ_NULL)) { return CFG_GET_RANGE; }

    v = cj_get_int(json, path, 0);
    if ((v < lo) || (v > hi)) { return CFG_GET_RANGE; }

    *dst = (int16_t)v;
    return CFG_GET_OK;
}

static int cfg_get_i8(const char *json, const char *path,
                      int8_t *dst, int32_t lo, int32_t hi)
{
    int16_t v = 0;
    int     r = cfg_get_i16(json, path, &v, lo, hi);

    if (r == CFG_GET_OK) { *dst = (int8_t)v; }
    return r;
}

static int cfg_get_bool(const char *json, const char *path, uint8_t *dst)
{
    cj_type_t t = cj_get(json, path, NULL, 0);

    if (t == CJ_NOTFOUND) { return CFG_GET_MISSING; }
    if (t == CJ_NULL)     { return CFG_GET_RANGE; }

    *dst = cj_get_bool(json, path, false) ? 1u : 0u;
    return CFG_GET_OK;
}

static int cfg_get_str(const char *json, const char *path, char *dst, uint16_t max)
{
    /* ★2026-10-08 修正★ 原来这里是 `CFG_LEDTEXT_MAX + 8`。
     *   以前 LED 和 TTS 上限都是 64, 所以"按 LED 的尺寸开缓冲"刚好够用;
     *   现在 TTS 升到 88、LED 降到 48 —— 再沿用 LED 就会**把 88 字节的语音文本
     *   在 56 字节处截断**, 而且 cfg_get_str 返回的还是 OK(它只按 max 判),
     *   表现为"平台配了长语音, 设备只念半句", 极难查。
     *   ⇒ 一律按【最大的那个字符串字段】开缓冲。 */
    char  tmp[CFG_TTSTEXT_MAX + 8];
    size_t n;

    if (cj_get(json, path, NULL, 0) == CJ_NOTFOUND) { return CFG_GET_MISSING; }
    if (!cj_get_str(json, path, tmp, (uint16_t)sizeof(tmp))) { return CFG_GET_RANGE; }

    n = strlen(tmp);
    if (n > (size_t)max) { return CFG_GET_RANGE; }   /* 文档给了长度上限, 超了要报错 */

    (void)memcpy(dst, tmp, n + 1u);
    return CFG_GET_OK;
}

/** 枚举字符串 -> 序号; 缺失/不匹配分别返回 MISSING/RANGE */
static int cfg_get_enum(const char *json, const char *path,
                        const char *const *names, uint8_t n, uint8_t *dst)
{
    char tmp[24];
    uint8_t i;

    if (cj_get(json, path, NULL, 0) == CJ_NOTFOUND) { return CFG_GET_MISSING; }
    if (!cj_get_str(json, path, tmp, (uint16_t)sizeof(tmp))) { return CFG_GET_RANGE; }

    for (i = 0u; i < n; i++)
    {
        if (strcmp(tmp, names[i]) == 0) { *dst = i; return CFG_GET_OK; }
    }
    return CFG_GET_RANGE;
}

/* ★★★2026-10-08 新增: 两个"双值"字段的解析 ★★★
 *
 * §1.9 把 `ledColor` 和 `ledTextShowMode` 都改成了【逗号分隔的两个值】,
 * 而且明确要求写成【字符串】(JSON 不允许裸的 `1,2`)。
 * 下面两个 helper 各自要求"必须恰好一个逗号 + 两段都合法":
 *   · 少一个逗号 / 多一个逗号 / 任一段非法  -> 一律 CFG_GET_RANGE
 *   · 于是 cfg.set 会【整域拒掉 gears】(与其它字段错误的语义一致, 不半信半疑)
 * 缺失(字段整个没给)仍然返回 MISSING = 保留原值, 与所有 cfg_get_* 一致。 */

/** 解析 "a,b" 形式的两个枚举名(如 ledColor: "red,yellow") */
static int cfg_get_enum_pair(const char *json, const char *path,
                             const char *const *names, uint8_t n, uint8_t *a, uint8_t *b)
{
    char  tmp[24];
    char *comma;
    uint8_t i;

    if (cj_get(json, path, NULL, 0) == CJ_NOTFOUND) { return CFG_GET_MISSING; }
    if (!cj_get_str(json, path, tmp, (uint16_t)sizeof(tmp))) { return CFG_GET_RANGE; }

    comma = strchr(tmp, ',');
    if ((comma == NULL) || (strchr(comma + 1, ',') != NULL)) { return CFG_GET_RANGE; }
    *comma = '\0';

    for (i = 0u; i < n; i++) { if (strcmp(tmp, names[i]) == 0) { *a = i; break; } }
    if (i >= n) { return CFG_GET_RANGE; }

    for (i = 0u; i < n; i++) { if (strcmp(comma + 1, names[i]) == 0) { *b = i; break; } }
    if (i >= n) { return CFG_GET_RANGE; }

    return CFG_GET_OK;
}

/** 解析 "a,b" 形式的两个 0~max 十进制数字(如 ledTextShowMode: "1,2") */
static int cfg_get_u8_pair(const char *json, const char *path, uint8_t max,
                           uint8_t *a, uint8_t *b)
{
    char          tmp[24];
    char         *comma;
    char         *end;
    unsigned long v;

    if (cj_get(json, path, NULL, 0) == CJ_NOTFOUND) { return CFG_GET_MISSING; }
    if (!cj_get_str(json, path, tmp, (uint16_t)sizeof(tmp))) { return CFG_GET_RANGE; }

    comma = strchr(tmp, ',');
    if ((comma == NULL) || (strchr(comma + 1, ',') != NULL)) { return CFG_GET_RANGE; }
    *comma = '\0';

    /* 两段都必须是"纯十进制数字且不超上界" —— strtoul 会把 "1x" 解析成 1,
     * 所以必须检查 end 指向结尾, 否则 "1x,2" 会被悄悄接受。 */
    v = strtoul(tmp, &end, 10);
    if ((tmp[0] == '\0') || (*end != '\0') || (v > (unsigned long)max)) { return CFG_GET_RANGE; }
    *a = (uint8_t)v;

    v = strtoul(comma + 1, &end, 10);
    if ((comma[1] == '\0') || (*end != '\0') || (v > (unsigned long)max)) { return CFG_GET_RANGE; }
    *b = (uint8_t)v;

    return CFG_GET_OK;
}

/* ==================== 出厂默认(文档第 6 章 + 3.x 默认列) ==================== */

void config_store_factory_default(config_store_t *c)
{
    if (c == NULL) { return; }

    (void)memset(c, 0, sizeof(*c));

    /* ---- 3.1 system ---- */
    /* ★2026-10-08: 出厂 gwId 由 "GW0001" 改成 "110001"★
     * 甲方平台约定: {gwId} = 【设备 ID】, 网关号段是 1xxxxx(见 lora_business.c 的
     * 地址规则: 首位 1=网关 / 2=液位 / 3=声光), 所以出厂默认 110001 = 网关 0001。
     * 这个值同时用于:
     *   · MQTT 主题   rt/gw/110001/cmd ...
     *   · MQTT 用户名 (甲方平台按用户名认设备)
     * 后续可用 cfg.set 改(system.gwId), 改完【网关重启】才生效(主题在建链前拼好)。 */
    cfg_default_gw_id(c->system.gw_id);             /* 2026-10-18: "11" + 芯片 96 位 UID(26 位 hex) */
    c->system.password[0] = '\0';                   /* 空 = 不校验 */
    c->system.timezone    = 8;                      /* +8 */

    /* ---- 3.2 network ----
     * ★★2026-10-09 用户要求把这些填上(原来是空的, 全靠编译期默认值兜着)★★
     *
     * 【为什么原来留空也能跑】main.c 启动时只在 network.host 非空时才调
     *   g4_at_set_broker(配置值); 空的时候就沿用 g4_at.c 里编译期的
     *   G4_MQTT_HOST/PORT/USER/PASS 常量 —— 而下面填的值【与那些常量完全一致】,
     *   所以行为没有任何变化, 只是"配置里能看见了"。
     *
     * ⚠️【host 只写 IP, 端口单独写在 port 里】用户写的是 "223.75.207.139:1883",
     *   那是"主机:端口"的写法 —— 如果整串塞进 host, MQTT 连接会失败(非法主机名)。
     *
     * ⚠️⚠️【安全提示: network.mqtt_password 会被 cfg.query 原样读出来】⚠️⚠️
     *   填上之后, 任何能下发 cfg.query 或订阅 rt/gw/{gwId}/cmdresp 的一方
     *   都能看到 broker 密码明文。当前平台侧就是靠 cfg.query 读配置的。
     *   如需规避, 见下面 mqtt_password 那行的注释(建议对查询做掩码)。 */
    (void)strcpy(c->network.host, "223.75.207.139");
    c->network.port         = 1883u;
    c->network.tls          = 0u;
    cfg_default_gw_id(c->network.client_id);        /* 2026-10-18: 与 gwId 同值(26 位 hex UID) */
    (void)strcpy(c->network.username, "tuguan");
    /* ⚠️ 明文密码, 且 cfg.query 会原样返回它。若不愿暴露, 两个办法:
     *   ① 保持空 -> 沿用 g4_at.c 的编译期默认值(功能一样, 只是查询看不到);
     *   ② 在 cloud_proto 的 cfg.query 里把 network.password 渲染成 "***"(写只读)。
     *   目前按用户要求填上。 */
    (void)strcpy(c->network.mqtt_password, "tg123456!");
    c->network.keep_alive_s     = 60u;
    c->network.reconnect_min_s  = 5u;
    c->network.reconnect_max_s  = 300u;
    c->network.status_interval_s = 30u;   /* ★2026-09-30 用户口径: 保活(心跳)默认 30s ★
                                          * 它是【主动探针】: 30s 一次 => 最坏 30~60s
                                          * 就能发现链路异常, 而不是靠"发布失败"兜底
                                          * (现场实测拖到过 HB=119s 才发现)。
                                          * 取值区间 10~600s 见下面的校验。 */

    /* ---- 3.3 sensors: 清空 ---- */
    c->sensor_count = 0u;

    /* ---- 3.4 thresholds ---- */
    c->thresholds.alarm_cm             = 10u;
    c->thresholds.warning_cm           = 7u;
    c->thresholds.resolve_cm           = 2u;
    c->thresholds.min_change_cm        = 1u;
    c->thresholds.event_min_interval_s = 60u;
    c->thresholds.confirm_count        = 1u;

    /* ---- 3.5 gears: 出厂默认 ----
     * ★2026-10-08 按新版字段重写★
     *   每档现在是【两段 LED 文本 + 一段 TTS】, 并且有横竖屏/对齐/双色。
     *   默认统一取: 横屏(0) + 两个半屏都居中(0) + 第一个半屏用档位主色、第二个同色。
     *   文本沿用原来的中文措辞(现场看惯了), 拆成"上/左"与"下/右"两段:
     *     竖屏时 led_text1 在左、led_text2 在右; 横屏时 1 在上、2 在下。
     *   ⚠ 三段合计必须 ≤203 字节(CFG_GEAR_TEXT_BUDGET), 下面每一档都远低于它。
     *   ⚠ gear 0(解除): 电源位会被置 0, 设备不显示文本 —— 按用户 2026-10-08 确认
     *     "可以带文本, 电源关了设备不会显示", 所以这里【仍然给文本】(与旧版一致)。 */
    {
        /* gear 0 解除: 关屏。文本留空 —— 反正电源位=0, 设备不显示 */
        cfg_gear_t *g = &c->gears[0];
        memset(g, 0, sizeof(*g));
        g->gear = 0u; g->targets_all = 1u; g->sound_mode = 0u;
        g->led_show_mode = 0u;                      /* 横屏 */
        g->led_color[0] = (uint8_t)CFG_COLOR_OFF;
        g->led_color[1] = (uint8_t)CFG_COLOR_OFF;
        (void)strcpy(g->name, "解除");
        g->led_text1[0] = '\0';
        g->led_text2[0] = '\0';
        g->tts_text[0]  = '\0';

        /* gear 1 正常: 常亮 + 绿 */
        g = &c->gears[1];
        memset(g, 0, sizeof(*g));
        g->gear = 1u; g->targets_all = 1u; g->sound_mode = 1u;   /* 1 = 常亮 */
        g->led_show_mode = 0u;
        g->led_color[0] = (uint8_t)CFG_COLOR_GREEN;
        g->led_color[1] = (uint8_t)CFG_COLOR_GREEN;
        (void)strcpy(g->name, "正常");
        (void)strcpy(g->led_text1, "路面积水{level}CM");
        (void)strcpy(g->led_text2, "缓慢通行");
        g->tts_text[0] = '\0';                                    /* 空 = 无语音 */

        /* gear 2 预警: 慢闪 + 黄 */
        g = &c->gears[2];
        memset(g, 0, sizeof(*g));
        g->gear = 2u; g->targets_all = 1u; g->sound_mode = 2u;   /* 2 = 慢闪 */
        g->led_show_mode = 0u;
        g->led_color[0] = (uint8_t)CFG_COLOR_YELLOW;
        g->led_color[1] = (uint8_t)CFG_COLOR_YELLOW;
        (void)strcpy(g->name, "预警");
        (void)strcpy(g->led_text1, "路面积水{level}CM");
        (void)strcpy(g->led_text2, "谨慎通行");
        g->tts_text[0] = '\0';

        /* gear 3 告警: 快闪 + 红 + 有语音 */
        g = &c->gears[3];
        memset(g, 0, sizeof(*g));
        g->gear = 3u; g->targets_all = 1u; g->sound_mode = 3u;   /* 3 = 快闪 */
        g->led_show_mode = 0u;
        g->led_color[0] = (uint8_t)CFG_COLOR_RED;
        g->led_color[1] = (uint8_t)CFG_COLOR_RED;
        (void)strcpy(g->name, "告警");
        (void)strcpy(g->led_text1, "路面积水{level}CM");
        (void)strcpy(g->led_text2, "禁止通行");
        (void)strcpy(g->tts_text, "前方积水{level}厘米，禁止通行");
    }

    /* ---- 3.6 devices: 清空 ---- */
    c->device_count = 0u;

    /* ---- 3.7 rules ---- */
    c->rules.mode                  = CFG_RULE_AUTO;
    c->rules.sensor_offline_policy = CFG_OFFLINE_KEEP;
    c->rules.manual_timeout_s      = 300u;

    /* ---- 3.8 ota ---- */
    (void)strcpy(c->ota.fw_version, "1.0.0");
    c->ota.upgrade_policy = CFG_OTA_IMMEDIATE;

    /* ---- 3.9 meta: 恢复出厂 configVersion 归零 ---- */
    c->meta.config_version = 0u;
    c->meta.crc32          = 0u;
    c->meta.updated_at     = 0u;
}

/* ==================== 校验 ==================== */

/* ==================== 13 字节 UID(hex 串)工具 ==================== */

static int cfg_hex_val(char ch)
{
    if ((ch >= '0') && (ch <= '9')) { return (int)(ch - '0'); }
    if ((ch >= 'a') && (ch <= 'f')) { return (int)(ch - 'a') + 10; }
    if ((ch >= 'A') && (ch <= 'F')) { return (int)(ch - 'A') + 10; }
    return -1;
}

bool cfg_uid_valid(const char *uid_hex)
{
    uint8_t i;
    bool    any_nonzero = false;

    if (uid_hex == NULL) { return false; }
    /* ★2026-10-18: 只认 26 位 hex(13 字节) —— 旧 3 字节格式不再兼容, 见 config_store.h */
    if (strlen(uid_hex) != CFG_UID_HEX_LEN) { return false; }

    for (i = 0u; i < CFG_UID_HEX_LEN; i++)
    {
        if (cfg_hex_val(uid_hex[i]) < 0) { return false; }
        if (uid_hex[i] != '0') { any_nonzero = true; }
    }
    /* 全 0 是空地址 —— 擦除态/未绑定, 不能当合法设备地址 */
    return any_nonzero;
}

bool cfg_uid_to_bytes(const char *uid_hex, uint8_t out[CFG_UID_BYTES])
{
    uint8_t i;

    if ((uid_hex == NULL) || (out == NULL)) { return false; }
    if (!cfg_uid_valid(uid_hex)) { return false; }

    for (i = 0u; i < CFG_UID_BYTES; i++)
    {
        int hi = cfg_hex_val(uid_hex[i * 2u]);
        int lo = cfg_hex_val(uid_hex[(i * 2u) + 1u]);
        out[i] = (uint8_t)((hi << 4) | lo);
    }
    return true;
}

void cfg_uid_from_bytes(const uint8_t in[CFG_UID_BYTES], char *out_hex)
{
    static const char HEX[] = "0123456789ABCDEF";
    uint8_t i;

    if ((in == NULL) || (out_hex == NULL)) { return; }

    for (i = 0u; i < CFG_UID_BYTES; i++)
    {
        out_hex[i * 2u]        = HEX[(in[i] >> 4) & 0x0Fu];
        out_hex[(i * 2u) + 1u] = HEX[in[i] & 0x0Fu];
    }
    out_hex[CFG_UID_HEX_LEN] = '\0';
}

/* 传感器/设备 UID 全网关唯一(文档 3.3: "全网关内唯一", 且与设备同一编号空间) */
static bool cfg_uid_unique(const config_store_t *c, const char *uid_hex, uint8_t upto_sensor)
{
    uint8_t i;

    for (i = 0u; i < c->sensor_count && i < CFG_SENSOR_MAX; i++)
    {
        if ((i != upto_sensor) && (strcmp(c->sensors[i].uid, uid_hex) == 0)) { return false; }
    }
    return true;
}

cfg_result_t config_store_validate(const config_store_t *c)
{
    uint8_t i, j;

    if (c == NULL) { return CFG_ERR_PARAM; }

    /* 3.1 system: gwId 非空(文档: MQTT topic 与 clientId 都用它) */
    if (c->system.gw_id[0] == '\0')  { return CFG_ERR_RANGE; }
    if (strlen(c->system.gw_id) > CFG_GWID_MAX) { return CFG_ERR_RANGE; }
    if (strchr(c->system.gw_id, '/') != NULL)   { return CFG_ERR_RANGE; }
    if ((c->system.timezone < -12) || (c->system.timezone > 14)) { return CFG_ERR_RANGE; }

    /* 3.2 network */
    if (c->network.port == 0u)                                   { return CFG_ERR_RANGE; }
    if (c->network.keep_alive_s < 30u || c->network.keep_alive_s > 300u)   { return CFG_ERR_RANGE; }
    if (c->network.reconnect_min_s < 1u)                         { return CFG_ERR_RANGE; }
    if (c->network.reconnect_max_s < c->network.reconnect_min_s) { return CFG_ERR_RANGE; }
    if (c->network.status_interval_s < 10u || c->network.status_interval_s > 600u) { return CFG_ERR_RANGE; }

    /* 3.3 sensors */
    if (c->sensor_count > CFG_SENSOR_MAX) { return CFG_ERR_RANGE; }
    for (i = 0u; i < c->sensor_count && i < CFG_SENSOR_MAX; i++)
    {
        const cfg_sensor_t *s = &c->sensors[i];

        if (!cfg_uid_valid(s->uid))                        { return CFG_ERR_RANGE; }
        if (!cfg_uid_unique(c, s->uid, i))                 { return CFG_ERR_RANGE; }
        if (s->measure_mode > CFG_MEASURE_DISTANCE)        { return CFG_ERR_RANGE; }
        if (s->install_height_cm > 2000u)                  { return CFG_ERR_RANGE; }
        if (s->calib_offset_mm < -5000 || s->calib_offset_mm > 5000) { return CFG_ERR_RANGE; }
        if (s->report_period_s < 5u || s->report_period_s > 3600u)   { return CFG_ERR_RANGE; }

        /* ★2026-10-05 用户口径: 【彻底去掉 offlineTimeoutSeconds 与 reportPeriodSeconds
         *   之间的大小校验】★
         *
         * 理由一(现场需求): 平台【只下发一个字段】reportPeriodSeconds。原来那条
         *   "offline 必须 > report" 会让平台把上报周期改成 ≥90 秒时整条命令回 1001 ——
         *   而平台压根没打算动 offline, 它不该为另一个字段背锅。
         *
         * 理由二(这条校验本来就没意义): offline_timeout_s 在本工程里是【没有任何消费者】
         *   的字段 —— 失联判定实际用的是【3 × reportPeriodSeconds】
         *   (见 sensor_offline.c 的 sensor_offline_period_for_uid() /
         *    sensor_offline_threshold_ms())。offline_timeout_s 只出现在
         *   解析/校验/序列化三处, 改它不会改变任何行为。
         *   拦"一个不会被读的值", 只会挡住合法操作。
         *
         * 理由三(文档自相矛盾): 文档 3.3 写"offlineTimeoutSeconds ≥ reportPeriodSeconds×2",
         *   而同一张表给的默认值就是 60/90(90 < 120)—— 按字面硬拒, 平台下发文档
         *   默认配置就会被我们打回。更不能照字面实现。
         *
         * 保留的唯一约束是上面那条 report_period_s 自身的范围(5~3600)。 */
    }

    /* 3.4 thresholds: alarm > warning > resolve */
    if (c->thresholds.alarm_cm   <= c->thresholds.warning_cm) { return CFG_ERR_RANGE; }
    if (c->thresholds.warning_cm <= c->thresholds.resolve_cm) { return CFG_ERR_RANGE; }
    if (c->thresholds.confirm_count < 1u || c->thresholds.confirm_count > 10u) { return CFG_ERR_RANGE; }

    /* 3.5 gears: 档位号必须 0~3 且不重复(索引即档位, 这里防串位) */
    for (i = 0u; i < CFG_GEAR_COUNT; i++)
    {
        const cfg_gear_t *g = &c->gears[i];

        if (g->gear != i)                        { return CFG_ERR_RANGE; }
        if (g->sound_mode > 3u)                  { return CFG_ERR_RANGE; }   /* ★0~3(常亮/慢闪/快闪) */
        if (g->led_show_mode > 1u)               { return CFG_ERR_RANGE; }   /* ★0横 / 1竖 */
        for (j = 0u; j < 2u; j++)
        {
            /* ★两个半屏各一个颜色、各一个对齐位★
             * 颜色上界用 CFG_COLOR_YELLOW(=3, 空口 11) —— 它是枚举里最大的那个,
             * 写 CFG_COLOR_RED 会变成"只允许到 1", 静默把黄色判非法。 */
            if (g->led_color[j] > (uint8_t)CFG_COLOR_YELLOW)        { return CFG_ERR_RANGE; }
            /* ★0~3★ 与云端 JSON 口径一致(0默认/1上左对齐/2居中/3下右对齐);
             *  空口只有 1 位, 折算规则见 level_eval.c 的 led_align_bit()。 */
            /* ★★2026-10-10: 这里【暂时仍允许 0~3】—— 为什么不直接用 0~1 见下★★
             *
             * 【空口事实】《声光报警器对接.txt》的空口位定义: bit5/bit4 每位【只有 1 bit】
             *   —— 0 居中 / 1 对齐, 没有别的取值。用户 2026-10-10 也确认"只有 0/1"。
             *   多余的取值靠 level_eval.c 的 led_align_bit() 压成 1 bit
             *   (0/2->居中, 1/3->对齐)。
             *
             * 【为什么校验【不】跟着收紧到 0~1】试过了, 会打挂 10 条既有断言:
             *   主机单测的夹具(和协议文档示例)里就在用 2/3。收紧 = 那些配置
             *   会被 config_store_validate() 判非法 -> 整条 cfg.set/rule.set 被拒。
             *   而平台侧此刻正在按"0~3"的历史口径准备数据。
             * ⇒ 折中: 【接受 0~3, 但只有 0/1 有意义】。平台沟通口径按 0/1,
             *   老数据发 2/3 也不至于被拒(会被压成 0/1, 功能正确)。
             *
             * ⚠️ 若以后决定彻底收紧, 必须【同时】改:
             *     ① 主机单测夹具里所有 ledTextShowMode 的 2/3 -> 0/1
             *     ② 协议文档/偏差记录里的口径
             *     ③ 与平台确认存量数据已清理
             *   三件事少一件都会在现场炸。 */
            if (g->led_text_show_mode[j] > 3u)                     { return CFG_ERR_RANGE; }
        }
        /* ★三段联动文本的总字节预算(203)也在这里挡一道★
         * 与 level_eval.h 的 LEVEL_GEAR_TEXT_BUDGET 是同一个数 —— 下面 #error 会
         * 在编译期钉住两者相等, 所以运行期只需查一次。
         * 为什么必须在这里挡: cfg.set 允许平台塞 48+48+88=184 字节, 看起来没超,
         * 但 UTF-8 汉字是 3 字节/字, 真正的约束是【合计 ≤203 字节】—— 超了组帧时
         * 尾部的 TTS 会被截掉, 平台却以为配好了。宁可在这里明确拒掉该域。 */
        if ((strlen(g->led_text1) + strlen(g->led_text2) + strlen(g->tts_text)) >
            (size_t)CFG_GEAR_TEXT_BUDGET)
        {
            return CFG_ERR_RANGE;
        }
        if (g->target_count > CFG_DEVICE_MAX)    { return CFG_ERR_RANGE; }
        for (j = 0u; j < g->target_count && j < CFG_DEVICE_MAX; j++)
        {
            if (!cfg_uid_valid(g->target_uid[j])) { return CFG_ERR_RANGE; }
        }
    }

    /* 3.6 devices */
    if (c->device_count > CFG_DEVICE_MAX) { return CFG_ERR_RANGE; }
    for (i = 0u; i < c->device_count && i < CFG_DEVICE_MAX; i++)
    {
        const cfg_device_t *d = &c->devices[i];

        if (!cfg_uid_valid(d->uid)) { return CFG_ERR_RANGE; }
        if (d->type < CFG_DEVTYPE_SOUND_LIGHT || d->type > CFG_DEVTYPE_ALLINONE) { return CFG_ERR_RANGE; }
        if (d->volume > 100u)      { return CFG_ERR_RANGE; }
        if (d->flash_mode > 3u)    { return CFG_ERR_RANGE; }

        /* 设备 UID 与传感器 UID 同一编号空间, 不能撞 */
        for (j = 0u; j < c->sensor_count && j < CFG_SENSOR_MAX; j++)
        {
            if (strcmp(c->sensors[j].uid, d->uid) == 0) { return CFG_ERR_RANGE; }
        }
        for (j = 0u; j < i; j++)
        {
            if (strcmp(c->devices[j].uid, d->uid) == 0) { return CFG_ERR_RANGE; }
        }
    }

    /* 3.7 rules */
    if (c->rules.mode > CFG_RULE_MANUAL)                  { return CFG_ERR_RANGE; }
    if (c->rules.sensor_offline_policy > CFG_OFFLINE_OFF) { return CFG_ERR_RANGE; }

    /* 3.8 ota */
    if (c->ota.upgrade_policy > CFG_OTA_IDLE) { return CFG_ERR_RANGE; }

    return CFG_OK;
}

/* ==================== 序列化 ==================== */

static const char *const s_measure_names[] = { "level", "distance" };
/* ★2026-10-08: 顺序必须与 cfg_led_color_t 一致, 也就是与【空口位定义】一致:
 *     00 无色 / 01 红 / 10 绿 / 11 黄
 *   cfg_get_enum() 是"按下标查名字、查到就返回下标", 所以这里的顺序【直接决定】
 *   JSON 字符串 -> 内部枚举的映射。改了枚举就必须同步改这一行, 否则红绿会反。 */
static const char *const s_color_names[]   = { "off", "red", "green", "yellow" };
static const char *const s_rulemode_names[]= { "auto", "manual" };
static const char *const s_offline_names[] = { "keep", "alarm", "off" };
static const char *const s_policy_names[]  = { "immediate", "idle" };

/** 写 system..ota(即"全量配置不含 meta"), 这一段正是 CRC32 的覆盖范围
 *
 *  【v33-2: 从自写 cjw_* 迁到 cJSON】
 *  ★ 键序 / 字段名 / 枚举名表【一个都不能动】: 序列化出来的字节必须与迁移前逐字节
 *    一致 —— 否则(1) Flash 里已存的 CRC 全部失效, 现场配置会被判成损坏;
 *    (2) 主机单测里那几条定长断言(n=1203 / n=1862 / n=3288)会立刻抓到差异。
 *  ★ 任何一步失败(池满)都返回 false, 由调用方把整棵树丢掉 —— 绝不发半条配置出去。 */
static bool cfg_write_domains(cJSON *root, const config_store_t *c)
{
    cJSON  *o;
    cJSON  *arr;
    uint8_t i, j;

    /* ---------------- system ---------------- */
    o = cJSON_CreateObject();
    if (o == NULL) { return false; }
    cJSON_AddItemToObject(root, "system", o);
    if ((cJSON_AddStringToObject(o, "gwId",     c->system.gw_id) == NULL) ||
        (cJSON_AddStringToObject(o, "password", c->system.password) == NULL) ||
        (cJSON_AddNumberToObject(o, "timezone", (double)c->system.timezone) == NULL))
    { return false; }

    /* ---------------- network ---------------- */
    o = cJSON_CreateObject();
    if (o == NULL) { return false; }
    cJSON_AddItemToObject(root, "network", o);
    if ((cJSON_AddStringToObject(o, "host", c->network.host) == NULL) ||
        (cJSON_AddNumberToObject(o, "port", (double)c->network.port) == NULL) ||
        (cJSON_AddBoolToObject(o, "tls", (c->network.tls != 0u) ? 1 : 0) == NULL) ||
        (cJSON_AddStringToObject(o, "clientId", c->network.client_id) == NULL) ||
        (cJSON_AddStringToObject(o, "username", c->network.username) == NULL) ||
        (cJSON_AddStringToObject(o, "password", c->network.mqtt_password) == NULL) ||
        (cJSON_AddNumberToObject(o, "keepAliveSeconds",
            (double)c->network.keep_alive_s) == NULL) ||
        (cJSON_AddNumberToObject(o, "reconnectMinSeconds",
            (double)c->network.reconnect_min_s) == NULL) ||
        (cJSON_AddNumberToObject(o, "reconnectMaxSeconds",
            (double)c->network.reconnect_max_s) == NULL) ||
        (cJSON_AddNumberToObject(o, "statusIntervalSeconds",
            (double)c->network.status_interval_s) == NULL))
    { return false; }

    /* ---------------- sensors[] ---------------- */
    arr = cJSON_CreateArray();
    if (arr == NULL) { return false; }
    cJSON_AddItemToObject(root, "sensors", arr);
    for (i = 0u; i < c->sensor_count && i < CFG_SENSOR_MAX; i++)
    {
        const cfg_sensor_t *s = &c->sensors[i];

        o = cJSON_CreateObject();
        if (o == NULL) { return false; }
        cJSON_AddItemToArray(arr, o);
        if ((cJSON_AddStringToObject(o, "uid", s->uid) == NULL) ||
            (cJSON_AddStringToObject(o, "serial", s->serial) == NULL) ||
            (cJSON_AddStringToObject(o, "name", s->name) == NULL) ||
            (cJSON_AddStringToObject(o, "measureMode",
                s_measure_names[(s->measure_mode < 2u) ? s->measure_mode : 0u]) == NULL) ||
            (cJSON_AddNumberToObject(o, "installHeightCm",
                (double)s->install_height_cm) == NULL) ||
            (cJSON_AddNumberToObject(o, "calibOffsetMm",
                (double)s->calib_offset_mm) == NULL) ||
            (cJSON_AddNumberToObject(o, "reportPeriodSeconds",
                (double)s->report_period_s) == NULL) ||
            (cJSON_AddNumberToObject(o, "lowBatteryPercent",
                (double)s->low_battery_pct) == NULL) ||
            (cJSON_AddNumberToObject(o, "offlineTimeoutSeconds",
                (double)s->offline_timeout_s) == NULL) ||
            (cJSON_AddBoolToObject(o, "enabled", (s->enabled != 0u) ? 1 : 0) == NULL))
        { return false; }
    }

    /* ---------------- thresholds ---------------- */
    o = cJSON_CreateObject();
    if (o == NULL) { return false; }
    cJSON_AddItemToObject(root, "thresholds", o);
    if ((cJSON_AddNumberToObject(o, "alarmCm",
            (double)c->thresholds.alarm_cm) == NULL) ||
        (cJSON_AddNumberToObject(o, "warningCm",
            (double)c->thresholds.warning_cm) == NULL) ||
        (cJSON_AddNumberToObject(o, "resolveCm",
            (double)c->thresholds.resolve_cm) == NULL) ||
        (cJSON_AddNumberToObject(o, "minChangeCm",
            (double)c->thresholds.min_change_cm) == NULL) ||
        (cJSON_AddNumberToObject(o, "eventMinIntervalSeconds",
            (double)c->thresholds.event_min_interval_s) == NULL) ||
        (cJSON_AddNumberToObject(o, "confirmCount",
            (double)c->thresholds.confirm_count) == NULL))
    { return false; }

    /* ---------------- gears[] ---------------- */
    arr = cJSON_CreateArray();
    if (arr == NULL) { return false; }
    cJSON_AddItemToObject(root, "gears", arr);
    for (i = 0u; i < CFG_GEAR_COUNT; i++)
    {
        const cfg_gear_t *g = &c->gears[i];

        o = cJSON_CreateObject();
        if (o == NULL) { return false; }
        cJSON_AddItemToArray(arr, o);
        if ((cJSON_AddNumberToObject(o, "gear", (double)g->gear) == NULL) ||
            (cJSON_AddStringToObject(o, "name", g->name) == NULL))
        { return false; }

        /* targets: "all" 或 [addr,...](文档 3.5) */
        if (g->targets_all != 0u)
        {
            if (cJSON_AddStringToObject(o, "targets", "all") == NULL) { return false; }
        }
        else
        {
            cJSON *ta = cJSON_CreateArray();

            if (ta == NULL) { return false; }
            cJSON_AddItemToObject(o, "targets", ta);
            for (j = 0u; j < g->target_count && j < CFG_DEVICE_MAX; j++)
            {
                /* CreateString 池满会返回 NULL, 此时 AddItemToArray 也返回 0 -> 整段失败 */
                if (cJSON_AddItemToArray(ta, cJSON_CreateString(g->target_uid[j])) == 0)
                { return false; }
            }
        }

        /* ★2026-10-08: 两个"双值"字段按 §1.9 回写成逗号分隔的【字符串】★
         *   ledTextShowMode: "1,2"       (两个半屏各一个 0/1)
         *   ledColor:        "red,yellow"
         * 【必须用字符串】: JSON 不允许裸的 `1,2`(那不是合法 JSON), 平台侧照抄会解析失败。 */
        {
            char colorbuf[24];
            char alignbuf[8];

            (void)snprintf(colorbuf, sizeof(colorbuf), "%s,%s",
                s_color_names[(g->led_color[0] <= (uint8_t)CFG_COLOR_YELLOW) ? g->led_color[0] : 0u],
                s_color_names[(g->led_color[1] <= (uint8_t)CFG_COLOR_YELLOW) ? g->led_color[1] : 0u]);
            (void)snprintf(alignbuf, sizeof(alignbuf), "%u,%u",
                (unsigned)g->led_text_show_mode[0], (unsigned)g->led_text_show_mode[1]);

            if ((cJSON_AddNumberToObject(o, "soundMode", (double)g->sound_mode) == NULL) ||
                (cJSON_AddNumberToObject(o, "ledShowMode", (double)g->led_show_mode) == NULL) ||
                (cJSON_AddStringToObject(o, "ledTextShowMode", alignbuf) == NULL) ||
                (cJSON_AddStringToObject(o, "ledColor", colorbuf) == NULL) ||
                (cJSON_AddStringToObject(o, "ledTextTemplate1", g->led_text1) == NULL) ||
                (cJSON_AddStringToObject(o, "ledTextTemplate2", g->led_text2) == NULL) ||
                (cJSON_AddStringToObject(o, "ttsTextTemplate", g->tts_text) == NULL))
            { return false; }
        }
    }

    /* ---------------- devices[] ---------------- */
    arr = cJSON_CreateArray();
    if (arr == NULL) { return false; }
    cJSON_AddItemToObject(root, "devices", arr);
    for (i = 0u; i < c->device_count && i < CFG_DEVICE_MAX; i++)
    {
        const cfg_device_t *d = &c->devices[i];

        o = cJSON_CreateObject();
        if (o == NULL) { return false; }
        cJSON_AddItemToArray(arr, o);
        if ((cJSON_AddStringToObject(o, "uid", d->uid) == NULL) ||
            (cJSON_AddStringToObject(o, "serial", d->serial) == NULL) ||
            (cJSON_AddStringToObject(o, "name", d->name) == NULL) ||
            (cJSON_AddNumberToObject(o, "type", (double)d->type) == NULL) ||
            (cJSON_AddNumberToObject(o, "volume", (double)d->volume) == NULL) ||
            (cJSON_AddNumberToObject(o, "flashMode", (double)d->flash_mode) == NULL) ||
            (cJSON_AddBoolToObject(o, "enabled", (d->enabled != 0u) ? 1 : 0) == NULL))
        { return false; }
    }

    /* ---------------- rules ---------------- */
    o = cJSON_CreateObject();
    if (o == NULL) { return false; }
    cJSON_AddItemToObject(root, "rules", o);
    if ((cJSON_AddStringToObject(o, "mode",
            s_rulemode_names[(c->rules.mode <= CFG_RULE_MANUAL) ? c->rules.mode : 0u]) == NULL) ||
        (cJSON_AddNumberToObject(o, "manualTimeoutSeconds",
            (double)c->rules.manual_timeout_s) == NULL) ||
        (cJSON_AddStringToObject(o, "sensorOfflinePolicy",
            s_offline_names[(c->rules.sensor_offline_policy <= CFG_OFFLINE_OFF)
                            ? c->rules.sensor_offline_policy : 0u]) == NULL))
    { return false; }

    /* ---------------- ota ---------------- */
    o = cJSON_CreateObject();
    if (o == NULL) { return false; }
    cJSON_AddItemToObject(root, "ota", o);
    if ((cJSON_AddStringToObject(o, "fwVersion", c->ota.fw_version) == NULL) ||
        (cJSON_AddStringToObject(o, "upgradePolicy",
            s_policy_names[(c->ota.upgrade_policy <= CFG_OTA_IDLE)
                           ? c->ota.upgrade_policy : 0u]) == NULL))
    { return false; }

    return true;
}

/** 写 meta(CRC 由调用方传入, 因为 CRC 覆盖的是它前面那一段) */
static bool cfg_write_meta(cJSON *obj, const config_store_t *c, uint32_t crc)
{
    cJSON *o;
    char   hex[9];

    (void)snprintf(hex, sizeof(hex), "%08lX", (unsigned long)crc);

    o = cJSON_CreateObject();
    if (o == NULL) { return false; }
    cJSON_AddItemToObject(obj, "meta", o);

    return ((cJSON_AddNumberToObject(o, "configVersion",
                (double)c->meta.config_version) != NULL) &&
            (cJSON_AddStringToObject(o, "crc32", hex) != NULL) &&  /* 文档: HEX 大写字符串 */
            (cJSON_AddNumberToObject(o, "updatedAt",
                (double)c->meta.updated_at) != NULL));
}

/**
 * @brief 序列化并回填 CRC
 * @param crc_out 可选, 输出算出的 CRC(供 bank 头部使用, 免得再算一遍)
 *
 * 【v33-2 迁到 cJSON 后为什么是"打两遍"】
 *   CRC 的覆盖范围是"全量配置不含 meta, 且【不含】根对象结尾的 '}'"
 *   ——也就是旧实现里"根对象还没闭合"时那一段。cJSON 是整树打印, 关不关根不由我们
 *   控制, 所以:
 *     第一遍: 只建 system..ota 的树 -> PrintPreallocated 拿到长度 n
 *             -> meta_start = n - 1(去掉 cJSON 自己补的那个 '}')
 *             -> crc = cfg_crc32(out, meta_start)
 *   这样 CRC 的输入与迁移前【逐字节一致】, Flash 里已存的 CRC 才不会集体失效。
 *     第二遍: 把 meta 挂到同一棵树上再打一遍(PrintPreallocated 直接覆盖 out)。
 *   PrintPreallocated 不需要额外缓冲, 所以"打两遍"只花时间、不花 RAM。
 */
static uint16_t cfg_serialize(const config_store_t *c, char *out, uint16_t cap,
                              uint32_t *crc_out)
{
    cJSON   *root;
    uint16_t meta_start;
    uint32_t crc;
    uint16_t n;

    if ((c == NULL) || (out == NULL) || (cap < 64u)) { return 0u; }

    root = cJSON_CreateObject();
    if (root == NULL) { return 0u; }
    if (!cfg_write_domains(root, c))
    {
        cJSON_Delete(root);
        return 0u;
    }

    if (cJSON_PrintPreallocated(root, out, (int)cap, 0) == 0)
    {
        cJSON_Delete(root);
        out[0] = '\0';
        return 0u;
    }
    n = (uint16_t)strlen(out);
    if (n < 2u)
    {
        cJSON_Delete(root);
        out[0] = '\0';
        return 0u;
    }
    meta_start = (uint16_t)(n - 1u);        /* 去掉结尾的 '}' —— 与旧实现同一覆盖范围 */
    crc        = crc32_ieee((const uint8_t *)out, (uint32_t)meta_start);

    if (!cfg_write_meta(root, c, crc))
    {
        cJSON_Delete(root);
        out[0] = '\0';
        return 0u;
    }
    if (cJSON_PrintPreallocated(root, out, (int)cap, 0) == 0)
    {
        cJSON_Delete(root);
        out[0] = '\0';
        return 0u;
    }
    cJSON_Delete(root);

    if (crc_out != NULL) { *crc_out = crc; }
    return (uint16_t)strlen(out);
}

uint16_t config_store_to_json(const config_store_t *c, char *out, uint16_t cap)
{
    return cfg_serialize(c, out, cap, NULL);
}

uint32_t config_store_calc_crc(const config_store_t *c)
{
    uint32_t crc = 0u;
    uint16_t n;

    /* 走专用缓冲 s_json_crc: 不能碰调用方(可能是 save 的载荷缓冲) */
    n = cfg_serialize(c, s_json_crc, (uint16_t)sizeof(s_json_crc), &crc);
    return (n == 0u) ? 0u : crc;
}

/* ==================== cfg.query 用: 把 9 个域写进已开好的应答 data ====================
 *
 * 【为什么不复用 config_store_to_json()】那个函数会自己建【一整棵以根对象为顶的树】
 * 并打印出一个完整的顶层对象; 而 cfg.query 的应答 data 只是信封里的一个成员:
 *     {"cmd":"cfg.query",...,"data":{ ...配置对象... }}
 * 若把它当"字符串值"塞进 data, 会得到 "data":"{...}" 或 "data":{ {...} } ——
 * 形状不对(这正是任务清单里标注的"数据形状冲突"之一)。所以这里只往调用方
 * 已经开好的对象里挂成员, 由调用方决定外层容器。
 *
 * 【crc32 必须重算】文档 meta.crc32 = "全量配置(不含 meta)的 CRC32"。这里写出来的
 * 就是当前内存配置, 所以直接调 config_store_calc_crc() —— 与存 Flash 时用的是
 * 同一个函数, 保证"cfg.query 看到的 CRC" == "Flash 里存的 CRC"。
 */
bool config_store_write_members(cJSON *obj, const config_store_t *c)
{
    uint32_t crc;

    if ((obj == NULL) || (c == NULL)) { return false; }

    if (!cfg_write_domains(obj, c)) { return false; }        /* system .. ota */
    crc = config_store_calc_crc(c);
    return cfg_write_meta(obj, c, crc);                     /* meta(configVersion/crc32/updatedAt) */
}

/* ==================== 反序列化(按域) ==================== */

#define CFG_RET_ON_RANGE(r)  do { if ((r) == CFG_GET_RANGE) { return CFG_ERR_RANGE; } } while (0)

/* 字符串超长必须报错, 不能当"没给"悄悄放过 —— 否则平台会以为设置成功了。
 * 文档每个字符串字段都标了长度上限(如 gwId ≤20 字节、ledTextTemplate ≤64)。 */
#define CFG_STR_ON_RANGE(json, path, dst, max)                            \
    do {                                                                  \
        if (cfg_get_str((json), (path), (dst), (max)) == CFG_GET_RANGE)   \
        {                                                                 \
            return CFG_ERR_RANGE;                                         \
        }                                                                 \
    } while (0)

/* 数组元素的缺省值(文档 3.3 / 3.6 的"默认"列)。
 * 必须在解析【之前】铺好: 数组元素是整体替换的, 不铺就是 0, 而
 * reportPeriodSeconds=0 连校验都过不去(文档要求 5~3600)。 */
static void cfg_sensor_defaults(cfg_sensor_t *s)
{
    (void)memset(s, 0, sizeof(*s));
    s->measure_mode      = CFG_MEASURE_LEVEL;   /* level */
    s->report_period_s   = 60u;
    s->offline_timeout_s = 90u;                 /* 对齐 Worker SensorOfflineTimeoutSeconds */
    s->enabled           = 1u;
}

static void cfg_device_defaults(cfg_device_t *d)
{
    (void)memset(d, 0, sizeof(*d));
    d->volume     = 50u;
    d->flash_mode = 1u;
    d->enabled    = 1u;
}

static cfg_result_t cfg_parse_system(const char *json, cfg_system_t *s, bool *present)
{
    int r;

    if (cj_get(json, "system", NULL, 0) != CJ_OBJ) { *present = false; return CFG_OK; }
    *present = true;

    CFG_STR_ON_RANGE(json, "system.gwId",     s->gw_id,    CFG_GWID_MAX);
    CFG_STR_ON_RANGE(json, "system.password", s->password, CFG_SYSPWD_MAX);
    r = cfg_get_i8(json, "system.timezone", &s->timezone, -12, 14);
    CFG_RET_ON_RANGE(r);

    return CFG_OK;
}

static cfg_result_t cfg_parse_network(const char *json, cfg_network_t *n, bool *present)
{
    int r;
    uint8_t b = 0u;

    if (cj_get(json, "network", NULL, 0) != CJ_OBJ) { *present = false; return CFG_OK; }
    *present = true;

    CFG_STR_ON_RANGE(json, "network.host",     n->host,      CFG_HOST_MAX);
    CFG_STR_ON_RANGE(json, "network.clientId", n->client_id, CFG_CLIENTID_MAX);
    CFG_STR_ON_RANGE(json, "network.username", n->username,  CFG_MQTTUSER_MAX);

    /* ★★2026-10-09: password 的取值要挡掉掩码值★★
     * cfg.query 现在把 network.password 固定返回 "*****"(见 cloud_proto 的
     * cloud_cmd_cfg_query)。平台若把整份配置【读回来再原样写回】(很常见的做法),
     * 就会把真密码改成 5 个星号 —— 之后 MQTT 再也连不上, 而且现场极难看出原因。
     * ⇒ 恰为 "*****" 时【当作"不修改"跳过】, 保留原值。
     * ⚠️ 代价: 真想把密码设成 "*****" 就做不到 —— 这是可接受的取舍(没人会这么设)。
     * 其余情况照常覆盖(所以设空串 "" = 清空密码, 仍然可用)。 */
    {
        /* 缓冲要比 CFG_MQTTPWD_MAX 大 1 —— cfg_get_str 在 n == max 时会
         * memcpy(dst, tmp, n+1)(连结尾 NUL), 刚好 max 字节会溢出 1 字节。 */
        char pwd[CFG_MQTTPWD_MAX + 2];
        int  pr;

        pr = cfg_get_str(json, "network.password", pwd, CFG_MQTTPWD_MAX);
        if (pr == CFG_GET_RANGE)
        {
            /* 超长: 照旧【拒绝整条命令】(与原 CFG_STR_ON_RANGE 的行为一致),
             * 不要在这里静默跳过 —— 那会让平台以为设置成功了。 */
            return CFG_ERR_RANGE;
        }
        if ((pr == CFG_GET_OK) && (strcmp(pwd, "*****") != 0))
        {
            (void)strncpy(n->mqtt_password, pwd, sizeof(n->mqtt_password) - 1u);
            n->mqtt_password[sizeof(n->mqtt_password) - 1u] = '\0';
        }
        /* pr == CFG_GET_MISSING(没带这个字段) 或 恰为 "*****"(掩码回传)
         * -> 都【不动】 n->mqtt_password, 保留原密码。 */
    }

    r = cfg_get_u16(json, "network.port", &n->port, 1u, 65535u);              CFG_RET_ON_RANGE(r);
    r = cfg_get_u16(json, "network.keepAliveSeconds", &n->keep_alive_s, 30u, 300u);
    CFG_RET_ON_RANGE(r);
    r = cfg_get_u16(json, "network.reconnectMinSeconds", &n->reconnect_min_s, 1u, 65535u);
    CFG_RET_ON_RANGE(r);
    r = cfg_get_u16(json, "network.reconnectMaxSeconds", &n->reconnect_max_s, 1u, 65535u);
    CFG_RET_ON_RANGE(r);
    r = cfg_get_u16(json, "network.statusIntervalSeconds", &n->status_interval_s, 10u, 600u);
    CFG_RET_ON_RANGE(r);

    r = cfg_get_bool(json, "network.tls", &b);
    CFG_RET_ON_RANGE(r);
    if (r == CFG_GET_OK) { n->tls = b; }

    return CFG_OK;
}

static cfg_result_t cfg_parse_sensors(const char *json, config_store_t *c, bool *present,
                                      bool merge)
{
    cj_type_t t;
    uint16_t cnt;
    uint16_t i;
    uint8_t  slot;

    t = cj_get(json, "sensors", NULL, 0);
    if (t == CJ_NOTFOUND) { *present = false; return CFG_OK; }
    /* 出现了就先把 present 置上: 逐域合并要靠它区分"没这个域"和"这个域坏了",
     * 后者必须计入 rejected 而不是被当成"没提到"悄悄跳过。 */
    *present = true;
    if (t != CJ_ARR) { return CFG_ERR_JSON; }

    cnt = cj_count(json, "sensors");
    if (cnt > CFG_SENSOR_MAX) { return CFG_ERR_RANGE; }

    if (!merge)
    {
        /* 全量语义: 整数组替换(存进去的就是完整数组) */
        (void)memset(c->sensors, 0, sizeof(c->sensors));
        c->sensor_count = (uint8_t)cnt;
        for (slot = 0u; slot < c->sensor_count; slot++)
        {
            cfg_sensor_defaults(&c->sensors[slot]);
        }
    }

    for (i = 0u; i < cnt; i++)
    {
        cfg_sensor_t *s;
        char    p[40];
        char    want_uid[CFG_UID_STR_MAX];
        int     r;
        uint8_t e = 0u;
        uint8_t k;
        bool    found = false;

        /* ---- 先取 uid, 用它决定"写到哪个槽" ----
         * 缺 uid / 格式非法 -> 拒该域: 没有地址就无法定位改的是哪一台
         * (新增设备应走 sen.bind / dev.bind, 不该从 cfg.set 混进来) */
        (void)snprintf(p, sizeof(p), "sensors[%u].uid", (unsigned)i);
        if (cfg_get_str(json, p, want_uid, CFG_UID_HEX_LEN) != CFG_GET_OK) { return CFG_ERR_RANGE; }
        if (!cfg_uid_valid(want_uid)) { return CFG_ERR_RANGE; }

        if (merge)
        {
            for (k = 0u; k < c->sensor_count && k < CFG_SENSOR_MAX; k++)
            {
                if (strcmp(c->sensors[k].uid, want_uid) == 0) { found = true; break; }
            }
            if (found)
            {
                slot = k;                       /* 命中: 只覆盖来包里出现过的字段 */
            }
            else
            {
                if (c->sensor_count >= CFG_SENSOR_MAX) { return CFG_ERR_RANGE; }
                slot = c->sensor_count;         /* 未命中: 追加, 先铺文档默认值 */
                cfg_sensor_defaults(&c->sensors[slot]);
                c->sensor_count++;
            }
        }
        else
        {
            slot = (uint8_t)i;
        }

        s = &c->sensors[slot];
        (void)strcpy(s->uid, want_uid);

        (void)snprintf(p, sizeof(p), "sensors[%u].serial", (unsigned)i);
        CFG_STR_ON_RANGE(json, p, s->serial, CFG_SERIAL_MAX);
        (void)snprintf(p, sizeof(p), "sensors[%u].name", (unsigned)i);
        CFG_STR_ON_RANGE(json, p, s->name, CFG_NAME_MAX);

        (void)snprintf(p, sizeof(p), "sensors[%u].measureMode", (unsigned)i);
        r = cfg_get_enum(json, p, s_measure_names, 2u, &e);    CFG_RET_ON_RANGE(r);
        if (r == CFG_GET_OK) { s->measure_mode = e; }

        (void)snprintf(p, sizeof(p), "sensors[%u].installHeightCm", (unsigned)i);
        r = cfg_get_u16(json, p, &s->install_height_cm, 0u, 2000u);  CFG_RET_ON_RANGE(r);
        (void)snprintf(p, sizeof(p), "sensors[%u].calibOffsetMm", (unsigned)i);
        r = cfg_get_i16(json, p, &s->calib_offset_mm, -5000, 5000);  CFG_RET_ON_RANGE(r);
        (void)snprintf(p, sizeof(p), "sensors[%u].reportPeriodSeconds", (unsigned)i);
        r = cfg_get_u16(json, p, &s->report_period_s, 5u, 3600u);    CFG_RET_ON_RANGE(r);
        (void)snprintf(p, sizeof(p), "sensors[%u].lowBatteryPercent", (unsigned)i);
        r = cfg_get_u8(json, p, &s->low_battery_pct, 0u, 100u);      CFG_RET_ON_RANGE(r);
        (void)snprintf(p, sizeof(p), "sensors[%u].offlineTimeoutSeconds", (unsigned)i);
        r = cfg_get_u16(json, p, &s->offline_timeout_s, 0u, 65535u); CFG_RET_ON_RANGE(r);

        (void)snprintf(p, sizeof(p), "sensors[%u].enabled", (unsigned)i);
        r = cfg_get_bool(json, p, &e);                               CFG_RET_ON_RANGE(r);
        if (r == CFG_GET_OK) { s->enabled = e; }
    }

    return CFG_OK;
}

static cfg_result_t cfg_parse_thresholds(const char *json, cfg_thresholds_t *t, bool *present)
{
    int r;
    uint8_t b = 0u;

    if (cj_get(json, "thresholds", NULL, 0) != CJ_OBJ) { *present = false; return CFG_OK; }
    *present = true;

    r = cfg_get_u16(json, "thresholds.alarmCm",   &t->alarm_cm,   0u, 65535u); CFG_RET_ON_RANGE(r);
    r = cfg_get_u16(json, "thresholds.warningCm", &t->warning_cm, 0u, 65535u); CFG_RET_ON_RANGE(r);
    r = cfg_get_u16(json, "thresholds.resolveCm", &t->resolve_cm, 0u, 65535u); CFG_RET_ON_RANGE(r);
    r = cfg_get_u16(json, "thresholds.minChangeCm", &t->min_change_cm, 0u, 65535u);
    CFG_RET_ON_RANGE(r);
    r = cfg_get_u16(json, "thresholds.eventMinIntervalSeconds", &t->event_min_interval_s,
                    0u, 65535u);
    CFG_RET_ON_RANGE(r);
    r = cfg_get_u8(json, "thresholds.confirmCount", &b, 1u, 10u);              CFG_RET_ON_RANGE(r);
    if (r == CFG_GET_OK) { t->confirm_count = b; }

    return CFG_OK;
}

/* ==================== gears[]: 按 【gear 字段】 定位槽位 ====================
 *
 * 【为什么不能按下标】来包里 `gears` 数组的下标与档位号【没有关系】:
 * 平台只想改 2 号档时下发的是 `{"gears":[{"gear":2,...}]}` —— 数组下标 0。
 * 若按下标写, 就会把 2 号档的内容写到 0 号档槽里; 而 validate 要求
 * gears[i].gear == i, 于是要么被拒(改不动), 要么把 gear 也改成 0(串档)。
 * 所以: 一律先读该元素的 `gear`, 再用它决定写哪个槽。
 *
 * 【gear 缺失/越界 -> CFG_ERR_RANGE】没有档位号就无法定位改的是哪一档。
 * 这条对"全量加载"和"增量合并"同样成立, 所以 gears 只需要这一个函数。
 *
 * 【槽位不变量】由于写入时 gear 与槽位索引始终一致, validate 里
 * `gears[i].gear == i` 这条检查天然成立; 全量加载时未出现在数组里的档位
 * 保持出厂默认(from_json 会先铺默认值), 不会出现半截档位。
 */
static cfg_result_t cfg_parse_gears(const char *json, cfg_gear_t *gears, bool *present)
{
    cj_type_t t;
    uint16_t cnt;
    uint8_t  i, j;
    int      r;

    t = cj_get(json, "gears", NULL, 0);
    if (t == CJ_NOTFOUND) { *present = false; return CFG_OK; }
    *present = true;
    if (t != CJ_ARR) { return CFG_ERR_JSON; }

    cnt = cj_count(json, "gears");
    if (cnt > CFG_GEAR_COUNT) { return CFG_ERR_RANGE; }

    for (i = 0u; i < (uint8_t)cnt; i++)
    {
        cfg_gear_t *g;
        char p[48];
        uint8_t e = 0u;
        uint8_t gg = 0u;

        /* ---- 先取 gear, 由它决定槽位 ----
         * CFG_RET_ON_RANGE 管"越界"; 这里还要单独管"缺失"(MISSING) ——
         * 两者都不能放行, 但原因不同, 所以合并成 r != CFG_GET_OK。 */
        (void)snprintf(p, sizeof(p), "gears[%u].gear", (unsigned)i);
        r = cfg_get_u8(json, p, &gg, 0u, 3u);
        CFG_RET_ON_RANGE(r);
        if (r != CFG_GET_OK) { return CFG_ERR_RANGE; }      /* 缺 gear: 无法定位 */

        g       = &gears[gg];                               /* ← 按档位号, 不是下标 i */
        g->gear = gg;

        (void)snprintf(p, sizeof(p), "gears[%u].name", (unsigned)i);
        CFG_STR_ON_RANGE(json, p, g->name, CFG_NAME_MAX);
        (void)snprintf(p, sizeof(p), "gears[%u].soundMode", (unsigned)i);
        /* ★2026-10-08: 上界 2 -> 3★ 0关闭/1常亮/2慢闪/3快闪 */
        r = cfg_get_u8(json, p, &e, 0u, 3u);             CFG_RET_ON_RANGE(r);
        if (r == CFG_GET_OK) { g->sound_mode = e; }

        (void)snprintf(p, sizeof(p), "gears[%u].ledShowMode", (unsigned)i);
        r = cfg_get_u8(json, p, &e, 0u, 1u);             CFG_RET_ON_RANGE(r);
        if (r == CFG_GET_OK) { g->led_show_mode = e; }   /* 0横 / 1竖 */

        /* ---- ★两个"双值"字段: 必须是 "a,b" 字符串★ ---- */
        (void)snprintf(p, sizeof(p), "gears[%u].ledTextShowMode", (unsigned)i);
        {
            uint8_t a = g->led_text_show_mode[0];
            uint8_t b = g->led_text_show_mode[1];
            /* ★上界是 3, 不是 1★ 云端 JSON 口径是 §1.9 的 0~3
             *   (0默认/1上对齐|左对齐/2居中/3下对齐|右对齐), 例如 "1,2"。
             *   空口上只有 1 位/半屏, 由 level_eval.c 的 led_align_bit() 折算 —— 
             *   【校验必须按 JSON 口径收 0~3】, 否则 "1,2" 会被这里误拒成 1001。 */
            r = cfg_get_u8_pair(json, p, 3u, &a, &b);    CFG_RET_ON_RANGE(r);
            if (r == CFG_GET_OK) { g->led_text_show_mode[0] = a; g->led_text_show_mode[1] = b; }
        }

        (void)snprintf(p, sizeof(p), "gears[%u].ledColor", (unsigned)i);
        {
            uint8_t a = g->led_color[0];
            uint8_t b = g->led_color[1];
            r = cfg_get_enum_pair(json, p, s_color_names, 4u, &a, &b); CFG_RET_ON_RANGE(r);
            if (r == CFG_GET_OK) { g->led_color[0] = a; g->led_color[1] = b; }
        }

        /* ★ledTextTemplate 拆成 1/2★ (原来是单字段 ledTextTemplate) */
        (void)snprintf(p, sizeof(p), "gears[%u].ledTextTemplate1", (unsigned)i);
        CFG_STR_ON_RANGE(json, p, g->led_text1, CFG_LEDTEXT_MAX);
        (void)snprintf(p, sizeof(p), "gears[%u].ledTextTemplate2", (unsigned)i);
        CFG_STR_ON_RANGE(json, p, g->led_text2, CFG_LEDTEXT_MAX);
        (void)snprintf(p, sizeof(p), "gears[%u].ttsTextTemplate", (unsigned)i);
        CFG_STR_ON_RANGE(json, p, g->tts_text, CFG_TTSTEXT_MAX);

        /* targets: "all" 或 [addr,...] */
        (void)snprintf(p, sizeof(p), "gears[%u].targets", (unsigned)i);
        if (cj_get(json, p, NULL, 0) == CJ_STR)
        {
            char tbuf[8];
            if (cj_get_str(json, p, tbuf, (uint16_t)sizeof(tbuf)) &&
                (strcmp(tbuf, "all") == 0))
            {
                g->targets_all  = 1u;
                g->target_count = 0u;
            }
            else
            {
                return CFG_ERR_RANGE;       /* 只认 "all" 这个字符串 */
            }
        }
        else if (cj_get(json, p, NULL, 0) == CJ_ARR)
        {
            uint16_t tc = cj_count(json, p);
            if (tc > CFG_DEVICE_MAX) { return CFG_ERR_RANGE; }

            g->targets_all  = 0u;
            g->target_count = (uint8_t)tc;
            for (j = 0u; j < (uint8_t)tc; j++)
            {
                char pa[56];
                (void)snprintf(pa, sizeof(pa), "gears[%u].targets[%u]", (unsigned)i, (unsigned)j);
                CFG_STR_ON_RANGE(json, pa, g->target_uid[j], CFG_UID_HEX_LEN);
                if (!cfg_uid_valid(g->target_uid[j])) { return CFG_ERR_RANGE; }
            }
        }
    }

    return CFG_OK;
}

/* ==================== devices[]: 与 sensors[] 完全对应的两种语义 ====================
 *
 *   merge=false -> 整数组替换(全量加载: 存进去的就是完整数组, 先铺文档默认值)
 *   merge=true  -> 按 uid 逐条合并:
 *                    命中  -> 只覆盖来包里【出现过】的字段, 其余保持原值
 *                    未命中-> 追加(受 CFG_DEVICE_MAX 约束), 追加前先铺
 *                            volume=50 / flashMode=1 / enabled=1
 *
 * 【为什么 merge=true 必须存在】cfg.set 的语义是"增量": 平台只发一条
 * {"devices":[{"uid":"310001","name":"东门报警器"}]} 时, 绝不能把另外几台设备
 * 从列表里删掉 —— 那正是整数组替换会干的事(与 sensors 曾经踩过的坑完全相同)。
 *
 * 【为什么"未命中"要报满而不是覆盖】设备只能从 dev.bind 进来(要配对/标识),
 * 从 cfg.set 偷偷塞一台新设备会让云和网关的绑定表不一致; 但文档又要求
 * "数组未出现的条目不变" —— 所以容量满时宁可整域拒绝(rejected), 让平台看得见。
 */
static cfg_result_t cfg_parse_devices(const char *json, config_store_t *c, bool *present,
                                     bool merge)
{
    cj_type_t t;
    uint16_t  cnt;
    uint16_t  i;
    uint8_t   slot;
    int       r;
    uint8_t   e = 0u;

    t = cj_get(json, "devices", NULL, 0);
    if (t == CJ_NOTFOUND) { *present = false; return CFG_OK; }
    *present = true;                    /* 同 sensors: 出现了就要计入 applied/rejected */
    if (t != CJ_ARR) { return CFG_ERR_JSON; }

    cnt = cj_count(json, "devices");
    if (cnt > CFG_DEVICE_MAX) { return CFG_ERR_RANGE; }

    if (!merge)
    {
        /* 全量语义: 整数组替换 */
        (void)memset(c->devices, 0, sizeof(c->devices));
        c->device_count = (uint8_t)cnt;

        /* 先铺文档 3.6 的默认值(volume=50 / flashMode=1 / enabled=true) */
        for (slot = 0u; slot < c->device_count; slot++)
        {
            cfg_device_defaults(&c->devices[slot]);
        }
    }

    for (i = 0u; i < cnt; i++)
    {
        cfg_device_t *d;
        char    p[40];
        char    want_uid[CFG_UID_STR_MAX];
        uint8_t k;
        bool    found = false;

        /* ---- 先取 uid, 用它决定"写到哪个槽" ----
         * 缺 uid / 格式非法 -> 拒该域: 没有地址就无法定位改的是哪一台 */
        (void)snprintf(p, sizeof(p), "devices[%u].uid", (unsigned)i);
        if (cfg_get_str(json, p, want_uid, CFG_UID_HEX_LEN) != CFG_GET_OK) { return CFG_ERR_RANGE; }
        if (!cfg_uid_valid(want_uid)) { return CFG_ERR_RANGE; }

        if (merge)
        {
            for (k = 0u; k < c->device_count && k < CFG_DEVICE_MAX; k++)
            {
                if (strcmp(c->devices[k].uid, want_uid) == 0) { found = true; break; }
            }
            if (found)
            {
                slot = k;                       /* 命中: 只覆盖来包里出现过的字段 */
            }
            else
            {
                if (c->device_count >= CFG_DEVICE_MAX) { return CFG_ERR_RANGE; }
                slot = c->device_count;         /* 未命中: 追加, 先铺文档默认值 */
                cfg_device_defaults(&c->devices[slot]);
                c->device_count++;
            }
        }
        else
        {
            slot = (uint8_t)i;
        }

        d = &c->devices[slot];
        (void)strcpy(d->uid, want_uid);

        (void)snprintf(p, sizeof(p), "devices[%u].serial", (unsigned)i);
        CFG_STR_ON_RANGE(json, p, d->serial, CFG_SERIAL_MAX);
        (void)snprintf(p, sizeof(p), "devices[%u].name", (unsigned)i);
        CFG_STR_ON_RANGE(json, p, d->name, CFG_NAME_MAX);

        (void)snprintf(p, sizeof(p), "devices[%u].type", (unsigned)i);
        r = cfg_get_u8(json, p, &d->type, 1u, 3u);             CFG_RET_ON_RANGE(r);
        (void)snprintf(p, sizeof(p), "devices[%u].volume", (unsigned)i);
        r = cfg_get_u8(json, p, &d->volume, 0u, 100u);         CFG_RET_ON_RANGE(r);
        (void)snprintf(p, sizeof(p), "devices[%u].flashMode", (unsigned)i);
        r = cfg_get_u8(json, p, &d->flash_mode, 0u, 3u);       CFG_RET_ON_RANGE(r);

        (void)snprintf(p, sizeof(p), "devices[%u].enabled", (unsigned)i);
        r = cfg_get_bool(json, p, &e);                         CFG_RET_ON_RANGE(r);
        if (r == CFG_GET_OK) { d->enabled = e; }
    }

    return CFG_OK;
}

static cfg_result_t cfg_parse_rules(const char *json, cfg_rules_t *r0, bool *present)
{
    int r;
    uint8_t e = 0u;

    if (cj_get(json, "rules", NULL, 0) != CJ_OBJ) { *present = false; return CFG_OK; }
    *present = true;

    r = cfg_get_enum(json, "rules.mode", s_rulemode_names, 2u, &e); CFG_RET_ON_RANGE(r);
    if (r == CFG_GET_OK) { r0->mode = e; }

    r = cfg_get_enum(json, "rules.sensorOfflinePolicy", s_offline_names, 3u, &e);
    CFG_RET_ON_RANGE(r);
    if (r == CFG_GET_OK) { r0->sensor_offline_policy = e; }

    r = cfg_get_u16(json, "rules.manualTimeoutSeconds", &r0->manual_timeout_s, 0u, 65535u);
    CFG_RET_ON_RANGE(r);

    return CFG_OK;
}

static cfg_result_t cfg_parse_ota(const char *json, cfg_ota_t *o, bool *present)
{
    int r;
    uint8_t e = 0u;

    if (cj_get(json, "ota", NULL, 0) != CJ_OBJ) { *present = false; return CFG_OK; }
    *present = true;

    /* fwVersion 只读(文档 3.8): 平台下发也不改, 由固件自己填 —— 这里刻意不解析 */
    r = cfg_get_enum(json, "ota.upgradePolicy", s_policy_names, 2u, &e); CFG_RET_ON_RANGE(r);
    if (r == CFG_GET_OK) { o->upgrade_policy = e; }

    return CFG_OK;
}

/* ==================== 增量合并 / 全量解析 ==================== */

cfg_result_t config_store_merge_json(config_store_t *c, const char *json,
                                     uint8_t *changed_mask)
{
    uint8_t      mask = 0u;
    bool         pres = false;
    cfg_result_t r;

    if ((c == NULL) || (json == NULL)) { return CFG_ERR_PARAM; }

    /* 在副本上改: 任何一步失败都让 *c 保持原样, 免得留下改了一半的配置 */
    (void)memcpy(&s_merge_tmp, c, sizeof(*c));

    r = cfg_parse_system(json, &s_merge_tmp.system, &pres);
    if (r != CFG_OK) { return r; }
    if (pres) { mask |= CFG_CH_SYSTEM; }

    r = cfg_parse_network(json, &s_merge_tmp.network, &pres);
    if (r != CFG_OK) { return r; }
    if (pres) { mask |= CFG_CH_NETWORK; }

    r = cfg_parse_sensors(json, &s_merge_tmp, &pres, false);
    if (r != CFG_OK) { return r; }
    if (pres) { mask |= CFG_CH_SENSORS; }

    r = cfg_parse_thresholds(json, &s_merge_tmp.thresholds, &pres);
    if (r != CFG_OK) { return r; }
    if (pres) { mask |= CFG_CH_THRESHOLDS; }

    r = cfg_parse_gears(json, s_merge_tmp.gears, &pres);
    if (r != CFG_OK) { return r; }
    if (pres) { mask |= CFG_CH_GEARS; }

    r = cfg_parse_devices(json, &s_merge_tmp, &pres, false);
    if (r != CFG_OK) { return r; }
    if (pres) { mask |= CFG_CH_DEVICES; }

    r = cfg_parse_rules(json, &s_merge_tmp.rules, &pres);
    if (r != CFG_OK) { return r; }
    if (pres) { mask |= CFG_CH_RULES; }

    r = cfg_parse_ota(json, &s_merge_tmp.ota, &pres);
    if (r != CFG_OK) { return r; }
    if (pres) { mask |= CFG_CH_OTA; }

    /* 合并后必须整体仍合法(例如只改 alarmCm 可能破坏 alarm>warning) */
    r = config_store_validate(&s_merge_tmp);
    if (r != CFG_OK) { return r; }

    (void)memcpy(c, &s_merge_tmp, sizeof(*c));
    if (changed_mask != NULL) { *changed_mask = mask; }
    return CFG_OK;
}

cfg_result_t config_store_apply_json(config_store_t *c, const char *json)
{
    return config_store_merge_json(c, json, NULL);
}

/* ==================== 逐域增量合并(cfg.set 用) —— 任务 P2-4 ==================== */

/** 在 s_merge_tmp 上只应用一个域; *present 置位 = 来包里确实出现了这个域 */
static cfg_result_t cfg_apply_one_section(const char *json, uint8_t ch, bool *present)
{
    switch (ch)
    {
        case CFG_CH_SYSTEM:     return cfg_parse_system(json, &s_merge_tmp.system, present);
        case CFG_CH_NETWORK:    return cfg_parse_network(json, &s_merge_tmp.network, present);
        case CFG_CH_SENSORS:    return cfg_parse_sensors(json, &s_merge_tmp, present, true);
        case CFG_CH_THRESHOLDS: return cfg_parse_thresholds(json, &s_merge_tmp.thresholds, present);
        case CFG_CH_GEARS:      return cfg_parse_gears(json, s_merge_tmp.gears, present);
        case CFG_CH_DEVICES:    return cfg_parse_devices(json, &s_merge_tmp, present, true);
        case CFG_CH_RULES:      return cfg_parse_rules(json, &s_merge_tmp.rules, present);
        case CFG_CH_OTA:        return cfg_parse_ota(json, &s_merge_tmp.ota, present);
        default:                *present = false; return CFG_OK;
    }
}

cfg_result_t config_store_merge_sections(config_store_t *c, const char *json,
                                         cfg_sections_result_t *res)
{
    /* 固定顺序 = 文档 5.2 的域列表(system..ota)。
     * 【meta 不在表里, 这是刻意的】meta(configVersion/crc32/updatedAt)是系统维护
     * 字段, 平台只读; 所以即使来包里带了 meta, 也既不算 applied 也不算 rejected。
     *
     * 【已知局限】域名拼错(如 "sensor")同样落在"不在表里"这一档, 会被静默忽略。
     * 想把它变成 rejected 需要能遍历对象的键, 而 cloud_json.h 目前没有这个接口 ——
     * 与其在这加一套半吊子扫描, 不如等真需要时给 cloud_json 补一个 cj_key_count/
     * cj_key_at。当前行为对平台是"这个域没生效"而不是"改错了别的域", 不算危险。 */
    static const uint8_t order[8] =
    {
        CFG_CH_SYSTEM, CFG_CH_NETWORK, CFG_CH_SENSORS, CFG_CH_THRESHOLDS,
        CFG_CH_GEARS, CFG_CH_DEVICES, CFG_CH_RULES, CFG_CH_OTA
    };
    cfg_sections_result_t out;
    uint8_t i;

    if (res != NULL) { res->applied = 0u; res->rejected = 0u; }
    if ((c == NULL) || (json == NULL)) { return CFG_ERR_PARAM; }

    out.applied  = 0u;
    out.rejected = 0u;

    for (i = 0u; i < (uint8_t)(sizeof(order) / sizeof(order[0])); i++)
    {
        bool         pres = false;
        cfg_result_t r;

        /* 每个域都从【当前 *c】重新拷贝一份副本, 保证失败域的改动不会污染下一个域,
         * 也不会污染 *c。s_merge_tmp 是静态的: 配置结构体 ~1.7KB, 不能在 4KB 栈上再来一份。 */
        (void)memcpy(&s_merge_tmp, c, sizeof(*c));

        r = cfg_apply_one_section(json, order[i], &pres);
        if (!pres)
        {
            continue;                       /* 来包里没有这个域 -> 原样不动 */
        }

        /* 应用完还要整体校验: 只改 alarmCm 可能破坏 alarm>warning 这类域内/域间约束 */
        if (r == CFG_OK) { r = config_store_validate(&s_merge_tmp); }

        if (r == CFG_OK)
        {
            (void)memcpy(c, &s_merge_tmp, sizeof(*c));      /* 成功 -> 提交这一个域 */
            out.applied |= order[i];
        }
        else
        {
            out.rejected |= order[i];                       /* 失败 -> 丢弃该域, 继续下一个 */
        }
    }

    if (res != NULL) { *res = out; }

    /* 全部域都被拒也返回 CFG_OK: 域级失败是【业务结果】, 写在 res 里给平台看;
     * 用错误码返回会让平台以为整条命令根本没被处理。只有入参非法才是 CFG_ERR_PARAM。 */
    return CFG_OK;
}

cfg_result_t config_store_from_json(config_store_t *c, const char *json)
{
    cfg_result_t r;
    cj_type_t    t;

    if ((c == NULL) || (json == NULL)) { return CFG_ERR_PARAM; }

    config_store_factory_default(c);

    /* meta 单独取密: configVersion / updatedAt 是系统维护字段, 平台只读,
     * 但上电从 Flash 读回时必须恢复, 否则版本号会一直归零。 */
    t = cj_get(json, "meta", NULL, 0);
    if (t == CJ_OBJ)
    {
        c->meta.config_version = cj_get_uint(json, "meta.configVersion", 0u);
        c->meta.updated_at     = cj_get_uint(json, "meta.updatedAt", 0u);
        c->meta.crc32          = 0u;
    }

    r = config_store_merge_json(c, json, NULL);
    if (r != CFG_OK) { return r; }

    /* CRC 校验必须在 merge 之后: 此时 c 才是这份 JSON 的完整解析结果。
     * 先重新序列化一次拿到"应有"的 CRC, 再和 JSON 里声明比 —— 这一步同时
     * 验证了【序列化器与解析器互为逆运算】: 任何一边丢字段/截断都会在这里暴露。 */
    if ((t == CJ_OBJ) && (cj_get(json, "meta.crc32", NULL, 0) == CJ_STR))
    {
        char     hex[12];
        uint32_t want;
        uint32_t have;

        if (cj_get_str(json, "meta.crc32", hex, (uint16_t)sizeof(hex)))
        {
            have = (uint32_t)strtoul(hex, NULL, 16);
            want = config_store_calc_crc(c);
            if ((want == 0u) || (want != have)) { return CFG_ERR_CRC; }
            c->meta.crc32 = want;
        }
    }

    return CFG_OK;
}

/* ==================== 双区 Flash 读 / 写 ==================== */

static cfg_result_t cfg_bank_read(uint8_t bank, config_store_t *c)
{
    cfg_bank_hdr_t h;
    uint32_t       want;
    cfg_result_t   r;

    r = cfg_port_read(bank, 0u, (uint8_t *)&h, (uint16_t)sizeof(h));
    if (r != CFG_OK) { return r; }

    if (h.magic != CFG_BANK_MAGIC) { return CFG_ERR_NOBANK; }   /* 空区(全 0xFF) */

    /* 头部自校验: 防住"擦到一半掉电"留下的半截头 */
    if (crc32_ieee((const uint8_t *)&h, (uint32_t)offsetof(cfg_bank_hdr_t, hdr_crc)) != h.hdr_crc)
    {
        return CFG_ERR_CRC;
    }
    if ((h.json_len == 0u) || (h.json_len >= CFG_JSON_MAX)) { return CFG_ERR_CRC; }

    r = cfg_port_read(bank, (uint32_t)sizeof(h), (uint8_t *)s_json_rd, (uint16_t)h.json_len);
    if (r != CFG_OK) { return r; }
    s_json_rd[h.json_len] = '\0';

    r = config_store_from_json(c, s_json_rd);
    if (r != CFG_OK) { return r; }

    /* 头部 CRC 与自己重算的必须一致 —— 抓"存进去的和读出来的不是一回事" */
    want = config_store_calc_crc(c);
    if ((want == 0u) || (want != h.crc32)) { return CFG_ERR_CRC; }

    /* 头部携带的版本号是权威(meta 不在 CRC 覆盖范围内, 以头部为准) */
    c->meta.config_version = h.version;
    c->meta.updated_at     = h.updated_at;
    c->meta.crc32          = h.crc32;

    return CFG_OK;
}

static cfg_result_t cfg_bank_write(uint8_t bank, const config_store_t *c,
                                   const char *json, uint16_t len, uint32_t crc)
{
    cfg_bank_hdr_t h;
    cfg_result_t   r;

    (void)memset(&h, 0, sizeof(h));
    h.magic      = CFG_BANK_MAGIC;
    h.version    = c->meta.config_version;
    h.crc32      = crc;
    h.updated_at = c->meta.updated_at;
    h.json_len   = len;
    h.hdr_crc    = crc32_ieee((const uint8_t *)&h, (uint32_t)offsetof(cfg_bank_hdr_t, hdr_crc));

    if (((uint32_t)len + (uint32_t)sizeof(h)) > CFG_BANK_SIZE) { return CFG_ERR_PARAM; }

    r = cfg_port_erase(bank);
    if (r != CFG_OK) { return r; }

    r = cfg_port_write(bank, 0u, (const uint8_t *)&h, (uint16_t)sizeof(h));
    if (r != CFG_OK) { return r; }

    return cfg_port_write(bank, (uint32_t)sizeof(h), (const uint8_t *)json, len);
}

cfg_result_t config_store_save(const config_store_t *c)
{
    cfg_result_t r;
    uint16_t     n;
    uint32_t     crc = 0u;
    config_store_t chk;

    if (c == NULL) { return CFG_ERR_PARAM; }

    /* 先校验再落盘。如果不校验, 一份非法配置会被写进 Flash, 然后【读回时】
     * 因范围检查失败而被判损坏 —— 表现为"存了但下次上电变回出厂默认",
     * 极难定位。宁可在这里就拒绝。 */
    {
        cfg_result_t v = config_store_validate(c);
        if (v != CFG_OK)
        {
            log_info("[CFG] 拒绝保存: 配置非法(code=%d), Flash 未被改动\r\n", (int)v);
            return v;
        }
    }

    n = cfg_serialize(c, s_json_wr, (uint16_t)sizeof(s_json_wr), &crc);
    if (n == 0u) { return CFG_ERR_PARAM; }

    /* --- 先写备份区 B, 回读校验 ---
     * 这一步失败时 A 还完好, 配置不会丢。 */
    r = cfg_bank_write(CFG_BANK_B, c, s_json_wr, n, crc);
    if (r != CFG_OK) { return r; }

    r = cfg_bank_read(CFG_BANK_B, &chk);
    if (r != CFG_OK) { return r; }

    /* --- 校验通过再写主区 A --- */
    r = cfg_bank_write(CFG_BANK_A, c, s_json_wr, n, crc);
    if (r != CFG_OK) { return r; }

    /* 写 A 之后不再回读 A: A 坏掉也不影响 —— 下次上电会回退到 B。
     * 少一次整页读, 掉电窗口也更短。 */
    return CFG_OK;
}

cfg_result_t config_store_load(config_store_t *c)
{
    cfg_result_t r;

    if (c == NULL) { return CFG_ERR_PARAM; }

    r = cfg_bank_read(CFG_BANK_A, c);
    if (r == CFG_OK)
    {
        log_info("[CFG] 主区 A 载入成功, configVersion=%lu\r\n",
                 (unsigned long)c->meta.config_version);
        return CFG_OK;
    }
    log_info("[CFG] 主区 A 不可用(code=%d), 试备份区 B\r\n", (int)r);

    r = cfg_bank_read(CFG_BANK_B, c);
    if (r == CFG_OK)
    {
        log_info("[CFG] 备份区 B 载入成功(主区损坏), configVersion=%lu\r\n",
                 (unsigned long)c->meta.config_version);
        return CFG_OK;
    }
    log_info("[CFG] 两区都不可用(code=%d) -> 回退出厂默认\r\n", (int)r);

    config_store_factory_default(c);
    return CFG_ERR_NOBANK;
}

config_store_t *config_store_get_mut(void)
{
    if (!s_loaded)
    {
        (void)config_store_load(&s_config);
        s_loaded = true;
    }
    return &s_config;
}

const config_store_t *config_store_get(void)
{
    /* 只读版本就是可写版本 —— 语义完全一致, 不复制第二份实例。
     * 分开两个入口是为了让"谁有权写"在代码里看得见: cloud_proto.c 的 cfg.set
     * 显式调用 get_mut(), 其它地方只能拿到 const 指针。 */
    return config_store_get_mut();
}

uint32_t config_store_version(void)
{
    return config_store_get()->meta.config_version;
}

void config_store_touch(config_store_t *c, uint32_t now_unix)
{
    if (c == NULL) { return; }
    c->meta.config_version++;
    c->meta.updated_at = now_unix;
}

cfg_result_t config_store_reset(config_store_t *c, uint32_t now_unix)
{
    if (c == NULL) { return CFG_ERR_PARAM; }

    config_store_factory_default(c);          /* configVersion 归零(文档第 6 章) */
    c->meta.updated_at = now_unix;

    return config_store_save(c);
}

/* ==================== 设备绑定/解绑(P5) ====================
 *
 * 见 config_store.h 的说明。这里只补实现口径:
 *   - 绑定时填文档默认值, 保证 config_store_validate 不因 0 值打回;
 *   - 解绑时连带清 gears[].target_uid 里指向该 uid 的条目(与 cloud_cmd_unbind 同口径),
 *     否则档位还会往一台已不存在的设备上下发。
 *   - 两函数都改【当前实例】(config_store_get_mut) 并落 Flash; 调用方不必再 save。 */

bool config_store_can_add(bool is_sensor)
{
    const config_store_t *c = config_store_get();
    uint8_t total = (uint8_t)(c->sensor_count + c->device_count);

    /* 用户约束: 总数≤5, 液位≤4, 声光≤3 */
    if (total >= 5u) { return false; }
    if (is_sensor)
    {
        return (c->sensor_count < 4u);
    }
    return (c->device_count < 3u);
}

/* 在 sensors[] 和 devices[] 两边找 uid, 找到返回 true */
static bool cfg_find_device(const config_store_t *c, const char *uid_hex,
                            bool *out_is_sensor, uint8_t *out_idx)
{
    uint8_t i;
    for (i = 0u; i < c->sensor_count && i < CFG_SENSOR_MAX; i++)
    {
        if (strcmp(c->sensors[i].uid, uid_hex) == 0)
        {
            if (out_is_sensor != NULL) { *out_is_sensor = true; }
            if (out_idx != NULL)        { *out_idx = i; }
            return true;
        }
    }
    for (i = 0u; i < c->device_count && i < CFG_DEVICE_MAX; i++)
    {
        if (strcmp(c->devices[i].uid, uid_hex) == 0)
        {
            if (out_is_sensor != NULL) { *out_is_sensor = false; }
            if (out_idx != NULL)        { *out_idx = i; }
            return true;
        }
    }
    return false;
}

/* ★★2026-10-14 新增: 重绑(同 UID 换新设备) —— 只更新元数据, 【不新增】条目 ★★
 * --------------------------------------------------------------------------
 * 【为什么必须单独一个函数】config_store_bind_device() 一开头就查重、已存在直接
 *   返回 CFG_ERR_RANGE(见它上面那段), 所以它【没法】用来覆盖一个已存在的 UID。
 *
 * 【用户场景(2026-10-14)】设备坏了换新, 重新烧录后回到同一默认地址
 *   ⇒ 网关必须允许同 UID 再绑一次, 但从机表里【不能出现两条】。
 *
 * 【用户口径, 三条】
 *   ① serial/name 用绑定窗口(sen.bind/dev.bind)里的新值【覆盖】;
 *   ② 【不】动 report_period_s 等运行参数 —— 平台一定会再下发一次上报周期,
 *      等于给新设备设一遍, 网关不需要动(动了反而会把平台设的值悄悄改回 60);
 *   ③ 条目数不变(本函数只改现有条目的字段)。
 *
 * ⚠️ 必须【保留】的字段: report_period_s / offline_timeout_s / enabled /
 *    low_battery_pct / measure_mode / calib_offset_mm / install_height_cm。
 *    它们与"这台设备是谁"无关, 是运行参数。
 *
 * ⚠️【不要】图省事写成 config_store_unbind_device() + config_store_bind_device()
 *    "先删再建" —— unbind 会连带清掉 gears[].target_uid 里指向该 UID 的条目
 *    (见 config_store.h 的说明), 那台设备以后收不到联动指令
 *    (现象: 能绑定、能上报, 就是不动灯 —— 极难查)。
 *
 * @return CFG_OK; CFG_ERR_PARAM(uid 非法); CFG_ERR_RANGE(该 UID 不在配置里)
 * @note   调用方(lora_business 的绑定状态机)只在【运行期从机表里已有该 UID】时
 *         才调它; 真"新设备"应走 config_store_bind_device()。 */
cfg_result_t config_store_rebind_device(const char *uid_hex, const char *serial,
                                        const char *name, uint8_t dev_type,
                                        uint32_t now_unix)
{
    config_store_t *c;
    uint8_t         idx       = 0u;
    bool            is_sensor = false;

    if ((uid_hex == NULL) || (!cfg_uid_valid(uid_hex)))
    {
        return CFG_ERR_PARAM;
    }
    /* serial/name 允许 NULL(当空串处理) —— 与 config_store_bind_device 同口径 */
    if (serial == NULL) { serial = ""; }
    if (name == NULL)   { name = ""; }

    c = config_store_get_mut();
    if (!cfg_find_device(c, uid_hex, &is_sensor, &idx))
    {
        return CFG_ERR_RANGE;       /* 未找到(与 config_store_unbind_device 同口径) */
    }

    if (is_sensor)
    {
        cfg_sensor_t *s = &c->sensors[idx];
        (void)strncpy(s->serial, serial, sizeof(s->serial) - 1u);
        s->serial[sizeof(s->serial) - 1u] = '\0';
        (void)strncpy(s->name, name, sizeof(s->name) - 1u);
        s->name[sizeof(s->name) - 1u] = '\0';
        /* ★ 刻意【不动】: report_period_s / offline_timeout_s / enabled /
         *   low_battery_pct / measure_mode / install_height_cm / calib_offset_mm
         *   —— 见上面口径②。 */
        (void)dev_type;             /* 液位传感器没有 type 字段 */
    }
    else
    {
        cfg_device_t *d = &c->devices[idx];
        (void)strncpy(d->serial, serial, sizeof(d->serial) - 1u);
        d->serial[sizeof(d->serial) - 1u] = '\0';
        (void)strncpy(d->name, name, sizeof(d->name) - 1u);
        d->name[sizeof(d->name) - 1u] = '\0';
        /* 设备类型允许在重绑时改(平台可能换了一台不同类型的设备) */
        d->type = dev_type;
    }

    config_store_touch(c, now_unix);    /* 版本 +1 + updated_at(平台的乐观锁靠它) */
    return config_store_save(c);
}

cfg_result_t config_store_bind_device(const char *uid_hex, const char *serial,
                                       const char *name, bool is_sensor,
                                       uint8_t dev_type, uint32_t now_unix)
{
    config_store_t *c;
    uint8_t         idx;
    bool            found;

    if ((uid_hex == NULL) || (!cfg_uid_valid(uid_hex)))
    {
        return CFG_ERR_PARAM;
    }
    /* serial/name 允许 NULL(当空串处理) */
    if (serial == NULL) { serial = ""; }
    if (name == NULL)    { name = ""; }

    c = config_store_get_mut();

    /* 查重: 同一 uid 不能重复绑 */
    found = cfg_find_device(c, uid_hex, NULL, NULL);
    if (found) { return CFG_ERR_RANGE; }

    /* 容量 */
    if (!config_store_can_add(is_sensor)) { return CFG_ERR_RANGE; }

    if (is_sensor)
    {
        cfg_sensor_t *s;
        if (c->sensor_count >= CFG_SENSOR_MAX) { return CFG_ERR_RANGE; }
        idx = c->sensor_count;
        s = &c->sensors[idx];
        (void)memset(s, 0, sizeof(*s));
        (void)strncpy(s->uid, uid_hex, sizeof(s->uid) - 1u);
        s->uid[sizeof(s->uid) - 1u] = '\0';
        (void)strncpy(s->serial, serial, sizeof(s->serial) - 1u);
        s->serial[sizeof(s->serial) - 1u] = '\0';
        (void)strncpy(s->name, name, sizeof(s->name) - 1u);
        s->name[sizeof(s->name) - 1u] = '\0';
        s->measure_mode     = CFG_MEASURE_LEVEL;
        s->enabled          = 1u;
        s->install_height_cm = 0u;        /* D-1 死配置 */
        s->calib_offset_mm   = 0;          /* D-1 死配置 */
        s->report_period_s  = 60u;        /* 文档默认 */
        s->offline_timeout_s = 90u;       /* 文档默认(注意 D-7 矛盾) */
        s->low_battery_pct  = 0u;         /* 0 = 禁用 */
        c->sensor_count++;
    }
    else
    {
        cfg_device_t *d;
        if (c->device_count >= CFG_DEVICE_MAX) { return CFG_ERR_RANGE; }
        idx = c->device_count;
        d = &c->devices[idx];
        (void)memset(d, 0, sizeof(*d));
        (void)strncpy(d->uid, uid_hex, sizeof(d->uid) - 1u);
        d->uid[sizeof(d->uid) - 1u] = '\0';
        (void)strncpy(d->serial, serial, sizeof(d->serial) - 1u);
        d->serial[sizeof(d->serial) - 1u] = '\0';
        (void)strncpy(d->name, name, sizeof(d->name) - 1u);
        d->name[sizeof(d->name) - 1u] = '\0';
        /* ★ 第 11 条: 用云端 dev.bind 给的类型, 不再硬编码 ★
         * 文档 5.4 第 11 条要求 dev.bind 额外必填 type(1 声光 / 2 LED / 3 一体)。
         * 以前这里恒写 CFG_DEVTYPE_SOUND_LIGHT —— 平台绑 LED 也会被记成声光, 而
         * devices[].type 文档标"只读", 平台自己改不了, 只能解绑重绑。
         * 越界/0(旧调用点) -> 回落到声光默认, 不给配置里留一个非法枚举值。 */
        d->type = ((dev_type >= (uint8_t)CFG_DEVTYPE_SOUND_LIGHT) &&
                   (dev_type <= (uint8_t)CFG_DEVTYPE_ALLINONE))
                      ? dev_type : (uint8_t)CFG_DEVTYPE_SOUND_LIGHT;
        d->volume    = 50u;                       /* 文档默认 */
        d->flash_mode = 1u;                       /* 文档默认 */
        d->enabled   = 1u;
        c->device_count++;
    }

    config_store_touch(c, now_unix);
    return config_store_save(c);
}

cfg_result_t config_store_unbind_device(const char *uid_hex, uint32_t now_unix)
{
    config_store_t *c;
    bool           is_sensor;
    uint8_t        idx;
    uint8_t        i;
    uint8_t        j;

    if ((uid_hex == NULL) || (!cfg_uid_valid(uid_hex)))
    {
        return CFG_ERR_PARAM;
    }

    c = config_store_get_mut();

    if (!cfg_find_device(c, uid_hex, &is_sensor, &idx))
    {
        return CFG_ERR_RANGE;          /* 未找到 */
    }

    if (is_sensor)
    {
        /* 数组保持紧凑: 后面前移, 末尾清零 */
        for (j = idx; (uint8_t)(j + 1u) < c->sensor_count; j++)
        {
            c->sensors[j] = c->sensors[j + 1u];
        }
        c->sensor_count--;
        (void)memset(&c->sensors[c->sensor_count], 0, sizeof(c->sensors[0]));
    }
    else
    {
        for (j = idx; (uint8_t)(j + 1u) < c->device_count; j++)
        {
            c->devices[j] = c->devices[j + 1u];
        }
        c->device_count--;
        (void)memset(&c->devices[c->device_count], 0, sizeof(c->devices[0]));
    }

    /* 连带清 gears[].target_uid 里指向该 uid 的条目(与 cloud_cmd_unbind 同口径) */
    for (i = 0u; i < CFG_GEAR_COUNT; i++)
    {
        uint8_t n = 0u;
        if (c->gears[i].targets_all != 0u) { continue; }
        for (j = 0u; j < c->gears[i].target_count; j++)
        {
            if (strcmp(c->gears[i].target_uid[j], uid_hex) != 0)
            {
                if (n != j)
                {
                    (void)strcpy(c->gears[i].target_uid[n], c->gears[i].target_uid[j]);
                }
                n++;
            }
        }
        c->gears[i].target_count = n;
    }

    config_store_touch(c, now_unix);
    return config_store_save(c);
}

/* ==================== 诊断 ==================== */

void config_store_dump(const config_store_t *c)
{
    if (c == NULL) { return; }

    log_info("[CFG] ver=%lu gwId=%s tz=%d host=%s:%u\r\n",
             (unsigned long)c->meta.config_version,
             c->system.gw_id, (int)c->system.timezone,
             (c->network.host[0] != '\0') ? c->network.host : "(空)",
             (unsigned)c->network.port);
    log_info("[CFG] 传感器=%u 联动设备=%u | 阈值 告警%u/预警%u/解除%u confirm=%u\r\n",
             (unsigned)c->sensor_count, (unsigned)c->device_count,
             (unsigned)c->thresholds.alarm_cm,
             (unsigned)c->thresholds.warning_cm,
             (unsigned)c->thresholds.resolve_cm,
             (unsigned)c->thresholds.confirm_count);
    log_info("[CFG] rules=%s 手动超时=%us 传感器失联策略=%s | ota=%s 固件=%s\r\n",
             s_rulemode_names[(c->rules.mode <= CFG_RULE_MANUAL) ? c->rules.mode : 0u],
             (unsigned)c->rules.manual_timeout_s,
             s_offline_names[(c->rules.sensor_offline_policy <= CFG_OFFLINE_OFF)
                             ? c->rules.sensor_offline_policy : 0u],
             s_policy_names[(c->ota.upgrade_policy <= CFG_OTA_IDLE)
                            ? c->ota.upgrade_policy : 0u],
             c->ota.fw_version);
}
