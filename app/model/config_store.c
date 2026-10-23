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
#include "config_internal.h"
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

static bool cfg_bank_hdr_peek(uint8_t bank, cfg_bank_hdr_t *h);
static bool cfg_bank_gen_peek(uint8_t bank, uint32_t *gen);
static void cfg_bank_gen_write(uint8_t bank, uint32_t gen);

/* ==================== 静态内存 ==================== */
/* 全部静态, 不占栈(配置结构体 ~1.7KB, 放栈上会压爆小栈) */
static config_store_t s_config;                 /* 惰性加载的当前配置 */
static bool           s_loaded;

static char s_json_wr[CFG_JSON_MAX];             /* 序列化(保存载荷)用 */
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
     *   默认统一取: 竖屏(1) + 两个半屏都居中(0) + 第一个半屏用档位主色、第二个同色。
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
        g->led_show_mode = 1u;                      /* 竖屏 */
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
        g->led_show_mode = 1u;                      /* 竖屏 */
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
        g->led_show_mode = 1u;                      /* 竖屏 */
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
        g->led_show_mode = 1u;                      /* 竖屏 */
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
    /* ★2026-10-23 P1: 这里原本是栈上局部变量, 而 config_store_t 实测 2640 字节,
     *   栈总共只有 4096 字节 —— 一次落盘就吃掉 2/3 栈。calldown 链再深一点
     *   (cloud 命令 -> cfg.set -> 落盘)就会把栈踩穿, 现象是随机的跑飞/看门狗,
     *   现场几乎查不出来。
     *   改成文件级 static: 只是"回读校验用的暂存", 不跨调用保留有效值。
     *   单线程主循环调用, 不存在重入; 落盘本来就串行(谁也不会并发存配置)。 */
    static config_store_t chk;

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

    /* ★2026-10-23 P1-5 根治: 双区“代”号★
     *
     * updated_at 是云端时间(不能动语义), 而 save 是先写 B 后写 A:
     * 若两次写之间掉电, A 是旧配置且 updated_at 可能与 B 相同 -> load 退回
     * “A 优先” -> 静默回退旧配置。时间戳相同/没有 RTC 时尤其明显。
     *
     * 修法: 每个配置区最后 16 字节写一个严格递增的“代号”(magic + gen + ~gen),
     * 不碰头部、不碰 JSON 格式; 老固件只读 header+json_len 字节, 忽略这块,
     * 所以升级/降级都不会丢 broker/账号/密码。下面两处 gen_write 与写区同序:
     * 先 B 后 A -> 掉电时 B 的 gen 更新, load 会选 B。 */
    {
        uint32_t ga = 0u;
        uint32_t gb = 0u;
        uint32_t gen;

        (void)cfg_bank_gen_peek(CFG_BANK_A, &ga);
        (void)cfg_bank_gen_peek(CFG_BANK_B, &gb);
        gen = ((ga > gb) ? ga : gb) + 1u;

        r = cfg_bank_write(CFG_BANK_B, c, s_json_wr, n, crc);
        if (r != CFG_OK) { return r; }
        cfg_bank_gen_write(CFG_BANK_B, gen);

        r = cfg_bank_read(CFG_BANK_B, &chk);
        if (r != CFG_OK) { return r; }

        r = cfg_bank_write(CFG_BANK_A, c, s_json_wr, n, crc);
        if (r != CFG_OK) { return r; }
        cfg_bank_gen_write(CFG_BANK_A, gen);

        return CFG_OK;
    }

}

/** 只读头部并做 magic/hdr_crc 校验: 用于"不解析 JSON 就判断哪一区更新" */
static bool cfg_bank_hdr_peek(uint8_t bank, cfg_bank_hdr_t *h)
{
    if (cfg_port_read(bank, 0u, (uint8_t *)h, (uint16_t)sizeof(*h)) != CFG_OK)
    {
        return false;
    }
    if (h->magic != CFG_BANK_MAGIC)
    {
        return false;
    }
    return (crc32_ieee((const uint8_t *)h,
                       (uint32_t)offsetof(cfg_bank_hdr_t, hdr_crc)) == h->hdr_crc);
}

/* ==================== 双区“代”号(旁路, 不改头部/JSON) ====================
 * 位置: 每区最后 16 字节。老固件只读 header + json_len 字节, 不会碰这里;
 * 新固件读到 magic/inv 不合法就把 gen 当 0(老配置) —— 双向兼容。
 */
#define CFG_GEN_MAGIC   0x47454E31u          /* 'GEN1' */
#define CFG_GEN_OFFSET  ((uint32_t)CFG_BANK_SIZE - 16u)

typedef struct
{
    uint32_t magic;
    uint32_t gen;
    uint32_t inv;       /* ~gen, 防半截写 */
    uint32_t rsv;
} cfg_bank_gen_t;

static bool cfg_bank_gen_peek(uint8_t bank, uint32_t *gen)
{
    cfg_bank_gen_t g;

    if (cfg_port_read(bank, CFG_GEN_OFFSET, (uint8_t *)&g, (uint16_t)sizeof(g)) != CFG_OK)
    {
        return false;
    }
    if ((g.magic != CFG_GEN_MAGIC) || ((g.gen ^ g.inv) != 0xFFFFFFFFu))
    {
        return false;
    }
    *gen = g.gen;
    return true;
}

static void cfg_bank_gen_write(uint8_t bank, uint32_t gen)
{
    cfg_bank_gen_t g;

    g.magic = CFG_GEN_MAGIC;
    g.gen   = gen;
    g.inv   = ~gen;
    g.rsv   = 0u;
    (void)cfg_port_write(bank, CFG_GEN_OFFSET, (const uint8_t *)&g, (uint16_t)sizeof(g));
}

cfg_result_t config_store_load(config_store_t *c)
{
    cfg_bank_hdr_t ha;
    cfg_bank_hdr_t hb;
    bool           va = cfg_bank_hdr_peek(CFG_BANK_A, &ha);
    bool           vb = cfg_bank_hdr_peek(CFG_BANK_B, &hb);
    uint8_t        order[2];
    cfg_result_t   r = CFG_OK;
    uint8_t        i;

    if (c == NULL) { return CFG_ERR_PARAM; }

    /* ★2026-10-23 P1: 不能无条件"先读 A"★
     *
     * save 的顺序是【先写 B, 再写 A】。若在两次写之间掉电, A 还是旧配置
     * (完整、CRC 合法), B 已经是新配置 —— 无条件优先 A 就等于"静默回退到
     * 上一次的配置", 而平台那边 configVersion 没变, 现场极难发现。
     *
     * 现在先比头部的 updated_at(云端下发时间戳), 取更新的那一份;
     * 时间戳相同再比 version(configVersion); 两者都相同(没对时 / 同一次落盘)
     * 时维持原来的"A 优先"。
     * 注意: 这里【没有】改 Flash 上的头部格式, 老设备上已有的配置照样能读。 */
    {
        uint32_t ga = 0u;
        uint32_t gb = 0u;
        bool     has_gen = cfg_bank_gen_peek(CFG_BANK_A, &ga);
        bool     b_newer;

        if (cfg_bank_gen_peek(CFG_BANK_B, &gb)) { has_gen = true; }

        if (has_gen)
        {
            /* 有代号就按代号(老区没有代号按 0), 它严格递增 */
            b_newer = (gb > ga);
        }
        else
        {
            /* 老配置(没有代号): 退回时间戳/版本比较 */
            b_newer = (hb.updated_at > ha.updated_at) ||
                      ((hb.updated_at == ha.updated_at) && (hb.version > ha.version));
        }

        if (va && vb && b_newer)
        {
            order[0] = CFG_BANK_B;
            order[1] = CFG_BANK_A;
        }
        else
        {
            order[0] = CFG_BANK_A;
            order[1] = CFG_BANK_B;
        }
    }

    for (i = 0u; i < 2u; i++)
    {
        uint8_t bank = order[i];

        r = cfg_bank_read(bank, c);
        if (r == CFG_OK)
        {
            log_info("[CFG] %s区载入成功, configVersion=%lu\r\n",
                     (bank == CFG_BANK_A) ? "主" : "备份",
                     (unsigned long)c->meta.config_version);
            return CFG_OK;
        }
        log_info("[CFG] %s区不可用(code=%d)%s\r\n",
                 (bank == CFG_BANK_A) ? "主" : "备份", (int)r,
                 (i == 0u) ? ", 试另一区" : "");
    }

    log_info("[CFG] 两区都不可用 -> 回退出厂默认\r\n");
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

/* 固件版本 -> 配置的 ota.fwVersion。只读字段, 平台不可写; 由固件自己维护。
 * 值没变就【不写 Flash】(每次上电都擦一页毫无意义), 变了才落盘,
 * 且【不动 configVersion】—— 升级不是"配置变更", 平台不该看到版本冲突。 */
cfg_result_t config_store_set_fw_version(const char *ver)
{
    config_store_t *c;
    char            tmp[CFG_FWVER_MAX + 1u];

    if ((ver == NULL) || (ver[0] == '\0')) { return CFG_ERR_PARAM; }

    c = config_store_get_mut();
    if (c == NULL) { return CFG_ERR_PARAM; }

    (void)snprintf(tmp, sizeof(tmp), "%s", ver);
    if (strcmp(c->ota.fw_version, tmp) == 0) { return CFG_OK; }

    (void)strcpy(c->ota.fw_version, tmp);
    return config_store_save(c);
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
        s->report_period_s  = 300u;       /* 新设备默认上报 300s(5 分钟) */
        s->offline_timeout_s = 900u;      /* 3 倍上报周期(值仅记录, 无消费者) */
        s->low_battery_pct  = 80u;        /* 电量 ≤80% 上报 low_voltage */
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

