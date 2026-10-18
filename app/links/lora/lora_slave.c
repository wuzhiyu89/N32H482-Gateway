/**
 * @file  lora_slave.c
 * @brief LoRa 从机表：hex 工具 / 设备类型判定 / 运行期表装载与增删 / 在线状态查询
 *
 * 表是【动态】的：lora_business_init() 时从 config_store 的 sensors[]/devices[]
 * 装载；运行期由绑定(0x04) / 解绑(0x05) 增删。
 *
 * 对外只通过 lora_internal.h 暴露：查找、摸一下(标记在线)、增、删、装载，
 * 以及"某台设备到底算在线还是离线"的查询。别的模块不要自己遍历 g_slave_list。
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "lora_business.h"
#include "lora_internal.h"
#include "config_store.h"
#include "bsp_tim.h"      /* get_tick_ms */
#include "log.h"

/* 从机表容量 = 传感器槽 + 设备槽(真正的容量约束在 config_store_can_add)。 */
#define LORA_SLAVE_MAX  (CFG_SENSOR_MAX + CFG_DEVICE_MAX)

lora_slave_t g_slave_list[LORA_SLAVE_MAX];
uint8_t      g_slave_count = 0U;

/* 字符 → 4bit 数值，非法返回 0xFF */
static uint8_t _hex_char2nibble(char c)
{
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
    return 0xFFU;
}

/**
 * \brief hex 字符串 → 二进制数组
 * \param hex      输入 hex 字符串（NULL 结尾）
 * \param hex_len  字符串长度（不含 NULL）
 * \param out      输出缓冲（必须 >= hex_len/2）
 * \param out_max  out 缓冲最大字节数
 * \return 实际转换的字节数; -1 输入长度为奇数或非法 hex
 */
int lora_hex_str2bin(const char *hex, int hex_len, uint8_t *out, int out_max)
{
    if (hex == NULL || (hex_len & 1) != 0) return -1;   /* 奇数长度 hex 无法成字节 */
    int byte_cnt = hex_len / 2;
    if (byte_cnt > out_max) return -1;

    int bi = 0;
    for (int i = 0; i < byte_cnt; i++)
    {
        uint8_t hi = _hex_char2nibble(hex[i * 2]);
        uint8_t lo = _hex_char2nibble(hex[i * 2 + 1]);
        if (hi == 0xFFU || lo == 0xFFU) return -1;
        out[bi++] = (uint8_t)((hi << 4) | lo);
    }
    return byte_cnt;
}

/**
 * \brief 二进制数组 → ASCII 十六进制字符串(大写)
 * \param in       输入字节
 * \param in_len   字节数
 * \param out      输出缓冲, 需要 (in_len*2 + 1) 字节
 * \param out_max  out 的容量(含结尾 '\0')
 * \return 写出的字符数(不含 '\0'); 0 = 参数非法/放不下
 *
 * 云端↔网关的 MQTT 载荷就是这种"文本 hex": 一帧 20 字节 → 40 个字符。
 * 走文本而不是裸二进制, 是因为 4G 模组的 +MQTTURC: "publish" 本身就是
 * 按行输出的文本通道 —— 用文本不用改底层驱动, 日志里也能直接看懂。
 */
static uint16_t lora_hex_bin2str(const uint8_t *in, uint16_t in_len, char *out, uint16_t out_max)
{
    static const char HEX[] = "0123456789ABCDEF";
    uint16_t i;
    uint16_t n = 0U;

    if ((in == NULL) || (out == NULL)) return 0U;
    if (((uint32_t)in_len * 2U + 1U) > (uint32_t)out_max) return 0U;

    for (i = 0U; i < in_len; i++)
    {
        out[n++] = HEX[(in[i] >> 4) & 0x0FU];
        out[n++] = HEX[in[i] & 0x0FU];
    }
    out[n] = '\0';
    return n;
}

/* ---- 以上为内部辅助函数 ---- */

/* ==================== 设备类型的判断规则(现场规则) ====================
 *
 * 【从机设备地址的"首位"就是设备类型】
 *     1xxxxx = 网关      2xxxxx = 液位传感器      3xxxxx = 声光报警器
 *   设备 ID 一共 16 个(首位 0~F)。
 *
 * 地址是 3 字节【BCD】(21100001 这种 6 位十进制数), 例如
 *     21 00 01 -> "210001"   31 00 01 -> "310001"
 * 所以"首位"就是 uid[0] 的【高半字节】。
 *
 * 【为什么要有这条规则】绑定/解绑场景里, 收到的 0x04 广播帧的 dev_uid 就是
 * 发起绑定的设备自己, 网关靠这条规则把它分到液位/声光槽里, 不用设备自报类型。
 */
#define LORA_UID_TYPE_GW      1U
#define LORA_UID_TYPE_LEVEL   2U
#define LORA_UID_TYPE_ALARM   3U

uint8_t lora_dev_type_of_uid(const uint8_t uid[CFG_UID_BYTES])
{
    return (uint8_t)(uid[0] >> 4);
}

/* 运行期从机表: dev_uid/dev_type 由 lora_slave_load 从 config_store 装载,
 * 其余字段跑起来才有意义。增删见 lora_slave_add / lora_slave_del。 */
/* (g_slave_list / g_slave_count 在文件开头已声明) */

/* ---------- 从机查找 ----------
 * 【只按地址匹配, 不看 is_online】: 在不在线都要能找到, 否则云端刚下发的指令
 * 会因为设备"暂时离线"而找不到目标被丢掉。 */
uint8_t lora_slave_find(const uint8_t uid[CFG_UID_BYTES])
{
    for (uint8_t i = 0U; i < g_slave_count; i++)
    {
        if (memcmp(g_slave_list[i].dev_uid, uid, CFG_UID_BYTES) == 0)
        {
            return i;
        }
    }
    return 0xFFU;
}

/* ---------- 收到帧时"摸一下"从机: 标记在线 + 记时间戳 ----------
 * 地址不在表里就返回 0xFF, 调用方据此丢弃该帧。
 *
 * ⚠【这里故意不碰 last_report_tick】: 本函数对【任何】帧(含 0x80/0x81 ACK)都调用,
 *   而失联检测要的是"上次【液位上报】时间"。假如在这里顺手更新它, 一台只回 ACK、
 *   再也不上报的液位设备就会被永远当成"刚上报过", 失联事件永远发不出来。 */
uint8_t lora_slave_touch(const uint8_t uid[CFG_UID_BYTES])
{
    uint8_t idx = lora_slave_find(uid);
    if (idx == 0xFFU) return idx;
    g_slave_list[idx].is_online     = true;
    g_slave_list[idx].last_recv_tick = get_tick_ms();
    return idx;
}

/* ---------- lora_slave_add: 往运行期表末尾追加一台 ----------
 * 由绑定流程在收到 0x04 广播帧且网关回了 ACK、设备也回 ACK 确认后调用。
 * 容量约束(总数≤5/液位≤4/声光≤3)由调用前 config_store_can_add 兜底,
 * 这里只保证 RAM 表不越界、不重复。返回索引; -1=表满/重复。 */
int lora_slave_add(const uint8_t uid[CFG_UID_BYTES], slave_type_t type)
{
    if (g_slave_count >= LORA_SLAVE_MAX) return -1;
    if (lora_slave_find(uid) != 0xFFU)       return -1;   /* 已在表里 */

    uint8_t idx = g_slave_count;
    lora_slave_t *s = &g_slave_list[idx];
    memset(s, 0, sizeof(*s));
    memcpy(s->dev_uid, uid, CFG_UID_BYTES);
    s->dev_type  = type;
    s->state     = SLAVE_STATE_IDLE;
    s->is_online = true;                          /* 绑定当场就在线 */
    s->retry_cnt = 0U;
    g_slave_count++;
    return (int)idx;
}

/* ---------- lora_slave_del: 删除指定地址的从机 ----------
 * 把末尾元素前移填补空洞, 保持数组紧凑。由解绑流程在收到 0x80 ACK(对 0x05)
 * 或重试耗尽后调用。返回 0=成功; -1=不在表里。 */
int lora_slave_del(const uint8_t uid[CFG_UID_BYTES])
{
    uint8_t idx = lora_slave_find(uid);
    if (idx == 0xFFU) return -1;

    uint8_t last = (uint8_t)(g_slave_count - 1U);
    if (idx != last)
    {
        /* 把最后一台搬到被删的那一格 —— 比 memmove 整段前移省事, 且本表
         * 外部只用 lora_slave_find 按地址查, 不认索引顺序, 搬动不影响别的逻辑。 */
        g_slave_list[idx] = g_slave_list[last];
    }
    memset(&g_slave_list[last], 0, sizeof(g_slave_list[last]));
    g_slave_count--;
    return 0;
}

/* ---------- 从 config_store 装载运行期表(init 时调一次) ----------
 * config_store 里的 sensors[](液位) 和 devices[](声光等联动设备) 是真相源,
 * 这里把它们的 uid 解成 13 字节填进 g_slave_list。
 * 顺序: 先 sensors 再 devices, 与 lora_slave_add 的末尾追加语义一致。 */
void lora_slave_load(void)
{
    const config_store_t *cfg = config_store_get();
    uint8_t i;

    memset(g_slave_list, 0, sizeof(g_slave_list));
    g_slave_count = 0U;

    /* 液位传感器 → SLAVE_TYPE_SENSOR */
    for (i = 0U; i < cfg->sensor_count && i < CFG_SENSOR_MAX; i++)
    {
        if (g_slave_count >= LORA_SLAVE_MAX) break;
        uint8_t uid3[CFG_UID_BYTES];
        if (!cfg_uid_to_bytes(cfg->sensors[i].uid, uid3)) continue;
        lora_slave_t *s = &g_slave_list[g_slave_count];
        memcpy(s->dev_uid, uid3, CFG_UID_BYTES);
        s->dev_type  = SLAVE_TYPE_SENSOR;
        s->state     = SLAVE_STATE_IDLE;
        s->is_online = false;
        g_slave_count++;
    }

    /* 联动设备(声光等) → SLAVE_TYPE_ALARM_OUT */
    for (i = 0U; i < cfg->device_count && i < CFG_DEVICE_MAX; i++)
    {
        if (g_slave_count >= LORA_SLAVE_MAX) break;
        uint8_t uid3[CFG_UID_BYTES];
        if (!cfg_uid_to_bytes(cfg->devices[i].uid, uid3)) continue;
        lora_slave_t *s = &g_slave_list[g_slave_count];
        memcpy(s->dev_uid, uid3, CFG_UID_BYTES);
        s->dev_type  = SLAVE_TYPE_ALARM_OUT;
        s->state     = SLAVE_STATE_IDLE;
        s->is_online = false;
        g_slave_count++;
    }

    log_info("[GW] 从机表装载完成, 共 %u 台:\r\n", (unsigned)g_slave_count);
    for (i = 0U; i < g_slave_count; i++)
    {
        log_info("[GW]   [%u] dev=%02X%02X%02X type=%s\r\n", (unsigned)i,
                 g_slave_list[i].dev_uid[0], g_slave_list[i].dev_uid[1],
                 g_slave_list[i].dev_uid[2],
                 (g_slave_list[i].dev_type == SLAVE_TYPE_SENSOR) ? "SENSOR(液位)"
                                                                 : "ALARM_OUT(声光)");
    }
}

/* ==================================================================
 * ★2026-10-18: 设备在线/离线状态查询(给 sen.query / dev.query 的 online 用)
 * ------------------------------------------------------------------
 * 【为什么不能继续用 last_value 快照】那张表只有液位 0x02 会写, 声光从不进表
 *   ⇒ 声光的 online 恒 false, 平台会一直以为它掉了。
 * 【为什么声光不用 last_recv_tick】声光平时不收帧(只有下发 0x03 时才回 0x80/
 *   0x82/0x83), 用"多久没收到帧"判它失联, 会在没下发动作时【一直误判离线】。
 *   所以对它只认一条证据: 3 条 0x03 都没成功(alarm_offline_reported)。
 * ================================================================== */
lora_dev_state_t lora_business_device_state_bytes(const uint8_t uid[LORA_UID_BYTES])
{
    uint8_t i;

    if (uid == NULL)
    {
        return LORA_DEV_STATE_UNKNOWN;
    }

    for (i = 0u; i < g_slave_count; i++)
    {
        const lora_slave_t *s = &g_slave_list[i];

        if (memcmp(s->dev_uid, uid, LORA_UID_BYTES) != 0) { continue; }

        if (s->dev_type == SLAVE_TYPE_SENSOR)
        {
            return s->is_online ? LORA_DEV_STATE_ONLINE : LORA_DEV_STATE_OFFLINE;
        }

        /* 声光: 只有"3 条 0x03 都没成功"这一条确凿证据 */
        return s->alarm_offline_reported ? LORA_DEV_STATE_OFFLINE
                                         : LORA_DEV_STATE_ONLINE;
    }

    return LORA_DEV_STATE_UNKNOWN;      /* 不在从机表里 */
}

lora_dev_state_t lora_business_device_state(const char *uid_hex)
{
    uint8_t uid[CFG_UID_BYTES];

    if ((uid_hex == NULL) || (!cfg_uid_to_bytes(uid_hex, uid)))
    {
        return LORA_DEV_STATE_UNKNOWN;
    }

    return lora_business_device_state_bytes(uid);
}


lora_slave_t *lora_business_get_slave_list(void)
{
    return g_slave_list;
}

/* 从机台数(运行期动态), 给 main 里的状态打印用 */
uint8_t lora_business_get_slave_count(void)
{
    return g_slave_count;
}
