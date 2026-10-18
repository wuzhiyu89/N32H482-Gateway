/**
 * \file  ota.c
 * \brief OTA 升级状态机 + 分包重组 + CRC 校验 + 缺包检测/超时
 *
 * 数据流:
 *   ota_on_mqtt_msg(topic="ota/cmd",  JSON 文本)  -> 解析 start/abort/reboot
 *   ota_on_mqtt_msg(topic="ota/data", 二进制分包) -> 校验 header + 写 Flash
 *   ota_poll()                                    -> 擦除/校验/超时检测
 *   超时缺包 -> ota_missing_cb(业务层据此 MQTT 上报请求重传)
 *
 * 升级流程:
 *   IDLE -> [收到 start] -> ERASING -> [完成] -> DOWNLOADING
 *        -> [包收齐] -> VERIFYING -> [CRC 通过] -> READY -> [reboot]
 *   任意步失败 -> ERROR(外部 abort 或超时回 IDLE)
 *
 * 健壮性:
 *   - 包乱序接受,重复幂等(位图去重)
 *   - 下载超时触发缺包上报,最多重传 OTA_RETRANSMIT_MAX 轮,超过进 ERROR
 *   - CRC 校验分块非阻塞(每轮 256 字节),软件逐位 CRC(正确性>速度,省硬件语义对齐)
 *   - 上下文持久化到 OTA_META_ADDR+40,掉电重进可恢复下载会话
 *   - ERROR 状态超时自动回 IDLE(OTA_ERROR_RECOVER_MS,0=不自动恢复)
 */
#include "ota.h"
#include "ota_port.h"
#include <string.h>
#include <stdlib.h>

/* ==================== 内部配置 ==================== */
#define OTA_MISSING_BATCH_MAX   32u   /* 单次缺包上报最大数量,超过分批 */
#define OTA_SAVE_CTX_INTERVAL   32u   /* 每 N 包持久化一次 ctx, 减少 Flash 擦写 */

/* ==================== 内部状态 ==================== */
static ota_state_t g_state;
static ota_meta_t   g_meta;
static uint32_t     g_total_pages;       /* 需擦除页数 */
static uint16_t     g_recv_packets;       /* 已收包数 */
static uint32_t     g_crc_acc;            /* CRC32 累加器(校验阶段) */
static uint32_t     g_verify_offset;      /* CRC 校验读偏移(非阻塞分块) */
static uint32_t     g_last_pkt_tick;      /* 最后收包时间戳 */
static uint16_t     g_retransmit_cnt;     /* 已触发重传轮次 */
static uint32_t     g_error_enter_tick;   /* 进入 ERROR 状态的时间戳(用于超时自动恢复) */
static ota_missing_cb_t g_missing_cb;

/* 包接收位图: 按 OTA_FLASH_SIZE 推算, 最大包数 = ceil(240KB / 512) = 480 */
#define OTA_BITMAP_WORDS  (((OTA_FLASH_SIZE / OTA_PACKET_PAYLOAD_MAX) + 31u) / 32u)
static uint32_t g_pkt_bitmap[OTA_BITMAP_WORDS];

/* 缺包上报临时缓冲(回调返回后失效) */
static uint16_t  g_missing_buf[OTA_MISSING_BATCH_MAX];

/* ==================== 位图辅助 ==================== */
static void reset_bitmap(void)
{
    memset(g_pkt_bitmap, 0, sizeof(g_pkt_bitmap));
}

static bool bitmap_get(uint16_t seq)
{
    if (seq >= (OTA_BITMAP_WORDS * 32u))
    {
        return true;    /* 越界视为已收,防溢出 */
    }
    return (g_pkt_bitmap[seq >> 5] & (1u << (seq & 31u))) != 0u;
}

static void bitmap_set(uint16_t seq)
{
    if (seq < (OTA_BITMAP_WORDS * 32u))
    {
        g_pkt_bitmap[seq >> 5] |= (1u << (seq & 31u));
    }
}

static void goto_error(void)
{
    g_error_enter_tick = ota_port_get_tick_ms();
    g_state = OTA_ERROR;
}

static bool topic_is(const char *topic, const char *expect)
{
    return (topic != NULL) && (strcmp(topic, expect) == 0);
}

/* ==================== JSON 最小化解析(不引入 JSON 库) ==================== */

/**
 * @brief 在 JSON 中找 "key" 后的值起始指针(跳过空白和冒号)
 *        key 传带引号形式,如 "\"cmd\""
 */
static const char *json_find_value(const char *json, const char *quoted_key)
{
    const char *p = strstr(json, quoted_key);
    if (p == NULL)
    {
        return NULL;
    }
    p = strchr(p + strlen(quoted_key), ':');
    if (p == NULL)
    {
        return NULL;
    }
    p++;
    while ((*p == ' ') || (*p == '\t'))
    {
        p++;
    }
    return p;
}

/** 提取字符串值(去引号),返回写入 out 的长度,失败返回 0 */
static uint16_t json_get_str(const char *json, const char *quoted_key,
                             char *out, uint16_t out_sz)
{
    const char *v = json_find_value(json, quoted_key);
    if ((v == NULL) || (*v != '"'))
    {
        return 0u;
    }
    v++;
    size_t n = strcspn(v, "\"");
    if (n >= out_sz)
    {
        n = out_sz - 1u;
    }
    memcpy(out, v, n);
    out[n] = '\0';
    return (uint16_t)n;
}

/** 提取数值(支持 0x 前缀),失败返回 default_val */
static uint32_t json_get_u32(const char *json, const char *quoted_key,
                             uint32_t default_val)
{
    const char *p = json_find_value(json, quoted_key);
    if (p == NULL)
    {
        return default_val;
    }
    if ((p[0] == '0') && ((p[1] == 'x') || (p[1] == 'X')))
    {
        return (uint32_t)strtoul(p, NULL, 16);
    }
    return (uint32_t)strtoul(p, NULL, 10);
}

/** 判断 "cmd" 字段值是否等于 expect(如 "start") */
static bool json_cmd_is(const char *json, const char *expect)
{
    char cmd[12];
    if (json_get_str(json, "\"cmd\"", cmd, sizeof(cmd)) == 0u)
    {
        return false;
    }
    return strcmp(cmd, expect) == 0;
}

/* ==================== 版本号比较 ====================
 * 简单 semver(major.minor.patch) 比较, 每个字段是无符号整数。
 * 输入格式约定: "1.2.3", "0.9.5", "10.20.30" 等, 点号分隔。
 * 返回值: -1 表示 a < b, 0 表示 a == b, 1 表示 a > b。
 * 格式异常(缺段/非数字)时, 缺失段视为 0, 多余段忽略。
 */
static int ver_next_int(const char **p)
{
    /* 跳过非数字字符(容错: "v1.2.3" → "1.2.3", "abc" → 全部返回 0) */
    while ((**p != '\0') && ((**p < '0') || (**p > '9')))
    {
        (*p)++;
    }
    int v = 0;
    while ((**p >= '0') && (**p <= '9'))
    {
        v = v * 10 + (**p - '0');
        (*p)++;
    }
    if (**p == '.')
    {
        (*p)++;
    }
    return v;
}

static int version_compare(const char *a, const char *b)
{
    const char *pa = a;
    const char *pb = b;
    for (int i = 0; i < 3; i++)
    {
        int va = ver_next_int(&pa);
        int vb = ver_next_int(&pb);
        if (va != vb)
        {
            return (va < vb) ? -1 : 1;
        }
    }
    return 0;
}

/* ==================== 数据包处理 ==================== */
static bool handle_data_packet(const uint8_t *data, uint16_t len)
{
    uint16_t seq, total, payload_len;
    uint32_t offset;
    const uint8_t *payload;

    if ((len < OTA_PACKET_HEADER_LEN) || (len > OTA_PACKET_TOTAL_MAX))
    {
        return false;
    }
    if ((data[0] != OTA_PACKET_MAGIC0) || (data[1] != OTA_PACKET_MAGIC1) ||
        (data[2] != OTA_PACKET_MAGIC2) || (data[3] != OTA_PACKET_MAGIC3))
    {
        return false;
    }
    seq         = (uint16_t)data[4] | ((uint16_t)data[5] << 8);
    total       = (uint16_t)data[6] | ((uint16_t)data[7] << 8);
    payload_len = (uint16_t)data[8] | ((uint16_t)data[9] << 8);

    /* CRC16 包头校验(前 10 字节) —— 字段填 0 表示跳过校验 */
    uint16_t pkt_crc16 = (uint16_t)data[10] | ((uint16_t)data[11] << 8);
    if (pkt_crc16 != 0u)
    {
        uint16_t calc_crc16 = ota_port_crc16(data, 10u);
        if (calc_crc16 != pkt_crc16)
        {
            return false;   /* 包头损坏, 丢弃 */
        }
    }

    if ((seq >= total) || (total != g_meta.packet_count))
    {
        return false;
    }
    if ((uint16_t)(len - OTA_PACKET_HEADER_LEN) != payload_len)
    {
        return false;
    }
    /* 中间包必须满 512,保证 offset 对齐(末尾包可小) */
    if ((seq < (total - 1u)) && (payload_len != OTA_PACKET_PAYLOAD_MAX))
    {
        return false;
    }
    if (bitmap_get(seq))
    {
        g_last_pkt_tick = ota_port_get_tick_ms();   /* 重复包也刷新超时 */
        return true;
    }

    payload = data + OTA_PACKET_HEADER_LEN;
    offset  = (uint32_t)seq * OTA_PACKET_PAYLOAD_MAX;

    if ((offset + payload_len) > g_meta.total_size)
    {
        return false;
    }
    /* offset = seq*512, 天然 8 字节对齐(ota_port_flash_write 要求) */
    if (!ota_port_flash_write(offset, payload, payload_len))
    {
        goto_error();
        return false;
    }

    bitmap_set(seq);
    g_recv_packets++;
    g_last_pkt_tick = ota_port_get_tick_ms();

    /* 节流持久化: 每 OTA_SAVE_CTX_INTERVAL 包存一次, 末尾包(recv_packets==total)必存
     * — 避免每包擦写整页, N32 Flash 擦写次数有限 */
    bool final_pkt = (g_recv_packets >= g_meta.packet_count);
    if (final_pkt || ((g_recv_packets % OTA_SAVE_CTX_INTERVAL) == 0u))
    {
        (void)ota_port_save_ctx(&g_meta, g_recv_packets,
                                g_pkt_bitmap, OTA_BITMAP_WORDS);
    }
    return true;
}

/* ==================== CMD 处理 ==================== */
static bool parse_start(const char *json, ota_meta_t *meta)
{
    char cmd[8];
    if (json_get_str(json, "\"cmd\"", cmd, sizeof(cmd)) == 0u)
    {
        return false;
    }
    if (strcmp(cmd, OTA_CMD_START) != 0)
    {
        return false;
    }
    json_get_str(json, "\"version\"", meta->version, sizeof(meta->version));
    meta->total_size   = json_get_u32(json, "\"size\"", 0u);
    meta->crc32        = json_get_u32(json, "\"crc32\"", 0u);
    meta->packet_count = (uint16_t)json_get_u32(json, "\"packets\"", 0u);

    /* 基础字段合法性 */
    if ((meta->total_size == 0u) || (meta->total_size > OTA_FLASH_SIZE) ||
        (meta->packet_count == 0u) ||
        (meta->packet_count > (OTA_BITMAP_WORDS * 32u)))
    {
        return false;
    }

    /* 版本比较: 拒绝同版本或更旧版本 */
    if (strlen(meta->version) > 0u)
    {
        const char *cur = ota_port_get_version();
        int cmp = version_compare(meta->version, cur);
        if (cmp <= 0)
        {
            /* 新版本号 <= 当前版本号 —— 拒绝升级 */
            return false;
        }
    }

    return true;
}

static void start_upgrade(const ota_meta_t *meta)
{
    g_meta           = *meta;
    g_recv_packets    = 0u;
    g_retransmit_cnt  = 0u;
    reset_bitmap();

    /* 先擦干净标记页: 清除上一个升级的 mark_valid / ctx 残留,
     * 让本次 save_ctx 可以在干净页上写, mark_valid 也有干净页擦。
     * 注意: 擦除标记页 ≠ 擦除固件缓存区, erase_begin 会管后者。 */
    ota_port_mark_invalid();

    g_total_pages = (meta->total_size + OTA_FLASH_PAGE_SIZE - 1u) / OTA_FLASH_PAGE_SIZE;
    if (g_total_pages > (OTA_FLASH_SIZE / OTA_FLASH_PAGE_SIZE))
    {
        goto_error();
        return;
    }
    (void)ota_port_flash_erase_begin(g_total_pages);
    g_last_pkt_tick = ota_port_get_tick_ms();
    g_state = OTA_ERASING;
}

static void handle_cmd_msg(const uint8_t *data, uint16_t len)
{
    char buf[192];
    if ((len == 0u) || (len >= sizeof(buf)))
    {
        return;
    }
    memcpy(buf, data, len);
    buf[len] = '\0';

    if (json_cmd_is(buf, OTA_CMD_START))
    {
        ota_meta_t meta;
        if ((g_state != OTA_IDLE) && (g_state != OTA_ERROR))
        {
            return;     /* 升级进行中,忽略重复 start */
        }
        if (!parse_start(buf, &meta))
        {
            goto_error();
            return;
        }
        start_upgrade(&meta);
    }
    else if (json_cmd_is(buf, OTA_CMD_ABORT))
    {
        ota_abort();
    }
    else if (json_cmd_is(buf, OTA_CMD_REBOOT))
    {
        if (g_state == OTA_READY)
        {
            if (ota_port_clear_ctx())
                {
                    ota_port_reboot();
                }
        }
    }
}

/* ==================== 缺包检测 ==================== */
/**
 * @brief 扫描位图收集缺失 seq 到 g_missing_buf
 * @return 实际存入数量(上限 OTA_MISSING_BATCH_MAX)
 */
static uint16_t collect_missing(void)
{
    uint16_t count = 0u;
    uint16_t seq;
    for (seq = 0u; seq < g_meta.packet_count; seq++)
    {
        if (!bitmap_get(seq))
        {
            if (count < OTA_MISSING_BATCH_MAX)
            {
                g_missing_buf[count] = seq;
            }
            count++;
        }
    }
    return (count > OTA_MISSING_BATCH_MAX) ? OTA_MISSING_BATCH_MAX : count;
}

static void trigger_retransmit(void)
{
    uint16_t n = collect_missing();
    if ((n > 0u) && (g_missing_cb != NULL))
    {
        g_missing_cb(g_missing_buf, n);
    }
    g_last_pkt_tick = ota_port_get_tick_ms();
    g_retransmit_cnt++;
    if (g_retransmit_cnt > OTA_RETRANSMIT_MAX)
    {
        goto_error();
    }
}

/* ==================== 对外 API ==================== */
void ota_init(void)
{
    g_state          = OTA_IDLE;
    g_recv_packets   = 0u;
    g_retransmit_cnt  = 0u;
    g_crc_acc         = 0xFFFFFFFFu;
    g_verify_offset   = 0u;
    g_error_enter_tick = 0u;
    g_missing_cb     = NULL;
    reset_bitmap();

    /* 尝试从 Flash 恢复未完成的升级会话(断点续传)
     * 条件: load_ctx 成功 + recv_packets > 0 + recv_packets < packet_count
     *       (即 ctx 有效但包没收齐 —— 收齐的情况应该已经走 VERIFYING/READY
     *        并写 mark_valid 了, ctx 应该被擦除; 如果 ctx 还存在说明是
     *        掉电中断在 DOWNLOADING 阶段) */
    ota_meta_t  resume_meta;
    uint16_t    resume_recv   = 0u;
    uint16_t    resume_bm_cap = OTA_BITMAP_WORDS;   /* 输入: 缓冲上限; 输出: 实际字数 */
    if (ota_port_load_ctx(&resume_meta, &resume_recv,
                          g_pkt_bitmap, &resume_bm_cap))
    {
        /* load_ctx 内部已校验 recv_packets ∈ (0, packet_count], 再额外确认没收齐 */
        if (resume_recv < resume_meta.packet_count)
        {
            g_meta           = resume_meta;
            g_recv_packets   = resume_recv;
            g_retransmit_cnt = 0u;              /* 重传轮次是会话内计数, 掉电清零 */
            g_last_pkt_tick  = ota_port_get_tick_ms();  /* 防刚恢复就因长时间无新包触发缺包 */
            g_state          = OTA_DOWNLOADING;
        }
    }
}

void ota_poll(void)
{
    switch (g_state)
    {
        case OTA_ERASING:
            if (ota_port_flash_erase_step())
            {
                if (!ota_port_flash_erase_ok())
                {
                    goto_error();
                }
                else
                {
                    g_state = OTA_DOWNLOADING;
                    g_last_pkt_tick = ota_port_get_tick_ms();
                }
            }
            break;

        case OTA_DOWNLOADING:
            /* 包收齐 -> 校验 */
            if (g_recv_packets >= g_meta.packet_count)
            {
                g_state        = OTA_VERIFYING;
                g_crc_acc      = 0xFFFFFFFFu;
                g_verify_offset = 0u;
                break;
            }
            /* 超时 -> 缺包重传 */
            if ((ota_port_get_tick_ms() - g_last_pkt_tick) > OTA_DOWNLOAD_TIMEOUT_MS)
            {
                trigger_retransmit();
            }
            break;

        case OTA_VERIFYING:
        {
            uint8_t  chunk[256];
            uint16_t n;
            if (g_verify_offset >= g_meta.total_size)
            {
                uint32_t final_crc = g_crc_acc ^ 0xFFFFFFFFu;
                if (final_crc == g_meta.crc32)
                {
                    /* CRC 通过 -> 写有效标记(bootloader 检测用) */
                    if (ota_port_mark_valid(&g_meta))
                    {
                        g_state = OTA_READY;
                    }
                    else
                    {
                        goto_error();
                    }
                }
                else
                {
                    /* CRC 不通过 -> 清除标记(以防有残留) + 进 ERROR */
                    ota_port_mark_invalid();
                    goto_error();
                }
                break;
            }
            n = ((g_meta.total_size - g_verify_offset) > sizeof(chunk))
                ? (uint16_t)sizeof(chunk)
                : (uint16_t)(g_meta.total_size - g_verify_offset);
            if (ota_port_flash_read(g_verify_offset, chunk, n))
            {
                g_crc_acc = ota_port_crc32_update(g_crc_acc, chunk, n);
                g_verify_offset += n;
            }
            else
            {
                goto_error();
            }
            break;
        }

        case OTA_ERROR:
        {
            /* ERROR 状态超时自动恢复到 IDLE, 避免永远卡死
             * (如果 OTA_ERROR_RECOVER_MS 设为 0 则永不自动恢复) */
            if (OTA_ERROR_RECOVER_MS > 0u)
            {
                uint32_t elapsed = ota_port_get_tick_ms() - g_error_enter_tick;
                if (elapsed >= OTA_ERROR_RECOVER_MS)
                {
                    ota_abort();
                }
            }
            break;
        }

        default:
            break;
    }
}

void ota_on_mqtt_msg(const char *topic, const uint8_t *data, uint16_t len)
{
    if ((topic == NULL) || (data == NULL) || (len == 0u))
    {
        return;
    }
    if (topic_is(topic, OTA_TOPIC_CMD))
    {
        handle_cmd_msg(data, len);
    }
    else if (topic_is(topic, OTA_TOPIC_DATA))
    {
        if (g_state != OTA_DOWNLOADING)
        {
            return;
        }
        (void)handle_data_packet(data, len);
    }
}

void ota_set_missing_cb(ota_missing_cb_t cb)
{
    g_missing_cb = cb;
}

ota_state_t ota_get_state(void)
{
    return g_state;
}

uint16_t ota_get_progress_permil(void)
{
    uint32_t p;
    switch (g_state)
    {
        case OTA_IDLE:
        case OTA_ERROR:
            return 0u;
        case OTA_ERASING:
            return 50u;    /* 擦除阶段无法知进度,固定值 */
        case OTA_DOWNLOADING:
            p = (g_meta.packet_count == 0u)
                ? 0u
                : (g_recv_packets * 850u) / g_meta.packet_count;
            return (uint16_t)(50u + p);   /* 50..900 */
        case OTA_VERIFYING:
            return 920u;
        case OTA_READY:
            return 1000u;
        default:
            return 0u;
    }
}

void ota_get_status(ota_status_t *out)
{
    if (out == NULL)
    {
        return;
    }
    out->state          = g_state;
    out->meta           = g_meta;
    out->recv_packets   = g_recv_packets;
    out->missing_count  = (g_state == OTA_DOWNLOADING) ? collect_missing() : 0u;
    out->retransmit_cnt = g_retransmit_cnt;
    out->progress_permil = ota_get_progress_permil();
}

void ota_abort(void)
{
    /* 擦除标记页: 同时清掉 mark_valid + ctx + 任何残留升级状态,
     * bootloader 读不到 magic 就跳过升级。
     * 仅在非 IDLE 状态下擦除 —— 在 IDLE 调 abort 不需要擦任何东西
     * (标记页本来就是干净的), 避免无意义地擦 8KB Flash 页。
     *
     * 注意: 本函数不在 reboot 路径上被调用 —— reboot 时 mark_valid
     * 必须保留, bootloader 才能正确搬运固件。 */
    if (g_state != OTA_IDLE)
    {
        ota_port_mark_invalid();
    }

    g_state          = OTA_IDLE;
    g_recv_packets   = 0u;
    g_retransmit_cnt  = 0u;
    g_crc_acc         = 0xFFFFFFFFu;
    g_verify_offset   = 0u;
    g_error_enter_tick = 0u;
    reset_bitmap();
}
