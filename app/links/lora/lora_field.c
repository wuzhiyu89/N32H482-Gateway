/**
 * @file  lora_field.c
 * @brief 出厂字段：写(0x06) / 读(0x07) / 应答(0x08) 的本地状态机
 *
 * 只在网关本地跑，不产生任何 MQTT 流量（字段不进 config_store、不进两种缓存）。
 * 目标设备是"从机表里唯一那台"（产线一次接一台），所以不需要按 uid 分发。
 *
 * 节奏：写 = 每 5 s 一条、最多 3 条；读 = 只发一次 0x07、等 12 s
 *       （设备自己会重发 0x08 两次，每次 5 s）。
 */
#include <string.h>

#include "log.h"
#include "bsp_tim.h"
#include "lora_business.h"
#include "lora_internal.h"
#include "lora_proto.h"
#include "config_store.h"   /* CFG_UID_BYTES */

/* ==================================================================
 * ★★2026-10-18 新增: 出厂字段的写入(0x06) / 读取(0x07)★★
 * ==================================================================
 * 【为什么单独一套状态机, 不复用 _try_send_alarm_idle / data_cache】
 *   1) 用户明确"只在本地跑, 不要让云端知道" ⇒ 不能进 data_cache(那条路会组帧
 *      发 MQTT / 走云端下发队列), 否则离线/在线的行为都不一样了;
 *   2) 重发节奏是【每 5 秒一次、只发 3 次】, 与 0x01/0x03 的"帧级重传 + 业务重试"
 *      完全不同, 硬塞进去会把两套节拍搅在一起;
 *   3) 目标设备是"从机表里唯一那台"(产线一次只接一台), 不需要按 uid 分发。
 *
 * 【0x80/0x81 的归属问题】
 *   0x80 是纯应答码, 不带"应答的是哪个 func"(见 lora_business_send_ack 的注释),
 *   所以只能靠"当前有没有在等"来判断。字段操作期间不并发下发别的 0x01/0x03,
 *   这个前提在产线场景下成立; 真要并发, 得给协议加"原功能码"字段(两端同时改)。
 * ================================================================== */
#define FIELD_WRITE_RETRY_MS   5000U    /* 写字段: 每 5 秒补发一次(用户指定) */
#define FIELD_WRITE_MAX_TRIES  3U       /* 写字段: 最多发 3 条(用户指定) */
#define FIELD_READ_WAIT_MS     12000U   /* 读字段: 只发一次 0x07, 等 12 秒
                                         * (设备侧自己会重发 0x08 两次, 各 5 秒) */

typedef enum
{
    FIELD_OP_IDLE = 0,
    FIELD_OP_WRITE,
    FIELD_OP_READ
} field_op_t;

static struct
{
    field_op_t state;
    uint8_t    dev_uid[CFG_UID_BYTES];
    char       field[LORA_FIELD_LEN + 1u];
    uint8_t    tries;
    uint32_t   last_tx_tick;
    uint32_t   deadline_tick;
} g_field;
_Static_assert(sizeof(g_field.dev_uid) == CFG_UID_BYTES, "g_field.dev_uid 必须 13 字节");

static lora_field_result_fn g_field_result_cb;

void lora_business_set_field_result_cb(lora_field_result_fn fn)
{
    g_field_result_cb = fn;
}

/** 收尾: 清状态并回调(ok/field 由调用方判定), 只回调一次 */
static void _field_finish(bool ok, const char *field10)
{
    char buf[LORA_FIELD_LEN + 1u];
    uint8_t i;

    g_field.state = FIELD_OP_IDLE;

    if ((field10 != NULL) && ok)
    {
        for (i = 0u; i < LORA_FIELD_LEN; i++) { buf[i] = field10[i]; }
        buf[LORA_FIELD_LEN] = '\0';
    }
    else
    {
        buf[0] = '\0';
    }

    if (g_field_result_cb != NULL)
    {
        g_field_result_cb(ok, buf);
    }
}

/** 组一帧直接发(不经 data_cache / 不进云端队列) */
static bool _field_send(uint8_t func, const uint8_t *data, uint8_t dlen)
{
    lora_frame_t f;

    (void)memset(&f, 0, sizeof(f));
    (void)memcpy(f.gw_uid,  g_gw_uid,       CFG_UID_BYTES);
    (void)memcpy(f.dev_uid, g_field.dev_uid, CFG_UID_BYTES);
    f.func     = func;
    f.data_len = dlen;
    if ((data != NULL) && (dlen > 0u))
    {
        (void)memcpy(f.data, data, dlen);
    }

    return lora_proto_send_frame(&f);
}

/** 目标设备 = 从机表里唯一那台; 不满足返回 false(调用方负责提示) */
static bool _field_pick_target(void)
{
    if (lora_business_get_slave_count() != 1u)
    {
        log_info("[FIELD] !! 从机表里有 %u 台设备, 字段操作要求正好 1 台"
                 "(先 #bind 绑进来, 或先解绑其它设备)\r\n",
                 (unsigned)lora_business_get_slave_count());
        return false;
    }
    (void)memcpy(g_field.dev_uid, lora_business_get_slave_list()[0].dev_uid, CFG_UID_BYTES);
    return true;
}

bool lora_business_field_write_begin(const char *field10)
{
    uint8_t i;

    if (g_field.state != FIELD_OP_IDLE)
    {
        log_info("[FIELD] !! 已有字段操作在进行, 先等它结束(#field 可看状态)\r\n");
        return false;
    }
    if ((field10 == NULL) || (strlen(field10) != (size_t)LORA_FIELD_LEN))
    {
        log_info("[FIELD] !! 字段必须恰好 %u 字节(实际 %u) -> 不发\r\n",
                 (unsigned)LORA_FIELD_LEN,
                 (unsigned)((field10 != NULL) ? strlen(field10) : 0u));
        return false;
    }
    if (!_field_pick_target()) { return false; }

    for (i = 0u; i < LORA_FIELD_LEN; i++) { g_field.field[i] = field10[i]; }
    g_field.field[LORA_FIELD_LEN] = '\0';

    g_field.state      = FIELD_OP_WRITE;
    g_field.tries      = 0u;
    g_field.last_tx_tick = get_tick_ms();

    log_info("[FIELD] 开始写字段 \"%s\" 到 %02X%02X%02X (每 %us 一条, 最多 %u 条)\r\n",
             g_field.field, g_field.dev_uid[0], g_field.dev_uid[1], g_field.dev_uid[2],
             (unsigned)(FIELD_WRITE_RETRY_MS / 1000u), (unsigned)FIELD_WRITE_MAX_TRIES);

    /* 立刻发第一条 */
    g_field.tries++;
    if (!_field_send((uint8_t)LORA_FUNC_SET_FIELD, (const uint8_t *)g_field.field,
                     (uint8_t)LORA_FIELD_LEN))
    {
        log_info("[FIELD] !! 第 1 条 0x06 发送失败(射频忙?), 继续按节奏重试\r\n");
    }
    return true;
}

bool lora_business_field_read_begin(void)
{
    if (g_field.state != FIELD_OP_IDLE)
    {
        log_info("[FIELD] !! 已有字段操作在进行\r\n");
        return false;
    }
    if (!_field_pick_target()) { return false; }

    g_field.state         = FIELD_OP_READ;
    g_field.tries         = 1u;
    g_field.last_tx_tick  = get_tick_ms();
    g_field.deadline_tick = get_tick_ms() + FIELD_READ_WAIT_MS;
    g_field.field[0]      = '\0';

    log_info("[FIELD] 开始读 %02X%02X%02X 的字段(0x07 只发一次, 等 %us)\r\n",
             g_field.dev_uid[0], g_field.dev_uid[1], g_field.dev_uid[2],
             (unsigned)(FIELD_READ_WAIT_MS / 1000u));

    if (!_field_send((uint8_t)LORA_FUNC_GET_FIELD, NULL, 0u))
    {
        log_info("[FIELD] !! 0x07 发送失败(射频忙?) -> 直接判失败\r\n");
        _field_finish(false, "");
        return false;
    }
    return true;
}

void lora_business_field_poll(void)
{
    uint32_t now = get_tick_ms();

    if (g_field.state == FIELD_OP_IDLE)
    {
        return;
    }

    if (g_field.state == FIELD_OP_WRITE)
    {
        if ((uint32_t)(now - g_field.last_tx_tick) < FIELD_WRITE_RETRY_MS)
        {
            return;                     /* 还没到 5 秒 */
        }

        if (g_field.tries >= FIELD_WRITE_MAX_TRIES)
        {
            /* 3 条都发完 + 最后一条也等满了 5 秒 ⇒ 判失败(用户口径) */
            log_info("[FIELD] !! 3 条 0x06 都没收到 0x80 -> 写字段失败\r\n");
            _field_finish(false, "");
            return;
        }

        g_field.tries++;
        g_field.last_tx_tick = now;
        log_info("[FIELD] 第 %u/%u 条 0x06 重发\r\n",
                 (unsigned)g_field.tries, (unsigned)FIELD_WRITE_MAX_TRIES);
        if (!_field_send((uint8_t)LORA_FUNC_SET_FIELD, (const uint8_t *)g_field.field,
                         (uint8_t)LORA_FIELD_LEN))
        {
            log_info("[FIELD] !! 本条 0x06 发送失败, 继续按节奏重试\r\n");
        }
        return;
    }

    /* FIELD_OP_READ */
    if ((int32_t)(now - g_field.deadline_tick) >= 0)
    {
        log_info("[FIELD] !! %us 内没收到 0x08 -> 读字段失败\r\n",
                 (unsigned)(FIELD_READ_WAIT_MS / 1000u));
        _field_finish(false, "");
    }
}

void lora_business_field_cancel(void)
{
    if (g_field.state != FIELD_OP_IDLE)
    {
        log_info("[FIELD] 字段操作被取消\r\n");
        g_field.state = FIELD_OP_IDLE;
    }
}

/**
 * @brief 收到设备的 0x08(字段应答) —— ★必须回 0x80 做 ack★
 *
 * 【为什么无论等不等都要回】设备侧的逻辑是"发 0x08 -> 开 5 秒窗口等网关 ack ->
 *   没收到就再发一次"。网关不回, 设备会白等两个 5 秒窗口(真实占用空口和电)。
 *   所以这里【先回 ack, 再判断要不要采用这个字段】。
 *
 * 【为什么立刻回, 不走 _reply_schedule 的 3 秒延迟】设备的窗口只有 5 秒,
 *   3 秒延迟虽然勉强够, 但一旦叠加处理/发射时间就容易踩线。0x08 是设备对
 *   我们 0x07 的应答, 不会引发"两边互相 ACK"的风暴(那是 0x80/0x81 才有的风险),
 *   所以可以立刻回。
 */
void lora_field_handle_resp(const lora_frame_t *frame, uint8_t idx)
{
    char    field[LORA_FIELD_LEN + 1u];
    uint8_t i;

    (void)idx;
    (void)lora_business_send_ack(g_gw_uid, frame->dev_uid,
                                 LORA_FUNC_FIELD_RESP, true, 0U);

    if (frame->data_len != (uint8_t)LORA_FIELD_LEN)
    {
        log_info("[FIELD] !! 0x08 的 data_len=%u, 期望 %u -> 丢弃"
                 "(但 ack 已经回了, 设备不会白等)\r\n",
                 (unsigned)frame->data_len, (unsigned)LORA_FIELD_LEN);
        return;
    }

    for (i = 0u; i < LORA_FIELD_LEN; i++) { field[i] = (char)frame->data[i]; }
    field[LORA_FIELD_LEN] = '\0';

    log_info("[FIELD] 设备 %02X%02X%02X 回报字段 = \"%s\"\r\n",
             frame->dev_uid[0], frame->dev_uid[1], frame->dev_uid[2], field);

    if ((g_field.state == FIELD_OP_READ) &&
        (memcmp(frame->dev_uid, g_field.dev_uid, CFG_UID_BYTES) == 0))
    {
        _field_finish(true, field);
    }
}

/* 0x80/0x81 的归属判断：写字段期间设备回的 ACK 由本状态机消费。
 * 返回 true = 这条 ACK 属于字段操作（调用方不要再走从机状态机）。 */
bool lora_field_on_ack(const lora_frame_t *frame)
{
    if ((g_field.state != FIELD_OP_WRITE) ||
        (memcmp(frame->dev_uid, g_field.dev_uid, CFG_UID_BYTES) != 0))
    {
        return false;
    }

    if (frame->func == (uint8_t)LORA_FUNC_RESP_OK)
    {
        log_info("[FIELD] 设备确认已写入(0x80), 停止重发\r\n");
        _field_finish(true, "");
    }
    else
    {
        log_info("[FIELD] !! 设备回 0x81(拒绝/失败) -> 写字段失败\r\n");
        _field_finish(false, "");
    }
    return true;
}
