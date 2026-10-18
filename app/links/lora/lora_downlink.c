/**
 * @file  lora_downlink.c
 * @brief 下发队列：投递 / 空口发送 / ACK 重传 / 两步错开的应答时序 / MQTT 下行
 *
 * 数据流：
 *   云或本地业务 --投递--> data_cache(每台一条) --lora_try_send_downlink--> 空口
 *     -> WAIT_ACK: 收到 ACK 清缓存；超时重传 3 次，耗尽丢弃（不无限重发）
 *
 * 两条时间线：
 *   · 液位是休眠设备：收到 0x02 后"先回 0x80，再等 3s 下发"（lora_reply_*）；
 *   · 声光是常驻 RX：lora_try_send_alarm_idle() 每秒扫 IDLE 的声光设备主动发。
 *
 * 0x03 的执行结果（0x82/0x83）不在本文件 —— 见 lora_alarm.c；本文件只在
 * 发送咽喉里给 alarm_attempts/alarm_tick 记账。
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "lora_business.h"
#include "lora_internal.h"
#include "lora_proto.h"
#include "data_cache.h"
#include "g4_urc.h"
#include "g4_at.h"
#include "bsp_tim.h"
#include "config_store.h"
#include "log.h"

bool lora_tx_downlink(uint8_t idx)
{
    lora_slave_t *s = &g_slave_list[idx];
    lora_frame_t frame;
    bool         ok;

    memset(&frame, 0, sizeof(frame));
    memcpy(frame.gw_uid,  g_gw_uid, CFG_UID_BYTES);
    memcpy(frame.dev_uid, s->dev_uid, CFG_UID_BYTES);
    frame.func      = s->pending_func;
    frame.data_len  = s->pending_data_len;
    if (frame.data_len > 0U)
    {
        memcpy(frame.data, s->pending_data, frame.data_len);
    }
    ok = lora_proto_send_frame(&frame);

    /* ==========================================================================
     * ★★2026-10-08: 声光(0x03)每次【真的发出去】就记一次尝试, 并启动 15 秒窗口★★
     * --------------------------------------------------------------------------
     * 为什么记在这里: 这是【唯一的发送咽喉】(lora_downlink_logic 的重发和
     *   lora_try_send_alarm_idle 的首次发送都走它), 记在这里不会漏也不会重复。
     * 为什么只在 ok 时记: TX 忙导致的发送失败不算一次尝试 —— 那条指令还在缓存里,
     *   下一拍会被 lora_try_send_alarm_idle 再捡起来, 不该白扣一次 3 条的配额。
     * ======================================================================== */
    if (ok && (s->pending_func == (uint8_t)LORA_FUNC_SET_ALARM))
    {
        s->alarm_attempts++;
        s->alarm_pending = true;
        s->alarm_tick    = get_tick_ms();
        s->alarm_gear    = g_alarm_gear_inflight;
        log_info("[GEAR] -> %02X%02X%02X 发出第 %u/%u 条 0x03(档位 %u), 等 0x82/0x83 %us\r\n",
                 s->dev_uid[0], s->dev_uid[1], s->dev_uid[2],
                 (unsigned)s->alarm_attempts, (unsigned)LORA_ALARM_MAX_ATTEMPTS,
                 (unsigned)s->alarm_gear, (unsigned)(LORA_ALARM_TIMEOUT_MS / 1000u));
    }
    return ok;
}


/* lora_try_send_downlink() 的结果, 让调用方能把日志说清楚 */
typedef enum
{
    DOWNLINK_SENT        =  1,   /* 真的发出去了 */
    DOWNLINK_BUSY        =  0,   /* 该从机正在等 ACK, 这轮不发 */
    DOWNLINK_NO_CACHE    = -1,   /* 下发缓存里没有这条设备的指令 */
    DOWNLINK_TX_FAIL     = -2,   /* 组帧/射频发送失败 */
    DOWNLINK_BAD_PAYLOAD = -3,   /* 缓存项格式不对(连 func 都没有) */
} downlink_result_t;

/**
 * \brief 收到上行帧后: 检查该设备是否有待下发指令, IDLE 状态则发送
 * \return downlink_result_t
 */
static int lora_try_send_downlink(uint8_t idx)
{
    lora_slave_t *s = &g_slave_list[idx];
    if (s->state != SLAVE_STATE_IDLE) return DOWNLINK_BUSY;

    cache_down_item_t item;
    if (!data_cache_down_fetch_by_uid(s->dev_uid, &item)) return DOWNLINK_NO_CACHE;

    /* payload 打包约定: payload[0] = func, payload[1..] = data
     * (lora_business_send_req 在存入 data_cache 时就这么拼好了) */
    if (item.pay_len < 1U) return DOWNLINK_BAD_PAYLOAD;

    s->pending_func     = item.payload[0];
    s->pending_data_len = (uint8_t)(item.pay_len - 1U);
    if (s->pending_data_len > 0U)
    {
        memcpy(s->pending_data, &item.payload[1], s->pending_data_len);
    }

    /* 【0x01 的校时必须在"要发的这一刻"重新向 4G 取】—— 现场协议要求:
     *   时间是【网关 lora 发送数据时】向 4G 模块取来组帧的。
     *
     *   为什么不能沿用"收到云端下行时"取的那个时间戳:
     *   液位是休眠设备, 上报间隔最长可配到 65535 秒(≈18.2 小时) —— 云端完全可能在
     *   它上报前很久就把参数发下来了。用那时的时间戳会把设备 RTC 校偏那么久。
     *
     *   取不到怎么办: 【沿用缓存里的旧值】并告警, 而不是把这条指令丢掉 ——
     *   时间戳略旧也好过"参数永远下不去"。
     *
     *   载荷布局(与设备端 app_cmd.h 一致, 共 13 字节, 16 位字段低字节在前):
     *     [0-1] 检测间隔 [2-3] 上报间隔 [4] 相同液位上报表 [5-6] 阈值
     *     [7]年 [8]月 [9]日 [10]周 [11]时 [12]分          <- 这 6 字节是网关补的 */
    if ((s->pending_func == LORA_FUNC_SET_SENSOR_PARAM) && (s->pending_data_len >= 13U))
    {
        g4_clock_t clk;

        if (g4_at_get_clock(&clk))
        {
            s->pending_data[7]  = clk.year;
            s->pending_data[8]  = clk.month;
            s->pending_data[9]  = clk.day;
            s->pending_data[10] = clk.week;     /* 4G 应答里没有星期, 固定周五(5) */
            s->pending_data[11] = clk.hour;
            s->pending_data[12] = clk.minute;

            log_info("[GW] 0x01 校时按【发送时刻】重取: 20%02u-%02u-%02u w%u %02u:%02u\r\n",
                     (unsigned)clk.year, (unsigned)clk.month, (unsigned)clk.day,
                     (unsigned)clk.week, (unsigned)clk.hour, (unsigned)clk.minute);
        }
        else
        {
            log_info("[GW] !! 发送前取不到 4G 时间(AT+CCLK?), 沿用缓存里的旧校时 "
                     "20%02u-%02u-%02u 继续下发\r\n",
                     (unsigned)s->pending_data[7], (unsigned)s->pending_data[8],
                     (unsigned)s->pending_data[9]);
        }
    }

    /* 打印下发帧内容(调试用) */
    log_info("[LoRA TX] dev=%02X%02X%02X func=0x%02X data_len=%u data=",
             s->dev_uid[0], s->dev_uid[1], s->dev_uid[2],
             s->pending_func, s->pending_data_len);
    for (uint8_t di = 0; di < s->pending_data_len && di < 32; di++) {
        log_info("%02X ", s->pending_data[di]);
    }
    log_info("\r\n");

    if (!lora_tx_downlink(idx)) return DOWNLINK_TX_FAIL;

    s->state       = SLAVE_STATE_WAIT_ACK;
    s->retry_cnt   = 0U;
    s->send_req_tick = get_tick_ms();

    return DOWNLINK_SENT;
}

/**
 * \brief 下行 ACK 超时重传检测 (轮询调用)
 *
 * 【重传耗尽 = 丢弃这条指令】—— 这是 2026-09 修掉的一个严重逻辑错误。
 *
 * 老实现耗尽后只把状态改回 IDLE、缓存【留着】, 注释写的是"等设备下次上报再试"。
 * 但声光报警器是【常驻 RX、且从不主动上报】的设备:
 *   lora_try_send_alarm_idle() 每秒扫一遍 IDLE 的报警器, 一看缓存里还有这条指令
 *   → 又发一轮(3 次重传) → 又耗尽 → 状态回 IDLE → 下一秒再发 …
 * 结果就是【网关永远在定时重发同一条 0x03】:
 *   1) 空口被这条帧刷满, 液位传感器的上报被挤掉 / 收到也来不及应答;
 *   2) 每次发送都要阻塞 ~500ms(LORA_TX_SETTLE_MS + 等 TX_DONE), 主循环被反复
 *      按停, 这期间 g4_at_poll() 不跑, 云端下行堆在串口缓冲里溢出丢失;
 *   3) 报警器那边收到的是同一条指令的无限副本。
 *
 * 正式业务逻辑: 3 次重传都没 ACK = 这次下发失败, 丢掉缓存并明确打日志;
 * 需要再试由云端重新下发(或等液位下次上报搭车 —— 液位缓存也一起丢,
 * 保持"两类设备语义一致", 免得又埋一个"某些设备会偷偷无限重试"的坑)。
 */
void lora_downlink_logic(void)
{
    uint32_t now = get_tick_ms();
    for (uint8_t i = 0U; i < g_slave_count; i++)
    {
        lora_slave_t *s = &g_slave_list[i];
        if (s->state != SLAVE_STATE_WAIT_ACK) continue;

        /* 【解绑(0x05)的 slave 由 g_unbind 状态机独立驱动, 不走这里的重传逻辑】
         *   g_unbind 启动时把目标 slave 置 WAIT_ACK+pending_func=0x05 当"锁",
         *   防止 lora_try_send_downlink / 本函数干扰; 这里直接 continue 跳过,
         *   发送/重发/超时/阶段切换全由 _unbind_logic() + _unbind_on_ack() 处理。
         *   (2026-09-30 改造: 原来走的 5s/3 次耗尽强制删路径与用户最终两阶段口径
         *   冲突, 已作废; 详见 g_unbind 上方注释) */
        if (s->pending_func == LORA_FUNC_UNBIND) continue;

        /* ==========================================================================
         * ★★2026-10-08: 声光(0x03)走【业务级】的 15 秒 / 3 条逻辑, 不走下面的帧级重传★★
         * --------------------------------------------------------------------------
         * 为什么必须分开:
         *   · 帧级重传(下面那套)语义是"发出去的帧没被解出来 ⇒ 重发同一串字节";
         *   · 声光的 0x80 只表示"收到了", 真正的结论在 0x82/0x83 ——
         *     所以"没结果"要重发的是【整条业务动作】, 而 0x80 到了也不能算成功。
         *   混在一起会出现: 设备回了 0x80 但 10 秒后才回 0x82, 中间被帧级重传
         *   又发了一遍 ⇒ 平台收到两条 event.action、设备执行两次。
         * ======================================================================== */
        if (s->pending_func == (uint8_t)LORA_FUNC_SET_ALARM)
        {
            if (!s->alarm_pending) continue;                /* 不在等结果: 还没发/已收尾 */
            if ((now - s->alarm_tick) < LORA_ALARM_TIMEOUT_MS) continue;

            log_info("[GEAR] !! %02X%02X%02X 等了 %us 没收到 0x82/0x83 -> 本条算失败"
                     "(已发 %u/%u)\r\n",
                     s->dev_uid[0], s->dev_uid[1], s->dev_uid[2],
                     (unsigned)(LORA_ALARM_TIMEOUT_MS / 1000u),
                     (unsigned)s->alarm_attempts, (unsigned)LORA_ALARM_MAX_ATTEMPTS);
            lora_alarm_on_fail(s);
            continue;
        }

        if ((now - s->send_req_tick) < LORA_ACK_TIMEOUT_MS) continue;

        s->retry_cnt++;
        if (s->retry_cnt > LORA_MAX_RETRY)
        {
            /* 重传耗尽: 丢弃下发缓存, 回到 IDLE(不会再被 lora_try_send_alarm_idle 捡起来重发) */
            s->state     = SLAVE_STATE_IDLE;
            s->retry_cnt = 0U;
            data_cache_down_del_by_uid(s->dev_uid);

            log_info("[GW] !! %02X%02X%02X 连发 %u 次没等到 ACK, 丢弃这条下发指令"
                     "(要重发请从云端再下发)\r\n",
                     s->dev_uid[0], s->dev_uid[1], s->dev_uid[2],
                     (unsigned)(LORA_MAX_RETRY + 1U));
        }
        else
        {
            lora_tx_downlink(i);
            s->send_req_tick = now;
        }
    }
}


/**
 * \brief 检查是否有任意从机正在等待 ACK (无线电忙)
 *        有从机在 WAIT_ACK 时不能抢发, 否则 TX 会打断接收 ACK
 */
static bool lora_any_slave_waiting_ack(void)
{
    for (uint8_t i = 0U; i < g_slave_count; i++)
    {
        if (g_slave_list[i].state == SLAVE_STATE_WAIT_ACK)
            return true;
    }
    return false;
}

/**
 * \brief 扫描所有 IDLE 的声光报警器从机, 尝试发送缓存中的待下发指令
 *        声光报警器常驻 RX, 可以随时主动发; 液位传感器休眠, 只能等上报搭车
 *        无线电忙(有从机在等ACK)时跳过, 避免打断接收 ACK
 *
 * \note 这里能捡起来的只可能是【云端刚下发、还没发出去过】的那一条:
 *       一旦发出去进了 WAIT_ACK, 要么收到 ACK 清缓存, 要么重传耗尽被
 *       lora_downlink_logic() 删缓存 —— 两种结局都不会让它再回到这里。
 *       (老版本耗尽后不删缓存, 于是这条函数每秒给它重发一轮, 永不停歇。)
 */
void lora_try_send_alarm_idle(void)
{
    if (lora_any_slave_waiting_ack())
        return;

    for (uint8_t i = 0U; i < g_slave_count; i++)
    {
        if (g_slave_list[i].state == SLAVE_STATE_IDLE &&
            g_slave_list[i].dev_type == SLAVE_TYPE_ALARM_OUT)
        {
            lora_try_send_downlink(i);
            /* 发了一台就停, 该从机进入 WAIT_ACK, 下次周期再发下一台 */
            break;
        }
    }
}

/* ==================== 收到请求帧后的"分两步错开"应答时序 ====================
 *
 * 【为什么不能收到就立刻回】
 *   对端刚把那一包发完, 它的射频正在从 TX 切回 RX(还要重新上电接收链路)。
 *   网关这时立刻回 ACK / 下发指令, 前导码正好压在对端接收链路还没稳定的那段,
 *   对端解不出来 —— 现象就是"网关明明发了, 从机却没反应"。
 *   所以把它拆成两步, 每步之间留一段空档:
 *
 *     收到液位上报(0x02) / 其他请求帧
 *          └─ 等 GW_REPLY_ACK_DELAY_MS(3s)      → 发 0x80 ACK
 *               └─ 等 GW_REPLY_CMD_DELAY_MS(3s) → 从下发缓存取这条指令发出, 进 WAIT_ACK
 *
 * 【为什么用时间戳而不是 delay_ms()】
 *   delay_ms(3000) 是阻塞的, 两步加起来会把整个主循环按死 6s ——
 *   这 6s 里 lora_proto_poll() 不跑(别的从机的帧只能靠单缓冲暂存, 会被覆盖),
 *   4G AT 轮询也停摆(而且 g4_at_poll 自己最长还要阻塞 1s)。
 *   用"状态 + 时间戳"就完全没有这个问题: 主循环照常转, 到点了才动手。
 *
 * 全局只有一份待应答记录: 同一时刻先来后到, 后来的会覆盖前面的。
 * 从机不会高频上报, 台架实测够用; 真要多台并发再把这里改成队列。
 */
#define GW_REPLY_ACK_DELAY_MS   3000U   /* 收到请求帧后, 隔多久回 ACK */
#define GW_REPLY_CMD_DELAY_MS   3000U   /* 回完 ACK 后, 隔多久下发指令 */

typedef enum
{
    GW_REPLY_IDLE = 0,
    GW_REPLY_WAIT_ACK_TX,   /* 已收到请求帧, 等 GW_REPLY_ACK_DELAY_MS 再回 ACK */
    GW_REPLY_WAIT_CMD_TX,   /* ACK 已发, 等 GW_REPLY_CMD_DELAY_MS 再下发指令 */
} gw_reply_state_t;

static gw_reply_state_t g_reply_state = GW_REPLY_IDLE;
static uint32_t         g_reply_tick  = 0U;
static uint8_t          g_reply_dev[CFG_UID_BYTES];
_Static_assert(sizeof(g_reply_dev) == CFG_UID_BYTES,
               "g_reply_dev 必须 13 字节(2026-10-18 踩过坑)");
/* ★★2026-10-18 修复(严重): 这里原来是 `[3]`★★
 *   lora_reply_schedule() 里是 `memcpy(g_reply_dev, dev_uid, CFG_UID_BYTES)`,
 *   UID 扩到 13 字节后, 往 3 字节数组里写 13 字节 ⇒
 *     ① 越界写坏紧跟其后的全局变量;
 *     ② 3 秒后 lora_reply_logic() 用 g_reply_dev 发 0x80 ACK 时,
 *        只有前 3 字节是对的, 后 10 字节是垃圾 ⇒ 地址根本不存在。
 *   现场表现(2026-10-18 实测): 设备上报 0x02 后, 网关的 ACK 发错地址,
 *   设备以为没收到, 连续重发 5 条 0x80(约 2 秒一条), 最后放弃。
 *   ⇒ 所有"存 13 字节 UID 的静态数组"都必须用 CFG_UID_BYTES 定尺寸。 */

static uint8_t          g_reply_func  = 0U;
static uint8_t          g_reply_idx   = 0xFFU;

/* on_frame 里调用: 只登记"要回谁", 真正的发送交给 lora_reply_logic() 分两步做 */
void lora_reply_schedule(const uint8_t dev_uid[CFG_UID_BYTES], uint8_t func, uint8_t idx)
{
    if (g_reply_state != GW_REPLY_IDLE)
    {
        log_info("[GW] !! 上一条待应答还没走完(dev=%02X%02X%02X), 被 %02X%02X%02X 覆盖\r\n",
                 g_reply_dev[0], g_reply_dev[1], g_reply_dev[2],
                 dev_uid[0], dev_uid[1], dev_uid[2]);
    }

    memcpy(g_reply_dev, dev_uid, CFG_UID_BYTES);
    g_reply_func  = func;
    g_reply_idx   = idx;
    g_reply_tick  = get_tick_ms();
    g_reply_state = GW_REPLY_WAIT_ACK_TX;
}

/* lora_business_poll() 每轮调用一次(1ms 精度) */
void lora_reply_logic(void)
{
    uint32_t waited;

    if (g_reply_state == GW_REPLY_IDLE)
    {
        return;
    }

    /* ---- 第一步: 等够 GW_REPLY_ACK_DELAY_MS 后回 0x80 ---- */
    if (g_reply_state == GW_REPLY_WAIT_ACK_TX)
    {
        waited = (uint32_t)(get_tick_ms() - g_reply_tick);
        if (waited < GW_REPLY_ACK_DELAY_MS)
        {
            return;
        }

        log_info("[GW] 收到 %02X%02X%02X 的帧后等 %lums, 现在回 0x80 ACK\r\n",
                 g_reply_dev[0], g_reply_dev[1], g_reply_dev[2],
                 (unsigned long)waited);
        (void)lora_business_send_ack(g_gw_uid, g_reply_dev, g_reply_func, true, 0);

        g_reply_tick  = get_tick_ms();
        g_reply_state = GW_REPLY_WAIT_CMD_TX;
        return;
    }

    /* ---- 第二步: 再等 GW_REPLY_CMD_DELAY_MS 后下发指令 ---- */
    waited = (uint32_t)(get_tick_ms() - g_reply_tick);
    if (waited < GW_REPLY_CMD_DELAY_MS)
    {
        return;
    }

    g_reply_state = GW_REPLY_IDLE;

    if (g_reply_idx == 0xFFU)
    {
        log_info("[GW] 该从机没登记进从机表, 无从下发\r\n");
        return;     /* 从机表没登记成功, 无从下发 */
    }

    log_info("[GW] 回完 ACK 再等 %lums, 现在尝试下发 %02X%02X%02X 的待发指令\r\n",
             (unsigned long)waited,
             g_reply_dev[0], g_reply_dev[1], g_reply_dev[2]);

    /* 该从机 IDLE 且有缓存指令才会真的发出去。
     * 【这里必须把结果打出来】: 缓存为空时它是"什么都不做"的,
     * 不打日志的话现象就是"从机收到 ACK 之后就没下文了、直接睡过去",
     * 完全看不出是"网关没指令可发"还是"发了但从机没收到"。 */
    {
        int r = lora_try_send_downlink(g_reply_idx);

        switch (r)
        {
        case DOWNLINK_SENT:
            log_info("[GW] 下发指令已发出, 等从机 ACK\r\n");
            break;
        case DOWNLINK_NO_CACHE:
            log_info("[GW] !! 下发缓存里没有 %02X%02X%02X 的指令 -> 这一步什么都没发\r\n",
                     g_reply_dev[0], g_reply_dev[1], g_reply_dev[2]);
            break;
        case DOWNLINK_BUSY:
            log_info("[GW] 该从机还在等上一条指令的 ACK, 这轮跳过\r\n");
            break;
        case DOWNLINK_TX_FAIL:
            log_info("[GW] !! 下发发送失败(没等到 TX_DONE), 查射频/供电\r\n");
            break;
        default:
            log_info("[GW] !! 下发缓存项格式不对(连 func 都没有)\r\n");
            break;
        }
    }
}


/* ========== MQTT 下行处理: 云端直接下发【协议帧】 ==========
 *
 * 【2026-09 改版 —— JSON 全部删除】
 *   以前云端下发的是嵌套 JSON({"target_dev":"210001","cmd":"1","data":{...}}),
 *   为了在 MCU 上解析它写了一整套字符串查找 + 一堆字段名常量, 光字段名字符串
 *   就占了可观的 Flash, 而且云端字段一旦拼错/改名, 现象只是"静默什么都不做"。
 *   现在改成: 云端把【一个和 LoRa 空口一模一样的帧】直接发下来(原始二进制),
 *   网关做 lora_proto_parse_raw()(帧头/版本/长度/CRC 四重校验) → 按功能码处理。
 *   少一整层解析, 也少一整类"解析失败却不报错"的坑。
 *
 *   (2026-09 早期版本用的是 ASCII hex 文本承载, 现在云端已改成原始二进制;
 *    代码里 hex 分支保留做兼容 —— 只有 payload 不是 AA55 开头时才走它。)
 *
 *   线上收到的 payload 格式就是这 20 个字节(CRC 已按协议重算):
 *     AA 55 01 11 00 01 21 00 01 01 07 05 00 0A 00 00 DC 05 98 E0
 *   对应的帧:
 *     AA 55 | 01 | 11 00 01 | 21 00 01 | 01 | 07 | 05 00 0A 00 00 DC 05 | 98 E0
 *     └头┘  ver  └─ 网关 ─┘  └─ 设备 ─┘  fc  dlen  └──── data 7B ────┘  └CRC┘
 *   ⚠️【2026-09 更正】原来这里写的是 ...DC 05 38 1B, 那个 CRC 是错的 ——
 *     现场抓包(见 _rev_GOOD_2026-09-22\v4...\说明.txt 第五节, 当时
 *     MAIN_LORA_ENABLE=0 只跑 4G, 所以没人校验过它)里云端发下来的就是
 *     38 1B。正确值是 98 E0; 照抄 38 1B 会让设备端 CRC 不过、整帧被丢弃,
 *     现象就是"参数下发了但设备一直没同步时间(上报帧里月日时分全 00)"。
 *     算法: crc16_modbus(版本字节..data 末尾), 结果低字节在前。
 *
 * 转发规则(《液位监测系统通讯协议说明》第五节):
 *   1) 0x01(设置液位): 云端的 data 只有前 7 字节, 【没有】那 6 字节校时;
 *      网关转发前必须补上 年 月 日 周 时 分 —— 取自 AT+CCLK?, 星期固定填周五。
 *   2) 0x03(设置声光): 一个字节都不改, 直接打包转发。
 *   3) 0x02 是【上行】功能码, 云端不会下发; 0x80/0x81 若收到只当"云端应答"记一笔。
 */

/* 【单位: 秒 —— ★2026-10-05 用户口径改版★】
 *   平台侧配置(sen.set 的 reportPeriodSeconds)本来就是【秒】, 所以网关
 *   【不做任何换算】, 原样填进 0x01 帧的对应字段; 设备端按秒解析
 *   (设备侧由用户自己对接)。
 *
 *   ⚠ 这条与 2026-09 的"分钟"版本【不兼容】, 网关与设备必须同一次改版:
 *     旧版: 协议字段=分钟, 网关原样转发, 设备端自己 ×60 换成内部秒;
 *     现在: 协议字段=秒,   网关原样转发, 设备端按秒直接用 —— 谁都不换算。
 *     任何一侧漏改 => 时间差 60 倍(平台 5 分钟 -> 设备 5 小时)。
 *
 *   16 位字段上限 65535 秒 ≈ 18.2 小时; 平台侧 reportPeriodSeconds 的配置范围
 *   是 5~3600 秒(config_store_validate), 落在里面。 */
#define LORA_SENSOR_PERIOD_MIN_S     1U      /* 间隔最小 1 秒 */

/* 液位传感器是【休眠】设备: 默认不立即发, 等它上报时搭车发。
 * 改成 1 就在收到下行后立刻发(只有它正好在接收窗口内才有意义)。 */
#ifndef LORA_SENSOR_SEND_IMMEDIATE
#define LORA_SENSOR_SEND_IMMEDIATE   0
#endif

/* 下行失败统一出口: 云端下发是强约束指令, 丢弃必须留下原因, 否则只能靠猜 */
static void _downlink_drop(uint8_t func, const char *why)
{
    log_info("[CLOUD->LoRa] !! func=0x%02X %s, 丢弃\r\n", (unsigned)func, why);
}

/* ---------- 0x01 参数的公共落地点(两条下行共用) ----------
 *
 * 【为什么要抽出来】0x01 这条参数有【两个入口】:
 *     ① 平台直接下发 0x01 原始帧(云端自己拼帧);
 *     ② 平台下发 JSON sen.set -> cloud_proto 调 lora_business_set_sensor_params()。
 *   两个入口的 范围检查 / 补校时 / 投递 / 日志 必须完全一样 ——
 *   否则就会出现"用原始帧能下发、用 JSON 下发不了"这种最难查的不一致。
 *   所以两条路都走本函数, 只有日志里的来源标签(src)不同。
 *
 * 参数单位全部是【秒】(见上面的改版说明)。
 * 返回 0 成功; -1 载荷过长; -2 地址不在从机表; -3 间隔越界。
 */
static int _sensor_params_commit(const uint8_t uid[CFG_UID_BYTES], uint16_t detect_s,
                                 uint16_t report_s, bool same, uint16_t thresh_mm,
                                 const char *src)
{
    uint8_t    payload[13];
    g4_clock_t clk;
    int        ret;

    if ((detect_s < LORA_SENSOR_PERIOD_MIN_S) || (report_s < LORA_SENSOR_PERIOD_MIN_S))
    {
        log_info("[CLOUD->LoRa] !! 0x01(%s) 间隔越界(检测/上报都必须 >= %us): "
                 "detect=%us report=%us\r\n",
                 src, (unsigned)LORA_SENSOR_PERIOD_MIN_S,
                 (unsigned)detect_s, (unsigned)report_s);
        return -3;
    }

    /* 13 字节载荷, 16 位字段一律【低字节在前】, 间隔字段原样填秒值(不换算) */
    payload[0]  = (uint8_t)(detect_s & 0xFFU);
    payload[1]  = (uint8_t)((detect_s >> 8) & 0xFFU);
    payload[2]  = (uint8_t)(report_s & 0xFFU);
    payload[3]  = (uint8_t)((report_s >> 8) & 0xFFU);
    payload[4]  = same ? 1U : 0U;
    payload[5]  = (uint8_t)(thresh_mm & 0xFFU);
    payload[6]  = (uint8_t)((thresh_mm >> 8) & 0xFFU);

    /* 校时(6 字节): 先按"现在"填一份。真正发出去之前 lora_try_send_downlink() 还会
     * 【再取一次刷新】——液位是休眠设备, 参数可能在它上报前很久就排好了队,
     * 用排队时刻的时间戳会把设备 RTC 校偏(最坏偏一个上报周期)。
     * 现在取不到【不丢弃】: 填 0 并告警, 到了真正发送那一刻还有机会取到
     * (旧版这里是直接丢弃, 那会让 sen.set 在 4G 刚断开时静默失效)。 */
    if (g4_at_get_clock(&clk))
    {
        payload[7]  = clk.year;  payload[8]  = clk.month; payload[9]  = clk.day;
        payload[10] = clk.week;  payload[11] = clk.hour;  payload[12] = clk.minute;

        log_info("[CLOUD->LoRa] 0x01(%s) 液位 %02X%02X%02X detect=%us report=%us "
                 "same=%u thresh=%umm time=20%02u-%02u-%02u w%u %02u:%02u\r\n",
                 src, uid[0], uid[1], uid[2],
                 (unsigned)detect_s, (unsigned)report_s, (unsigned)payload[4],
                 (unsigned)thresh_mm,
                 (unsigned)clk.year, (unsigned)clk.month, (unsigned)clk.day,
                 (unsigned)clk.week, (unsigned)clk.hour, (unsigned)clk.minute);
    }
    else
    {
        (void)memset(&payload[7], 0, 6U);
        log_info("[CLOUD->LoRa] !! 0x01(%s) 现在取不到 4G 时间(AT+CCLK?), 校时先填 0"
                 "(真正发出前会再取一次)\r\n", src);
    }

    ret = lora_business_send_req(uid, SLAVE_TYPE_SENSOR,
                                 LORA_FUNC_SET_SENSOR_PARAM,
                                 payload, (uint8_t)sizeof(payload));
    if (ret != 0)
    {
        /* 把真实原因分开报 —— 地址没登记和缓存满完全是两个方向的排查 */
        log_info("[CLOUD->LoRa] !! 0x01(%s) 投递失败: %s\r\n", src,
                 (ret == -2) ? "该地址不在固定从机表里(液位必须先在表里登记)" :
                 (ret == -1) ? "载荷过长(>239字节)" : "下发缓存满/写入失败");
        return ret;
    }
    return 0;
}

/* ---------- 0x01 入口②: 平台 JSON(sen.set) ----------
 * 见 lora_business.h 的说明。单位【秒】, 不换算。 */
bool lora_business_set_sensor_params(const char *uid_hex,
                                     uint16_t detect_period_s,
                                     uint16_t report_period_s,
                                     bool     report_same_level,
                                     uint16_t alarm_thresh_mm)
{
    uint8_t uid[CFG_UID_BYTES];

    if (uid_hex == NULL) { return false; }
    if (!cfg_uid_to_bytes(uid_hex, uid))
    {
        log_info("[CLOUD->LoRa] !! sen.set: uid \"%s\" 不是合法的 6 位 hex 串\r\n", uid_hex);
        return false;
    }
    return (_sensor_params_commit(uid, detect_period_s, report_period_s,
                                  report_same_level, alarm_thresh_mm, "sen.set") == 0);
}

/* ---------- 0x01 入口①: 云端直接下发的原始帧 ----------
 *
 * 云端 data(7 字节, 单位【秒】, 见上面的改版说明):
 *   off0-1 检测间隔(小端) off2-3 上报间隔(小端) off4 相同液位是否上报
 *   off5-6 液位告警阈值 mm(小端)
 *
 * 下发给设备的 13 字节(app_cmd.h):
 *   off0-1 检测间隔(小端, 秒) off2-3 上报间隔(小端, 秒) off4 相同上报表
 *   off5-6 阈值(小端) off7 年 off8 月 off9 日 off10 周 off11 时 off12 分
 *   ⚠️ 16 位字段必须【低字节在前】, 写成大端会被设备解成 512/256/62465 这种乱值。
 */
static void _downlink_set_sensor(const lora_frame_t *frame)
{
    uint32_t detect_s, report_s, thresh_mm;
    int      ret;

    if (frame->data_len < 7U)
    {
        _downlink_drop(frame->func, "载荷不足 7 字节");
        return;
    }

    detect_s  = ((uint32_t)frame->data[0] | ((uint32_t)frame->data[1] << 8));
    report_s  = ((uint32_t)frame->data[2] | ((uint32_t)frame->data[3] << 8));
    thresh_mm = ((uint32_t)frame->data[5] | ((uint32_t)frame->data[6] << 8));

    ret = _sensor_params_commit(frame->dev_uid, (uint16_t)detect_s, (uint16_t)report_s,
                                (frame->data[4] != 0U), (uint16_t)thresh_mm,
                                "云端原始帧");
    if (ret != 0)
    {
        return;     /* 具体原因 _sensor_params_commit() 已经打过了, 不重复刷日志 */
    }

#if LORA_SENSOR_SEND_IMMEDIATE
    if (!lora_any_slave_waiting_ack())
    {
        uint8_t idx = lora_slave_find(frame->dev_uid);
        if (idx != 0xFFU)
        {
            (void)lora_try_send_downlink(idx);
        }
    }
    log_info("[CLOUD->LoRa] 已立即下发(液位立即模式)\r\n");
#else
    log_info("[CLOUD->LoRa] 已缓存, 等液位下次上报时随应答下发\r\n");
#endif
}

/* ---------- 0x03 云端设置声光报警器: 原样转发 ----------
 *
 * 云端 data 顺序即线上字节顺序, 与设备端完全一致 —— 网关【不解析、不加工】:
 *   power_ctrl 1B | light_ctrl 1B | volume 1B | led_mode 1B | led_brightness 1B |
 *   led_scroll_speed 1B | led_text_len 1B | led_text(len 字节 GB2312) |
 *   tts_text_len 1B | tts_text(len 字节 GB2312)
 *
 * 报警器常驻 RX, 缓存完立刻发; 无线忙就留在缓存里, 由 poll 里的
 * lora_try_send_alarm_idle() 捡起来发。
 */
static void _downlink_set_alarm(const lora_frame_t *frame, uint8_t idx)
{
    int ret;

    if (frame->data_len == 0U)
    {
        _downlink_drop(frame->func, "载荷为空");
        return;
    }

    /* 【不在固定从机表里 -> 直接转发】(现场规则: 地址首位是 3 就是声光报警器)
     *   报警器【常驻接收】, 不必等它上报, 所以直接上发即可 ——
     *   而且数据【一个字节都不改】: 现场要求"声光报警的数据我们不处理, 只判断"。
     *   (在表里的走下面那条"缓存 + 立即发 + ACK 重传"的路径, 多一层保障。) */
    if (idx == 0xFFU)
    {
        log_info("[CLOUD->LoRa] 0x03 声光 %02X%02X%02X 不在固定表里 -> 【直接转发】"
                 "%u 字节(数据不改)\r\n",
                 frame->dev_uid[0], frame->dev_uid[1], frame->dev_uid[2],
                 (unsigned)frame->data_len);

        /* 这帧就是从云端解出来的原帧; 组帧是幂等的(字段不变 -> CRC 不变),
         * 所以重发一次等于把云端那串字节原样送出去。
         * 去 const 是安全的: 实参是 lora_mqtt_recv_cb 里的非 const 局部变量。 */
        if (lora_proto_send_frame((lora_frame_t *)frame))
        {
            log_info("[CLOUD->LoRa] 直接转发完成\r\n");
        }
        else
        {
            log_info("[CLOUD->LoRa] !! 直接转发失败(射频没发出去) -> 查 LoRa 供电/天线\r\n");
        }
        return;
    }

    /* 下发缓存每台设备只有 64 字节(其中 1 字节给 func), 所以 data 最多 63 字节。
     * 声光那条带 LED(12B) + TTS(32B) 的实测载荷是 52 字节, 够用。 */
    ret = lora_business_send_req(frame->dev_uid, SLAVE_TYPE_ALARM_OUT,
                                 LORA_FUNC_SET_ALARM, frame->data, frame->data_len);
    if (ret != 0)
    {
        _downlink_drop(frame->func, "投递失败(载荷>63字节或缓存满)");
        return;
    }

    log_info("[CLOUD->LoRa] 0x03 声光 %02X%02X%02X %u 字节已缓存\r\n",
             frame->dev_uid[0], frame->dev_uid[1], frame->dev_uid[2],
             (unsigned)frame->data_len);

    if (!lora_any_slave_waiting_ack())
    {
        (void)lora_try_send_downlink(idx);
    }
}

/* ---------- MQTT 下行回调(由 lora_business_init 注册给 g4_urc) ----------
 *
 * 全模块【唯一】的下行入口: 4G URC → g4_io 收包泵 → g4_urc 队列 → g4_urc_poll → 这里。
 * 没有对外函数, 也没有"串口手敲当云端下发"的台架后门。
 *
 * 【2026-09 台架联调改版】云端现在直接下发【原始二进制帧】(AA 55 ... CRC),
 * 不再是 ASCII hex 文本 —— 所以这里先按二进制收, 同时保留 hex 兼容分支。
 *
 * 【LORA_DOWNLINK_RAW_FORWARD = "粗略直转"开关】
 *   1(台架联调用) = 校验通过后【原样把这一帧发到空口】: 不查白名单、不解释
 *       func、不取 4G 时钟、不等从机上报搭车。目的只有一个: 先确认
 *       "云端 -> 网关 -> 空口" 这条路通不通, 别被业务解释(校时/参数范围/等待
 *       上报)干扰判断。发出去的字节和云端发下来的完全一样(parse 后重新组帧,
 *       字段一致 -> CRC 一致)。
 *   0 = 走原来的业务解释: 0x01 设置液位传感器参数(需要 AT+CCLK? 校时),
 *       0x03 设置声光报警器(常驻 RX, 立即下发)。
 *
 * ★2026-09-22 默认从 1 改回 0★ —— 台架联调结束, 回到正式协议流程:
 *   云端下发 0x01(7 字节, 不带时间) -> 网关补齐 6 字节校时 -> 等液位下次上报
 *   时随应答下发。详见下面 LORA_DOWNLINK_PASSTHROUGH 的说明。
 */

/* 解出来的二进制帧放静态区: 一次 253 字节, 不占本来就紧张的栈 */
static uint8_t s_down_bin[LORA_FRAME_RAW_MAX];

#ifndef LORA_DOWNLINK_RAW_FORWARD
#define LORA_DOWNLINK_RAW_FORWARD   0
#endif

/* ==================== 【最简透传开关 LORA_DOWNLINK_PASSTHROUGH】 ====================
 *
 * 1 = 云端下发什么字节, 空口就原样发什么字节:
 *     · 不看帧头 AA55、不校验版本/长度/CRC、不解释 func、不查从机白名单;
 *     · 不补 0x01 那 6 字节校时, 不等从机上报搭车, 收到就发;
 *     · 从机表 / 上行缓存 / ACK 重传那套业务逻辑照旧保留(上行和"收到从机帧后
 *       回 0x80"都不受影响), 只是【下行不再走业务解释】。
 *   目的: 先把"云端 → 网关 → 空口"这条路单独走通, 别被业务解释(校时/参数范围/
 *   等待上报)干扰判断。对端(声光报警器 / 任意 LoRa 接收机)收到什么就是什么。
 *
 * 0 = 回到带帧校验/业务解释的路径(下面的 #else 分支): 下行必须是以 AA55 开头的
 *     合法协议帧, 再由 LORA_DOWNLINK_RAW_FORWARD 决定"校验后直转"还是业务解释。
 *
 * ★2026-09-22 曾从 1 改回 0★ —— 当时是"透传属台架临时手段, 回正式协议"。
 * ★2026-10-08 又改回 1★ —— 甲方平台要下发自定义载荷(非 LoRa 帧), 见上面的说明。
 *   下面这段"正式下行流程"仍然有效, 只是**现在默认不走它**; 把本宏改回 0 即可恢复。
 *
 *   【正式下行流程(宏=0 时的行为, 现场确认)】
 *     1) 云端下发一帧协议帧到 ★rt/gw/{gwId}/cmd★
 *          (2026-10-08 前是 mod/pc_down —— 该主题已随甲方要求整体删除)
 *          AA 55 | 01 | 网关3B | 从机3B | func | dlen | data | CRC16(2B)
 *        func: 0x01 设置液位传感器 / 0x03 设置声光报警器 /
 *              0x02 上报(上行) / 0x80 应答OK / 0x81 应答错误
 *        0x01 的 data 固定 7 字节, 【不带时间】:
 *          [0-1] 检测参数间隔 [2-3] 上报时间间隔 (单位【秒】, 小端)
 *          [4]   相同液位是否上报  [5-6] 液位告警阈值 mm (小端)
 *     2) 网关校验(帧头/版本/长度/CRC)、查从机白名单、把这条指令放进【下发缓存】;
 *     3) 网关【等液位下次上报】—— 液位是休眠设备, 只在醒来那一小段窗口里能收;
 *     4) 收到液位上报(0x02)后: 等 3s 回 0x80 ACK -> 再等 3s 把缓存的指令发下去,
 *        正好落在液位的接收窗口内(见 lora_reply_logic / GW_REPLY_*_DELAY_MS);
 *     5) 0x01 实际发出去时, 由网关补上 6 字节校时:
 *          [7]年 [8]月 [9]日 [10]周 [11]时 [12]分  —— 共 13 字节
 *        时间【在要发的这一刻】向 4G 模块取(AT+CCLK?), 应答里没有星期, 固定周五。
 *
 * 【透传的代价(宏=1 时)】lora_proto_send_raw() 会阻塞 LORA_TX_SETTLE_MS(500ms) +
 *   等 TX_DONE(≤1s) —— 最长约 1.5 秒卡在主循环里, 期间 AT 轮询/日志都停下。
 *   宏=0 时走组帧路径(同样阻塞), 但不再是"收到就发", 而是搭在液位上报窗口里,
 *   不影响 4G 下行的投递节奏。这是切回 0 的主要理由, 现在为了自定义载荷接受了它。
 *
 * ★2026-10-08 用户决定: 默认改回 1(最简透传)★
 *   用户口径: "我发不是 lora 的 hex, lora 直接转发" ——
 *   即平台往 rt/gw/{gwId}/cmd 发【任意字节】(不是 AA55 协议帧、也不是合法 hex),
 *   网关【不做任何帧校验/业务解释】, 原样推到 LoRa 空口。
 *
 *   这是 2026-09-22 之前长期使用的行为(那次是为了回正式协议才改成 0);
 *   现在因为甲方平台要下发自定义载荷, 又切回透传。
 *
 *   ⚠️ 透传仍然【只影响下行】: 上行(0x02 解析 → last_value → rt/gw/{gwId}/telemetry)、
 *      从机表、绑定窗口、0x80/0x82/0x83 的内部处理【全部照旧】, 一点没变。
 *   ⚠️ 前置换算(order 很重要): JSON 仍然走云协议! 见 lora_mqtt_recv_cb 开头 ——
 *      `main_cloud_downlink_try()` 先嗅探, 像 JSON(`{`...`}` 且后面有 `}`/`"`)
 *      就交给 cloud_proto, 【根本到不了这里】。所以透传只吃"非 JSON"的下行。
 *   ⚠️ 单包上限 LORA_FRAME_DATA_MAX(240 字节), 超了丢并打日志。
 *   ⚠️ 这条路【不回平台任何应答】(本文件没有任何 g4_at_mqtt_publish 调用),
 *      从机的 0x80/0x82/0x83 也只在网关内部处理, 不转发给平台。
 */
#ifndef LORA_DOWNLINK_PASSTHROUGH
#define LORA_DOWNLINK_PASSTHROUGH   1
#endif

/* 下行前置过滤器(见 lora_business.h 的说明)。
 * 不注册时行为与本改动前完全一致。 */
static lora_dl_prefilter_t g_dl_prefilter;

void lora_business_set_dl_prefilter(lora_dl_prefilter_t fn)
{
    g_dl_prefilter = fn;
}

void lora_mqtt_recv_cb(const char *topic, const uint8_t *data, uint16_t len)
{
    /* 先让订阅者(云协议层)判断这条归不归它管。
     * 归它管就到此为止, 不再走下面的 LoRa 透传 —— 这是过渡期两种协议共存的关键。 */
    if ((g_dl_prefilter != NULL) && g_dl_prefilter(topic, data, len))
    {
        return;
    }

    (void)topic;        /* 下面的分支不一定用到 topic */
#if LORA_DOWNLINK_PASSTHROUGH
    /* ---------------- 最简透传: 云端发什么, 空口发什么 ---------------- */
    uint16_t i;

    if ((data == NULL) || (len == 0U))
    {
        log_info("[CLOUD->LoRa] !! 下行空载荷, 不发\r\n");
        return;
    }

    log_info("[CLOUD->LoRa] 透传 %u 字节到空口:", (unsigned)len);
    for (i = 0U; (i < len) && (i < 64U); i++)
    {
        log_info(" %02X", (unsigned)data[i]);
    }
    if (len > 64U)
    {
        log_info(" ...(只打印前 64 字节)");
    }
    log_info("\r\n");

    /* 空口单包上限: 超过就只能丢(留着也比发半个包强)。
     * LORA_FRAME_DATA_MAX 见 lora_proto.h。 */
    if (len > (uint16_t)LORA_FRAME_DATA_MAX)
    {
        log_info("[CLOUD->LoRa] !! 下行 %u 字节超过空口单包上限 %u, 丢弃\r\n",
                 (unsigned)len, (unsigned)LORA_FRAME_DATA_MAX);
        return;
    }

    if (lora_proto_send_raw(data, (uint8_t)len))
    {
        log_info("[CLOUD->LoRa] 透传完成\r\n");
    }
    else
    {
        log_info("[CLOUD->LoRa] !! 透传失败(射频没发出去) -> 查 LoRa 供电/EN/BUSY/天线\r\n");
    }

    /* 下面这些是"带帧校验的业务路径"才用到的(#else 分支), 透传模式下不引用它们,
     * 显式引一下, 免得编译器报一堆 "declared but never referenced"。 */
    (void)s_down_bin;
    (void)lora_hex_str2bin;
    (void)_downlink_drop;
    (void)_downlink_set_sensor;
    (void)_downlink_set_alarm;
    return;
#else
    lora_frame_t frame;
    int          n;
    uint8_t      idx;

    /* ---- 1. 取出原始帧字节 ---- */
    if ((data == NULL) || (len < LORA_FRAME_OVERHEAD))
    {
        log_info("[MQTT->LoRa] !! 下行过短(%u 字节, 至少要 %u)\r\n",
                 (unsigned)len, (unsigned)LORA_FRAME_OVERHEAD);
        return;
    }

    if ((data[0] == LORA_FRAME_HEAD0) && (data[1] == LORA_FRAME_HEAD1))
    {
        /* 原始二进制帧(当前云端的下发方式) */
        if (len > (uint16_t)sizeof(s_down_bin))
        {
            log_info("[MQTT->LoRa] !! 下行过长(%u 字节, 缓冲 %u)\r\n",
                     (unsigned)len, (unsigned)sizeof(s_down_bin));
            return;
        }
        memcpy(s_down_bin, data, len);
        n = (int)len;
        log_info("[MQTT DOWN] 原始二进制帧 %u 字节 (帧头 AA55)\r\n", (unsigned)len);
    }
    else
    {
        /* 兼容分支: 旧格式 ASCII hex 文本 */
        log_info("[MQTT DOWN] %u bytes(按 hex 文本解析): %.*s\r\n",
                 (unsigned)len, (int)((len < 200U) ? len : 200U), (const char *)data);
        n = lora_hex_str2bin((const char *)data, (int)len, s_down_bin, (int)sizeof(s_down_bin));
        if (n < 0)
        {
            log_info("[MQTT->LoRa] !! 下行既不是 AA55 开头的二进制帧, 也不是合法 hex\r\n");
            return;
        }
    }

    /* 帧头 AA55 + 版本 + 长度自洽 + CRC16, 四项全过才认 (n = 上面解出的字节数) */
    if (!lora_proto_parse_raw(s_down_bin, (uint8_t)n, &frame))
    {
        log_info("[MQTT->LoRa] !! 帧校验失败(帧头/版本/长度/CRC), 共 %d 字节\r\n", n);
        return;
    }

    /* 【设备类型按"地址首位"判断, 不再只认固定从机表】(现场规则)
     *   1xxxxx 网关 / 2xxxxx 液位传感器 / 3xxxxx 声光报警器
     * 固定表还是要查 —— 在表里就能享受"在线跟踪 + ACK 重传"那套有状态的处理;
     * 不在表里【也不代表这帧没用】: 只要地址首位是认得的类型, 后面按类型分发,
     * 声光报警器会走"直接转发"分支(见 _downlink_set_alarm)。
     * (老代码在这里直接 return 丢弃 —— 那样一台没登记的报警器就永远收不到指令。) */
    idx = lora_slave_find(frame.dev_uid);
    if (idx == 0xFFU)
    {
        log_info("[GW] 从机 %02X%02X%02X 不在固定表里, 按【地址首位=%u】判断设备类型\r\n",
                 frame.dev_uid[0], frame.dev_uid[1], frame.dev_uid[2],
                 (unsigned)lora_dev_type_of_uid(frame.dev_uid));
    }

    /* 帧里的"网关设备地址"应该是本机。不是也照样转发(要不要拦是策略问题),
     * 但必须打出来 —— 否则云端把网关号填错会一直没人发现。 */
    if (memcmp(frame.gw_uid, g_gw_uid, CFG_UID_BYTES) != 0)
    {
        log_info("[MQTT->LoRa] !! 帧里网关地址 %02X%02X%02X 与本机 %02X%02X%02X 不一致\r\n",
                 frame.gw_uid[0], frame.gw_uid[1], frame.gw_uid[2],
                 g_gw_uid[0], g_gw_uid[1], g_gw_uid[2]);
    }

    log_info("[CLOUD->LoRa] 收到 %d 字节帧: dev=%02X%02X%02X func=0x%02X dlen=%u\r\n",
             n, frame.dev_uid[0], frame.dev_uid[1], frame.dev_uid[2],
             (unsigned)frame.func, (unsigned)frame.data_len);

#if LORA_DOWNLINK_RAW_FORWARD
    /* ---------------- 粗略直转(台架联调) ----------------
     * 校验通过(帧头/版本/长度/CRC)之后【原样把这一帧发到空口】, 不做业务解释:
     *   · 不取 4G 时钟 —— 0x01 本来要靠 AT+CCLK? 校时, 先跳过;
     *   · 不等从机上报搭车下发;
     *   · 不建 WAIT_ACK/重传状态机(收到 ACK 也当普通帧处理)。
     * 目的只有一个: 先确认 "云端 -> 网关 -> 空口" 这条路通不通。
     * 发出去的字节 = 云端发下来的字节(parse 后重新组帧, 字段一致 -> CRC 一致)。
     *
     * 【代价】lora_proto_send_frame() 会阻塞 LORA_TX_SETTLE_MS(500ms) + 等 TX_DONE,
     *        这几百毫秒主循环是停住的。它跑在 g4_urc_poll()(收包泵之外), 所以不会
     *        丢 4G 接收, 但下行连发时会把投递排到后面。台架联调可接受。
     * 想回到"带业务解释"的老路径: 把 LORA_DOWNLINK_RAW_FORWARD 改成 0。 */
    (void)idx;
    /* 老的业务解释路径仍在编译单元里(把 LORA_DOWNLINK_RAW_FORWARD 改成 0 即可切回),
     * 这里显式引用一下, 免得编译器报"定义未使用"。 */
    (void)_downlink_set_sensor;
    (void)_downlink_set_alarm;
    log_info("[CLOUD->LoRa] 直转 %d 字节原始帧到空口: dev=%02X%02X%02X func=0x%02X dlen=%u\r\n",
             n, frame.dev_uid[0], frame.dev_uid[1], frame.dev_uid[2],
             (unsigned)frame.func, (unsigned)frame.data_len);

    if (lora_proto_send_frame(&frame))
    {
        log_info("[CLOUD->LoRa] 直转完成\r\n");
    }
    else
    {
        log_info("[CLOUD->LoRa] !! 直转失败(组帧或射频没发出去)\r\n");
    }
    return;
#else
    switch (frame.func)
    {
    case LORA_FUNC_SET_SENSOR_PARAM:            /* 0x01 设置液位传感器参数 */
        /* 类型判断改看【地址首位】: 2xxxxx = 液位传感器 */
        if (lora_dev_type_of_uid(frame.dev_uid) != LORA_UID_TYPE_LEVEL)
        {
            _downlink_drop(frame.func, "设备地址首位不是 2 -> 不是液位传感器");
            break;
        }
        _downlink_set_sensor(&frame);
        break;

    case LORA_FUNC_SET_ALARM:                   /* 0x03 设置声光报警器 */
        /* 类型判断改看【地址首位】: 3xxxxx = 声光报警器
         * 声光的数据【网关不处理, 只判断】—— 认出类型后原样转发。 */
        if (lora_dev_type_of_uid(frame.dev_uid) != LORA_UID_TYPE_ALARM)
        {
            _downlink_drop(frame.func, "设备地址首位不是 3 -> 不是声光报警器");
            break;
        }
        _downlink_set_alarm(&frame, idx);
        break;

    case LORA_FUNC_RESP_OK:                     /* 0x80 云端对上行上报的应答 */
    case LORA_FUNC_RESP_ERR:                    /* 0x81 */
        log_info("[CLOUD->LoRa] 云端应答 0x%02X(%s), 无需处理\r\n",
                 (unsigned)frame.func,
                 (frame.func == LORA_FUNC_RESP_OK) ? "OK" : "ERR");
        break;

    default:
        _downlink_drop(frame.func, "功能码不支持(下行只认 0x01/0x03)");
        break;
    }
#endif /* LORA_DOWNLINK_RAW_FORWARD */
#endif /* LORA_DOWNLINK_PASSTHROUGH */
}


bool lora_business_send_ack(const uint8_t gw_uid[CFG_UID_BYTES], const uint8_t dev_uid[CFG_UID_BYTES],
                            uint8_t func, bool ok, uint8_t err_code)
{
    lora_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    if (gw_uid)  memcpy(frame.gw_uid,  gw_uid, CFG_UID_BYTES);
    if (dev_uid) memcpy(frame.dev_uid, dev_uid, CFG_UID_BYTES);
    frame.func = ok ? LORA_FUNC_RESP_OK : LORA_FUNC_RESP_ERR;

    /* 【0x80/0x81 是纯应答码, 没有数据域, data_len 必须为 0】
     * 正确帧 13 字节: AA 55 01 <gw3> <dev3> 80 00 <CRC>
     *
     * 这里原来塞了 2 字节 (data[0]=err_code, data[1]=原功能码)。
     * 设备端 app_cmd.c 收到 0x80/0x81 是直接把 ack_len 置 0 返回的, 不看载荷,
     * 所以那 2 字节一直是白带的 —— 把它去掉, 和协议/设备端对齐。
     *
     * ⚠️ 如果以后确实要让应答携带信息(比如 err_code 给报警器判读),
     *    必须【两端同时】改协议, 不能只改这里。 */
    frame.data_len = 0U;

    (void)func;      /* 载荷不带原功能码了, 参数保留是为了不改调用方 */
    (void)err_code;

    return lora_proto_send_frame(&frame);
}

int lora_business_send_req(const uint8_t uid[CFG_UID_BYTES], slave_type_t dev_type,
                           uint8_t func, const uint8_t *data, uint8_t data_len)
{
    uint8_t idx;

    if (data_len > (LORA_FRAME_DATA_MAX - 1U)) return -1;  /* 留 1B 给 func */

    /* 【从机表是动态白名单】: 地址不在表里就直接拒掉 —— 设备必须先经 0x04 绑定
     * 进表(写 config_store + g_slave_list)才会被这里找到。 */
    idx = lora_slave_find(uid);
    if (idx == 0xFFU) return -2;

    /* 表里的 dev_type 在绑定时确定, 运行期不变。调用方传进来的如果和表不一致,
     * 以表为准并告警(避免一个地址被当成两种设备) */
    if (g_slave_list[idx].dev_type != dev_type)
    {
        log_info("[GW] !! %02X%02X%02X 在从机表里是 %s, 调用方却按 %s 下发, 以表为准\r\n",
                 uid[0], uid[1], uid[2],
                 (g_slave_list[idx].dev_type == SLAVE_TYPE_SENSOR) ? "SENSOR" : "ALARM_OUT",
                 (dev_type == SLAVE_TYPE_SENSOR) ? "SENSOR" : "ALARM_OUT");
        dev_type = g_slave_list[idx].dev_type;
    }

    /* 把 func 塞进 payload[0], data 接在后面 —— 这样 data_cache_down_push
     * 只需要存一段连续字节流, fetch 时取出来直接当 frame.data[] */
    uint8_t payload[LORA_FRAME_DATA_MAX];
    payload[0] = func;
    if (data_len > 0U && data != NULL)
    {
        memcpy(&payload[1], data, data_len);
    }

    int ret = data_cache_down_push(uid, dev_type, payload, (uint8_t)(data_len + 1U));
    if (ret != 0) return -3;

    /* 实际发送时机:
     *   声光报警器(常驻 RX) -> _downlink_set_alarm 里立即 lora_try_send_downlink
     *   液位传感器(休眠)   -> 等它上报时 lora_reply_logic 第二步搭车下发 */
    return 0;
}
