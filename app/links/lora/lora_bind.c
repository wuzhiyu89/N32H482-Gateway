/**
 * @file  lora_bind.c
 * @brief 设备绑定（LoRa 0x04）：窗口状态机 + 握手 + 落配置
 *
 * 时序（用户 2026-09-29 决策）：
 *   云端 dev.bind -> lora_business_open_bind_window() 开窗口（BIND_LISTENING）
 *   设备发 0x04   -> 网关回 0x80、转 BIND_WAIT_DEV_ACK
 *   设备回 0x80   -> 写 config_store + 从机表、上报 event.device{bound}
 *   窗口到期      -> 没走完就上报 bind_failed
 *
 * 同一个 UID 重复绑定时【只更新元数据、不新增条目】（用户 2026-10-14 口径），
 * 细节见 lora_bind_handle_dev_ack() 里的注释。
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "lora_business.h"
#include "lora_internal.h"
#include "config_store.h"
#include "last_value.h"
#include "cloud_proto.h"      /* cloud_proto_get_now_ts */
#include "bsp_tim.h"
#include "log.h"

/* ========== 内部: 设备绑定窗口状态机(P5, LoRa 0x04) ==========
 *
 * 【为什么绑定只在云端开窗口时才监听 0x04】(用户 2026-09-29 决策):
 *   平时 0x04 是个"陌生设备广播" —— 任何附近设备都可能发, 网关常驻监听就
 *   会被无关帧打扰并回 ACK(空口被刷)。所以常态是 BIND_IDLE, 只有当云端
 *   下发 dev.bind{serial,name,pairWindowSeconds}、经 pair_hook 调
 *   lora_business_open_bind_window() 时才进入 BIND_LISTENING 打开窗口,
 *   到点自动回 IDLE。
 *
 * 【状态转移】(tick 用 get_tick_ms, 32 位 ms, 用无符号差值判回绕):
 *   BIND_IDLE         -- open_bind_window -->  BIND_LISTENING
 *   BIND_LISTENING    -- 收到 0x04      -->  BIND_WAIT_DEV_ACK (记 pending_uid, 立即发一次 0x80)
 *   BIND_WAIT_DEV_ACK -- 收到对端 0x80 -->  BIND_IDLE  + 写 config_store + 触发 BOUND
 *   BIND_WAIT_DEV_ACK -- 每 5s          -->  重发 0x80 ACK(给设备机会重收)
 *   任意 LISTENING/WAIT_DEV_ACK -- 窗口到点 -->  BIND_IDLE (WAIT_DEV_ACK 时还要触发 BIND_FAILED)
 *
 * 【只允许一个绑定并发】: 同一时刻只跟踪一个发起绑定的设备(pending_uid),
 *   窗口里第二个不同地址的 0x04 直接被 0x81 拒掉(不打断当前握手)。
 */
typedef enum
{
    BIND_IDLE         = 0,   /* 未开窗口 */
    BIND_LISTENING    = 1,   /* 窗口已开, 在等 0x04 */
    BIND_WAIT_DEV_ACK = 2,   /* 已回 0x80, 等设备回 0x80 确认 */
} bind_state_t;

static struct
{
    bind_state_t state;
    uint32_t     deadline_tick;      /* 窗口到期 tick(进入 LISTENING 时设定) */
    uint32_t     last_ack_tx_tick;   /* 上次向设备发 0x80 ACK 的 tick */
    char         expected_serial[CFG_SERIAL_MAX + 1U];   /* 落盘用, 来自云端 dev.bind */
    char         expected_name[CFG_NAME_MAX + 1U];
    bool         is_sensor;          /* sen.bind=true 液位 / dev.bind=false 声光 */
    uint8_t      dev_type;           /* ★ 第 11 条: dev.bind 给的联动设备类型
                                      * (cfg_dev_type_t 1/2/3); sen.bind 时无意义 */
    uint8_t      pending_uid[CFG_UID_BYTES];     /* 发起绑定的设备地址(收到 0x04 后登记) */
} g_bind;
_Static_assert(sizeof(g_bind.pending_uid) == CFG_UID_BYTES,
               "g_bind.pending_uid 必须 13 字节");


/* ========== 内部: 设备绑定窗口状态机(P5, LoRa 0x04) ==========
 *
 * 下列 7 个静态函数配合 on_frame / lora_business_poll 实现:
 *   lora_bind_fire_event()              —— 统一出口: 触发事件回调(没注册只打日志)
 *   lora_bind_send_ack_to_pending()—— 向发起绑定的设备发 0x80 ACK
 *   lora_bind_handle_0x04()       —— 收到 0x04 绑定请求
 *   lora_bind_handle_dev_ack()    —— 收到设备回的 0x80 确认(绑定成立)
 *   lora_bind_logic()             —— 周期调用: 重发 ACK + 窗口到期
 *   lora_business_set_bind_event_cb() / lora_business_open_bind_window() —— 对外 API
 *
 * 时序概要(详见 g_bind 上方注释):
 *   设备发 0x04 → 网关回 0x80 → 设备回 0x80 → 绑定成功。
 *   网关每 5s 重发一次 0x80(LORA_BIND_ACK_INTERVAL_MS), 直到设备回 0x80 或窗口到期。
 */

static lora_bind_event_fn g_bind_event_cb = NULL;

/* 统一的事件出口。uid_hex 必须是 6 字符大写 hex 串, 或 NULL 表示"无具体设备"(如超时) */
void lora_bind_fire_event(const char *uid_hex, bool is_sensor, lora_bind_event_t event)
{
    const char *ev_str =
        (event == LORA_BIND_EVENT_BOUND)       ? "bound"       :
        (event == LORA_BIND_EVENT_BIND_FAILED) ? "bind_failed" :
        (event == LORA_BIND_EVENT_UNBOUND)     ? "unbound"     : "unknown";

    log_info("[BIND] event=%s uid=%s type=%s\r\n",
             ev_str,
             (uid_hex != NULL) ? uid_hex : "(none)",
             is_sensor ? "sensor" : "device");

    if (g_bind_event_cb != NULL)
    {
        g_bind_event_cb(uid_hex, is_sensor, event);
    }
}

/* 向发起绑定的设备(g_bind.pending_uid)发 0x80 ACK(应答它的 0x04 绑定请求) */
void lora_bind_send_ack_to_pending(void)
{
    (void)lora_business_send_ack(g_gw_uid, g_bind.pending_uid,
                                 LORA_FUNC_BIND_REQ, true, 0U);
    g_bind.last_ack_tx_tick = get_tick_ms();
    log_info("[BIND] 已向 %02X%02X%02X 发 0x80 ACK(应答 0x04 绑定请求)\r\n",
             g_bind.pending_uid[0], g_bind.pending_uid[1], g_bind.pending_uid[2]);
}

/* 收到 0x04 绑定请求: 检查容量 → 登记 pending_uid → 立即回 0x80 */
void lora_bind_handle_0x04(const lora_frame_t *frame)
{
    /* ★2026-10-27: UID 首字节必须与窗口类型一致★
     *
     * 0x04 帧本身【不带设备类型】, 网关只能靠开窗命令区分液位/声光。以前
     * 本地 `#bind` 默认按液位开窗, 声光会被写进 sensors[]; 绑定成功后
     * lora_bind_queue_sensor_sync() 立刻排队 0x01(含 6 字节校时) ->
     * 现场就是"声光绑定后被校时"。
     *
     * 这里按 UID 首字节兜底: 液位窗口只收 21 开头, 声光窗口只收 31 开头;
     * 不匹配就回 0x81 拒绝, 窗口保持打开(可以继续等正确的设备)。
     */
    {
        uint8_t expect = g_bind.is_sensor ? 0x21U : 0x31U;

        if (frame->dev_uid[0] != expect)
        {
            log_info("[BIND] !! %02X%02X%02X 的 UID 首字节 0x%02X 与窗口类型(%s)不符 -> 回 0x81 拒绝\r\n",
                     frame->dev_uid[0], frame->dev_uid[1], frame->dev_uid[2],
                     (unsigned)frame->dev_uid[0],
                     g_bind.is_sensor ? "液位/21" : "声光/31");
            (void)lora_business_send_ack(g_gw_uid, frame->dev_uid,
                                         LORA_FUNC_BIND_REQ, false, 0x07U); /* 0x07 = 类型不符 */
            return;
        }
    }

    /* 容量检查(总数≤5 / 液位≤4 / 声光≤3) —— 不够直接回 0x81, 不进入握手 */
    if (!config_store_can_add(g_bind.is_sensor))
    {
        log_info("[BIND] !! %02X%02X%02X 请求绑定, 但%s已达上限, 回 0x81 拒绝\r\n",
                 frame->dev_uid[0], frame->dev_uid[1], frame->dev_uid[2],
                 g_bind.is_sensor ? "液位传感器" : "声光设备");
        (void)lora_business_send_ack(g_gw_uid, frame->dev_uid,
                                     LORA_FUNC_BIND_REQ, false, 0x06U);  /* err 0x06 = 容量满 */
        return;
    }

    /* 已在等另一台设备的 ACK → 直接拒掉这次(单绑定并发) */
    if (g_bind.state == BIND_WAIT_DEV_ACK)
    {
        log_info("[BIND] !! %02X%02X%02X 请求绑定, 但已在等 %02X%02X%02X 回 ACK, 回 0x81 拒绝\r\n",
                 frame->dev_uid[0], frame->dev_uid[1], frame->dev_uid[2],
                 g_bind.pending_uid[0], g_bind.pending_uid[1], g_bind.pending_uid[2]);
        (void)lora_business_send_ack(g_gw_uid, frame->dev_uid,
                                     LORA_FUNC_BIND_REQ, false, 0x05U);  /* err 0x05 = 忙 */
        return;
    }

    /* 登记 pending_uid → 转入 WAIT_DEV_ACK → 立即发 0x80 */
    memcpy(g_bind.pending_uid, frame->dev_uid, CFG_UID_BYTES);
    g_bind.state = BIND_WAIT_DEV_ACK;
    log_info("[BIND] 收到 %02X%02X%02X 的 0x04 绑定请求(type=%s), 进入 WAIT_DEV_ACK\r\n",
             g_bind.pending_uid[0], g_bind.pending_uid[1], g_bind.pending_uid[2],
             g_bind.is_sensor ? "sensor" : "device");
    lora_bind_send_ack_to_pending();
}

/* ---------- 绑定成功后的"首次配置同步" ----------
 *
 * 【为什么要有它 —— 用户 2026-10-22 报的 bug】
 *   绑定成功只写了 config_store + 从机表, 没有把 0x01 参数投进下发缓存。
 *   于是液位第一次上报时, 网关的应答里没有配置, 设备只能一直用自己的出厂参数,
 *   直到云端再下发一次 sen.set / cfg.set。现场表现就是"绑完第一包不上配置"。
 *
 * 【为什么投缓存而不是立刻发】
 *   液位是休眠设备, 平时不在接收窗口; 只能等它上报时搭车下发
 *   (lora_reply_logic 第二步)。投进去之后, 它第一次上报就能收到。
 *
 * 【值从哪来】
 *   上报周期取 config_store 里该设备的值 —— 新绑定是默认 300s; 重绑时保留平台
 *   此前设过的值, 换的新设备也能立刻对齐。其余三项用 lora_business.h 的共用默认。
 */
static void lora_bind_queue_sensor_sync(const char *uid_hex)
{
    const config_store_t *cfg;
    uint16_t report_s = LORA_SENSOR_REPORT_DEFAULT_S;
    uint8_t  i;

    cfg = config_store_get();
    if (cfg != NULL)
    {
        for (i = 0u; (i < cfg->sensor_count) && (i < (uint8_t)CFG_SENSOR_MAX); i++)
        {
            if (strcmp(cfg->sensors[i].uid, uid_hex) == 0)
            {
                if (cfg->sensors[i].report_period_s != 0u)
                {
                    report_s = cfg->sensors[i].report_period_s;
                }
                break;
            }
        }
    }

    if (lora_business_set_sensor_params(uid_hex,
                                        report_s,
                                        LORA_SENSOR_ALARM_DEFAULT_MM))
    {
        log_info("[BIND] 已为 %s 排入 0x01 配置同步(上报=%us 相同上报=1 "
                 "阈值=%umm), 它下次上报时随应答下发\r\n",
                 uid_hex, (unsigned)report_s, (unsigned)LORA_SENSOR_ALARM_DEFAULT_MM);
    }
    else
    {
        log_info("[BIND] !! %s 的 0x01 配置同步排入失败(原因见上一行 [CLOUD->LoRa])\r\n",
                 uid_hex);
    }
}

/* 收到设备回的 0x80 确认: 写 config_store + 追加运行期从机表 + 触发 BOUND */
void lora_bind_handle_dev_ack(const lora_frame_t *frame)
{
    /* 校验是不是 pending_uid 回的(避免别的设备 0x80 蹭进来) */
    if (memcmp(frame->dev_uid, g_bind.pending_uid, CFG_UID_BYTES) != 0)
    {
        log_info("[BIND] !! 收到 %02X%02X%02X 的 0x80, 但 pending 是 %02X%02X%02X, 忽略\r\n",
                 frame->dev_uid[0], frame->dev_uid[1], frame->dev_uid[2],
                 g_bind.pending_uid[0], g_bind.pending_uid[1], g_bind.pending_uid[2]);
        return;
    }

    /* 1) hex 串形式(供 config_store / 事件回调用) */
    char uid_hex[CFG_UID_STR_MAX];
    /* cfg_uid_from_bytes 把 13 字节转 hex, 不会失败(非全 0 即有效) */
    (void)cfg_uid_from_bytes(g_bind.pending_uid, uid_hex);

    /* ★2026-10-27: 绑定前残留的下发缓存一律作废★
     * 新绑定不继承旧指令 —— 尤其是"曾被误绑成液位"时排下的 0x01(带校时),
     * 否则重绑成声光后它还会被当成待发指令发出去。 */
    (void)data_cache_down_del_by_uid(g_bind.pending_uid);

    /* 2) 追加到运行期从机表(失败就别写 config_store, 免得两边不一致) */
    slave_type_t type = g_bind.is_sensor ? SLAVE_TYPE_SENSOR : SLAVE_TYPE_ALARM_OUT;
    int idx = lora_slave_add(g_bind.pending_uid, type);
    if (idx < 0)
    {
        /* ==================================================================
         * ★★2026-10-14 重绑: 已在从机表 ⇒ 允许再绑, 但【不新增】条目 ★★
         * ------------------------------------------------------------------
         * 【用户场景】设备坏了换新, 重新烧录后回到【同一默认地址】
         *   ⇒ 网关必须允许同 UID 再绑一次; 若拒绝, 现场换设备就必须先解绑,
         *     很难操作。且从机表里【不能出现两条同 UID】。
         *
         * 【用户口径, 三条】
         *   ① serial/name 用本次绑定窗口里的新值【覆盖】(换了设备, 标识也该换);
         *   ② 【不】动 report_period_s 等运行参数 —— 平台一定会再下发一次上报周期,
         *      等于给新设备设一遍(动了反而把平台设的值悄悄改回默认 60);
         *   ③ 【不继承】旧设备的历史值 ⇒ 清掉该 UID 的快照。
         *      理由: 否则新设备还没上报时 sen.query 就回旧水位、level_supervisor
         *      会拿旧水位算档位(新设备一上电就按旧水位点灯)、失联判定也可能因
         *      "缓存里的 rx_tick_ms 很新"而误判在线。
         *
         * ⚠️ 这里【不能】用 config_store_bind_device() —— 它对已存在的 UID 直接
         *    返回 CFG_ERR_RANGE(见该函数开头), 会静默什么都不改。
         * ⚠️ 也【不能】用 config_store_unbind_device() + bind "先删再建" ——
         *    unbind 会连带清掉 gears[].target_uid 里指向该 UID 的条目,
         *    那台设备以后收不到联动指令(能绑定、能上报, 就是不动灯)。
         *
         * ⚠️⚠️ 【本分支目前没有主机单测覆盖】⚠️⚠️
         *   _host_test 里没有任何套件链接【真的】app/links/lora/lora_business.c
         *   (test_cloud 用的是 _host_test/fake_lora_business.c 那个假实现),
         *   因为本文件依赖 LoRa/radio 时序。⇒ 改这段代码【必须上板验证】,
         *   "主机单测全绿"对它没有意义。
         *   已有的主机覆盖只到它调用的两个新函数:
         *     · config_store_rebind_device() -> test_cfg 用例 [14]
         *     · last_value_clear_uid()       -> test_last_value 用例 [10]
         * ================================================================== */
        if ((idx == -1) && (lora_slave_find(g_bind.pending_uid) != 0xFFU))
        {
            cfg_result_t rb;

            log_info("[BIND] %s %s 重新绑定(已在从机表, 不新增): serial=%s name=%s\r\n",
                     g_bind.is_sensor ? "液位传感器" : "声光设备",
                     uid_hex,
                     g_bind.expected_serial[0] ? g_bind.expected_serial : "(none)",
                     g_bind.expected_name[0]    ? g_bind.expected_name    : "(none)");

            /* ① 覆盖 serial/name(设备侧连 type 一起更新; 运行参数一律不动) */
            rb = config_store_rebind_device(uid_hex, g_bind.expected_serial,
                                            g_bind.expected_name, g_bind.dev_type,
                                            cloud_proto_get_now_ts());
            if (rb != CFG_OK)
            {
                /* 落盘失败: 运行期表已经是"已存在"状态, 不必回滚, 但要让现场看见 */
                log_info("[BIND] !! 重绑写 config_store 失败(code=%d), 本次只更新了运行期表\r\n",
                         (int)rb);
            }

            /* ③ 清该 UID 的历史快照(新设备不继承旧值; 其它 UID 不受影响) */
            (void)last_value_clear_uid(g_bind.pending_uid);

            /* ④ 新设备必须拿到当前参数: 排一条 0x01, 首次上报时随应答下发 */
            if (g_bind.is_sensor)
            {
                lora_bind_queue_sensor_sync(uid_hex);
            }

            lora_bind_fire_event(uid_hex, g_bind.is_sensor, LORA_BIND_EVENT_BOUND);
            g_bind.state = BIND_IDLE;
            return;
        }

        /* 到这里就是真失败: 从机表【满了】 */
        log_info("[BIND] !! 从机表【已满】, 无法绑定 %s(idx=%d) —— 请先解绑不用的设备\r\n",
                 uid_hex, idx);
        lora_bind_fire_event(uid_hex, g_bind.is_sensor, LORA_BIND_EVENT_BIND_FAILED);
        g_bind.state = BIND_IDLE;
        return;
    }

    /* 3) 写 config_store(失败回滚运行期表)
     * ★★ 必须与 CFG_OK(== 0) 比较, 不能用 ! 取反 ★★
     * cfg_result_t 的语义是"0 = 成功"(CFG_OK=0, 其余全是错误码), 写成
     * `if (!config_store_bind_device(...))` 会把【每一次绑定成功】都当成失败:
     *   刚加进运行期表的设备被回滚掉、事件报 bind_failed, 但 config_store 内部
     *   其实已经 save 落 Flash 了 —— 结果"云端看到绑定失败, 重启后设备又冒出来"。
     * (2026-09-29 评审修正) */
    uint32_t     now_ts = cloud_proto_get_now_ts();
    cfg_result_t wr     = config_store_bind_device(uid_hex, g_bind.expected_serial,
                                                   g_bind.expected_name, g_bind.is_sensor,
                                                   g_bind.dev_type, now_ts);

    if (wr == CFG_ERR_RANGE)
    {
        /* ★2026-10-24 bugfix: 配置里已经有这个 UID★
         *
         * 典型场景: 平台先用 cfg.set 把 sensors[]/devices[] 下发下来(设备此时
         * 还没完成 0x04 绑定), 随后设备发起绑定 -> 运行期表是空的, lora_slave_add
         * 成功, 但 config_store_bind_device 的查重命中 -> 返回 CFG_ERR_RANGE。
         *
         * 旧代码把它当失败: 【删掉刚加进运行期表的条目】+ 发 bind_failed
         *   -> 现象正是"云端 cfg.query 里明明有设备, 网关从机表却是空的,
         *      而且绑定报失败"。
         *
         * 正确语义: 这不是新设备, 是"配置已存在、只差一次握手" -> 走重绑,
         * 只覆盖 serial/name(运行参数一律不动), 运行期表保留。 */
        log_info("[BIND] %s 已在配置里(平台先下发过) -> 走重绑, 不新增条目\r\n", uid_hex);
        wr = config_store_rebind_device(uid_hex, g_bind.expected_serial,
                                        g_bind.expected_name, g_bind.dev_type, now_ts);
    }

    if (wr != CFG_OK)
    {
        log_info("[BIND] !! 写 config_store 失败(code=%d), 回滚运行期表\r\n", (int)wr);
        (void)lora_slave_del(g_bind.pending_uid);
        lora_bind_fire_event(uid_hex, g_bind.is_sensor, LORA_BIND_EVENT_BIND_FAILED);
        g_bind.state = BIND_IDLE;
        return;
    }

    /* 配置里可能还有别的设备(平台下发过) -> 顺手补登记, 保证后续下发都能找到 */
    lora_business_ensure_slaves();

    /* 4) 绑定成功 */
    /* 给液位排一条 0x01 配置同步 —— 修"绑完第一包不带配置" */
    if (g_bind.is_sensor)
    {
        lora_bind_queue_sensor_sync(uid_hex);
    }

    log_info("[BIND] %s %s 绑定成功(uid=%s, serial=%s, name=%s), 已写 config_store + 从机表\r\n",
             g_bind.is_sensor ? "液位传感器" : "声光设备",
             uid_hex, uid_hex,
             g_bind.expected_serial[0] ? g_bind.expected_serial : "(none)",
             g_bind.expected_name[0]    ? g_bind.expected_name    : "(none)");
    lora_bind_fire_event(uid_hex, g_bind.is_sensor, LORA_BIND_EVENT_BOUND);

    g_bind.state = BIND_IDLE;
    memset(g_bind.pending_uid, 0, sizeof(g_bind.pending_uid));
}

/* 周期调用(挂在 lora_business_poll):
 *   - 窗口未到期时, 若在 WAIT_DEV_ACK 且距上次发 0x80 已过 5s → 重发 0x80
 *   - 窗口到期: 若在 WAIT_DEV_ACK → 触发 BIND_FAILED; 一律回 IDLE
 * 用 (int32_t)(now - deadline_tick) >= 0 判到期 —— tick 32 位无符号回绕时
 * "已过 deadline" 的差值是小正数, 未过时是大负数(强转 int32 为负), 这样判才对。
 */
void lora_bind_logic(void)
{
    if (g_bind.state == BIND_IDLE) return;

    uint32_t now = get_tick_ms();

    /* 窗口未到期: 看是否该重发 0x80 */
    if ((int32_t)(now - g_bind.deadline_tick) < 0)
    {
        if (g_bind.state == BIND_WAIT_DEV_ACK)
        {
            if ((uint32_t)(now - g_bind.last_ack_tx_tick) >= LORA_BIND_ACK_INTERVAL_MS)
            {
                lora_bind_send_ack_to_pending();
            }
        }
        return;
    }

    /* 窗口到期 */
    if (g_bind.state == BIND_WAIT_DEV_ACK)
    {
        char uid_hex[CFG_UID_STR_MAX];
        /* cfg_uid_from_bytes 不会失败, 直接走失败回调 */
        (void)cfg_uid_from_bytes(g_bind.pending_uid, uid_hex);
        lora_bind_fire_event(uid_hex, g_bind.is_sensor, LORA_BIND_EVENT_BIND_FAILED);
        log_info("[BIND] 窗口到期, 未收到 %02X%02X%02X 的确认, 绑定失败\r\n",
                 g_bind.pending_uid[0], g_bind.pending_uid[1], g_bind.pending_uid[2]);
    }
    else
    {
        log_info("[BIND] 窗口到期, 期间没有设备发起 0x04, 绑定失败(无具体设备)\r\n");
        lora_bind_fire_event(NULL, g_bind.is_sensor, LORA_BIND_EVENT_BIND_FAILED);
    }

    g_bind.state = BIND_IDLE;
    memset(g_bind.pending_uid, 0, sizeof(g_bind.pending_uid));
}


void lora_business_set_bind_event_cb(lora_bind_event_fn fn)
{
    g_bind_event_cb = fn;
}

bool lora_business_open_bind_window(const char *serial, const char *name,
                                     uint16_t window_s, bool is_sensor,
                                     uint8_t dev_type)
{
    /* 参数校验: 允许 serial/name 为空(用 "" 落盘), 但 window_s 必须在合理区间 */
    if (window_s == 0U || window_s > 300U) return false;
    if (g_bind.state != BIND_IDLE) return false;   /* 已有窗口在进行 */

    /* 落盘用的元数据(serial/name), 截断到数组上限, 空指针当 "" 处理 */
    if (serial != NULL)
    {
        strncpy(g_bind.expected_serial, serial, CFG_SERIAL_MAX);
        g_bind.expected_serial[CFG_SERIAL_MAX] = '\0';
    }
    else
    {
        g_bind.expected_serial[0] = '\0';
    }
    if (name != NULL)
    {
        strncpy(g_bind.expected_name, name, CFG_NAME_MAX);
        g_bind.expected_name[CFG_NAME_MAX] = '\0';
    }
    else
    {
        g_bind.expected_name[0] = '\0';
    }

    g_bind.is_sensor       = is_sensor;
    g_bind.dev_type        = dev_type;      /* 第 11 条: dev.bind 的类型, 绑定时写进 devices[].type */
    g_bind.deadline_tick   = get_tick_ms() + (uint32_t)window_s * 1000U;
    g_bind.last_ack_tx_tick = 0U;
    memset(g_bind.pending_uid, 0, sizeof(g_bind.pending_uid));
    g_bind.state           = BIND_LISTENING;

    log_info("[BIND] 开放绑定窗口: window=%us, type=%s, serial=%s, name=%s\r\n",
             (unsigned)window_s,
             is_sensor ? "sensor" : "device",
             g_bind.expected_serial[0] ? g_bind.expected_serial : "(none)",
             g_bind.expected_name[0]    ? g_bind.expected_name    : "(none)");
    return true;
}

bool lora_bind_accepts_0x04(void)
{
    return (g_bind.state != BIND_IDLE);
}

bool lora_bind_waiting_dev_ack(const uint8_t uid[CFG_UID_BYTES])
{
    return ((g_bind.state == BIND_WAIT_DEV_ACK) &&
            (memcmp(uid, g_bind.pending_uid, CFG_UID_BYTES) == 0));
}
