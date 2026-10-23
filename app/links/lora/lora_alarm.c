/**
 * @file  lora_alarm.c
 * @brief 声光联动：0x03 执行结果状态机（0x80/0x82/0x83）+ 档位下发
 *
 * 两块内容：
 *   1) 0x03 的"三段式应答"状态机：0x80 = 收到了，0x82 = 执行成功，0x83 = 执行失败；
 *      15 秒窗口、最多 3 条，3 条都没成功就判设备离线（详见下面宏上方的注释）。
 *   2) 档位联动下发：把 level_eval 算好的档位/文本按 gears[] 投递到下发缓存，
 *      真正的空口发送与 ACK 重试在 lora_downlink.c（本文件只管"投递"）。
 *
 * 本文件不含帧分发；入口是 handle_ack_from_device()（在 lora_business.c）和
 * level_supervisor（经 lora_business_apply_gear()）。
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "lora_business.h"
#include "lora_internal.h"
#include "config_store.h"
#include "level_eval.h"
#include "data_cache.h"
#include "bsp_tim.h"      /* get_tick_ms */
#include "log.h"

/* ==================== ★2026-10-08 声光(0x03)执行结果状态机的公共定义 ====================
 * 放在 lora_tx_downlink 之前: 它是发送咽喉, 每次真的发出去都要记账。
 * 状态机主体在下面 handle_ack_from_device 之前(见 lora_alarm_on_fail)。 */
#define LORA_ALARM_TIMEOUT_MS   15000u   /* 单条 0x03 等 0x82/0x83 的窗口(用户指定 15s) */
#define LORA_ALARM_MAX_ATTEMPTS 3u       /* 单条动作最多发几条(用户指定 3 条) */
#define LORA_ALARM_RETRY_GAP_MS 500u     /* 重发前的最小间隔(给设备射频恢复时间) */

/* 正在下发的这条动作是哪个档位。
 * 【为什么要这个全局】lora_tx_downlink 只知道 slave(uid/func/data), 不知道档位 ——
 *   档位只存在于 gears[] 的索引里, 不在空口载荷里。而上报 event.action 需要它。
 *   单线程 + 一条指令从入队到发出只隔最多一个 poll 周期, 所以这里存一份是安全的。
 *   (由 lora_business_apply_gear_ex() 在入队时写。) */
uint8_t g_alarm_gear_inflight;


static lora_alarm_result_fn g_alarm_result_cb = NULL;

void lora_business_set_alarm_result_cb(lora_alarm_result_fn fn)
{
    g_alarm_result_cb = fn;
}

/** 拿到 uid 的 hex 串并回调主程序(没注册就只打日志) */
void lora_alarm_fire_result(const lora_slave_t *s, bool ok, bool gave_up,
                            bool recovered)
{
    char uid_hex[CFG_UID_STR_MAX];

    (void)cfg_uid_from_bytes(s->dev_uid, uid_hex);

    log_info("[GEAR] 上报执行结果 uid=%s gear=%u ok=%u gave_up=%u recovered=%u\r\n",
             uid_hex, (unsigned)s->alarm_gear, ok ? 1u : 0u, gave_up ? 1u : 0u,
             recovered ? 1u : 0u);

    if (g_alarm_result_cb != NULL)
    {
        g_alarm_result_cb(uid_hex, s->alarm_gear, ok, gave_up, recovered);
    }
}

/**
 * @brief 这一条 0x03 失败了(收到 0x83 或 15s 超时): 还有配额就重发, 用完就收尾
 *
 * 【为什么重发是"重新走一遍下发"而不是"重发同一串字节"】
 *   0x82/0x83 是【设备执行完】的结论, 重发同一帧是没意义的(设备已经执行过了);
 *   真正要做的是把这条业务动作【再试一次】。所以这里回到 IDLE + 保留下发缓存,
 *   由 lora_try_send_alarm_idle() 捡起来重发 —— 与既有的重发路径完全一致。
 *
 * 【为什么清 alarm_pending 但保留 attempts】
 *   attempts 是"这一轮动作已经试了几条", 跨重发保留; alarm_pending 表示
 *   "正在等结果", 清掉后 lora_downlink_logic 就不会再对它判超时。
 */
void lora_alarm_on_fail(lora_slave_t *s)
{
    s->alarm_pending = false;
    s->state         = SLAVE_STATE_IDLE;    /* 让 lora_try_send_alarm_idle 能再捡起来 */
    s->retry_cnt     = 0U;                  /* 帧级重传计数与业务重试无关, 一并归零 */

    if (s->alarm_attempts >= LORA_ALARM_MAX_ATTEMPTS)
    {
        /* ★重试用尽 ⇒ 收尾并报"设备离线"★ 缓存也要丢掉, 否则会被无限重发 */
        log_info("[GEAR] !! %02X%02X%02X 连续 %u 条都没执行成功 -> 判【声光报警器离线】\r\n",
                 s->dev_uid[0], s->dev_uid[1], s->dev_uid[2],
                 (unsigned)s->alarm_attempts);
        /* ★2026-10-23 bugfix: 和 0x82/重传耗尽两处保持一致 —— 只有缓存里还是
         * 刚发出去的那条才删。15 秒窗口里云端可能已经下发了【新档位】,
         * 无条件按 uid 删会把那条新指令一起删掉(平台以为成功, 设备停在旧档位)。 */
        if (!data_cache_down_del_if_match(s->dev_uid, s->pending_func,
                                          s->pending_data, s->pending_data_len))
        {
            log_info("[GEAR] %02X%02X%02X 缓存里已是更新的指令 -> 保留待发\r\n",
                     s->dev_uid[0], s->dev_uid[1], s->dev_uid[2]);
        }
        /* ★2026-10-18 (丙): 记下"已报离线", 等它下次 0x82 成功时补报 recovered ★ */
        s->alarm_offline_reported = true;
        lora_alarm_fire_result(s, false, true, false);  /* ok=false, gave_up=true */
        s->alarm_attempts = 0U;
        return;
    }

    /* 还有配额: 记一次"这一条失败了", 等 lora_try_send_alarm_idle 重发 */
    s->send_req_tick = get_tick_ms() - (LORA_ALARM_TIMEOUT_MS - LORA_ALARM_RETRY_GAP_MS);
    log_info("[GEAR] %02X%02X%02X 本条失败, 准备重发(已发 %u/%u)\r\n",
             s->dev_uid[0], s->dev_uid[1], s->dev_uid[2],
             (unsigned)s->alarm_attempts, (unsigned)LORA_ALARM_MAX_ATTEMPTS);
}

/* ============================================================================
 * 档位联动下发 —— 边缘自治的"最后一跳" (2026-09-30, 用户第 2 条)
 * ----------------------------------------------------------------------------
 * 【为什么要有它】
 *   文档第 4 章的自治核心: 评估 -> 档位 -> 按 gears[] 生成动作 -> 经 LoRa 下发
 *   联动设备。前四步早就做完并单测过了(level_eval.c / level_supervisor.c),
 *   但【下发】这一步一直空着 —— 于是:
 *     · gears[] 的 targets/soundMode/ledText/ttsText 全是死配置;
 *     · 唯一真正到设备的 0x03 是"把云端原始字节原样转发" -> 断网就不联动。
 *   本函数就是补上那一跳。
 *
 * 【本文件只做"取 uid + 投递", 组帧不在这里】
 *   组帧(9 字节头 + 文本)与"选哪些目标设备"是【纯逻辑】, 放在 level_eval.c 的
 *   level_gear_build_payload() / level_gear_collect_targets() —— 那样能在主机上
 *   跑单测, 而且"待声光设备协议确认"的布局只集中在一个函数里, 设备侧定稿后
 *   只改那一处。
 *
 * 【复用了什么(没另造轮子)】
 *   lora_business_send_req() —— 已有的"投递到下发缓存"入口: 负责找从机 ->
 *   写 pending_func/pending_data -> 由 poll 里的 lora_try_send_alarm_idle()(联动设备
 *   常驻 RX, 立即发) / lora_downlink_logic()(5s 超时重发、最多 4 次、耗尽丢弃)
 *   完成发送与 ACK 跟踪。所以 ACK、重试、窗口适配一行都不用重写。
 *
 * 【时序为什么要守】(网关液位lora规则.txt, 液位侧口径)
 *   液位上报后【立即开 5s 接收窗口】等主机 ack; 收到 ack 再开【10s 窗口】收云端
 *   控制指令; 收到指令后回【两遍 ack(间隔 3s)】再休眠。
 *   -> 本条链只对【联动设备】(常驻 RX, 随时可发), 不涉及液位窗口; 液位那侧的下发
 *      仍走 lora_reply_logic() 的 3s+3s 两步错开(落在 5s/10s 窗口内)。
 *      设备回的第二遍 ack 会落在 state=IDLE, handle_ack_from_device() 直接忽略
 *      (不是等 ACK 状态) —— 这是安全的, 不需要特判。
 * ========================================================================== */

/* 只对【声光/联动设备】下发 */
#define GEAR_TARGET_TYPE   SLAVE_TYPE_ALARM_OUT

/* 诊断计数(给状态行/排障用) */
static uint16_t g_gear_tx_ok;            /* 成功投递到缓存的目标台数 */
static uint16_t g_gear_tx_skip;          /* 跳过/投递失败的台数 */
static uint8_t  g_last_gear_tx = 0xFFU;  /* 上次真正下发过的档位 */

/**
 * @brief 档位联动下发(最后一跳的入口)
 * @param gear     目标档位 0~3
 * @param level_mm 当前有效水位(mm, 只用于日志)
 * @param led_text 渲染后的 LED 文本(可 NULL/空)
 * @param tts_text 渲染后的语音文本(可 NULL/空)
 * @param out_total 可选输出: 目标设备总台数(区分"没设备"和"发失败")
 * @return >=0 成功投递的设备台数; -1 参数非法/档位越界
 * @note  只投递到下发缓存, 不阻塞; 空口发送由 lora_business_poll() 完成。
 */
/* ==================== ★2026-10-10: dev.ctrl 的本次下发样式覆盖 ====================
 * 三个静态: 登记区(g_style_valid/g_style) + 组装用的临时档位副本(g_style_tmp)。
 * 【为什么是静态】cfg_gear_t 约 250 字节, 本工程栈只有 4KB, 放栈上会吃掉一大块;
 *   本工程单线程、apply_gear_ex 不重入, 用静态是安全的(同 g_alarm_gear_inflight 先例)。
 * 【为什么副本】绝不能改 cfg->gears[] —— 那是持久配置, 改了就"临时覆盖"变成"永久改了"。 */
static uint8_t    g_style_valid;
static cfg_gear_t g_style;
static cfg_gear_t g_style_tmp;

void lora_business_set_gear_style(uint8_t valid_mask, uint8_t show_mode,
                                  uint8_t text_mode_h1, uint8_t text_mode_h2,
                                  uint8_t color_h1, uint8_t color_h2)
{
    g_style_valid                 = valid_mask;
    g_style.led_show_mode         = show_mode;
    g_style.led_text_show_mode[0] = text_mode_h1;
    g_style.led_text_show_mode[1] = text_mode_h2;
    g_style.led_color[0]          = (cfg_led_color_t)color_h1;
    g_style.led_color[1]          = (cfg_led_color_t)color_h2;
}

void lora_business_clear_gear_style(void)
{
    g_style_valid = 0u;
}

/** 取"本台实际要用"的档位样式: 有覆盖就返回改过的副本, 否则直接返回配置 */
static const cfg_gear_t *gear_style_effective(const config_store_t *cfg, uint8_t gear)
{
    if (g_style_valid == 0u)
    {
        return &cfg->gears[gear];
    }

    g_style_tmp = cfg->gears[gear];          /* 整份拷贝, 再按 mask 覆盖 */

    if ((g_style_valid & LORA_STYLE_SHOW_MODE) != 0u)
    {
        g_style_tmp.led_show_mode = g_style.led_show_mode;
    }
    if ((g_style_valid & LORA_STYLE_TEXT_MODE) != 0u)
    {
        g_style_tmp.led_text_show_mode[0] = g_style.led_text_show_mode[0];
        g_style_tmp.led_text_show_mode[1] = g_style.led_text_show_mode[1];
    }
    if ((g_style_valid & LORA_STYLE_COLOR) != 0u)
    {
        g_style_tmp.led_color[0] = g_style.led_color[0];
        g_style_tmp.led_color[1] = g_style.led_color[1];
    }
    return &g_style_tmp;
}

int lora_business_apply_gear_ex(uint8_t gear, const char *only_uid,
                                const char *led_text1, const char *led_text2, const char *tts_text,
                                lora_gear_result_t *res, uint8_t res_cap,
                                uint8_t *out_total)
{
    const config_store_t *cfg = config_store_get();
    uint8_t idx[CFG_DEVICE_MAX];
    uint8_t n_targets;
    uint8_t sent = 0U;
    uint8_t i;

    if (cfg == NULL) { return -1; }
    if (gear >= (uint8_t)CFG_GEAR_COUNT) { return -1; }

    /* (1) 选目标: only_uid==NULL -> 按 gears[].targets(自动联动);
     *             only_uid!=NULL -> 只这一台(dev.ctrl 手动控制) */
    n_targets = level_gear_collect_targets(cfg, gear, only_uid,
                                           idx, (uint8_t)CFG_DEVICE_MAX);
    if (out_total != NULL) { *out_total = n_targets; }

    /* (2) 逐台组帧 + 投递 */
    for (i = 0U; (i < n_targets) && (i < (uint8_t)CFG_DEVICE_MAX); i++)
    {
        const cfg_device_t *d = &cfg->devices[idx[i]];
        uint8_t uid3[CFG_UID_BYTES];
        uint8_t payload[LEVEL_GEAR_PAYLOAD_MAX];
        uint8_t plen;
        uint16_t code = 0U;          /* ★ uint16: 错误码是 1003/1007 三位数 */
        int     r;

        if (!cfg_uid_to_bytes(d->uid, uid3))
        {
            g_gear_tx_skip++;
            code = 1007U;
            log_info("[GEAR] !! devices[%u].uid=%s 不是 26 位 hex 串, 跳过\r\n",
                     (unsigned)idx[i], d->uid);
        }
        else
        {
            /* 每台的音量可能不同, 所以载荷要逐台组(文本部分是一样的)
             * ★2026-10-10: 样式(ledShowMode/ledTextShowMode/ledColor)走
             *   gear_style_effective() —— 有 dev.ctrl 覆盖就用改过的副本, 否则用配置。*/
            plen = level_gear_build_payload(gear_style_effective(cfg, gear), d->volume,
                                            led_text1, led_text2, tts_text,
                                            payload, (uint16_t)sizeof(payload));
            if (plen == 0U)
            {
                g_gear_tx_skip++;
                code = 1007U;
                log_info("[GEAR] !! 档位 %u 给 %s 组帧失败(缓冲不够)\r\n",
                         (unsigned)gear, d->uid);
            }
            else
            {
                /* ★2026-10-08: 记下这条动作的档位★
                 * lora_tx_downlink 只知道 uid/func/data, 不知道档位(档位不在空口载荷里),
                 * 而上报 event.action 需要它。单线程 + 入队到发出只隔一个 poll 周期, 安全。 */
                g_alarm_gear_inflight = gear;

                r = lora_business_send_req(uid3, GEAR_TARGET_TYPE,
                                           LORA_FUNC_SET_ALARM, payload, plen);
                if (r == 0)
                {
                    sent++;
                    g_gear_tx_ok++;
                }
                else
                {
                    g_gear_tx_skip++;
                    /* -1 载荷过长(不太可能: 组帧已按 63 上限截过) / -2 从机表里没有它 */
                    code = (r == -1) ? 1007U : 1003U;
                    log_info("[GEAR] !! 档位 %u 投递给 %s 失败(r=%d: %s)\r\n",
                             (unsigned)gear, d->uid, r,
                             (r == -1) ? "载荷过长" : "从机表里没有它(未绑定?)");
                }
            }
        }

        if ((res != NULL) && (i < res_cap))
        {
            (void)snprintf(res[i].uid, sizeof(res[i].uid), "%s", d->uid);
            res[i].code = code;
        }
    }

    if (sent > 0U)
    {
        g_last_gear_tx = gear;
        log_info("[GEAR] 档位 %u 已投递 %u/%u 台联动设备%s(LED1=\"%s\" LED2=\"%s\")\r\n",
                 (unsigned)gear, (unsigned)sent, (unsigned)n_targets,
                 (only_uid != NULL) ? "(手动控制)" : "",
                 (led_text1 != NULL) ? led_text1 : "",
                 (led_text2 != NULL) ? led_text2 : "");
    }
    else
    {
        log_info("[GEAR] 档位 %u 没有可投递的联动设备(启用 %u 台)%s\r\n",
                 (unsigned)gear, (unsigned)n_targets,
                 (only_uid != NULL) ? " 或指定的那台不存在/已停用" : "");
    }

    return (int)sent;
}

int lora_business_apply_gear(uint8_t gear, int32_t level_mm,
                             const char *led_text1, const char *led_text2, const char *tts_text,
                             uint8_t *out_total)
{
    int sent = lora_business_apply_gear_ex(gear, NULL, led_text1, led_text2, tts_text,
                                           NULL, 0U, out_total);

    if (sent > 0)
    {
        log_info("[GEAR] (自动联动) 水位 %ldmm -> 档位 %u\r\n",
                 (long)level_mm, (unsigned)gear);
    }
    return sent;
}

/** 上次成功下发过的档位(0~3); 0xFF = 还没有下发过 */
uint8_t lora_business_last_gear(void)
{
    return g_last_gear_tx;
}

/** 联动下发计数: ok = 成功投递台数, skip = 跳过/失败台数 */
void lora_business_gear_stats(uint16_t *ok, uint16_t *skip)
{
    if (ok   != NULL) { *ok   = g_gear_tx_ok; }
    if (skip != NULL) { *skip = g_gear_tx_skip; }
}
