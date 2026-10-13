#include "level_supervisor.h"
#include "last_value.h"
#include "sensor_offline.h"
#include <string.h>

static level_eval_t           s_ev;         /* confirmCount 防抖状态机 */
static level_event_gate_t     s_gate;       /* 水位事件去抖(minChange/间隔) */
static level_dispatch_state_t s_disp;       /* "上次成功下发的那份"防重发 */
static cfg_thresholds_t       s_thr;        /* 上次评估用的阈值(用于发现"阈值变了") */
static uint32_t               s_fed_seq;    /* 已经喂到哪一次上报 */
static bool                   s_init;

/* 最近一次【有效】评估结果(给 level_supervisor_last() 用, 例如 dev.ctrl 渲染模板) */
static int32_t                s_last_mm;
static uint8_t                s_last_gear;
static bool                   s_last_ok;
static bool                   s_force;      /* 下一次 poll 强制评估一次(无视"没有新上报") */
static bool                   s_fail_active; /* 上一拍是否处于"全部失联"fail-safe 态(做边沿) */

void level_supervisor_force_next(void)
{
    s_force = true;
}

void level_supervisor_init(void)
{
    level_eval_init(&s_ev);
    level_event_gate_init(&s_gate);
    level_dispatch_init(&s_disp);
    (void)memset(&s_thr, 0, sizeof(s_thr));     /* 让下一次 poll 重新认阈值 */
    s_fed_seq = last_value_update_seq();    /* 不把"上电前就有"的旧样本算成新样本 */
    s_last_mm   = 0;
    s_last_gear = 0u;
    s_last_ok   = false;
    s_init = true;
}

void level_supervisor_mark_sent(uint8_t gear, const char *led_text1, const char *led_text2)
{
    level_dispatch_mark(&s_disp, gear, led_text1, led_text2);
}

void level_supervisor_mark_event(int32_t level_mm, uint32_t now_ms)
{
    level_event_mark(&s_gate, level_mm, now_ms);
}

/* 该台是否"在线": 复用本工程失联判定的【同一条规则】——
 * 超过 3×上报周期没上报就算不在线(sensor_offline_threshold_ms)。
 * 刻意不自己再定一套"在线": 两套定义迟早会打架。
 * 用无符号差值比较, tick 回绕安全。 */
static bool lsup_online(uint32_t now_ms, uint32_t last_ms, uint16_t period_s)
{
    return ((uint32_t)(now_ms - last_ms) < sensor_offline_threshold_ms(period_s)) ? true : false;
}

void level_supervisor_poll(const config_store_t *cfg, uint32_t now_ms,
                           level_decision_t *out)
{
    int32_t levels[CFG_SENSOR_MAX];
    uint8_t select[CFG_SENSOR_MAX];
    uint8_t n = 0u;
    uint8_t i;
    int32_t eff = 0;

    if (out == NULL)
    {
        return;
    }
    (void)memset(out, 0, sizeof(*out));
    out->state = LEVEL_ST_UNKNOWN;

    if (!s_init)
    {
        level_supervisor_init();
    }

    /* ==================================================================
     * 0) fail-safe: 液位传感器【全部失联】(文档第 5 章, 用户第 7 条)
     * ------------------------------------------------------------------
     * 【为什么放在"有没有新上报"之前】"全部失联"是【时间】条件(超过 3×上报周期
     *   没收到上报), 不是"来了新样本"才成立 —— 恰恰相反, 一台都不报的时候
     *   永远不会有新样本。所以必须在 new_sample 门之前算, 否则这条策略形同不存在。
     * 【怎么区分"从没报过"和"全掉线"】只看 s_last_ok: 上电至今没有过有效水位时
     *   s_last_ok=false -> 不算 fail-safe(没有"保持/强制"的对象, 也不该乱报警);
     *   曾经有过水位、现在一台都不在线 -> 才是真 fail-safe。
     * 【策略】rules.sensorOfflinePolicy: keep=保持当前档位 / alarm=强制档位3 /
     *   off=执行档位0(映射在 level_eval.c 的 level_failsafe_gear(), 纯函数可单测)。
     * ================================================================== */
    if ((cfg != NULL) && (s_last_ok))
    {
        uint8_t n_valid = 0u;

        for (i = 0u; (i < cfg->sensor_count) && (i < (uint8_t)CFG_SENSOR_MAX); i++)
        {
            const cfg_sensor_t *s = &cfg->sensors[i];
            uint8_t      uid[3];
            last_value_t v;

            if (s->enabled == 0u)               { continue; }
            if (!cfg_uid_to_bytes(s->uid, uid)) { continue; }
            if (!last_value_get(uid, &v))       { continue; }
            if (lsup_online(now_ms, v.rx_tick_ms, s->report_period_s)) { n_valid++; }
        }

        if (n_valid == 0u)
        {
            out->failsafe      = true;
            out->failsafe_gear = level_failsafe_gear(cfg->rules.sensor_offline_policy,
                                                     s_last_gear);
            if (!s_fail_active)
            {
                out->failsafe_enter = true;     /* 边沿: 只在这一拍为真 */
            }
        }
        s_fail_active = out->failsafe;
    }

    /* ---- 1) 有没有新上报? 没有就原样返回(不做任何推进) ----
     * ★ 例外: 被 force_next() 强制过一次 —— rule.mode 从 manual 切回 auto 时文档要求
     *   "立即按当前水位重评估一次并刷新联动状态", 而那时通常没有新上报。 */
    if ((last_value_update_seq() == s_fed_seq) && (!s_force))
    {
        return;                             /* new_sample 保持 false */
    }
    s_force         = false;
    s_fed_seq       = last_value_update_seq();
    out->new_sample = true;

    if (cfg == NULL)
    {
        return;
    }

    /* ---- 1.5) 阈值变了就重建评估状态机 ----
     * 触发场景: cfg.set 改 thresholds、恢复出厂、以后可能的热更新。
     * 【为什么必须重建】旧阈值下"已确认"的状态, 在新阈值下可能根本不成立;
     *   拿它继续做迁移判断会得出错误档位 —— 而且是那种"现场看着档位对不上、
     *   复位一下就好了"的难查问题。
     * ★ 刻意【不动】s_disp(下发缓存) ★ —— 那里存的是"上次成功发给报警器的那份",
     *   与阈值无关; 清掉它会让文档要求的"档位未变不重发"失效, 白白多下发一次。
     *   若档位模板文本因改配置而变, level_dispatch_should_send() 会自己判成
     *   "文本变了"并下发, 不需要这里插手。 */
    if (memcmp(&s_thr, &cfg->thresholds, sizeof(s_thr)) != 0)
    {
        level_eval_init(&s_ev);         /* have=false -> 下一次 feed 按"首次确认"处理 */
        s_thr = cfg->thresholds;
    }

    /* ---- 2) 取每台的水位 + 是否参与(在线 且 enabled) ---- */
    n = (cfg->sensor_count > (uint8_t)CFG_SENSOR_MAX) ? (uint8_t)CFG_SENSOR_MAX
                                                      : cfg->sensor_count;
    for (i = 0u; i < n; i++)
    {
        const cfg_sensor_t *s = &cfg->sensors[i];
        last_value_t        v;
        uint8_t             uid[3];

        levels[i] = 0;
        select[i] = 0u;

        if (s->enabled == 0u)           { continue; }
        if (!cfg_uid_to_bytes(s->uid, uid)) { continue; }
        if (!last_value_get(uid, &v))   { continue; }   /* 从没上报过: 不参与 */

        levels[i] = (int32_t)v.level_mm;
        select[i] = lsup_online(now_ms, v.rx_tick_ms, s->report_period_s) ? 1u : 0u;
    }

    /* ---- 3) 有效水位 = 参与集合里取 max ---- */
    if (!level_effective(levels, select, n, &eff))
    {
        return;     /* have_level 保持 false: 一台有效的都没有, fail-safe 交给调用方 */
    }
    out->have_level = true;
    out->level_mm   = eff;

    /* ---- 4) confirmCount 防抖 -> 已确认状态 ----
     * ★ 先存一份"迁移前"的状态 ★: level_eval_feed() 成功时会把 confirmed 覆盖成
     * 新状态, 之后就再也拿不到 from 了 —— 而产生 event.status_change 需要 from/to。 */
    {
        level_state_t before = s_ev.confirmed;

        if (level_eval_feed(&s_ev, &cfg->thresholds, eff, &out->state))
        {
            out->migrated = true;
        }
        out->prev_state = before;
    }
    out->state = s_ev.confirmed;
    out->gear  = level_state_to_gear(s_ev.confirmed);

    /* ---- 5) 档位模板渲染({level} -> 当前水位 cm 整数) ----
     * ★2026-10-08: 两段 LED 模板各渲一次★ */
    if (out->gear < (uint8_t)CFG_GEAR_COUNT)
    {
        (void)level_render_template(out->led_text1, (uint16_t)sizeof(out->led_text1),
                                    cfg->gears[out->gear].led_text1, eff);
        (void)level_render_template(out->led_text2, (uint16_t)sizeof(out->led_text2),
                                    cfg->gears[out->gear].led_text2, eff);
        (void)level_render_template(out->tts_text, (uint16_t)sizeof(out->tts_text),
                                    cfg->gears[out->gear].tts_text, eff);
    }

    /* ---- 6) 该不该下发(档位变了 / 文本变了 / 从没成功发过) ----
     * ★ manual 模式抑制【自动】联动(文档 3.7 + 5.5, 用户 2026-09-30 第 6 条) ★
     *   rules.mode == manual 时: 只算状态/档位/文案(照常上报 status、telemetry 能用),
     *   但【不派发】联动 —— 手动模式下设备状态由平台的 dev.ctrl 决定, 阈值变化不再
     *   自动改灯。以前这个字段根本没人读, manual 只是"记了个状态"。 */
    if (cfg->rules.mode == (uint8_t)CFG_RULE_MANUAL)
    {
        out->dispatch = false;
        out->auto_suppressed = true;      /* 让调用方能在日志里说清"为什么没下发" */
    }
    else
    {
        out->dispatch = level_dispatch_should_send(&s_disp, out->gear,
                                                   out->led_text1, out->led_text2);
    }

    /* ---- 7) 该不该生成一条水位类事件 ---- */
    out->event_due = level_event_should_fire(&s_gate, &cfg->thresholds, eff, now_ms);

    /* ---- 8) 记一份"最近有效结果", 供 dev.ctrl 渲染档位模板时取当前水位 ---- */
    s_last_mm   = eff;
    s_last_gear = out->gear;
    s_last_ok   = true;
}

bool level_supervisor_last(int32_t *out_level_mm, uint8_t *out_gear)
{
    if (!s_last_ok) { return false; }
    if (out_level_mm != NULL) { *out_level_mm = s_last_mm; }
    if (out_gear     != NULL) { *out_gear     = s_last_gear; }
    return true;
}
