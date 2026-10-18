/**
 * @file  app_state.c
 * @brief 编排层共享状态：缓存时钟 / 最近一次水位评估 / 周期换算
 */
#include "app_state.h"
#include "bsp_tim.h"        /* get_tick_ms */
#include "g4_at.h"          /* g4_at_get_clock */
#include "cloud_proto.h"    /* cloud_proto_calendar_to_unix / cloud_proto_state_name */

/* ==================== 缓存时钟 ====================
 * g4_at_get_clock() 是【阻塞调用】(最长 1s), 而主循环里 LoRa 每 1ms、4G 每 5ms
 * 都要轮询一次。如果每 30s 都去问模组, 就会把这两个节拍各压 1 秒。
 * 所以: 只在缓存过期(10 分钟)时才问一次, 平时用 tick 差值外推。
 * 模组不在网时 g4_at_get_clock() 会立刻返回 false, 不会白等。 */
static uint32_t s_epoch_base;      /* 上次问到的 Unix 秒 */
static uint32_t s_epoch_tick;      /* 问到的时刻(get_tick_ms) */
static bool     s_epoch_valid;

#define APP_EPOCH_STALE_MS   600000u   /* 10 分钟 */

uint32_t app_now_unix(void)
{
    uint32_t tick = get_tick_ms();

    if (s_epoch_valid)
    {
        return s_epoch_base + ((uint32_t)(tick - s_epoch_tick) / 1000u);
    }
    return tick / 1000u;            /* 还没问到模组时间: 退回开机秒数 */
}

void app_refresh_epoch(void)
{
    g4_clock_t clk;

    if (s_epoch_valid && ((uint32_t)(get_tick_ms() - s_epoch_tick) < APP_EPOCH_STALE_MS))
    {
        return;
    }
    if (g4_at_get_clock(&clk))
    {
        /* g4_at_get_clock() 给的是【北京时间】, 传 +28800 折成 UTC 的 Unix 秒 */
        uint32_t ts = cloud_proto_calendar_to_unix((uint16_t)(2000u + clk.year),
                                                   clk.month, clk.day,
                                                   clk.hour, clk.minute, 0u, 28800);
        if (ts != 0u)
        {
            s_epoch_base  = ts;
            s_epoch_tick  = get_tick_ms();
            s_epoch_valid = true;
        }
    }
}

void app_invalidate_epoch(void)
{
    s_epoch_valid = false;
}

bool app_epoch_valid(void)
{
    return s_epoch_valid;
}

/* ==================================================================
 * 最近一次边缘评估结果(给状态上报 / 遥测取【实时】档位与状态)
 * ==================================================================
 * 【为什么要缓存它】level_supervisor 只对外给 poll()/mark_sent()/mark_event(),
 * 没有"当前档位"的 getter。而 status.report 与 telemetry 都需要【实时】档位。
 * 以前这里是硬编码: st->mode="auto"、st->current_gear=0 —— 于是平台看到的档位
 * 永远是 0(哪怕现场已经红灯禁行)。2026-09-30 第 10 条一并修掉。
 * ★ 纯 RAM, 不写 Flash(用户第 14 条决定: 网关不会断电)。 */
static level_decision_t s_last_ld;
static bool             s_last_ld_valid = false;

void app_note_decision(const level_decision_t *ld)
{
    if (ld == NULL)
    {
        return;
    }
    s_last_ld       = *ld;
    s_last_ld_valid = true;
}

uint8_t app_live_gear(void)
{
    return s_last_ld_valid ? s_last_ld.gear : 0U;
}

uint8_t app_live_state(void)
{
    return s_last_ld_valid ? (uint8_t)s_last_ld.state : 0U;
}

const char *app_live_state_name(void)
{
    if (!s_last_ld_valid) { return "Normal"; }
    /* level_state_t 与 event_cache 的 EC_ST_* 数值完全一致(1..4), 直接复用同一份
     * 名字映射, 免得"同一个状态两处叫法不同"。 */
    return cloud_proto_state_name((uint8_t)s_last_ld.state);
}

/** 心跳/遥测共用的"周期秒数"取值(带兜底) */
uint32_t app_cfg_period_ms(uint16_t sec, uint32_t fallback_ms)
{
    if ((sec == 0U) || (sec > 3600U)) { return fallback_ms; }
    return (uint32_t)sec * 1000U;
}
