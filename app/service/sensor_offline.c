/**
 * @file  sensor_offline.c
 * @brief 液位传感器"失联"判定实现(见 sensor_offline.h)
 *
 * 【本文件只有纯逻辑】: 不 include bsp_tim.h / g4_at.h / cloud_proto.h, 不读硬件、
 * 不发网络 —— 所以 _host_test 可以直接把它编进去跑边界用例(打桩都不需要)。
 * 时间戳由调用方传进来, 这也是它能被主机测试的前提。
 */
#include "sensor_offline.h"
#include <string.h>

uint32_t sensor_offline_threshold_ms(uint16_t report_period_s)
{
    uint32_t p;

    /* 0 视为"未配置": 用兜底周期, 而不是把阈值算成 0(那会"永远失联") */
    p = (report_period_s == 0u) ? (uint32_t)SENSOR_OFFLINE_DEFAULT_PERIOD_S
                                : (uint32_t)report_period_s;

    /* 最大 65535 × 3000 = 196,605,000 < 2^32, 不会溢出 */
    return p * (uint32_t)(SENSOR_OFFLINE_PERIOD_MULT * 1000u);
}

bool sensor_offline_should_report(uint32_t now_ms, uint32_t last_report_ms,
                                  uint16_t report_period_s, bool was_online)
{
    /* 不在线就没什么"失联"可报:
     *   上电后从没上报过 -> 不是"失联", 是"从没上线";
     *   已经报过并置了 offline_reported -> 由调用方把 was_online 传 false, 不重复报。 */
    if (!was_online)
    {
        return false;
    }

    /* 【回绕安全】只做无符号差值比较。写成 now_ms >= last_report_ms + 阈值 的话,
     * tick 从 0xFFFFFFFF 翻回 0 的那一下会得出"还差得很远"的结论(漏报),
     * 或在别的写法下误报。 */
    return ((uint32_t)(now_ms - last_report_ms) >= sensor_offline_threshold_ms(report_period_s));
}

static sensor_offline_action_t sensor_offline_update_impl(bool *is_online,
                                                        bool *offline_reported,
                                                        uint32_t now_ms,
                                                        uint32_t last_report_ms,
                                                        uint16_t report_period_s,
                                                        bool got_report,
                                                        uint32_t *last_offline_event_ms)
{
    if ((is_online == NULL) || (offline_reported == NULL))
    {
        return SENSOR_OFFLINE_ACT_NONE;
    }

    /* ---- 情况一: 本帧就是这台设备的上报 ---- */
    if (got_report)
    {
        *is_online = true;

        if (*offline_reported)
        {
            *offline_reported = false;
            if (last_offline_event_ms != NULL) { *last_offline_event_ms = 0u; }
            return SENSOR_OFFLINE_ACT_REPORT_ONLINE;
        }
        return SENSOR_OFFLINE_ACT_NONE;
    }

    /* ---- 情况二: 已判失联, 还没等到它自己的新上报 ---- */
    if (*offline_reported)
    {
        *is_online = false;         /* ACK 可能把标志刷成 true, 这里纠正 */

        /* ★2026-10-27: 离线事件按 3xperiod 重复上报 ★
         * 传了 last_offline_event_ms 才启用; 没传(NULL)=旧行为"只报一次"。
         * 用无符号差值, 与阈值判据同一套回绕安全写法。 */
        if (last_offline_event_ms != NULL)
        {
            uint32_t thr = sensor_offline_threshold_ms(report_period_s);

            if ((uint32_t)(now_ms - *last_offline_event_ms) >= thr)
            {
                *last_offline_event_ms = now_ms;
                return SENSOR_OFFLINE_ACT_REPORT_OFFLINE;
            }
        }
        return SENSOR_OFFLINE_ACT_NONE;
    }

    if (sensor_offline_should_report(now_ms, last_report_ms, report_period_s, *is_online))
    {
        *is_online        = false;
        *offline_reported = true;
        if (last_offline_event_ms != NULL) { *last_offline_event_ms = now_ms; }
        return SENSOR_OFFLINE_ACT_REPORT_OFFLINE;
    }

    return SENSOR_OFFLINE_ACT_NONE;
}

sensor_offline_action_t sensor_offline_update(bool *is_online, bool *offline_reported,
                                             uint32_t now_ms, uint32_t last_report_ms,
                                             uint16_t report_period_s, bool got_report)
{
    return sensor_offline_update_impl(is_online, offline_reported, now_ms,
                                      last_report_ms, report_period_s, got_report, NULL);
}

sensor_offline_action_t sensor_offline_update_ex(bool *is_online, bool *offline_reported,
                                                uint32_t now_ms, uint32_t last_report_ms,
                                                uint16_t report_period_s, bool got_report,
                                                uint32_t *last_offline_event_ms)
{
    return sensor_offline_update_impl(is_online, offline_reported, now_ms,
                                      last_report_ms, report_period_s, got_report,
                                      last_offline_event_ms);
}

uint16_t sensor_offline_period_for_uid(const config_store_t *cfg, const char *uid_hex)
{
    uint8_t i;

    if ((cfg == NULL) || (uid_hex == NULL))
    {
        return SENSOR_OFFLINE_DEFAULT_PERIOD_S;
    }

    for (i = 0u; (i < cfg->sensor_count) && (i < (uint8_t)CFG_SENSOR_MAX); i++)
    {
        if (strcmp(cfg->sensors[i].uid, uid_hex) == 0)
        {
            /* 找到人了, 但周期没配(0) -> 仍然用兜底值, 不把阈值算成 0 */
            return (cfg->sensors[i].report_period_s == 0u)
                       ? (uint16_t)SENSOR_OFFLINE_DEFAULT_PERIOD_S
                       : cfg->sensors[i].report_period_s;
        }
    }

    /* 配置里没有这台设备: 用户口径是"3 倍上报周期", 没有周期就用兜底周期。
     * 【注意这只影响阈值, 不影响"要不要监控"】: 监控对象是 LoRa 从机表里的液位从机,
     * 与 sensors[] 里配了几台无关。 */
    return SENSOR_OFFLINE_DEFAULT_PERIOD_S;
}
