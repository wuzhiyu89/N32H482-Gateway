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

sensor_offline_action_t sensor_offline_update(bool *is_online, bool *offline_reported,
                                             uint32_t now_ms, uint32_t last_report_ms,
                                             uint16_t report_period_s, bool got_report)
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
            /* 之前报过"失联", 现在又活了 -> 补一条"恢复", 并清标志。
             * 只在这里清: 必须收到【真的上报】才算恢复(不能因为收到一个 ACK 就算)。 */
            *offline_reported = false;
            return SENSOR_OFFLINE_ACT_REPORT_ONLINE;
        }
        return SENSOR_OFFLINE_ACT_NONE;     /* 上电第一次上报: 不发"恢复" */
    }

    /* ---- 情况二: 别的设备在上报, 顺带检查这台 ---- */

    /* 已经报过失联、还没等到它自己的新上报 -> 不再重复报。
     * 这一条还挡住了"失联后它只回了个 ACK": 那时 _touch_slave 会把 is_online
     * 置回 true, 若不看 offline_reported, 每来一帧别人的上报就会重发一次失联。 */
    if (*offline_reported)
    {
        return SENSOR_OFFLINE_ACT_NONE;
    }

    if (sensor_offline_should_report(now_ms, last_report_ms, report_period_s, *is_online))
    {
        *is_online        = false;      /* status.report 的在线计数跟着变诚实 */
        *offline_reported = true;
        return SENSOR_OFFLINE_ACT_REPORT_OFFLINE;
    }

    return SENSOR_OFFLINE_ACT_NONE;
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
