/**
 * @file  sensor_offline.h
 * @brief 液位传感器"失联"判定 —— 纯逻辑(不碰硬件/不碰网络)
 *
 * 依据: 用户 2026-09-27 补充要求第 3 条(原文):
 *   "液位传感器最多三个；不用断言还在，只用记录上报时间；
 *    每次一个液位上报数据，就把所有液位从机的上次上报时间拿出来对比，
 *    如果有很久没上报的（比如 3 倍上报周期没上报），就向云端发送传感器失联。"
 *
 * 【为什么要单独一个文件】
 *   判定规则(阈值/边界/回绕/跳变去重)是整件事里唯一"容易算错又难在现场看出"的部分,
 *   而它【完全可以不依赖硬件】。把它拆出来后:
 *     · 主机上能一次跑完边界用例(刚好 3 倍 / 差 1ms / tick 回绕 / 不重复报);
 *     · lora_business.c 里只剩"读状态 → 调这里 → 发事件"这几行胶水, 不掺算法。
 *
 * 【时间戳一律是 get_tick_ms() 的毫秒 tick】
 *   它会回绕(uint32 最大约 49.7 天), 所以比较【必须】用无符号差值
 *   (uint32_t)(now - last), 绝不能写成 now > last + 阈值 —— 回绕那一下会漏判/误判。
 *   本模块内部全部用差值写法。
 */
#ifndef __SENSOR_OFFLINE_H__
#define __SENSOR_OFFLINE_H__

#include <stdint.h>
#include <stdbool.h>
#include "config_store.h"      /* cfg_sensor_t / config_store_t(old 指针查上报周期) */

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 判定口径 ==================== */

/** 失联阈值 = 该传感器上报周期 × 这个倍数(用户原话: "比如 3 倍上报周期没上报") */
#define SENSOR_OFFLINE_PERIOD_MULT       3U

/**
 * config_store 里【查不到该 UID】或该传感器【reportPeriodSeconds = 0】(视为未配置)时
 * 兜底用的上报周期(秒), 阈值 = 60 × 3 = 180 秒。
 *
 * 为什么是 60:
 *   · 文档《网关配置项设计》3.3 与 config_store 的出厂默认(cfg_sensor_defaults)
 *     给传感器的 reportPeriodSeconds 默认值就是 60s —— 用文档默认值兜底, 不会有
 *     "同一个传感器在两个地方得到两个周期"的问题;
 *   · 兜底值取太小 → 传感器只是晚到一点就误报失联; 取太大 → 真失联迟迟不报。
 *     60s 与出厂默认一致, 属于"最不意外"的选择。
 *   · 宁可偏保守(不轻易误报), 因为误报"失联"会让平台做无谓的工单。
 */
#define SENSOR_OFFLINE_DEFAULT_PERIOD_S  60U

/* ==================== 判定动作 ==================== */

/** 一次判定后需要对外做的事(由调用方决定怎么发事件) */
typedef enum
{
    SENSOR_OFFLINE_ACT_NONE = 0,        /* 什么都不用做 */
    SENSOR_OFFLINE_ACT_REPORT_OFFLINE,  /* 在线 -> 失联: 上报一次"失联"(只在跳变时) */
    SENSOR_OFFLINE_ACT_REPORT_ONLINE    /* 已报失联 -> 又收到上报: 上报一次"恢复" */
} sensor_offline_action_t;

/* ==================== 阈值 / 判定(纯函数) ==================== */

/**
 * @brief 失联阈值(毫秒) = 上报周期 × 3
 * @param report_period_s 该传感器的上报周期(秒); 0 = 未配置 -> 用 SENSOR_OFFLINE_DEFAULT_PERIOD_S
 * @note  最大 65535s × 3000ms = 1.97e8 < 2^32, 不会溢出。
 */
uint32_t sensor_offline_threshold_ms(uint16_t report_period_s);

/**
 * @brief 【单点判据】当前是否已达到"失联"条件
 * @param now_ms        当前 tick(get_tick_ms())
 * @param last_report_ms 该传感器上次 0x02 上报的 tick
 * @param report_period_s 该传感器的上报周期(秒); 0 = 未配置 -> 用默认
 * @param was_online     判定前它是不是"在线"。false 直接返回 false —— 这一条同时
 *                       覆盖两种情况(所以"不重复发"是判据的一部分, 不是调用方的事):
 *                         a) 上电后还没上报过(本来就不在线, 不该报"失联");
 *                         b) 已经判过失联并已上报, 还没等到新的上报。
 * @return true = 应当上报"失联"; 恰好在 3 倍那一刻算【已失联】(>=)。
 * @note   回绕安全: 内部用 (uint32_t)(now_ms - last_report_ms) 比较, 不做大小比较。
 */
bool sensor_offline_should_report(uint32_t now_ms, uint32_t last_report_ms,
                                  uint16_t report_period_s, bool was_online);

/**
 * @brief 【状态机】失联/恢复判定的唯一入口: 就地更新状态并给出该做什么
 *
 * 语义(与 lora_business.c 的用法一一对应):
 *   · got_report = true (本帧就是这台设备的 0x02 上报):
 *       置在线; 若之前报过失联 -> 返回 REPORT_ONLINE(恢复), 并清掉"已报失联";
 *       从未报过失联 -> NONE(上电第一次上报不该发"恢复")。
 *   · got_report = false:
 *       只有"在线 且 还没报过失联"才可能跳变成失联; 一旦报过就 NONE ——
 *       这就是"同一个传感器失联后不要每收一条别人的上报就重复发一次"的落点。
 *
 * @param is_online        输入/输出: 在线标志(调用方持有, 例如 lora_slave_t.is_online)
 * @param offline_reported 输入/输出: 是否"已上报过失联、还没等到新上报"
 * @param now_ms           当前 tick
 * @param last_report_ms   该传感器上次 0x02 上报的 tick(本函数不改它, 由调用方在收到上报时更新)
 * @param report_period_s  该传感器的上报周期(秒); 0 = 未配置 -> 用默认
 * @param got_report       本帧是否为这台设备的 0x02 上报
 * @return 需要对外做的动作
 */
sensor_offline_action_t sensor_offline_update(bool *is_online, bool *offline_reported,
                                             uint32_t now_ms, uint32_t last_report_ms,
                                             uint16_t report_period_s, bool got_report);

/**
 * @brief 按 UID 从配置里查该液位传感器的上报周期(秒)
 * @param cfg     配置(可为 NULL, 返回默认值)
 * @param uid_hex 6 字符 hex 串, 如 "210001"
 * @return reportPeriodSeconds; 查不到该 UID / 该字段为 0 / 入参非法 -> SENSOR_OFFLINE_DEFAULT_PERIOD_S
 */
uint16_t sensor_offline_period_for_uid(const config_store_t *cfg, const char *uid_hex);

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_OFFLINE_H__ */
