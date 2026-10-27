/**
 * @file  app_core.h
 * @brief 编排层内部接口（app/core/*.c 之间互相调用）
 *
 * 只给 app/core 内部用；其它层不要 include 它。
 * 依赖方向：core -> cloud / links / service / model，反向不成立。
 */
#ifndef APP_CORE_H
#define APP_CORE_H

#include <stdint.h>
#include <stdbool.h>

#include "g4_at.h"            /* sys_err_t */
#include "cloud_proto.h"      /* cloud_status_t */
#include "level_supervisor.h" /* level_decision_t */
#include "lora_business.h"    /* lora_bind_event_t / LORA_UID_BYTES */

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- 日志口（USART1，115200）：控制台 + 周期状态 ------------- */
void app_console_init(void);            /* 打开 USART1 接收中断 */
void app_console_poll(void);            /* 主循环每 10ms：拼行 + 分发 # 命令 */
bool app_console_ext(const char *line); /* 扩展命令；返回 true = 已处理 */
void app_console_help(void);            /* 打印扩展命令列表 */
void app_status_report(void);           /* 每秒一行状态（[STAT]） */
void app_report_reset_cause(void);      /* 开机：打印上次复位原因 */

/* ---------------- 下行：云端 JSON 命令 / 原始字节 ------------------------- */
/**
 * @brief 处理一条下行：JSON 走云协议层，二进制帧走 LoRa 透传
 * @return true = 已处理（调用方不要再往下传）
 */
bool app_downlink_try(const char *topic, const uint8_t *data, uint16_t len);

/** LoRa 关闭时的 MQTT 下行回调：JSON 照走云协议，其余只打印 + 回执 */
void app_downlink_log_cb(const char *topic, const uint8_t *data, uint16_t len);

/* ---------------- 上行（status / telemetry / event / ota.progress） -------- */
/** 组一条 status.report 并按档位发布（心跳 / #status 共用） */
sys_err_t app_publish_status_json(void);

/** 填一份网关状态快照（sys.query_status 的 provider，也供 status.report 用） */
void app_fill_status(cloud_status_t *st);

/** 每秒：水位遥测（周期 / 变化触发） */
void app_telemetry_step(void);

/** 每秒：断线期间缓存的逐条液位上报，一次补一条 */
void app_telemetry_backfill_poll(void);

/** 每秒：断线期间缓存的事件，一页最多 16 条 */
void app_backfill_poll(void);

/** 每秒：OTA over HTTP 的进度上报 / 就绪提示 */
void app_ota_progress_poll(void);

/** sys.time_sync 的钩子：向 4G 模组要时间 */
bool app_time_sync(uint32_t unix_ts);

/* ---- 注册给 links/lora 的事件回调 ---- */
void app_on_level_report(const uint8_t uid[LORA_UID_BYTES],
                         uint16_t level_mm, uint8_t bat_pct, int8_t rssi_dbm);
void app_on_sensor_event(const char *uid_hex, bool online);

/* ★2026-10-22: 液位低电量 -> event.device / low_voltage */
void app_on_sensor_low_battery(const char *uid_hex, uint8_t bat_pct);
void app_on_field_result(bool ok, const char *field10);
void app_publish_bind_event(const char *uid_hex, bool is_sensor,
                            lora_bind_event_t event);
void app_on_alarm_result(const char *uid_hex, uint8_t gear,
                         bool ok, bool gave_up, bool recovered);

/* ---- 水位状态迁移（主循环直接调） ---- */
void app_cache_status_change(uint8_t from_code, uint8_t to_code,
                             uint8_t gear, int32_t level_mm);
void app_publish_status_change(uint8_t from_code, uint8_t to_code,
                               uint8_t gear, int32_t level_mm);

/* ---------------- 启动与主循环（都不返回） -------------------------------- */
void app_boot_run(void);   /* 上电初始化序列 */
void app_loop_run(void);   /* 主循环，永不返回 */

/* ---------------- 工程模式（不返回） -------------------------------------- */
void app_mode_rawtest_run(void);   /* MAIN_MODE_LORA_TEST */
void app_mode_pairing_run(void);   /* MAIN_MODE_PAIRING  */

/* ---------------- 本地自治（水位 -> 联动） -------------------------------- */
void app_failsafe_step(const level_decision_t *ld);   /* 全部失联 fail-safe */
void app_event_action_failsafe(void);
bool app_level_off_failsafe_active(void);              /* off 策略的故障画面期 */
void app_manual_step(void);                            /* manual 窗口到期回归 */

#ifdef __cplusplus
}
#endif

#endif /* APP_CORE_H */
