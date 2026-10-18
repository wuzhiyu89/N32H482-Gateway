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
#include "g4_at.h"      /* sys_err_t */

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

/* ---------------- 上行 --------------------------------------------------- */
/** 组一条 status.report 并按档位发布（心跳 / #status 共用） */
sys_err_t app_publish_status_json(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_CORE_H */
