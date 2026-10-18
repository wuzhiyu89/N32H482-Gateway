/**
 * @file  app_config.h
 * @brief 编译期功能开关（默认值）
 *
 * 规则：
 *   1. 每个开关都写成 `#ifndef`，这样也可以在 Keil 的
 *      Options for Target → C/C++ → Define 里覆盖，不必改源码；
 *   2. 改默认值只改这里 —— 编排层的 .c 不再各自定义同一批开关；
 *   3. 射频/模式的开关不在这里：那三个模式宏在
 *      app/drivers/lora/project_config.h（必须被射频层同时看到）。
 *
 * 说明：本文件只放"编不编"的开关，不放可调参数（周期、超时）——
 *       可调参数应尽量来自 config_store，而不是重新编译。
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 日志口手敲 AT 转发（台架调试用，量产可关） */
#ifndef MAIN_AT_CONSOLE_ENABLE
#define MAIN_AT_CONSOLE_ENABLE   1
#endif

/* 心跳/事件用 status.report JSON 信封；0 = 退回裸字符串 "GW_ONLINE" */
#ifndef MAIN_STATUS_JSON_ENABLE
#define MAIN_STATUS_JSON_ENABLE  1
#endif

/* 4G 链路总开关（调 4G 时可单独关掉射频，反之亦然） */
#ifndef MAIN_G4_ENABLE
#define MAIN_G4_ENABLE           1
#endif

/* LoRa 链路总开关 */
#ifndef MAIN_LORA_ENABLE
#define MAIN_LORA_ENABLE         1
#endif

#endif /* APP_CONFIG_H */
