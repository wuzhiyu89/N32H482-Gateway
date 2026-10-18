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

#ifndef MAIN_FW_VERSION
#define MAIN_FW_VERSION          "1.0.0"
#endif

/* 其余编译期参数（尽量少：能进 config_store 的就别编译死） */
#ifndef MAIN_OTA_PROGRESS_MS
#define MAIN_OTA_PROGRESS_MS     10000u  /* ota.progress 最长静默期(文档 7.6: 10% 或 10s) */
#endif

/* 主循环节拍 / 通信参数（所有编排单元共用） */
#define TICK_G4_POLL_MS          5U
#define TICK_CONSOLE_MS          10U
#define TICK_STATUS_MS           1000U
#define MAIN_SERIAL_BAUD         115200U

/* 串口 / LED / 4G 上电 / 周期心跳 */
#ifndef MAIN_SERIAL_INIT_ENABLE
#define MAIN_SERIAL_INIT_ENABLE  1   /* 初始化 RS232/RS485(协议待定, 仅建通道) */
#endif
#ifndef MAIN_HEARTBEAT_LED
#define MAIN_HEARTBEAT_LED       0   /* PA8 是 4G 电源脚, 必须 0 */
#endif
#ifndef MAIN_G4_POWER_AT_BOOT
#define MAIN_G4_POWER_AT_BOOT    0   /* 只控"提前上电+浪涌错峰"; 4G 最终由 g4_at_init 上电 */
#endif
#ifndef MAIN_GW_ONLINE_ENABLE
#define MAIN_GW_ONLINE_ENABLE    1
#endif
#ifndef MAIN_GW_ONLINE_MS
#define MAIN_GW_ONLINE_MS        30000U  /* 兜底值; 实际周期取配置 network.statusIntervalSeconds */
#endif

#endif /* APP_CONFIG_H */
