/**
 * \file  main.c
 * \brief N32H482 4G + LoRa 网关一体机入口
 *
 * ============================ 整机数据流 ============================
 *
 *   [液位传感器/声光报警器] --LoRa--> 本板 --4G MQTT--> 云平台
 *                                      ^
 *   云平台 --4G MQTT(下行)--> 本板 --LoRa--> [从机]
 *
 *   上行: radio RxDone → lora_proto_poll → 帧解析(AA55+CRC16) → on_frame
 *           └─ 0x02 传感器上报 → 回 ACK(0x80) → data_cache_up_push
 *           └─ 0x80/0x81 从机 ACK → 从机状态 IDLE + 清下发缓存
 *         ★2026-10-08: 本行原写"4G 在线时由 lora_business_poll() 里的 data_cache_poll() 取缓存" —— 那条 hex 上行通道已整体删除, 现无此路径
 *         → 组 0x02 整帧 → hex → MQTT 发布, 【发成功了才出队】。
 *         (上行只有这一个出口, 不存在"立即发一遍 + 缓存再发一遍"的重复。)
 *
 *   下行: 4G URC(+MQTTURC: "publish") → g4_io 收包泵(按 <payload_len> 收定长二进制)
 *         → g4_urc 入队 → g4_urc_poll() → lora_mqtt_recv_cb
 *           └─ payload 是【一整个协议帧的二进制】(含 0x00/0x0A/0x0D, 不能当字符串),
 *              例: AA 55 01 11 00 01 21 00 01 01 07 05 00 0A 00 00 DC 05 98 E0
 *                  (⚠️ 2026-09 更正: 这帧的 CRC 原写作 38 1B, 是错的;
 *                   正确值 98 E0 —— 覆盖 ver..data 末尾, 低字节在前。)
 *           └─ 校验(帧头/版本/长度/CRC) → lora_business_send_req
 *                                        └→ 声光报警器常驻 RX: 立即下发
 *                                        └→ 液位传感器休眠:   等它上报时搭车下发
 *
 *   定时上行: 每 MAIN_GW_ONLINE_MS 往云端 pub 一条 "GW_ONLINE" 心跳,
 *             告诉云端本网关在线(见下面 (E) 那段)。
 *
 * ============================ 任务节拍 ============================
 *   本工程无 RTOS, 用 ATIM1 的 1ms 时基做分频调度, 主循环与 1ms 对齐:
 *     每 1ms   lora_business_poll()  radio 中断处理 + 帧分发 + ACK 超时/重传(内部 1s 限频)
 *     每 5ms   g4_at_poll()          收包泵 + 云端下行投递 + 非阻塞 AT 状态机 + 发布子流程
 *     每 10ms  app_console_poll()        日志口(USART1)手敲 AT 转发给模组(台架调试用)
 *     每 1s    app_status_report()       状态打印(PA8 心跳已取消: 那是 4G_RST)
 *     每 30s   在线心跳 "GW_ONLINE" + 命令行/缓存重投
 *
 * ============================ 声光报警器怎么下发 ============================
 *   【只在云端下发了 0x03 时才发】, 网关自己不会定时去叫它。
 *   以前上电会主动下发一次(MAIN_ALARM_BOOT_PUSH), 而且重传耗尽后缓存不删,
 *   于是 _try_send_alarm_idle() 每秒捡起来重发一遍 —— 空口被刷满、主循环被
 *   反复阻塞, 直接连累 4G 下行和液位上报。现在:
 *     - 去掉上电主动下发;
 *     - 重传 3 次没 ACK → 丢弃该条缓存并打日志(要重发由云端再下发)。
 *
 * ============================ 硬件资源 ============================
 *   USART1  PA9-TX  PA10-RX   日志(115200), 同时作 4G AT 手敲控制台
 *   USART2  PC8-TX  PC9-RX    4G 模组 ML307R AT 口(115200)
 *   UART6   PA0-TX  PA1-RX    RS232 (PA2-EN)
 *   USART4  PB0/PB1           485_0        UART7  PB11/PB10   485_1
 *   SPI3    PC10-SCK PC11-MISO PC12-MOSI  PA15-NSS(软)   LLCC68
 *           PD2-RST  PB3-BUSY  PB4-DIO1(EXTI4)  PB5-EN
 *   PA8     4G 模组【电源开关】(板上没引出 RST; "复位"=断一下电再上电)
 *           —— 开机第一件事就配成推挽输出并按当前极性上电
 *           ⚠️ 原"LED 心跳"已取消: 那会每秒翻转 PA8 = 每秒给模组断一次电
 *           极性未知 → g4_at.c 的供电极性自检会两个方向都试一遍
 *
 * ============================ 4G 建链顺序 ============================
 *   g4_at.c 的建链流程照搬现场验证过的 ml307_app.c/ml307_init:
 *     AT → ATE0 → 关深睡眠(MLPMCFG) → CPIN → IMEI(CGSN) → IMSI(CIMI)
 *     → ICCID → CSQ → 注册(CEREG/CGREG/CREG 轮换, 2s 一条共 30s)
 *     → APN(CGDCONT) → 激活 PDP(CGACT) → 查 IP(CGPADDR)
 *     → MQTTDISC → MQTTCONN(clientID=IMEI) → 等 conn URC → MQTTSUB → 在线
 *   要点: ① 主路是标准 3GPP 的 APN+PDP, AT+MDIALUP(USB/RNDIS 拨号)只做兜底;
 *         ② "拿不拿到 IP"是通不通的唯一判据, 不是 CGACT 回没回 OK;
 *         ③ MQTT clientID 用 IMEI, 多台网关不会互相顶下线。
 *   日志里关键行: "AT_SYNC: OK" / "[G4] PDP 已分配 IP: x.x.x.x" /
 *                 "===== MQTT ONLINE! ====="。
 *
 * ============================ 已知限制 ============================
 *   1) g4_at.c 目前只订阅一个主题(G4_MQTT_SUB_TOPIC), 云端下发必须发到该主题
 *      才能被 lora_mqtt_recv_cb 收到; OTA 的 ota/cmd、ota/data 主题尚未订阅。
 *   2) OTA 已改用 HTTP 通道(ota_http.c); 但当前是分区模式 1
 *      (FM_HAS_BOOTLOADER=0), 板上没有 bootloader, 校验通过后只能停在 READY,
 *      不能真把新固件装进运行区。要真安装: 切模式 2 + 先烧 BL。
 *   3) 【2026-09 结构性改版后】g4_at_poll() 内的 AT 收发已全部非阻塞, 接收由
 *      g4_io_pump() 统一消费; 仍然"阻塞"的只剩日志口手工诊断(#at/#vbat/#dial)
 *      和 g4_at_get_clock() —— 它们内部照样跑收包泵, 因此不会丢接收, 只会让
 *      主循环多等一会儿。
 *   4) 云端下发必须是【原始二进制帧】, 长度对上协议帧(data_len 在帧里)。若云端
 *      改发 ASCII hex 文本, g4_io 的收包器会按 <payload_len> 把 hex 文本的前一半
 *      当 payload —— 两种格式必须和云端约定死一种(见 g4_io.h 顶部)。
 *   5) USB 主机功能已停用并移出工程(archive/legacy/usb-host/); json_proto 同样
 *      已归档(archive/legacy/json-proto/)。
 *
 * ==================== 【最小 LoRa 收发隔离测试】MAIN_LORA_RAW_TEST ====================
 *   现场"网关收不到对端任何一帧"的排障用开关(默认见 app/drivers/lora/project_config.h):
 *     1 = main() 只起 时基/日志/射频, 每 5 秒发两条测试帧, 其余时间一直连续接收,
 *         并在【射频 IRQ 寄存器层】把收到的原始字节打出来(见本文件末尾 app_mode_rawtest_run()
 *         与 app/drivers/lora/radio.c 的 [RAW-IRQ]/[RAW-RX]); 4G / 业务 / 协议层 /
 *         参数 Flash / 心跳 全部不执行。切回正常固件: 把那个开关改回 0 重新编译,
 *         正常流程一行都没删(整体放在 main() 的 #else 里)。
 *     0 = 正常固件(与加这个开关之前完全一致)。
 */
#include "n32h47x_48x.h"
#include "n32h47x_48x_gpio.h"
#include "n32h47x_48x_rcc.h"
#include "n32h47x_48x_usart.h"
#include "misc.h"
#include <string.h>
#include <stdio.h>      /* snprintf: 状态行里拼模组自测供电(VBAT) */

#include "log.h"
#include "bsp_tim.h"
#include "bsp_uart.h"
#include "ring_buf.h"
#include "utils.h"

#include "local_alarm.h"
#include "param_save.h"
#include "n32h47x_48x_dbg.h"   /* 2026-10-18: GetUID() —— 读芯片 96 位唯一 ID */
#include "data_cache.h"
#include "bsp_gpio.h"   /* G4_PowerSwitch_Init(): 4G 模组电源开关 PA8 */

#include "g4_io.h"
#include "g4_at.h"
#include "g4_urc.h"

#include "lora_proto.h"
#include "lora_business.h"
#include "serial_proto.h"
#include "ota_http.h"     /* OTA over HTTP(S): AT+MHTTP* + MD5, 见 app/ota/ota_http.c */
#include "ota_port.h"     /* ota_port_reboot(): 校验通过后交给 bootloader 搬运 */
#include "ota_health.h"   /* 2026-10-11: A/B 回滚判定+计数(开机 boot_step / 主循环 poll) */
#include "cjson_port.h"      /* cJSON 入口: 静态池分配器 + hooks(见 app/cjson/cjson_port.h) */
#include "app_config.h"      /* 功能开关(唯一默认值源) */
#include "app_state.h"       /* 共享运行时状态(时钟/最近档位) */
#include "app_core.h"        /* 编排层内部接口(控制台/状态/下行/上行) */

#include "cloud_proto.h"     /* P1-2 信封/路由 + status.report 数据段 */
#include "last_value.h"      /* v53: sen.query/dev.query 的"最近一次上报"时刻 */
#include "level_supervisor.h"/* v54: 边缘水位评估的取数+串接层 */
#include "event_cache.h"     /* v69: 断线期间把事件存起来, 重连后补报 */
#include "telemetry_cache.h" /* ★2026-10-18: 断线期间把【每次液位上报】存起来, 重连后补报 */
#include "cloud_profile.h"   /* P1-0 本地/文档 档位(topic / QoS / retained) */
#include "config_store.h"    /* P2-4: cfg.query 应答要用 CFG_JSON_MAX 定缓冲 */
#include "sensor_offline.h"  /* 液位失联判定(#offline 诊断用; 事件发布在下面) */

/* MAIN_LORA_RAW_TEST(最小 LoRa 收发隔离测试开关)的真相源。
 * 它的【本体与说明】写在 app/drivers/lora/project_config.h: 那个头是
 * main.c / lora_proto.c / radio.c 三个编译单元都会包含的射频工程配置头,
 * 开关放那里才能被三处同时看到(C 的宏不跨 .c 传递)。
 * 这里显式包含一次, 免得下面的 #if 依赖别的头文件的包含顺序。 */
#include "project_config.h"

/* ---- 【只在最小收发测试模式下需要】射频层的只读回读接口 ----
 * 放在 #if 里: 开关=0 时这几个头一行都不引入, 保证正常固件的目标码
 * 与加这个开关之前【逐字节等价】(见文件末尾 app_mode_rawtest_run() 与 main())。 */
#if MAIN_LORA_RAW_TEST
#include "radio.h"          /* RadioIrqStat()/RadioGetRxContinuous()/LLCC68GetOperatingMode() */
#include "llcc68-board.h"   /* LLCC68BusyLevel()/LLCC68BusyTimeoutCount()/LLCC68IsStuck() */
#include "llcc68.h"         /* LLCC68GetStats()/GetRssiInst()/GetStatus()/GetDeviceErrors() */
#endif

/* ==================== 功能开关 ====================
 * 都写成 #ifndef, 这样也能在 Keil 的 C/C++ -> Define 里覆盖,
 * 不用为了改一个开关去动代码。 */
#ifndef MAIN_SERIAL_INIT_ENABLE
#define MAIN_SERIAL_INIT_ENABLE  1   /* 初始化 RS232/RS485(协议待定, 仅建通道) */
#endif
/* OTA 只有一条通道: over HTTP(S)(ota_http.c), 由 cloud_proto 的 ota.start 驱动。
 * 旧的 MQTT 分包版 ota.c 已于 2026-10-18 移入 archive/legacy/ota-mqtt/。
 * 这里只留进度上报的节拍参数。 */
/* ⚠️⚠️ PA8 在新板上是 4G 模组的【电源开关】(不是 LED, 也不是 RST):
 * 打开这个开关 = 主循环每 1s 翻转一次 PA8 = 每 1~2s 给 4G 模组断一次电,
 * 模组永远启动不完, 现象就是"AT_SYNC 一直重试、模组一个字节都不回"。
 * 所以默认必须是 0。要心跳灯请另找一根空闲脚。 */
#ifndef MAIN_HEARTBEAT_LED
#define MAIN_HEARTBEAT_LED       0
#endif

/* ---- P1-8: 心跳载荷改成文档要求的 status.report JSON ----
 * 1(默认) = 周期心跳发 {"event":"status.report",...} JSON
 *           (本地档位下发到 rt/gw/{gwId}/status, 所以不用换 broker 就能在云上看到;
 *            文档档位下发到 rt/gw/{gwId}/status)
 * 0       = 退回原来的裸字符串 "GW_ONLINE"
 *
 * 【注意这是可见的改变】云端侧原本收到的是 9 字节的 "GW_ONLINE"; 打开后收到的是
 * JSON。两者语义等价(JSON 里含 "online":true), 但如果云端有硬编码判断, 就把它关掉。
 */

/* 上报给云端的固件版本(status.report 的 fwVersion 字段) */

/* 注: 这里只用 MAIN_STATUS_JSON_ENABLE 当守卫。
 * MAIN_G4_ENABLE / MAIN_GW_ONLINE_ENABLE 是在本文件【更靠后】才 #define 的,
 * 在这里用它们做条件会恒为 0, 整个函数会被编译掉(踩过一次)。 */

/* 4G AT/MQTT 状态机开关。
 *
 * ⚠️ 4G 模组没接 / 没调通时把它改成 0:
 *   1) 日志会被 [G4_TX]/[G4_RX] 刷满: AT_SYNC 每 500ms 敲一次, 敲满
 *      G4_AT_SYNC_RETRY_MAX(默认 60 次 = 30s) 还不回就转 4G_RST 断电重启模组,
 *      重启后重新预热再敲 —— 模组一直不理人时这个循环会持续。
 *   2) 关掉后 4G 相关代码一行都不跑, 只保留 LoRa。
 *
 * 【注意】现在 g4_at_poll() 里的 AT 收发是【非阻塞】的(每次只推进一步),
 * 模组不应答也不会再把主循环按到 ~1Hz —— 那条"阻塞拖慢 LoRa"的顾虑没有了。
 * (只剩日志口手工诊断 #at/#vbat/#dial 和 g4_at_get_clock() 还是阻塞封装,
 *  它们内部照常跑收包泵, 不会再丢接收。) */

/* ==================== LoRa 射频开关 ====================
 *
 * ⚠️ 为什么要做成开关: LoRa 模组一旦不应答(BUSY(PB3) 一直是高), 射频层每条 SPI
 * 命令都会卡在等 BUSY。原来的等 BUSY 是【死等】, 于是 lora_business_init() 永远
 * 返回不了, main() 根本走不到 g4_at_init() —— 【LoRa 坏了会把 4G 一起拖死】,
 * 日志里只剩一秒钟一行 "wait busy pin timeout", 4G 那边一点动静都没有。
 *
 * 现在等 BUSY 已经有上限(见 LLCC68N32H482-board.c), 默认不会再拖死主循环;
 * 这个开关是"彻底绕开 LoRa"的保险:
 *   1 = 正常: 初始化 LoRa + 主循环跑 lora_business_poll()(当前默认)
 *   0 = 只跑 4G: 完全不碰射频(不 init、不 poll), 专门用来单独调 4G 模组。
 *       ⚠️ 此时 param_save_init()(Flash 参数/gw_uid)也不会执行, 业务参数是默认值。
 *
 * 【2026-09 打开 LoRa 的原因】4G 那边已经稳定(心跳 + 下行都通), 现在加空口:
 *   · LoRa 常态【一直在收】(lora_proto_init -> LoraApp_StartRxContinuous),
 *     收到什么帧都在日志里打原始字节([LoRa RX] size=.. data:), 不挑格式;
 *   · 云端 MQTT 下发一条消息 -> 回调 lora_mqtt_recv_cb -> 【原样透传】到空口
 *     (见 lora_business.c 的 LORA_DOWNLINK_PASSTHROUGH, 默认 1: 不校验帧格式)。
 */

/* ==================== 隔离测试开关: 开机要不要给 4G 模组上电 ====================
 *
 * 【为什么需要】: "模组一上电, 板子就自己重启"这种问题必须二分定位 ——
 * 到底是 4G 那边的浪涌, 还是 LoRa 那边的。两个开关组合一下就够了:
 *
 *   MAIN_G4_POWER_AT_BOOT   MAIN_LORA_ENABLE     说明
 *        1(默认)                1(默认)          正常
 *        0                      1                只给 LoRa 上电 → 还重启 = 不是 4G
 *        1                      0                只给 4G 上电  → 还重启 = 不是 LoRa
 *        0                      0                两个都不上电 → 还重启 = 跟模组无关,
 *                                               (那就要怀疑代码/复位电路本身)
 *
 * 注意: 0 只是"不上电", PA8 依然会配成推挽输出并驱到【断电】电平(不会浮空)。
 */
#ifndef MAIN_G4_POWER_AT_BOOT
#define MAIN_G4_POWER_AT_BOOT    0
#endif

/* ==================== 主功能模式(三选一, 2026-09-27 起) ====================
 *
 * 把 main() 按用途分成三种, 用宏切换(同一个 .uvprojx, 不必改工程文件 ——
 * 在 Keil 的 Options for Target → C/C++ → Define 里加一个宏即可):
 *
 *   MAIN_MODE_CLOUD      1 = 4G + LoRa + 云端联调   ← 现场默认(现状行为)
 *   MAIN_MODE_PAIRING    1 = 从机配对模式           ← 从机发配对请求, 网关回 ACK
 *   MAIN_MODE_LORA_TEST  1 = LoRa 收发测试           ← 每 5s 发一帧(报文可改), 其余时间收
 *
 * 【为什么用"模式"而不是再加一堆开关】
 *   三种模式的初始化与主循环差异很大(4G 上不上电 / 有没有云协议层 / 主循环里跑什么),
 *   用"模式"表达比"开关组合"清楚; 以后要加第四种也只需再加一行。
 *
 * 【向后兼容】老的 MAIN_LORA_RAW_TEST=1 等价于 MAIN_MODE_LORA_TEST=1
 *   (v30 起的"LoRa 最小收发隔离测试固件"就是它在编)。
 *
 * 【编译期自检】三个宏必须恰好选一个 —— 现场最容易出的事故是"以为在跑 A, 实际编的是 B";
 *   多选/少选在这里直接 #error, 编不过就不会误烧进板子。
 */
/* ★ 2026-09-28: CLOUD 改为【兜底模式】★
 * 顺序很重要: 先让 PAIRING / LORA_TEST 取默认值, 再据此决定 CLOUD 的默认值。
 * 这样在 Keil 里【只加一个新宏】(例如 MAIN_MODE_PAIRING=1) 就能切过去,
 * 不必再手动把 MAIN_MODE_CLOUD 改成 0 —— 之前那样"只加新宏"会撞下面的 #error,
 * 现场表现就是"设了模式却没进模式"(2026-09-28 用户实际踩到)。
 *
 * 当前三个默认值 = 全 0(只由 CLOUD 兜底为 1) —— 即【不加任何宏 = 云联调模式】,
 * 与现场生产行为一致。切模式请在 Keil → Options for Target → C/C++ → Define 里加:
 *     MAIN_MODE_LORA_TEST=1     只跑 LoRa 收发测试
 *     MAIN_MODE_PAIRING=1       从机配对模式
 */

/* ★★★ 这里【不是】模式开关 —— 不要再在本文件里写 #define MAIN_MODE_xxx ★★★
 *
 * 模式宏的唯一真相源是 app/drivers/lora/project_config.h(经 lora_proto.h 在本文件
 * 前面就被包含了)。写在这里【一律无效】, 两个原因:
 *   1) project_config.h 早在本文件之前就用 #ifndef 把它们定好了;
 *   2) radio.c / lora_proto.c 是【各自单独编译】的, 根本看不到本文件里的定义 ——
 *      写在这里会出现"主流程进了测试模式, 射频层的测试代码却没被编译"的不一致。
 * 2026-09-28 现场踩过两次(v100: 分发看老宏; v103: 新宏写在 main.c), 所以现在改成:
 *   本文件只留【说明】+ 一条【兜底自检】, 开关本身全部放在 project_config.h。
 *
 * 【怎么切模式】改 project_config.h 里那三个 0/1, 或在 Keil → Options for Target →
 *   C/C++ → Define 里加(工程 Define 优先):
 *       MAIN_MODE_PAIRING=1      从机配对模式
 *       MAIN_MODE_LORA_TEST=1    LoRa 收发测试
 *       MAIN_MODE_CLOUD=1        云端联调(不加任何宏也是它, 因为 CLOUD 是兜底)
 *   ✅ 只加/只改【一个】宏即可 —— 另两个自动让开(CLOUD 兜底 + 老宏自动映射)。
 *   ✅ 多选会【编译报错】(三选一自检), 不会静默进错模式。
 *
 * 本处自检只是兜底保险: 真正的校验已放在 project_config.h 里, 任何包含它的编译
 * 单元都会更早、更明确地报出来。 */
#if ((MAIN_MODE_CLOUD + MAIN_MODE_PAIRING + MAIN_MODE_LORA_TEST) != 1)
#error "主功能模式必须恰好选一个: 见 app/drivers/lora/project_config.h 的『主功能模式(三选一)』"
#endif

/* ==================== 网关在线心跳 ====================
 *
 * ★现在恢复开启(默认 1)★ —— 原因很实际:
 * 之前把它关掉、只靠"收到云端消息回 ACK"之后, 一旦【下发这条路出问题】
 * (解析卡住 / 模组截断 / 链路异常), 就既没有 ACK 也没有心跳, 云端那边看着这台
 * 设备彻底死了, 连"它还在不在"都判断不了。
 * 所以两条一起留, 缺一个都会在故障时失去可观测性:
 *   · 定时心跳(GW_ONLINE, 每 30s) → 云端【始终知道设备活着】, 与下发无关;
 *   · 收到即回执(ACK)              → 云端【知道指令送达了】。
 *
 * 只想留回执、不要定时心跳: 改成 0(不建议, 见上)。 */
#ifndef MAIN_GW_ONLINE_ENABLE
#define MAIN_GW_ONLINE_ENABLE    1
#endif
#ifndef MAIN_GW_ONLINE_MS
/* 心跳周期的【兜底值】。真正的周期来自配置 network.statusIntervalSeconds
 * (2026-09-30 第 5 条: 以前只用这个宏, 于是云平台改了配置一点反应都没有)。
 * 读不到/越界时才回落到这个值。 */
#define MAIN_GW_ONLINE_MS        30000U
#endif

/* ==================== 主循环节拍 ==================== */








/* ==================== main ==================== */
/* ★2026-10-18: 把厂商的 GetUID() 包装成 param_save 要的 12 字节形式。
 * GetUID(uint8_t*) 填 12 字节(96 位, 3 个 32 位字小端) —— 正好是我们要的。 */
static void main_read_chip_uid(uint8_t out[12])
{
    GetUID(out);
}

int main(void)
{
    /* ★2026-10-18: 必须在任何 param_save 使用之前注册 —— 出厂默认的 gw_uid
     * 就是靠这个 hook 填入芯片 96 位 UID 的(main.c 拿不到 UID 的话,
     * 网关发出的帧里 gw_uid 后 12 字节会是 0, 设备端会认为不是本网关)。 */
    param_save_set_uid_hook(main_read_chip_uid);
    /* 2026-10-18 (甲): 出厂 gwId / clientId 也用它 => 26 位 hex(主题 rt/gw/<26位>/... ) */
    config_store_set_uid_hook(main_read_chip_uid);
#if (MAIN_MODE_LORA_TEST != 0) || (MAIN_MODE_PAIRING != 0)
    /* ★ 2026-09-28: 把"4G 断电"提到【main 的第一件事】★
     * 原因(见 bsp_gpio.h:155): MCU 复位后 PA8 是【输入悬空】, 而本板【悬空 = 给模组上电】
     *   (现场已确认 PA8 低=上电 / 高=断电)。所以等到 app_mode_rawtest_run()/app_mode_pairing_run() 里
     *   再调 G4_PowerSwitch_InitNoPower() 其实已经晚了 —— 前面 bsp_tim1_init_1ms()、
     *   log_init() 与好几行日志(115200bps, 每行毫秒级)的这段时间里, 模组早已被通电启动。
     * 纯软件只能把这个窗口缩到最短; 要彻底消除需在 PA8 上加【上拉电阻】(高=断电)。
     * 各模式内原来那次调用保留(重复无害, 也保证单独调用该模式函数时同样安全)。 */
    G4_PowerSwitch_InitNoPower();
#endif

#if MAIN_MODE_PAIRING
    /* ---- 【从机配对模式】只起 时基/日志/射频, 然后进配对循环(永不返回) ---- */
    app_mode_pairing_run();
#elif MAIN_MODE_LORA_TEST
    /* ---- 【LoRa 收发测试模式】只起 时基/日志/射频, 然后进测试循环(永不返回) ----
     * ⚠️ 这里用【新宏 MAIN_MODE_LORA_TEST】而不是老宏 MAIN_LORA_RAW_TEST:
     *    两者虽然由 project_config.h 映射成一致, 但分发处统一用新宏才不会再出现
     *    "上面的 #if 看新宏、下面的 #elif 看老宏"那种各看各的坑(v100 就是这么错的)。
     * 正常固件的主流程【一行都没删】, 它整体在 #else 里: 开关=0 时逐字参与编译,
     * 开关=1 时完全不参与(4G 初始化 / param_save / 业务轮询 / 心跳 一次都不跑)。 */
    app_mode_rawtest_run();
#else
    /* 正常固件：按顺序初始化，然后进主循环（两个函数都不返回） */
    app_boot_run();
    app_loop_run();
#endif /* MAIN_LORA_RAW_TEST */
}
