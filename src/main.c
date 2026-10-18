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
#define TICK_G4_POLL_MS          5U
#define TICK_CONSOLE_MS          10U
#define TICK_STATUS_MS           1000U
#define MAIN_SERIAL_BAUD         115200U








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
    uint32_t now;
#if MAIN_G4_ENABLE
    uint32_t t_g4;
#endif
#if MAIN_G4_ENABLE && MAIN_GW_ONLINE_ENABLE
    uint32_t t_online;
#endif
    uint32_t t_console;
    uint32_t t_status;

    /* ---------- 0. 先把时基和日志起来 ----------
     * 【顺序改过】: 原来第一件事是给 4G 上电, 但那时候 log 还没起来 —— 结果
     * "上电把板子拉重启"这种问题一个字都留不下, 只能靠猜。
     * 现在改成: 时基 → 日志 → 【每一步都打标记】→ 再动电源。
     * 这样一旦重启, 最后打印的那一行就是"死在哪个动作上"。 */
    bsp_tim1_init_1ms();    /* 必须最先: delay_ms / get_tick_ms 都依赖 ATIM1 时基 */

    log_init();
    app_console_init();

    log_info("\r\n==== N32H482 4G+LoRa 网关 模式=%s  %s %s ====\r\n",
             MAIN_MODE_NAME, __DATE__, __TIME__);

    /* ---------- 0.5 装 cJSON 的静态池分配器(v33) ----------
     * 【必须在这里, 而且必须在这条 #else 分支里】
     *  · 信封层已经改用 cJSON, 而 cJSON 默认走裸 malloc/free —— 本工程堆只有
     *    512 字节, 不装 hooks 就是一跑就挂。装上之后全程无堆分配(见
     *    app/cjson/cjson_port.h);
     *  · 放在 #else 里: 隔离测试固件(MAIN_LORA_RAW_TEST=1)用不到 cJSON,
     *    放外面会让那份固件白背 cJSON 的代码 + 24 KB 池。
     *  · 位置要早于任何可能处理下行命令的地方 —— 所以紧跟日志之后。 */
    cjson_port_init();

    app_report_reset_cause();   /* 上一轮到底是谁把我复位的 */

    /* ---------- 1. 给 4G 模组上电 ----------
     * PA8 控的是模组【供电】(板上没引出 RST, "复位"=断一下电再上电)。
     * 极性已实测确认: ★PA8 低电平 = 上电★(bsp_gpio.h 的 G4_PWR_SW_ON_HIGH=0)。
     * 必须放在 lora_business_init() 之前 —— radio 初始化要几百毫秒,
     * 这段时间模组应该已经在跑了。
     * ⚠️ 若一给模组上电板子就重启(BOR), 用 MAIN_G4_POWER_AT_BOOT=0 隔离验证。 */
#if MAIN_G4_POWER_AT_BOOT
    log_info("[BOOT] 给 4G 模组上电: PA8 -> %s ...\r\n",
             G4_PowerSwitch_GetOnHigh() ? "高电平" : "低电平");
    G4_PowerSwitch_Init();
    log_info("[BOOT] 4G 已上电, PA8=%s; 等 300ms 让上电浪涌过去\r\n",
             G4_PowerSwitch_GetLevel() ? "H" : "L");
    delay_ms(300);
    log_info("[BOOT] 4G 上电后 300ms 仍然活着(没有欠压复位)\r\n");
#else
    log_info("[BOOT] !! 隔离测试: 不给 4G 模组上电(MAIN_G4_POWER_AT_BOOT=0)\r\n");
    G4_PowerSwitch_InitNoPower();
#endif

    /* ---------- 2. 本地告警表 ---------- */
    local_alarm_init();

    /* ---------- 3. LoRa 协议层 + 网关业务层 ----------
     * lora_business_init() 内部依次完成:
     *   param_save_init()            读 Flash 参数(含 gw_uid)
     *   lora_proto_init(on_frame)    Radio.Init + 持续 RX + 注册帧回调
     *   data_cache_init()            上行/下发缓存清零
     *   g4_urc_set_recv_cb(...)      注册 MQTT 下行回调 lora_mqtt_recv_cb
     */
#if MAIN_LORA_ENABLE && MAIN_G4_POWER_AT_BOOT
    /* 【错开两股上电浪涌】: 4G 刚上电的这几百毫秒里再给 LoRa 上电, 两个模组的
     * 输入电容同时充电 + 各自内部 LDO 起动, 浪涌叠在一起最容易把电源拉塌 ——
     * 表现正是"MCU 反复 BOR 欠压复位、连 main 都跑不完"。
     * 隔开 500ms 再看: 如果这样就不重启了, 那就确认是供电能力/储能不足。 */
    log_info("[BOOT] 等 500ms(错开两个模组的上电浪涌)再给 LoRa 上电...\r\n");
    delay_ms(500);
#endif
    log_info("[BOOT] LoRa 初始化...\r\n");
#if MAIN_LORA_ENABLE
    lora_business_init();
    {
        device_param_t *p = param_get_ptr();
        log_info("[BOOT] LoRa OK, 网关 UID=%02X%02X%02X\r\n",
                 p->gw_uid[0], p->gw_uid[1], p->gw_uid[2]);

        /* 源地址全 0 的帧协议上非法, 对端会当坏帧丢掉(这个坑踩过: ACK 的
         * gw_uid 留了 00 00 00, 设备一律不回)。这里只大声报警, 不拦。 */
        if ((p->gw_uid[0] == 0U) && (p->gw_uid[1] == 0U) && (p->gw_uid[2] == 0U))
        {
            log_info("[BOOT] !! 警告: 网关 UID = 00 00 00, 外发帧源地址非法,\r\n");
            log_info("[BOOT] !!       对端会直接丢弃 -> 永远收不到应答。\r\n");
            log_info("[BOOT] !!       请检查 param_save(Flash) 里的 gw_uid。\r\n");
        }
    }

    /* P5 设备绑定/解绑(LoRa 0x04/0x05):
     *   - cloud_proto 的 pair_hook 接到 lora_business_open_bind_window, 让云端
     *     sen.bind/dev.bind 命令能打开绑定窗口;
     *   - 业务层在 0x04 绑定成功/失败、0x05 解绑完成时回调 app_publish_bind_event,
     *     由它组 event.device{bound/bind_failed/unbound} 发到 event topic。
     * 这两条注册都要在 lora_business_init() 之后(网关 UID 已读出)、
     * 在云端命令处理之前完成。
     * ⚠️【别删】这两行 + app_publish_bind_event(): 少任何一个, sen.bind/dev.bind
     *    都会恒回 1006(云端再也绑不上设备) —— 2026-09-29 误删过一次。 */
    cloud_proto_set_pair_hook(lora_business_open_bind_window);
    lora_business_set_bind_event_cb(app_publish_bind_event);
    /* ★2026-10-08 新增★ 声光(0x03)执行结果回调 -> event.action(+ 离线时补 event.device)
     * 声光设备对 0x03 回 0x80(收到了) -> 0x82(成功)/0x83(失败), 见 lora_proto.h。
     * ⚠️【别删】少了这一行, 声光状态机照样跑完 15 秒 x3 条的重试, 但结果
     *   一个字都发不出去 —— 平台永远不知道灯亮没亮。 */
    lora_business_set_alarm_result_cb(app_on_alarm_result);
    log_info("[BOOT] 绑定/解绑回调已挂载(pair_hook + bind_event -> event topic)\r\n");
    log_info("[BOOT] 声光执行结果回调已挂载(0x82/0x83 -> event.action)\r\n");

    /* 射频健康自检: BUSY(PB3) 一直高 = LoRa 模组不应答(没电/EN没拉/没焊/线断)。
     * 单独打一行, 免得现场要去数 "wait busy pin timeout" 才知道射频是坏的。 */
    if (lora_business_radio_stuck())
    {
        log_info("[BOOT] !! LoRa 模组不应答(BUSY 一直高): 射频功能不可用, 4G 不受影响\r\n");
    }
    else
    {
        log_info("[BOOT] LoRa 自检 OK (BUSY=%u)\r\n", (unsigned)lora_business_radio_busy());
    }
#else  /* !MAIN_LORA_ENABLE */
    log_info("[BOOT] LoRa 已关闭(MAIN_LORA_ENABLE=0): 只跑 4G, 没有射频收发\r\n");
#endif

    /* ---------- 4. 4G 链路: USART2 + AT/MQTT 状态机 ---------- */
#if MAIN_G4_ENABLE
    log_info("[BOOT] 4G AT 状态机启动...\r\n");
    g4_at_init();

    /* v79: 把配置域 network 的 broker/账号灌给 4G 层 —— 文档 3.2 要求"整域重启生效"。
     * 此前 g4_at 只认编译期常量(broker.emqx.io), 配置里那几个字段是【死配置】:
     * 用 cfg.set 改了也不生效。现在只在配置里 host 非空时才覆盖, 空则沿用编译期默认
     * —— 默认行为不变(出厂配置 host 为空时仍然连原来的 broker)。
     *
     * ★★2026-10-08 用户重新定的口径★★
     *   · clientID = 【固定 "110001"】(不再是 IMEI) —— 编译期默认, 配置可覆盖;
     *   · 用户名   = 【"tuguan"】(固定)     —— 编译期默认, 配置可覆盖;
     *   · 密码     = 【空】;
     *   · topic 里的 {gwId} = 【仍跟 system.gwId 走】(出厂 110001, 可改)。
     *   ⇒ 【三者已经解耦】: 主题的 gwId 可以改, 而 clientID/用户名保持固定值。
     *     (之前是"用户名 = 设备 ID = 主题 gwId"三者同一个值, 现在不是了。)
     *
     *   ⚠️ 唯一要警惕的: 固定 clientID + 同一个 broker = 多台网关互相顶下线。
     *      单台测试没问题; 上第二台时把 `network.clientId` 配成唯一值(或 "IMEI")。 */
    {
        const config_store_t *cfg = config_store_get();
        const char *dev_id = ((cfg != NULL) && (cfg->system.gw_id[0] != '\0'))
                                 ? cfg->system.gw_id : "11000000000000000000000000";   /* 2026-10-18: 兜底也 26 位 */

        /* (1) ★先把遗留档位迁移到文档档位★
         * Flash(param_save 页)里可能存着旧固件写下的 LOCAL 档(mod/cmd ...),
         * 而它【优先于】代码里的出厂默认, 且【重烧 App 带不走】——
         * 现场表现极具迷惑性: 心跳发到 mod/status 看得到, 但平台往
         * rt/gw/110001/cmd 发的指令网关一条都收不到(它订的是 mod/cmd)。
         * 这里一次性改成 DOC 档并落盘, 下次开机不再需要迁移。 */
        if (cloud_profile_migrate_to_doc())
        {
            log_info("[BOOT] ★检测到 Flash 里是遗留的 LOCAL 档(mod/xxx) —— 已迁移到"
                     " DOC 档(rt/gw/{gwId}/xxx)并落盘★\r\n");
            log_info("[BOOT]   原因: 平台用 rt/gw/{gwId}/cmd 等主题, 旧档位会导致"
                     "【心跳能发、指令收不到】\r\n");
        }

        /* (2) 设备 ID -> cloud_profile(决定 rt/gw/{gwId}/xxx 里的 {gwId}) */
        cloud_profile_set_gw_id(dev_id);

        /* (3) clientID: 默认用编译期的 "110001"; 配置里给了 network.clientId 就覆盖它。
         *     "IMEI" 是特殊值 -> 用模组 IMEI(多台网关时的正解), 见 g4_at_set_client_id。 */
        if ((cfg != NULL) && (cfg->network.client_id[0] != '\0'))
        {
            g4_at_set_client_id(cfg->network.client_id);
            log_info("[BOOT] clientID 来自配置 network.clientId = %s\r\n",
                     cfg->network.client_id);
        }

        /* (4) broker host/port/账号。用户名/密码规则:
         *     配置里给了 network.username / network.mqtt_password 就用配置的;
         *     没给就【保持编译期默认】(用户名 "tuguan"、密码空) ——
         *     因为 g4_at_set_broker() 的约定是"空串 = 不覆盖"。 */
        if ((cfg != NULL) && (cfg->network.host[0] != '\0'))
        {
            char portbuf[8];

            (void)snprintf(portbuf, sizeof(portbuf), "%u", (unsigned)cfg->network.port);
            g4_at_set_broker(cfg->network.host, portbuf,
                             cfg->network.username,      /* 空 = 保持 "tuguan" */
                             cfg->network.mqtt_password);/* 空 = 保持 "" */
            /* ★network.tls 接线(2026-10-04 第 3 步)★
             * 这个字段以前是【死配置】: config_store 解析/校验/序列化它, 但全工程
             * 没有任何消费者 —— 于是"配置里开了 TLS"实际上一点用都没有。
             * 【为什么放在这个 if 里面】: 与 broker 其余字段保持同一个语义 ——
             * "配置里给了 host = 整条用配置的一套(含 tls); 没给 = 整条用编译期默认"。
             * 不放进去的话, 出厂默认配置(network.host 为空、tls=false)会把
             * g4_at 里编译期的 TLS=1 给覆盖掉, 于是私有 broker 的 8883 就连不上了。 */
            g4_at_set_tls(cfg->network.tls ? 1u : 0u);
            log_info("[BOOT] broker 来自配置 network: %s:%s user=%s pass=%s tls=%s\r\n",
                     cfg->network.host, portbuf,
                     (cfg->network.username[0] != '\0') ? cfg->network.username : "(默认 tuguan)",
                     (cfg->network.mqtt_password[0] != '\0') ? "已设" : "空",
                     cfg->network.tls ? "开" : "关");
        }
        else
        {
            /* 用编译期默认: host/port 见 g4_at.c 的 G4_MQTT_*, user="tuguan", pass=""
             * —— 这里【不需要】再调 g4_at_set_broker(), 那些默认值本来就是生效值。 */
            log_info("[BOOT] broker 用编译期默认(配置 network.host 为空), "
                     "用户名用默认 tuguan\r\n");
        }

        /* 无论走哪条路径, 都把【真正生效】的值打出来 —— 连不上时第一眼就看这一行。
         * ★2026-10-08: pass 之前这里硬编码成 "空" —— 密码改成 tg123456! 之后
         * 那行日志就是错的(会让人以为没发密码)。改成按实际值打"已设/空",
         * 但仍然【不打印明文】。 */
        log_info("[BOOT] 生效 broker = %s:%s user=%s pass=%s clientID=%s\r\n",
                 g4_at_get_broker_host(), g4_at_get_broker_port(),
                 (g4_at_get_broker_user() != NULL) ? g4_at_get_broker_user() : "-",
                 (g4_at_get_broker_pass() != NULL) && (g4_at_get_broker_pass()[0] != '\0')
                     ? "已设" : "空",
                 (g4_at_get_client_id() != NULL) ? g4_at_get_client_id() : "-");
    }

    /* P1-3: 下行订阅 topic 按档位注入。
     * 本地档位算出来就是 "rt/gw/{gwId}/cmd"(与 g4_at 的默认值相同) -> 行为不变;
     * 文档档位是 "rt/gw/{gwId}/cmd"。 */
    {
        char subt[64];
        if (cloud_profile_build_topic(CLOUD_TOPIC_CMD, subt, (uint16_t)sizeof(subt)) > 0u)
        {
            g4_at_set_sub_topic(subt);
            log_info("[BOOT] 下行订阅 topic = %s (%s档位)\r\n",
                     subt, cloud_profile_is_doc() ? "文档" : "本地");
        }
    }

    /* MQTT 遗嘱(LWT): 正文 {"online":false}。
     * ★★2026-10-08: 遗嘱 topic 由 status 改成【专用主题 rt/gw/{gwId}/lwt】★★
     *
     * 【为什么改 —— 现场真的踩了】
     *   原来遗嘱发到 status, 于是【每次网关上电】平台都会先收到一条
     *   {"online":false}, 然后才是 status.report(online:true), 看起来像
     *   "网关刚掉线又上线"。根因是 MQTT 协议行为: clientID 固定时设备掉电后
     *   broker 侧旧会话要等 keepalive 超时才过期, 网关重连时 broker 踢掉旧会话,
     *   那一刻【旧会话的遗嘱才真正发出】—— 也就是"上电瞬间收到 offline"。
     *   而 status 是 retained 的, 这条遗嘱还会把平台上的状态短暂改成 offline。
     *
     * 【现在】lwt 独立一条主题、也 retained, 两者互不覆盖:
     *   · status 的 retained 只由本机主动上报维护;
     *   · 平台要【多订阅一个】rt/gw/{gwId}/lwt 才能拿到掉线通知
     *     (《云端对接偏差说明》§2.4 提过, 用户 2026-10-08 拍板采用)。
     *
     * 这里只"备好"(并把要用的值打进日志); 是否真的拼进 AT+MQTTCONN 由 g4_at.c 的
     * MAIN_MQTT_LWT_ENABLE 控制(默认 0, 语法待台架确认, 见 MQTT_LWT_台架测试.txt)。 */
    {
        char lwtt[64];
        if (cloud_profile_build_topic(CLOUD_TOPIC_LWT, lwtt, (uint16_t)sizeof(lwtt)) > 0u)
        {
            g4_at_set_lwt(lwtt, "{\"online\":false}");
        }
    }

    /* ---------- 断网判定的第二条腿: 保活停滞看门狗 ----------
     * 【为什么由 main.c 来设】: 这条判据只有在"有周期性上行"时才成立, 所以窗口
     * 必须跟着心跳周期走, 不能用固定值:
     *     心跳开着 -> 5 × 30s = 150s: 连着几次心跳都没发成功才触发 4G_RST 断电重启;
     *     心跳关着 -> 0(= 关闭这条判据): 否则没有周期上行时必然误判, 会一直断电。
     * (第一条腿是 g4_at 里的"连续 3 次发布失败", 见 G4_PUB_FAIL_MAX。)
     *
     * ★2026-09-22 3 × -> 5 × (依据现场日志 78.txt)★
     *   模组自己重启一次要 40~60 秒才回到 ONLINE, 而 90s 的窗口意味着
     *   "它还没回来, 我们就又判它死、再给它断一次电" —— 看门狗本身成了又一个
     *   "自己拆自己链路"的来源。窗口必须明显长于一个完整的重建周期。
     *   150s 仍然远快于"云端一直等不到数据", 不至于让人等太久。 */
#if MAIN_GW_ONLINE_ENABLE
    /* 保活停滞看门狗窗口 = 5 ×【实际配置的】心跳周期(以前用编译期宏, 配置改了就不匹配) */
    g4_at_set_link_watchdog(5U * app_cfg_period_ms(config_store_get()->network.status_interval_s,
                                                    MAIN_GW_ONLINE_MS));
#else
    g4_at_set_link_watchdog(0U);
#endif

    /* ---------- 用上【模组自己】的 MQTT 能力(2026-10-04 第 2 步) ----------
     * ① keepalive: 模组级保活时间。以前 network.keepAliveSeconds 是【死配置】
     *    (只有 config_store 读写, 没有任何消费者) —— 模组一直用它的默认 120s。
     *    现在真正下发; 手册下限 60s, 配置给 30~59 会被 g4_at 夹紧并在日志里说明。
     * ② reconnect min/max: 恢复动作的指数退避。这两个也一直是死配置, 于是
     *    "连续失败就断电"没有任何减速带 —— 现场 1.6 小时断电 27 次就是这么来的。
     * ③ 遗嘱 / MQTT心跳间隔 / 心跳回显 / 模组自带重连 都在 g4_at 的建链流程里
     *    统一下发(MQTTCONN 之前), 不在这里 —— 那几条每条建链都要重发一遍。 */
    g4_at_set_keepalive_s(config_store_get()->network.keep_alive_s);
    g4_at_set_reconnect(config_store_get()->network.reconnect_min_s,
                        config_store_get()->network.reconnect_max_s);

#if MAIN_LORA_ENABLE
    /* P1-3: 给 LoRa 的下行回调挂一个前置过滤器 —— 内容是 JSON 就当云协议命令处理,
     * 其余(二进制帧)照旧走 LoRa 透传。不注册时行为与本改动前完全一致。 */
    lora_business_set_dl_prefilter(app_downlink_try);

    /* 把 sys.query_status 要的运行时状态接上(v40): 协议层不 include 4G/LoRa 的模块,
     * 由这里提供一个回调 —— 这样协议层仍然能在 PC 上脱机单测。
     * ⚠ 不注册的话 sys.query_status 会明确回 1006(而不是拿一堆 0 冒充状态)。 */
    cloud_proto_set_status_provider(app_fill_status);

    /* sys.time_sync 的写时钟钩子(v91)。★ 不注册的话这条命令会明确回 1006 ★ ——
     * 此前正是如此（自 v49 实现该命令起一直没接上，见 v82 说明）。 */
    cloud_proto_set_time_sync_hook(app_time_sync);

    /* sen.query / dev.query 的 lastTs 需要"收到那一帧时的 Unix 秒"(v53)。
     * 用钩子注入 —— last_value 模块要能在 PC 上脱机单测, 不能直接调 app_now_unix()。
     * ⚠ 不注册的话 lastTs 在应答里【根本不出现】, 不拿 0 冒充 1970。 */
    last_value_set_now_hook(app_now_unix);
#if MAIN_STATUS_JSON_ENABLE
    /* 液位失联/恢复事件: lora_business 只负责"什么时候该报"(判定+跳变去重),
     * 真正的组 JSON + 发 event topic 在本文件的 uplink_publish_device_event()。
     * 这样 LoRa 业务层不用依赖云协议层, 主机测试也就不用为它造 4G 桩。 */
    lora_business_set_sensor_event_cb(app_on_sensor_event);   /* 2026-10-18: 经适配器固定 devType="sensor" */

    /* ★2026-10-18: 每次液位上报(0x02)都通知一次 —— 4G 离线时存进
     *   telemetry_cache, 重连后按原时间戳逐条补报(用户口径: 补的是"每次上报",
     *   不是 30s 聚合遥测)。在线时这个回调直接返回, 不影响原有遥测。 */
    lora_business_set_level_report_cb(app_on_level_report);

    /* *2026-10-18: 出厂字段(0x06/0x07)的结果回调 —— 只打串口, 不上云 */
    lora_business_set_field_result_cb(app_on_field_result);
    log_info("[BOOT] 液位失联事件回调已挂载(event.device -> event topic)\r\n");
#endif
    log_info("[BOOT] MQTT 下行回调 = lora_mqtt_recv_cb, 前置过滤器 = 云协议层(JSON)\r\n");
#else
    /* LoRa 关闭时也必须注册下行回调, 否则云端下发的消息会被静默丢掉,
     * 现场会误判成"模组收不到下发"(见上面 app_downlink_log_cb 的说明)。 */
    g4_at_set_recv_cb(app_downlink_log_cb);
    log_info("[BOOT] LoRa 已关闭 -> MQTT 下行回调 = 只打印(JSON 命令仍走云协议层)\r\n");
#endif

#if MAIN_AT_CONSOLE_ENABLE
    /* 把云协议离线自测命令挂到日志口(#json / #status / #topic / #offline) + 射频诊断(#lora)。
     * 弱网/离线时靠它验证协议层; 射频收不到时靠 #lora 看芯片里的实际配置。 */
    g4_at_set_console_ext(app_console_ext, app_console_help);
    log_info("[BOOT] 日志口离线自测命令已挂载: #json #status #topic #offline #lora\r\n");
#endif
#else  /* !MAIN_G4_ENABLE */
    log_info("[BOOT] 4G 状态机已关闭(MAIN_G4_ENABLE=0): 只跑 LoRa, 云端下发收不到\r\n");
#endif

    /* ---------- 5. RS232 / RS485 通道(协议待定, 仅初始化) ---------- */
#if MAIN_SERIAL_INIT_ENABLE
    serial_init_all(MAIN_SERIAL_BAUD);
    log_info("[BOOT] RS232/RS485 通道已初始化 (%lu bps)\r\n",
             (unsigned long)MAIN_SERIAL_BAUD);
#endif

    /* ---------- 6. OTA ----------
     * 【两条通道, 别混】:
     *   ota.c       MQTT 分包版(按包收 + 位图 + CRC32) —— 保留, 但 cloud_proto
     *               已经不再调它了(2026-10-02 起 ota.start 走 HTTP 版)。
     *   ota_http.c  HTTP 拉取版(ML307R 原生 AT+MHTTP* + MD5) —— ★当前通道★
     *
     * ota_http_init() 【无条件调用】, 不放在 MAIN_OTA_ENABLE 里:
     *   它只做 memset + 把自己注册成 g4_urc 的 +MHTTPURC 钩子, 本身不发任何 AT。
     *   注册这件事【必须】做 —— 否则 +MHTTPURC 家族(里面是固件正文的 HEX)
     *   会落到 AT 应答匹配里去, 要么被当垃圾行丢掉, 要么误命中关键字把 AT 流带偏。
     * 注: 它不碰 Flash、不碰分区, 所以与 MAIN_OTA_ENABLE 那个"分区顾虑"无关。 */
    ota_http_init();
    log_info("[BOOT] OTA(HTTP) 已初始化: chunk=%u frag=%u/%ums\r\n",
             (unsigned)OTA_HTTP_CHUNK, (unsigned)OTA_HTTP_FRAG_SIZE,
             (unsigned)OTA_HTTP_FRAG_INTV);

    /* ==========================================================================
     * ★★2026-10-11: A/B 回滚判定(开机一次)★★
     * --------------------------------------------------------------------------
     * 【什么时候做】必须在进主循环之前 —— 越早扣机会越准(晚扣的话, 一个"起来就崩"
     *   的固件可能还没扣到就崩了, 计数永远涨不上去, 回滚就永远不触发)。
     *
     * 【(乙) 方案: 模式 1 下【只记录、不复位】】
     *   回滚动作本身是 bootloader 的职责(把另一槽搬回运行区)。而当前是
     *   FM_HAS_BOOTLOADER=0 的单镜像模式 —— 板上没有 BL, 复位之后跑的【还是同一个
     *   slot】⇒ 会变成无限重启循环, 比不回滚严重得多。
     *   ⇒ 用 FM_HAS_BOOTLOADER 条件编译: 有 BL 才复位。
     *   ⇒ 好处: 现在(模式 1)就能把判定+计数逻辑跑起来、用日志验证;
     *     将来切模式 2 只需改 flash_map.h 一个宏, 这段代码不用动。
     * ======================================================================== */
    /* 无条件编译: 模式 1 下只记录日志, 零副作用(见下面的 (乙) 说明)。
     * (旧的 MQTT 分包版 ota.c 已归档, 不再有 MAIN_OTA_ENABLE 这个开关。) */
    if (ota_health_boot_step() == OTA_HEALTH_BOOT_ROLLBACK)
    {
#if (FM_HAS_BOOTLOADER != 0)
        log_info("[BOOT] !! A/B 回滚: 本槽机会已用完 -> 复位, 让 bootloader 搬回另一槽\r\n");
        NVIC_SystemReset();          /* 不返回 */
#else
        /* ⚠️ 模式 1(无 BL): 复位也是同一个槽 ⇒ 只记录, 不复位。
         * 这一行日志就是"这台设备本该被回滚"的证据, 现场可据此判断固件是否有问题。 */
        log_info("[BOOT] !! A/B 回滚【已判定但不执行】: 当前是模式 1(无 bootloader), "
                 "复位只会重跑同一槽 -> 若切成模式 2 这里会真的回滚\r\n");
#endif
    }
    log_info("[OTAHL] A/B: 槽=%u 剩=%u 次 %s\r\n",
             (unsigned)ota_health_boot_slot(),
             (unsigned)ota_health_tries_left(),
             ota_health_confirmed() ? "(已定案)" : "(待确认)");

    log_info("[BOOT] 进入主循环\r\n");

    /* ==================== 主循环 ==================== */
    now        = get_tick_ms();
    t_console  = now;
    t_status   = now;
#if MAIN_G4_ENABLE
    t_g4       = now;
#endif
#if MAIN_G4_ENABLE && MAIN_GW_ONLINE_ENABLE
    t_online   = now;
#endif

    while (1)
    {
        now = get_tick_ms();

        /* ★★2026-10-11: 健康判定(每圈都调, 但代价只有 3 次比较)★★
         * 【判据(三条缺一不可)】
         *   ① 4G 在线                      -> g4_at_is_online()
         *   ② 至少一条上行被 broker puback  -> g4_urc_puback_count() > 0
         *      ★ 用 puback 而不是"发布成功": 模组回 OK 只说明它自己收下了,
         *        puback 才是"云端确实收到了"的证据(见上面 [STAT2] 的注释)。
         *   ③ uptime > 60s(OTA_HEALTH_MIN_UPTIME_S) -> 挡"起来就崩"
         * 三条都满足就把本槽"定案"(写一次 Flash), 之后本函数立即变成空操作。
         * 【为什么每圈调而不是挂 1s 节拍】: 定案后 s_done=true 直接 return,
         *   定案前也只是 3 次比较 —— 比"再找一个 1s 节拍插入点"简单且不易错。 */
        ota_health_poll(g4_at_is_online(),
                        (g4_urc_puback_count() > 0u),
                        (uint32_t)(now / 1000u));

        /* (A) 每 1ms: LoRa radio 中断处理 + 帧解析分发 + 从机表/重传/上行缓存 */
#if MAIN_LORA_ENABLE
        lora_business_poll();
#endif

        /* (A2) sys.reboot 的延迟重启(v37+): 命令只登记"什么时候重启", 真正的软件复位
         *      在这里做 —— 这样 cloud_proto 不用直接碰 NVIC_SystemReset, 主机单测
         *      也能把这条命令的逻辑验完。纯比较, 每个循环跑一次的开销可以忽略。 */
        if (cloud_proto_reboot_due(app_now_unix()))
        {
            log_info("[SYS] sys.reboot 到期, 执行软件复位(NVIC_SystemReset)...\r\n");
            delay_ms(50);               /* 让上面那行日志从串口发完 */
            NVIC_SystemReset();
        }

        /* (B) 每 5ms: 收包泵 + 云端下行投递 + 非阻塞 AT 状态机 + 发布子流程
         *     ⚠️ 这里【不会阻塞】: 每条 AT 命令每次只推进一步, 收包由 g4_io_pump()
         *        一次性消费干净。模组不应答也只是状态机停在原地重试, 主循环节拍不变。
         *        模组没调通就把 MAIN_G4_ENABLE 改成 0。 */
#if MAIN_G4_ENABLE
        if ((uint32_t)(now - t_g4) >= TICK_G4_POLL_MS)
        {
            t_g4 = now;
            g4_at_poll();

            /* OTA over HTTP 必须跟【同一节拍】推进, 不能放到下面 1s 的 (D) 块:
             * 下载状态机每拍要发一条 AT(HEADER/REQUEST) 并消化 URC 字节队列,
             * 而模组按 fragment 配置以 200 B/块往外吐。若 1s 才推一次, 就变成
             * 200 B/s —— 64 KB 要 5 分钟以上, 还能把 8192 的环形缓冲灌爆。
             * 本函数纯粹是状态机推进, 不阻塞、空闲时什么都不做。 */
            ota_http_poll();
        }
#endif

        /* (C) 每 10ms: 日志口手敲 AT 转发给 4G 模组(台架调试)
         *     注: 手敲命令会插入 AT 流, 可能被状态机的关键字匹配"看到",
         *         排障时尽量避开正在握手/发布的时间窗口。 */
        if ((uint32_t)(now - t_console) >= TICK_CONSOLE_MS)
        {
            t_console = now;
            app_console_poll();
        }

        /* (D) 每 1s: 状态上报 + LED 心跳 */
        if ((uint32_t)(now - t_status) >= TICK_STATUS_MS)
        {
            t_status = now;

            app_status_report();

            /* ---- 边缘水位评估(v54): 复用这同一个 1s 节拍, 【不新增定时器】 ----
             * 文档 4 章的自治核心: 有效水位(max) -> confirmCount 防抖 -> 档位 ->
             * 模板渲染 -> 该不该下发。算法在 level_eval.c, 取数+串接在 level_supervisor.c。
             * ★ 每秒调一次【不会】把 confirmCount 变成"立刻满足": level_supervisor_poll()
             *   只在检测到【新上报】时才推进算法(按 last_value 的更新计数判)。
             *
             * 【2026-09-30 用户第 2 条: 最后一跳接上了】
             *   以前这里只"评估 + 记日志"。现在真的把档位动作下发到联动设备:
             *     level_supervisor_poll() 算出 dispatch -> lora_business_apply_gear()
             *     按 gears[] 模板组帧投递 -> 发【成功】才 level_supervisor_mark_sent()
             *     (失败不 mark, 下次有新上报时会自动重试)。
             *   注意 dispatch 与 migrated 是【两个判据】: migrated 管"要不要上报
             *   status_change", dispatch 管"要不要下发联动"(档位变【或】文案变),
             *   所以下面分成两个 if, 不能合并。 */
            {
                level_decision_t ld;

                level_supervisor_poll(config_store_get(), get_tick_ms(), &ld);

                /* 缓存最近一次【有效】评估结果: status.report / telemetry 要用它取
                 * 实时档位与状态(第 10 条: 以前那里硬编码 0/"auto") */
                if (ld.new_sample && ld.have_level)
                {
                    app_note_decision(&ld);
                }

                /* ---- (D1) 联动下发: 判据是 dispatch ---- */
                if (ld.new_sample && ld.have_level && ld.dispatch)
                {
                    uint8_t total = 0U;
                    int     sent  = lora_business_apply_gear(ld.gear, ld.level_mm,
                                                             ld.led_text1, ld.led_text2,
                                                             ld.tts_text,
                                                             &total);

                    if (sent > 0)
                    {
                        /* 真发出去了 -> 记下档位/文案, 下次同样的内容不再重复下发 */
                        level_supervisor_mark_sent(ld.gear, ld.led_text1, ld.led_text2);
                    }
                    else if (total == 0U)
                    {
                        /* 压根没有"启用的"联动设备: 这是配置事实, 不是发送失败。
                         * 也 mark(否则每来一次上报都会重试一次, 日志被刷满)。 */
                        level_supervisor_mark_sent(ld.gear, ld.led_text1, ld.led_text2);
                        log_info("[LEVEL] 档位 %u 无启用的联动设备, 视为已处理\r\n",
                                 (unsigned)ld.gear);
                    }
                    /* sent==0 且 total>0: 投递失败(载荷过长/从机表里没有) ->
                     * 不 mark, 等下次上报重试。lora_business 那边已打原因。 */
                }

                /* ---- (D1b) 全部失联 fail-safe(第 7 条): 边沿触发, 只做一次 ---- */
                app_failsafe_step(&ld);

                /* ---- (D2) 状态迁移上报: 判据是 migrated ---- */
                if (ld.new_sample && ld.have_level && ld.migrated)
                {
                    log_info("[LEVEL] 迁移 -> state=%u gear=%u level=%ldmm led1=\"%s\" led2=\"%s\" 该下发=%u\r\n",
                             (unsigned)ld.state, (unsigned)ld.gear,
                             (long)ld.level_mm, ld.led_text1, ld.led_text2,
                             (unsigned)ld.dispatch);

                    /* ---- 【第 8 条】事件去抖真正生效 ----
                     * migrated 只说明"状态被确认变了"; 文档 7.2 还要求
                     * event.status_change 受 minChangeCm 与 eventMinIntervalSeconds 约束。
                     * 这两个判据 level_eval 早就算好放在 ld.event_due 里了,
                     * 但这里【以前从来没读它】—— 于是去抖形同不存在。
                     * 现在: 不 due 就不发(也不 mark, 让下一条够格的样本能发出去)。 */
                    if (!ld.event_due)
                    {
                        log_info("[LEVEL] 迁移事件被去抖拦下(变化 < %ucm 或间隔 < %us)\r\n",
                                 (unsigned)config_store_get()->thresholds.min_change_cm,
                                 (unsigned)config_store_get()->thresholds.event_min_interval_s);
                    }
                    else
                    {
                        /* ★ 在线直发 / 断线入缓存, 两支互斥 (v72) ★
                         * 相关函数见上面 app_publish_status_change / app_cache_status_change。
                         * 之前这里只有"入缓存"一支; 现在在线时也真的把 status_change 发出去了。 */
                        if (g4_at_is_online())
                        {
                            app_publish_status_change((uint8_t)ld.prev_state, (uint8_t)ld.state,
                                                       ld.gear, ld.level_mm);
                        }
                        else
                        {
                            app_cache_status_change((uint8_t)ld.prev_state, (uint8_t)ld.state,
                                                     ld.gear, ld.level_mm);
                        }

                        /* 事件【已生成】(直发或入缓存都算) -> 记下基准, 供下一次去抖比较。
                         * ★ 不在这里留"失败重试"的口子: 入缓存那条本来就是断线补报的入口;
                         *   直发失败也只影响这一条, 下一个够格的样本会再发一条新事件。 */
                        level_supervisor_mark_event(ld.level_mm, get_tick_ms());
                    }
                }

                /* ---- (D3) 水位遥测上行(第 3 条): 周期 or 变化量触发 ---- */
                app_telemetry_step();

                /* ---- (D4) manual 模式窗口(第 6 条): 到期自动回归 auto ---- */
                app_manual_step();
            }

            /* ---- 断线补报: 每次最多补一页(16 条), 缓存空了立刻返回 (v70) ---- */
            app_backfill_poll();

            /* ---- ★2026-10-18: 逐条液位上报的补报(一次一条) ----
             * 与上面的"事件补报"分开跑: 两者缓存不同、topic 不同。
             * 先事件后遥测 —— 事件量小、时效性强; 平台侧各自按 ts 排序。 */
            app_telemetry_backfill_poll();

#if MAIN_LORA_ENABLE
            /* ---- 液位失联检测的【定时兜底腿】(2026-09-27 用户拍板加) ----
             * 【为什么光有"上报驱动"不够】原口径是"每收到一路液位上报就顺带检查
             * 全部液位从机", 但那只在【至少有一台还在报】时才成立。现场只挂了一台
             * 液位(210001), 它彻底不报时【没有任何帧能触发检查】—— 而"唯一那台死了"
             * 恰恰是最该报的 case, 于是最需要它的场景反而检测不到。
             * 所以再加这条腿: 复用【已有的 1s 节拍】, 【不新增任何定时器】。
             * check 函数是幂等的(只在"在线->失联"跳变时发一次事件), 每秒调用不会
             * 造成重复上报; 上报驱动那条腿保留不动, 两条腿互为补充。 */
            lora_business_check_sensor_offline();

            /* *2026-10-18: 出厂字段操作(0x06 重发 / 0x07 超时)推进
             * 1 秒粒度足够: 写字段要 5s 才补发一条, 读字段要等 12s。 */
            lora_business_field_poll();
#endif

#if MAIN_HEARTBEAT_LED
            GPIO_TogglePin(GPIOA, GPIO_PIN_8);
#endif

            /* OTA over HTTP 的进度上报 + 就绪后复位。
             * 放在 1s 块里是【刻意的】: 文档 7.6 要求 ota.progress 每 10% 或 10s
             * 一条, 状态机本身的推进在 5ms 块里(见上面 (B)), 两件事分开。
             * 本函数只读状态 + 发 MQTT, 不发 AT、不阻塞; 空闲时第一件事就是 return。
             * 刻意【不】加 #if MAIN_G4_ENABLE: 那样在 4G 关掉时它就成了没人调的
             * static 函数, 编译器会报 unused-function。 */
            app_ota_progress_poll();
        }

        /* (E) 网关在线心跳 —— 往云端 pub 一条 status.report(或裸 "GW_ONLINE")
         *
         * 为什么要有: 云端只认"有没有收到报文"。业务数据都来自下属从机, 从机可能
         * 几十分钟不上报, 这段时间云端会以为网关掉线了。这条心跳就是网关的
         * "我还活着"信号, 同时它也是 4G 链路的探针 —— 发不出去会累加 g4_at 里的
         * 发布失败计数, 连续失败就自动重新建链(见 g4_pub_mark_fail)。
         *
         * 【第 5 条 2026-09-30】周期改成读配置 network.statusIntervalSeconds
         *   (文档 3.2: 10~600s, 默认 60, 生效方式"立即")。以前这里是编译期宏,
         *   平台改了配置【一点反应都没有】。读不到/越界才回落到 MAIN_GW_ONLINE_MS。 */
#if MAIN_G4_ENABLE && MAIN_GW_ONLINE_ENABLE
        {
            uint32_t hb_period_ms = app_cfg_period_ms(config_store_get()->network.status_interval_s,
                                                      MAIN_GW_ONLINE_MS);

            /* ★2026-10-07: 刚上线【立刻】发第一条 retained 状态, 把遗嘱盖掉★
             *
             * 【现场现象】每次上电连上 broker 后, 平台会先收到一条
             *     {"online": false}     (发在 retained 的 mod/status 上)
             * 这不是固件发的, 是 MQTT 的【遗嘱(LWT)】被 broker 代发的 —— 机制是:
             *   本机 client id 就是 IMEI(固定), 上次掉电时 broker 那边那个会话还没
             *   走到 keepalive 超时, 它的遗嘱一直挂着; 这次重连 broker 发现"同名旧
             *   会话还在", 就把它踢掉 —— 【踢掉的那一刻旧会话的遗嘱才真正发出去】。
             *   所以它看起来"跟着上电一起来", 其实是上一次掉线欠下的。
             * 见 g4_at 里的 AT+MQTTCFG="willoption",0,1,1,1 (遗嘱 retain=1) 与
             * willpayload 的 mod/status / {"online":false}。
             *
             * 【为什么必须立刻发】平台按"最后一条 retained 消息"判断在线状态。遗嘱也
             * 是 retained 的, 且和 status.report 用同一个 topic。原先第一条心跳要等
             * 满一个周期(默认 30s)才发, 于是这 30 秒里平台一直显示离线 —— 现场看着
             * 就像"上电后先报离线"。
             * 现在: 一进入 ONLINE 就把 t_online 拨到"已到期", 下一拍立刻发一条
             * retained 的 status.report(online:true), 100ms 级就把遗嘱盖回去。 */
            {
                static bool s_hb_was_online;
                bool        online_now = g4_at_is_online();

                if (online_now && (!s_hb_was_online))
                {
                    t_online = now - hb_period_ms;      /* 让下面的判据立刻成立 */
                }
                s_hb_was_online = online_now;
            }

            /* ★2026-10-07: OTA 下载期间心跳【让路】★
             *
             * 现场(19:09) 实测的死法: 心跳/事件发布和 OTA 共用同一条 AT 引擎, 而
             * 发布一旦撞上 '>' 丢失, 光是等 '>' 就要占住引擎 15 秒
             * (G4_PUB_PROMPT_TIMEOUT_MS); 超时后立刻重发, 又一个 15 秒 —— 连着几轮
             * 就把 OTA 的下一片 AT+MHTTPHEADER 一直挤在门外:
             *   19:09:20  第一片 8192 收满(percent 52)
             *   19:09:20 ~ 19:10:20  【60 秒里一条 AT 都没发出去, 全是 MQTTPUB 在重试】
             *   19:10:20  片内超时(60s) -> 回滚到 7680 重下 7764 字节
             * 而就在这段"抢引擎"的时间里串口还丢了字节(丢字节 1->5),
             * 最终 MD5 对不上(算出 dd4a7606..., 云端 d9c8750b...) —— 前功尽弃。
             *
             * OTA 期间 ota.progress 仍然每 10 秒上报一次, 云端不会以为网关死了,
             * 所以这里让路是安全的; 升级一结束(READY/FAILED)就自动恢复。 */
            if (g4_at_is_online() && (!ota_http_busy()) &&
                ((uint32_t)(now - t_online) >= hb_period_ms))
            {
            sys_err_t hb_res;

#if MAIN_STATUS_JSON_ENABLE
            /* P1-8: 心跳载荷换成文档要求的 status.report JSON(替代裸 "GW_ONLINE") */
            hb_res = app_publish_status_json();
#else
            static const char gw_online[] = "GW_ONLINE";
            hb_res = g4_at_mqtt_publish(NULL, (uint8_t *)gw_online,
                                        (uint16_t)(sizeof(gw_online) - 1u));
#endif
            if (hb_res == SYS_OK)
            {
                t_online = now;
#if MAIN_STATUS_JSON_ENABLE
                log_info("[GW] status.report 已发 (距上次发布成功 %lus)\r\n",
                         (unsigned long)(g4_at_pub_age_ms() / 1000u));
#else
                log_info("[GW] 在线心跳已发: GW_ONLINE (距上次发布成功 %lus)\r\n",
                         (unsigned long)(g4_at_pub_age_ms() / 1000u));
#endif
            }
            else
            {
                /* 【这一条以前是静默的, 而它就是"4G 好像不会保活"最可能的真相】:
                 * 原来失败时什么都不打印, 于是"心跳发不出去"这件事在现场完全看不见,
                 * 只能看到云端那边这台设备慢慢不理人了。
                 * 现在: ① 失败一定打日志; ② 不等满 30s, 2s 后就重试;
                 *       ③ g4_at 里【连续 3 次】(G4_PUB_FAIL_MAX)发布失败就判链路断,
                 *          走 g4_link_recover 直接 4G_RST 断电重启模组重建连接;
                 *       ④ 另有保活停滞看门狗(5 × 心跳周期 = 150s 没成功过一次)兜底。 */
                t_online = now - hb_period_ms + 2000U;
                log_info("[GW] !! 在线心跳【发布登记失败】(上一条还在发 / 状态机不在 ONLINE), "
                         "2s 后重试; 距上次成功 %lus\r\n",
                         (g4_at_pub_age_ms() == 0xFFFFFFFFu)
                             ? 0UL : (unsigned long)(g4_at_pub_age_ms() / 1000u));
            }
            }
        }
#endif

        /* 与 1ms 时基对齐, 避免空转(中断仍可抢占) */
        while (get_tick_ms() == now)
        {
        }
    }
#endif /* MAIN_LORA_RAW_TEST */
}
