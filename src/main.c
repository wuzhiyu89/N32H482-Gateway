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
 *     每 10ms  console_poll()        日志口(USART1)手敲 AT 转发给模组(台架调试用)
 *     每 1s    status_report()       状态打印(PA8 心跳已取消: 那是 4G_RST)
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
 *   2) OTA 需要 App 链接到 0x08002000(bootloader 跳转地址), 当前 Keil IROM1
 *      仍是 0x08000000, 因此默认关闭(MAIN_OTA_ENABLE=0)。
 *   3) 【2026-09 结构性改版后】g4_at_poll() 内的 AT 收发已全部非阻塞, 接收由
 *      g4_io_pump() 统一消费; 仍然"阻塞"的只剩日志口手工诊断(#at/#vbat/#dial)
 *      和 g4_at_get_clock() —— 它们内部照样跑收包泵, 因此不会丢接收, 只会让
 *      主循环多等一会儿。
 *   4) 云端下发必须是【原始二进制帧】, 长度对上协议帧(data_len 在帧里)。若云端
 *      改发 ASCII hex 文本, g4_io 的收包器会按 <payload_len> 把 hex 文本的前一半
 *      当 payload —— 两种格式必须和云端约定死一种(见 g4_io.h 顶部)。
 *   5) USB 主机功能已停用: 不调用 USBH_Init(), 但 usbh_*.c/usbhs_*.c 仍在工程里
 *      参与编译(usbh_user.c 引用 USBHS_Core/USB_Host, 符号定义在
 *      src/usbhs_bsp.c 末尾)。要彻底瘦身就把这些源文件从工程里移出, 连同那块
 *      定义一起删。
 *   6) json_proto.c 已无调用者(云端改成发二进制帧, 不再发 JSON), 可以把它从
 *      Keil 工程里移出; 不移也不占 Flash(armlink 会丢掉未引用段)。
 *
 * ==================== 【最小 LoRa 收发隔离测试】MAIN_LORA_RAW_TEST ====================
 *   现场"网关收不到对端任何一帧"的排障用开关(默认见 app/radio/llcc68/project_config.h):
 *     1 = main() 只起 时基/日志/射频, 每 5 秒发两条测试帧, 其余时间一直连续接收,
 *         并在【射频 IRQ 寄存器层】把收到的原始字节打出来(见本文件末尾 rawtest_run()
 *         与 app/radio/llcc68/radio.c 的 [RAW-IRQ]/[RAW-RX]); 4G / 业务 / 协议层 /
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
#include "data_cache.h"
#include "bsp_gpio.h"   /* G4_PowerSwitch_Init(): 4G 模组电源开关 PA8 */

#include "g4_io.h"
#include "g4_at.h"
#include "g4_urc.h"

#include "lora_proto.h"
#include "lora_business.h"
#include "serial_proto.h"
#include "ota.h"
#include "ota_http.h"     /* OTA over HTTP(S): AT+MHTTP* + MD5, 见 app/ota/ota_http.c */
#include "ota_port.h"     /* ota_port_reboot(): 校验通过后交给 bootloader 搬运 */
#include "ota_health.h"   /* 2026-10-11: A/B 回滚判定+计数(开机 boot_step / 主循环 poll) */
#include "cjson_port.h"      /* cJSON 入口: 静态池分配器 + hooks(见 app/cjson/cjson_port.h) */

#include "cloud_proto.h"     /* P1-2 信封/路由 + status.report 数据段 */
#include "last_value.h"      /* v53: sen.query/dev.query 的"最近一次上报"时刻 */
#include "level_supervisor.h"/* v54: 边缘水位评估的取数+串接层 */
#include "event_cache.h"     /* v69: 断线期间把事件存起来, 重连后补报 */
#include "cloud_profile.h"   /* P1-0 本地/文档 档位(topic / QoS / retained) */
#include "config_store.h"    /* P2-4: cfg.query 应答要用 CFG_JSON_MAX 定缓冲 */
#include "sensor_offline.h"  /* 液位失联判定(#offline 诊断用; 事件发布在下面) */

/* MAIN_LORA_RAW_TEST(最小 LoRa 收发隔离测试开关)的真相源。
 * 它的【本体与说明】写在 app/radio/llcc68/project_config.h: 那个头是
 * main.c / lora_proto.c / radio.c 三个编译单元都会包含的射频工程配置头,
 * 开关放那里才能被三处同时看到(C 的宏不跨 .c 传递)。
 * 这里显式包含一次, 免得下面的 #if 依赖别的头文件的包含顺序。 */
#include "project_config.h"

/* ---- 【只在最小收发测试模式下需要】射频层的只读回读接口 ----
 * 放在 #if 里: 开关=0 时这几个头一行都不引入, 保证正常固件的目标码
 * 与加这个开关之前【逐字节等价】(见文件末尾 rawtest_run() 与 main())。 */
#if MAIN_LORA_RAW_TEST
#include "radio.h"          /* RadioIrqStat()/RadioGetRxContinuous()/LLCC68GetOperatingMode() */
#include "llcc68-board.h"   /* LLCC68BusyLevel()/LLCC68BusyTimeoutCount()/LLCC68IsStuck() */
#include "llcc68.h"         /* LLCC68GetStats()/GetRssiInst()/GetStatus()/GetDeviceErrors() */
#endif

/* ==================== 功能开关 ====================
 * 都写成 #ifndef, 这样也能在 Keil 的 C/C++ -> Define 里覆盖,
 * 不用为了改一个开关去动代码。 */
#ifndef MAIN_AT_CONSOLE_ENABLE
#define MAIN_AT_CONSOLE_ENABLE   1   /* 日志口手敲 AT 转发(台架调试, 量产可关) */
#endif
#ifndef MAIN_SERIAL_INIT_ENABLE
#define MAIN_SERIAL_INIT_ENABLE  1   /* 初始化 RS232/RS485(协议待定, 仅建通道) */
#endif
#ifndef MAIN_OTA_ENABLE
#define MAIN_OTA_ENABLE          0   /* 旧的 MQTT 分包版 ota.c —— 保留但不再接线 */
#endif
/* OTA over HTTP(S) 通道(ota_http.c) —— 2026-10-02 起是【当前通道】, 由
 * cloud_proto 的 ota.start 驱动, 不受上面 MAIN_OTA_ENABLE 控制。
 * 这里只留两个上报节拍相关的参数。 */
#ifndef MAIN_OTA_PROGRESS_MS
#define MAIN_OTA_PROGRESS_MS     10000u  /* ota.progress 最长静默期(文档 7.6: 10% 或 10s) */
#endif
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
#ifndef MAIN_STATUS_JSON_ENABLE
#define MAIN_STATUS_JSON_ENABLE  1
#endif

/* 上报给云端的固件版本(status.report 的 fwVersion 字段) */
#ifndef MAIN_FW_VERSION
#define MAIN_FW_VERSION          "1.0.0"
#endif

/* 注: 这里只用 MAIN_STATUS_JSON_ENABLE 当守卫。
 * MAIN_G4_ENABLE / MAIN_GW_ONLINE_ENABLE 是在本文件【更靠后】才 #define 的,
 * 在这里用它们做条件会恒为 0, 整个函数会被编译掉(踩过一次)。 */
#if MAIN_STATUS_JSON_ENABLE
/* ---- 缓存时钟 ----
 * g4_at_get_clock() 是【阻塞调用】(最长 1s), 而主循环里 LoRa 每 1ms、4G 每 5ms
 * 都要轮询一次。如果每 30s 都去问模组, 就会把这两个节拍各压 1 秒。
 * 所以: 只在缓存过期(10 分钟)时才问一次, 平时用 tick 差值外推。
 * 模组不在网时 g4_at_get_clock() 会立刻返回 false, 不会白等。 */
static uint32_t s_epoch_base;      /* 上次问到的 Unix 秒 */
static uint32_t s_epoch_tick;      /* 问到的时刻(get_tick_ms) */
static bool     s_epoch_valid;

#define MAIN_EPOCH_STALE_MS   600000u   /* 10 分钟 */

static uint32_t main_now_unix(void)
{
    uint32_t tick = get_tick_ms();

    if (s_epoch_valid)
    {
        return s_epoch_base + ((uint32_t)(tick - s_epoch_tick) / 1000u);
    }
    return tick / 1000u;            /* 还没问到模组时间: 退回开机秒数 */
}

static void main_refresh_epoch(void)
{
    g4_clock_t clk;

    if (s_epoch_valid && ((uint32_t)(get_tick_ms() - s_epoch_tick) < MAIN_EPOCH_STALE_MS))
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

/**
 * @brief 组一条 status.report 并按档位发到 status topic(文档 7.7 / 任务 P1-8)
 *
 *   本地档位 -> 发到 "rt/gw/{gwId}/status"(现网正在看的那个), QoS1, 不 retained
 *              —— 不用换 broker 就能在云上看到 JSON
 *   文档档位 -> 发到 "rt/gw/{gwId}/status", QoS1 + retained(文档 2 章)
 *
 * ⚠ 字段口径(与 cloud_proto.h 的说明一致):
 *     rsrp = 用 AT+CSQ 换算的 RSSI(3GPP: rssi=-113+2*csq), 本工程拿不到真 RSRP;
 *     configVersion / mode / currentGear 在 P2(配置) / P3(边缘评估) 落地前是占位值。
 */
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

/** 当前实时档位(还没有过有效评估时返回 0) */
static uint8_t main_live_gear(void)
{
    return s_last_ld_valid ? s_last_ld.gear : 0U;
}

/** 当前实时状态名("Alarm"/"Warning"/"Normal"/"Resolved"); 没有有效评估时 "Normal" */
static const char *main_live_state_name(void)
{
    if (!s_last_ld_valid) { return "Normal"; }
    /* level_state_t 与 event_cache 的 EC_ST_* 数值完全一致(1..4), 直接复用同一份
     * 名字映射, 免得"同一个状态两处叫法不同"。 */
    return cloud_proto_state_name((uint8_t)s_last_ld.state);
}

/*!
 * \brief 填一份网关状态(供 status.report 与 sys.query_status 共用)
 * \note  ★ 只留这一处 ★ 在线数/信号/运行时长如果两处各算一份, 迟早会对不上
 *        (比如一边按从机表数、另一边按配置数)。sys.query_status 通过
 *        cloud_proto_set_status_provider() 直接调本函数。
 */
static void main_fill_status(cloud_status_t *st)
{
    static const char fw_ver[] = MAIN_FW_VERSION;
    lora_slave_t     *sl;
    uint8_t           cnt;
    uint8_t           i;

    (void)memset(st, 0, sizeof(*st));

    /* 子设备在线/总数 —— 来自 LoRa 从机表(已在维护 is_online) */
    sl  = lora_business_get_slave_list();
    cnt = lora_business_get_slave_count();
    for (i = 0u; (i < cnt) && (sl != NULL); i++)
    {
        if (sl[i].dev_type == SLAVE_TYPE_ALARM_OUT)
        {
            st->devices_total++;
            if (sl[i].is_online)
            {
                st->devices_online++;
            }
        }
        else
        {
            st->sensors_total++;
            if (sl[i].is_online)
            {
                st->sensors_online++;
            }
        }
    }

    st->online         = true;
    st->fw_version     = fw_ver;
    st->config_version = config_store_version();   /* P2 已落地, 不再是占位的 0 */
    st->rsrp_dbm       = g4_at_get_info()->rssi_dbm;
    st->uptime_s       = get_tick_ms() / 1000u;
    /* ★ 2026-09-30: 不再是占位值 —— 取边缘评估算出的【实时】档位/状态。
     *   以前这里恒填 "auto"/0, 平台看到的档位永远是 0。
     *   mode 暂时仍恒 "auto": manual 模式要等第 6 条(dev.ctrl)落地后才有第二种取值。 */
    st->mode           = "auto";
    st->current_gear   = main_live_gear();
}

static sys_err_t main_publish_status_json(void)
{
    static uint16_t   seq;
    cloud_status_t    st;
    char              data[CLOUD_DATA_MAX];
    char              payload[CLOUD_RESP_MAX];
    char              topic[64];
    char              msg_id[20];
    uint16_t          dlen;
    uint16_t          plen;
    uint16_t          tlen;

    main_refresh_epoch();

    /* 状态字段统一由 main_fill_status() 填 —— 与 sys.query_status 共用一处 */
    main_fill_status(&st);

    dlen = cloud_proto_build_status_data(data, (uint16_t)sizeof(data), &st);
    if (dlen == 0u)
    {
        return SYS_ERR;
    }

    seq++;
    (void)snprintf(msg_id, sizeof(msg_id), "h%03u", (unsigned)(seq % 1000u));

    plen = cloud_proto_build_uplink(payload, (uint16_t)sizeof(payload),
                                    "status.report", msg_id, main_now_unix(), data);
    if (plen == 0u)
    {
        return SYS_ERR;
    }

    tlen = cloud_profile_build_topic(CLOUD_TOPIC_STATUS, topic, (uint16_t)sizeof(topic));
    if (tlen == 0u)
    {
        return SYS_ERR;
    }

    /* 【弱网关键】上报正文也打到串口: 离线时 MQTT 发不出去, 但"我们准备发的到底
     * 是什么"必须看得见, 否则无法判断是协议层错了还是网络没通。 */
    log_info("[CLOUD] 上报 %s (%u 字节): %s\r\n", topic, (unsigned)plen, payload);

    return g4_at_mqtt_publish_ex(topic, (const uint8_t *)payload, plen,
                                 cloud_profile_topic_qos(CLOUD_TOPIC_STATUS),
                                 cloud_profile_topic_retained(CLOUD_TOPIC_STATUS));
}

/* ==================================================================
 * telemetry.report —— 水位遥测上行(文档 7.1, 用户 2026-09-30 第 3 条)
 * ==================================================================
 * 【触发条件】(文档: "周期上报(周期 = network.statusIntervalSeconds) 或水位变化
 *   超过 minChangeCm 时触发")
 *     · 周期 = config_store.network.statusIntervalSeconds(10~600s, 默认 60);
 *     · 变化量 = |本次有效水位 - 上次遥测的水位| >= thresholds.minChangeCm。
 *   上电第一条恒发(不然云端要等一个周期才看得到水位)。
 *
 * 【★ 水位怎么算: 取【平均值】, 以及为什么 ★】
 *   用户口径(2026-09-30 第 3 条): 遥测里的水位取各液位传感器的【平均值】
 *   (最多 4 台), 不按文档那套"逐台 + 有效水位取 max + 逐台防抖"来做。
 *   文档 3.3 末的"有效水位 = 取 max"仍然保留给【告警判定/联动】用
 *   (level_eval / level_supervisor 一行没动) —— 也就是:
 *       告警按 max(保守, 有一点淹就报), 遥测按 mean(反映整体水面)。
 *   这个差别是有意的, 写在这里免得以后有人"顺手统一"。
 *
 * 【★ 各路液位不是同一时刻的 —— 处理方式 ★】
 *   这是平均值最容易被忽略的坑: 4 台传感器各自 60s 上报一次, 收到某台的新值时,
 *   另外 3 台手里的是【几十秒前的旧值】。如果直接平均, 就会出现"水面正在涨,
 *   平均却在降"的错觉。本实现这样处理(都是显式规则, 不是"差不多就行"):
 *     ① 只把【在线且 enabled】的算进来 —— 在线判据复用 sensor_offline 那一套
 *        (超过 3×reportPeriodSeconds 没上报 = 离线), 与"失联检测"口径一致;
 *     ② 每台在 data.sensors[] 里【各自带自己的 ts】(文档 7.1 本来就有这个字段)
 *        —— 平台能直接看出这几包是不是同一时刻的, 这是最诚实的做法:
 *        网关不假装它们同时到达, 也不替平台去加权;
 *     ③ 平均值是"快照"(各台的最新值), 不是"同一时刻的插值" —— 所以本字段适合
 *        看趋势/展示, 不适合当"某一瞬间的真实水位"做计量。要更严谨得上时间加权,
 *        那属于平台侧的事(它拿得到每台的 ts)。
 *     ④ 只有一台在线时, 平均值就等于它自己(退化成单点值), 这是可接受的。
 * ================================================================== */

#define MAIN_TELEMETRY_RETRY_MS   5000U   /* 发布登记失败后多久重试(不占满整个周期) */

/* 遥测的时基与状态(纯 RAM) */
static struct
{
    uint32_t attempt_ms;   /* 上次【尝试】发布的 tick */
    uint32_t ok_ms;        /* 上次【成功】发布的 tick(周期基准) */
    int32_t  last_mm;      /* 上次成功发布时的平均值(变化量基准) */
    bool     have;         /* 成功发过至少一条 */
    bool     fail;         /* 上次尝试失败(只影响"提前重试") */
} s_tel;
static uint16_t s_tel_seq;

/** 心跳/遥测共用的"周期秒数"取值(带兜底) */
static uint32_t main_cfg_period_ms(uint16_t sec, uint32_t fallback_ms)
{
    if ((sec == 0U) || (sec > 3600U)) { return fallback_ms; }
    return (uint32_t)sec * 1000U;
}

/**
 * @brief 组装并发布 telemetry.report
 * @param avg_mm 各在线液位传感器的平均水位(mm)
 * @return SYS_OK = 已交给模组
 */
static sys_err_t main_publish_telemetry(int32_t avg_mm)
{
    static cloud_telemetry_sensor_t sens[CFG_SENSOR_MAX];
    static char        data[CLOUD_RESP_MAX];
    static char        payload[CLOUD_RESP_MAX];
    static char        topic[64];
    const config_store_t *cfg = config_store_get();
    cloud_telemetry_t  t;
    uint8_t            n = 0u;
    uint8_t            i;
    char               msg_id[20];
    uint16_t           dlen;
    uint16_t           plen;
    uint32_t           now = get_tick_ms();

    if (cfg == NULL) { return SYS_ERR; }

    (void)memset(&t, 0, sizeof(t));
    (void)memset(sens, 0, sizeof(sens));

    /* ---- 逐台填: 只列【启用的 + 有最近值 + 还没超时】的液位传感器 ---- */
    for (i = 0u; (i < cfg->sensor_count) && (i < (uint8_t)CFG_SENSOR_MAX); i++)
    {
        const cfg_sensor_t *s = &cfg->sensors[i];
        uint8_t      uid3[3];
        last_value_t lv;
        uint32_t     thr;

        if (s->enabled == 0u) { continue; }
        if (!cfg_uid_to_bytes(s->uid, uid3)) { continue; }
        if (!last_value_get(uid3, &lv)) { continue; }        /* 从没上报过 */

        /* 在线判据与"失联检测"完全同一套: 3 × reportPeriodSeconds */
        thr = sensor_offline_threshold_ms(s->report_period_s);
        if ((uint32_t)(now - lv.rx_tick_ms) >= thr) { continue; }   /* 已超时 -> 不参与 */

        sens[n].uid         = s->uid;
        sens[n].level_mm    = (int32_t)lv.level_mm;
        sens[n].raw_mm      = (uint32_t)lv.level_mm;   /* 液位透传(D-1): 网关不做换算, 原始值就是它 */
        sens[n].battery_pct = (uint8_t)((lv.bat_pct > 100u) ? 100u : lv.bat_pct);
        sens[n].rssi_dbm    = lv.rssi_dbm;              /* 第 10 条: 该帧的 LoRa RSSI */
        sens[n].ts          = lv.rx_unix;               /* 各自的时间戳, 不假装同时 */
        n++;
    }

    t.sensor_count        = n;
    t.sensors             = sens;
    t.effective_level_mm  = avg_mm;
    t.effective_status    = main_live_state_name();
    t.effective_gear      = main_live_gear();

    dlen = cloud_proto_build_telemetry_data(data, (uint16_t)sizeof(data), &t);
    if (dlen == 0u)
    {
        log_info("[CLOUD] !! telemetry.report data 段组装失败\r\n");
        return SYS_ERR;
    }

    s_tel_seq++;
    (void)snprintf(msg_id, sizeof(msg_id), "m%03u", (unsigned)(s_tel_seq % 1000u));

    plen = cloud_proto_build_uplink(payload, (uint16_t)sizeof(payload),
                                    "telemetry.report", msg_id, main_now_unix(), data);
    if (plen == 0u)
    {
        log_info("[CLOUD] !! telemetry.report 信封组装失败\r\n");
        return SYS_ERR;
    }

    if (cloud_profile_build_topic(CLOUD_TOPIC_TELEMETRY, topic,
                                  (uint16_t)sizeof(topic)) == 0u)
    {
        log_info("[CLOUD] !! telemetry topic 拼装失败\r\n");
        return SYS_ERR;
    }

    log_info("[CLOUD] 上报 %s (%u 字节): %s\r\n", topic, (unsigned)plen, payload);

    return g4_at_mqtt_publish_ex(topic, (const uint8_t *)payload, plen,
                                 cloud_profile_topic_qos(CLOUD_TOPIC_TELEMETRY),
                                 cloud_profile_topic_retained(CLOUD_TOPIC_TELEMETRY));
}

/**
 * @brief 算平均值 + 判"该不该发遥测"
 * @param out_avg 输出: 平均值(mm); 没有任何有效传感器时不改它
 * @return true = 本次应发; false = 不发
 * @note  只有【在线且 enabled】的传感器参与平均(规则见上面长注释)。
 */
static bool main_telemetry_ready(const config_store_t *cfg, uint32_t now_ms,
                                 int32_t *out_avg)
{
    int32_t  sum = 0;
    uint8_t  n   = 0u;
    uint8_t  i;
    int32_t  delta;

    if ((cfg == NULL) || (out_avg == NULL)) { return false; }

    for (i = 0u; (i < cfg->sensor_count) && (i < (uint8_t)CFG_SENSOR_MAX); i++)
    {
        const cfg_sensor_t *s = &cfg->sensors[i];
        uint8_t      uid3[3];
        last_value_t lv;

        if (s->enabled == 0u) { continue; }
        if (!cfg_uid_to_bytes(s->uid, uid3)) { continue; }
        if (!last_value_get(uid3, &lv)) { continue; }
        if ((uint32_t)(now_ms - lv.rx_tick_ms) >= sensor_offline_threshold_ms(s->report_period_s))
        {
            continue;
        }
        sum += (int32_t)lv.level_mm;
        n++;
    }

    if (n == 0u) { return false; }                 /* 一台有效的都没有 -> 没什么可报 */

    *out_avg = sum / (int32_t)n;                   /* 整数平均, 单位 mm, 不引浮点 */

    /* ① 第一条恒发 */
    if (!s_tel.have) { return true; }

    /* ② 上次登记失败 -> 5s 后重试(不占满整个周期, 弱网下水位能早点上去) */
    if (s_tel.fail
        && ((uint32_t)(now_ms - s_tel.attempt_ms) >= MAIN_TELEMETRY_RETRY_MS))
    {
        return true;
    }

    /* ③ 周期到(周期 = network.statusIntervalSeconds, 与心跳同一个字段) */
    if ((uint32_t)(now_ms - s_tel.ok_ms)
        >= main_cfg_period_ms(cfg->network.status_interval_s, 60000U))
    {
        return true;
    }

    /* ④ 变化量 >= minChangeCm(文档 7.1 第二条腿) */
    delta = *out_avg - s_tel.last_mm;
    if (delta < 0) { delta = -delta; }
    if (delta >= (int32_t)cfg->thresholds.min_change_cm * 10) { return true; }

    return false;
}

/* ==================================================================
 * 液位传感器【全部失联】时的 fail-safe 动作(文档第 5 章, 用户第 7 条)
 * ==================================================================
 * 【什么时候进来】level_supervisor 判出"曾经有水位、现在一台在线的都没有",
 *   并且在【刚跌进去的那一拍】给 failsafe_enter=true(边沿, 只做一次)。
 * 【做什么】按 rules.sensorOfflinePolicy:
 *     keep  -> 保持当前档位(什么都不发, 只记日志)
 *     alarm -> 强制档位 3(告警)
 *     off   -> 执行档位 0(解除)
 * 【文本用什么水位渲染】全失联时没有"当前水位", 用【最后一次已知水位】——
 *   这是唯一合理的选择; 档位 0 的模板通常是空文本, 所以 off 基本只会关屏/关声。
 * 【为什么要 mark_sent】强制改档之后去重基准得跟着走 —— 否则传感器恢复回来时会
 *   拿"上一次自动下发的档位"去比, 该发的不发或白重发一条。
 * ================================================================== */
static void main_failsafe_step(const level_decision_t *ld)
{
    const config_store_t *cfg = config_store_get();
    uint8_t policy;
    char    led1[CFG_LEDTEXT_MAX + 1u];
    char    led2[CFG_LEDTEXT_MAX + 1u];
    char    tts[CFG_TTSTEXT_MAX + 1u];
    int32_t lv = 0;
    uint8_t g  = 0u;

    if ((cfg == NULL) || (ld == NULL) || (!ld->failsafe_enter)) { return; }

    policy = cfg->rules.sensor_offline_policy;

    log_info("[LEVEL] !! 液位传感器【全部失联】-> fail-safe 策略=%s, 应执行档位 %u\r\n",
             (policy == (uint8_t)CFG_OFFLINE_ALARM) ? "alarm" :
             (policy == (uint8_t)CFG_OFFLINE_OFF)   ? "off"   : "keep",
             (unsigned)ld->failsafe_gear);

    if (policy == (uint8_t)CFG_OFFLINE_KEEP)
    {
        return;                     /* 保持当前档位 = 什么都不做 */
    }

    led1[0] = '\0';
    led2[0] = '\0';
    tts[0]  = '\0';
    if (level_supervisor_last(&lv, &g) && (ld->failsafe_gear < (uint8_t)CFG_GEAR_COUNT))
    {
        (void)level_render_template(led1, (uint16_t)sizeof(led1),
                                    cfg->gears[ld->failsafe_gear].led_text1, lv);
        (void)level_render_template(led2, (uint16_t)sizeof(led2),
                                    cfg->gears[ld->failsafe_gear].led_text2, lv);
        (void)level_render_template(tts, (uint16_t)sizeof(tts),
                                    cfg->gears[ld->failsafe_gear].tts_text, lv);
    }

    if (lora_business_apply_gear_ex(ld->failsafe_gear, NULL, led1, led2, tts,
                                    NULL, 0u, NULL) > 0)
    {
        level_supervisor_mark_sent(ld->failsafe_gear, led1, led2);   /* 见上方说明 */
    }
}

/* ==================================================================
 * manual 模式窗口(用户第 6 条)
 * ==================================================================
 * 【分工】"manual 期间抑制自动联动"在 level_supervisor.c 里(它直接读 rules.mode);
 *   这里只管【回归】: manualTimeoutSeconds 到期把 mode 写回 auto, 并强制重评估一次
 *   (文档 5.5: "切回 auto 时网关立即按当前水位重评估一次并刷新联动状态")。
 * 【计时基准】进入 manual 的时刻; 若 manual 期间【配置版本变了】(又下发了一次
 *   dev.ctrl / rule.mode), 窗口重新计时 —— 这符合"每次手动控制后重新开始计时"的
 *   直觉, 代价是任何别的配置改动也会顺带刷新窗口(已在此注明)。
 * 【0 = 不回归】(文档 3.7: manualTimeoutSeconds 0 = 不回归)。
 * 【为什么放在这里而不是 cloud_proto】回归是"时间"触发的, 主循环才是时间的主人;
 *   cloud_proto 只负责"用户下命令"这一侧。
 * ================================================================== */
static uint32_t s_man_start_ms;      /* 本次 manual 的起点 */
static uint32_t s_man_cfg_ver;       /* 起点时的配置版本(变了就重新计时) */
static bool     s_man_was;           /* 上一拍是不是 manual(用于检测"回到 auto") */

static void main_manual_step(void)
{
    const config_store_t *cfg = config_store_get();
    uint32_t now = get_tick_ms();
    uint32_t ver;

    if (cfg == NULL) { return; }
    ver = config_store_version();

    /* ---- 不在 manual: 若刚从 manual 回来, 强制重评估一次 ---- */
    if (cfg->rules.mode != (uint8_t)CFG_RULE_MANUAL)
    {
        if (s_man_was)
        {
            s_man_was = false;
            log_info("[CTRL] 已回到 auto 模式(自动联动恢复), 强制重评估一次\r\n");
            level_supervisor_force_next();
        }
        return;
    }

    /* ---- 刚进 manual, 或 manual 期间配置又变过 -> (重新)开始计时 ---- */
    if ((!s_man_was) || (ver != s_man_cfg_ver))
    {
        s_man_was      = true;
        s_man_start_ms = now;
        s_man_cfg_ver  = ver;
        log_info("[CTRL] 进入 manual 模式: 自动联动已抑制, manualTimeoutSeconds=%us%s\r\n",
                 (unsigned)cfg->rules.manual_timeout_s,
                 (cfg->rules.manual_timeout_s == 0u) ? "(0=不回归)" : "");
        return;
    }

    if (cfg->rules.manual_timeout_s == 0u) { return; }      /* 不回归 */

    if ((uint32_t)(now - s_man_start_ms)
        >= ((uint32_t)cfg->rules.manual_timeout_s * 1000U))
    {
        config_store_t tmp;

        (void)memcpy(&tmp, cfg, sizeof(tmp));
        tmp.rules.mode = (uint8_t)CFG_RULE_AUTO;

        if (config_store_save(&tmp) == CFG_OK)
        {
            /* ★★2026-10-14 修复: 必须把 tmp 提交给"当前配置"实例 ★★
             * ------------------------------------------------------------------
             * 【漏了什么】config_store_save() 只负责【落 Flash】, 它【不更新】
             *   RAM 里那份"当前配置"。调用方必须自己 memcpy 到
             *   config_store_get_mut() —— 对比 cloud_proto.c 的 sys.factory_reset:
             *   那处 save 之后有一句 "落盘成功才提交给当前配置实例" 的 memcpy, 这里漏了。
             *   那处有, 这里漏了。
             *
             * 【后果(2026-10-14 用户实测, 三条症状同时出现)】
             *   ① 云端 status.report 的 mode 恒为 manual —— 因为它读的是 RAM;
             *   ② manual 抑制一直生效 -> 液位变化不再联动声光设备;
             *   ③ 本次 save 抬了配置版本 -> 下一拍 line 682 的
             *      `ver != s_man_cfg_ver` 成立 -> 走"刚进 manual, 重新计时"分支
             *      -> 每 manual_timeout_s 重复一次, 永远回不到 auto。
             *   ⇒ 一个遗漏解释全部三个现象。
             *
             * 【提交之后】下一拍 line 670 会走"已回到 auto"那支:
             *   打 "[CTRL] 已回到 auto 模式(自动联动恢复), 强制重评估一次"
             *   并调 level_supervisor_force_next() -> 档位按当前水位重算下发。 */
            (void)memcpy(config_store_get_mut(), &tmp, sizeof(tmp));

            log_info("[CTRL] manual 超时(%us) -> 自动回归 auto\r\n",
                     (unsigned)cfg->rules.manual_timeout_s);
            /* 不在这里置 s_man_was=false: 下一拍会发现 mode 已经不是 manual,
             * 走上面那支"回到 auto"并强制重评估 —— 一处收口, 不会漏。 */
        }
        else
        {
            log_info("[CTRL] !! manual 超时, 但写回 auto 失败(Flash), 下一拍重试\r\n");
        }
    }
}

/**
 * @brief 遥测的一步(算平均 -> 判据 -> 发布 -> 回填时基)
 * @note  只在主循环 1s 节拍里调; 4G 不在线时直接跳过(模组自己会拒),
 *        但【不】刷新成功基准, 免得"没发出去也算发过"。
 */
static void main_telemetry_step(void)
{
    const config_store_t *cfg = config_store_get();
    uint32_t now = get_tick_ms();
    int32_t  avg = 0;

    if ((cfg == NULL) || (!g4_at_is_online())) { return; }
    if (!main_telemetry_ready(cfg, now, &avg)) { return; }

    s_tel.attempt_ms = now;

    if (main_publish_telemetry(avg) == SYS_OK)
    {
        s_tel.ok_ms   = now;
        s_tel.last_mm = avg;
        s_tel.have    = true;
        s_tel.fail    = false;
    }
    else
    {
        s_tel.fail = true;
        log_info("[CLOUD] !! telemetry.report 发布登记失败, %us 后重试\r\n",
                 (unsigned)(MAIN_TELEMETRY_RETRY_MS / 1000U));
    }
}

/**
 * @brief 组一条 event.device(文档 7.4) 发到 event topic —— 液位失联/恢复
 *
 * 文档 7.4 原文:
 *   { "event": "event.device", "msgId": "v001", "ts": 1735689600,
 *     "data": { "addr": 1, "devType": "sensor", "event": "offline" } }
 *   event 取值: bound / bind_failed / unbound / online / offline / low_voltage。
 *
 * 【字段口径】: 本工程子设备寻址是 3 字节 UID 的 hex 串(偏差 D-8), 文档那个
 * uint16 的 "addr" 装不下, 所以 data 里用 "uid":"210001"; "devType" 与 "event"
 * 两个字段名与取值【严格照文档】。
 *
 * 【这个函数是被 lora_business 回调进来的】: 判定与去重都在
 * lora_business.c + sensor_offline.c 里, 这里只管"组包 + 登记发布"。
 *
 * 【弱网】: 发布登记失败只打日志 —— g4_at_mqtt_publish_ex() 本身是非阻塞登记队列,
 * 而且事件不像配置应答那样必须让平台知道, 绝不能因为它去阻塞主循环。
 */
static void main_publish_device_event(const char *uid_hex, bool online)
{
    /* 一律 static: 栈只有 4KB(见 startup_n32h47x_48x.s 的 Stack_Size) */
    static char     data[96];
    static char     payload[CLOUD_RESP_MAX];
    static char     topic[64];
    static uint16_t seq;
    char            msg_id[20];
    cjw_t           w;
    uint16_t        plen;
    uint16_t        tlen;

    cjw_init(&w, data, (uint16_t)sizeof(data));
    cjw_obj_begin(&w);
    cjw_kv_str(&w, "uid", (uid_hex != NULL) ? uid_hex : "");
    cjw_kv_str(&w, "devType", "sensor");
    cjw_kv_str(&w, "event", online ? "online" : "offline");
    cjw_obj_end(&w);
    if (!cjw_ok(&w))
    {
        log_info("[CLOUD] !! event.device 的 data 段组装失败(缓冲不足?)\r\n");
        return;
    }
    (void)cjw_len(&w);      /* v91: 本函数不再需要长度(信封自带) —— 原来算给 dlen 但没用到 */

    seq++;
    (void)snprintf(msg_id, sizeof(msg_id), "v%03u", (unsigned)(seq % 1000u));

    plen = cloud_proto_build_uplink(payload, (uint16_t)sizeof(payload),
                                    "event.device", msg_id, main_now_unix(), data);
    if (plen == 0u)
    {
        log_info("[CLOUD] !! event.device 信封组装失败\r\n");
        return;
    }

    tlen = cloud_profile_build_topic(CLOUD_TOPIC_EVENT, topic, (uint16_t)sizeof(topic));
    if (tlen == 0u)
    {
        log_info("[CLOUD] !! event topic 拼装失败, 事件丢弃\r\n");
        return;
    }

    /* 发不出去也要把正文打出来: 弱网下串口是唯一能确认"我们准备发什么"的地方
     * (与 main_publish_status_json / main_cloud_downlink_try 同一套做法) */
    log_info("[CLOUD] 上报 %s (%u 字节): %s\r\n", topic, (unsigned)plen, payload);

    if (g4_at_mqtt_publish_ex(topic, (const uint8_t *)payload, plen,
                              cloud_profile_topic_qos(CLOUD_TOPIC_EVENT),
                              cloud_profile_topic_retained(CLOUD_TOPIC_EVENT)) != SYS_OK)
    {
        log_info("[CLOUD] !! 事件发布登记失败(离线/队列满), 这条事件只记在本机\r\n");
    }
}

/* ==================================================================
 * event.device: bound / bind_failed / unbound(P5, 设备绑定/解绑)
 * ==================================================================
 * 与上面的 main_publish_device_event(online/offline)【同 topic 同 data 结构】,
 * 只是 event 字段不同。由 lora_business_set_bind_event_cb 注册给业务层,
 * 在 0x04 绑定成功/失败、0x05 解绑完成时各调一次。
 *
 * 异步: 0x04/0x05 完成后才触发本回调, 那时云端命令响应(sen.bind/dev.unbind 的
 * 应答 code 0)早就发回去了 —— 云端靠这条事件确认"真绑/真删"完成。
 *
 * 与 main_publish_device_event 一样, 静态缓冲 + 发布失败只记本机不阻塞。
 * ★ 不进 event_cache(断线补报队列): 绑定/解绑是即时事件, 平台收不到时会再下发
 *   命令重试, 不需要补报(补报反而可能让平台收到"重复 bound"造成误判)。
 *
 * 【2026-09-29 补回】本函数与下面那两行注册在 14:06 的一次改动里被误删,
 *   导致 sen.bind/dev.bind 恒回 1006(pair_hook 为空)、解绑完成事件也发不出去。
 *   从 v105 归档原样取回。 */
static void main_publish_bind_event(const char *uid_hex, bool is_sensor,
                                     lora_bind_event_t event)
{
    static char     data[96];
    static char     payload[CLOUD_RESP_MAX];
    static char     topic[64];
    static uint16_t seq;
    char            msg_id[20];
    cjw_t           w;
    uint16_t        plen;
    uint16_t        tlen;
    const char     *ev_str;

    ev_str =
        (event == LORA_BIND_EVENT_BOUND)          ? "bound"          :
        (event == LORA_BIND_EVENT_BIND_FAILED)    ? "bind_failed"    :
        (event == LORA_BIND_EVENT_UNBOUND)        ? "unbound"        :
        (event == LORA_BIND_EVENT_UNBIND_FAILED)  ? "unbind_failed"  :
        (event == LORA_BIND_EVENT_UNBIND_FAULT)   ? "unbind_fault"   : "unknown";

    cjw_init(&w, data, (uint16_t)sizeof(data));
    cjw_obj_begin(&w);
    cjw_kv_str(&w, "uid",     (uid_hex != NULL) ? uid_hex : "");
    cjw_kv_str(&w, "devType", is_sensor ? "sensor" : "device");
    cjw_kv_str(&w, "event",   ev_str);
    cjw_obj_end(&w);
    if (!cjw_ok(&w))
    {
        log_info("[CLOUD] !! event.device(bind) data 段组装失败\r\n");
        return;
    }

    seq++;
    (void)snprintf(msg_id, sizeof(msg_id), "b%03u", (unsigned)(seq % 1000u));

    plen = cloud_proto_build_uplink(payload, (uint16_t)sizeof(payload),
                                    "event.device", msg_id, main_now_unix(), data);
    if (plen == 0u)
    {
        log_info("[CLOUD] !! event.device(bind) 信封组装失败\r\n");
        return;
    }

    tlen = cloud_profile_build_topic(CLOUD_TOPIC_EVENT, topic, (uint16_t)sizeof(topic));
    if (tlen == 0u)
    {
        log_info("[CLOUD] !! event topic 拼装失败, 绑定事件丢弃\r\n");
        return;
    }

    log_info("[CLOUD] 上报 %s (%u 字节): %s\r\n", topic, (unsigned)plen, payload);

    if (g4_at_mqtt_publish_ex(topic, (const uint8_t *)payload, plen,
                              cloud_profile_topic_qos(CLOUD_TOPIC_EVENT),
                              cloud_profile_topic_retained(CLOUD_TOPIC_EVENT)) != SYS_OK)
    {
        log_info("[CLOUD] !! 绑定事件发布登记失败, 这条事件只记在本机\r\n");
    }
}

/* ★ 第 12 条(2026-09-30): 事件 msgId 的【唯一序号】—— 在线发送与"断线入缓存"
 * 两条路【共用同一个计数器】。
 * 【为什么必须共用】以前两边各有一个 static seq: 同一个事件在线时叫 `s001`,
 * 断线入缓存、之后补报时被重新分配成 `b001` —— 平台按 msgId 做幂等/去重时对不上
 * (该去重的去不掉, 或者把同一条当成两条)。
 * 现在: 入缓存时就把号码记进记录(ec_record_t.msg_seq, 2 B/条), 补报时原样还原。
 * ★2026-10-08: 声明【上移到本文件靠前处】★ —— 声光动作结果(main_on_alarm_result)
 *   也要用它, 那个函数在 status_change 之前。原来它定义在下面, 编译报未声明。 */
static uint16_t s_evt_seq;

/* ==================================================================
 * event.action：声光联动【执行结果】上报（2026-10-08 用户 A1/A3）
 * ==================================================================
 * 由 lora_business 的声光(0x03)状态机回调 —— 设备对 0x03 会先后回
 * 0x80(收到了) → 0x82(成功) / 0x83(失败), 语义见 lora_proto.h。
 *
 * 【三种回调情形, 只有两种要发 MQTT】
 *   ok=true              -> 收到 0x82: 发 event.action{code 0}
 *   ok=false, gave_up=假 -> 中间失败(还会重发): 【什么都不发】——
 *                           否则平台会看到三次"失败", 而其实第三次成功了
 *   ok=false, gave_up=真 -> 三条都没成功: 发 event.action{失败}
 *                           【并且额外】发一条 event.device{"offline"}
 *                           (用户口径: "三次都没响应需要上报声光报警器离线")
 *
 * 【trigger 怎么来的】声光状态机不知道"谁触发的"(那是 level_supervisor 的事),
 *   这里按当前 rules.mode 反推: manual -> "manual", 其余 -> "auto"。
 *   够用且不撒谎 —— failsafe 触发的动作在 manual 模式下本来也不会发。
 * ================================================================== */
static void main_on_alarm_result(const char *uid_hex, uint8_t gear,
                                 bool ok, bool gave_up)
{
    static char            data[192];
    static char            payload[CLOUD_RESP_MAX];
    static char            topic[64];
    static uint16_t        seq;
    char                   msg_id[20];
    cloud_action_result_t  res;
    cloud_action_event_t   ae;
    const config_store_t  *cfg = config_store_get();
    uint16_t               plen;
    uint16_t               tlen;

    if (!ok && !gave_up)
    {
        return;                     /* 中间失败: 不打扰平台 */
    }
    if ((uid_hex == NULL) || (uid_hex[0] == '\0'))
    {
        return;
    }

    res.uid  = uid_hex;
    /* 0 = 成功; 非 0 = 失败。0x83 的 data 为空, 拿不到更细的原因,
     * 所以统一用 3001 表示"设备未能完成动作 / 无响应"。 */
    res.code = ok ? 0 : 3001;

    ae.gear    = gear;
    ae.trigger = ((cfg != NULL) && (cfg->rules.mode == (uint8_t)CFG_RULE_MANUAL))
                     ? "manual" : "auto";
    ae.result_count = 1u;
    ae.results      = &res;

    if (cloud_proto_build_action_data(data, (uint16_t)sizeof(data), &ae) == 0u)
    {
        log_info("[CLOUD] !! event.action data 段组装失败\r\n");
        return;
    }

    /* ★★2026-10-08: 断线时【入缓存补报】, 在线时直发★★
     * 判据与 status_change 完全一致(只有一个: 4G 在线), 不引入第二套"能发吗"。
     * 用户 2026-10-08 选定"甲"方案: 每台设备单独一条记录(seq = 0), 结果码存 aux。
     * 渲染/补报那一侧已支持(见 cloud_proto.c 里 EC_KIND_ACTION 的两个分支)。
     *
     * ⚠️ 为什么联动结果值得补报: 平台靠它判断"灯到底亮没亮"。
     *   状态迁移事件丢了顶多晚一点知道, 这个丢了就永远不知道那次联动失败了。 */
    if (!g4_at_is_online())
    {
        ec_record_t rec;
        uint8_t     uid3[3];

        if (!cfg_uid_to_bytes(uid_hex, uid3))
        {
            log_info("[CLOUD] !! %s 不是合法 uid, 动作结果无法入缓存\r\n", uid_hex);
            return;
        }

        (void)memset(&rec, 0, sizeof(rec));
        rec.kind = (uint8_t)EC_KIND_ACTION;
        (void)memcpy(rec.uid, uid3, 3U);
        rec.a    = gear;                    /* 档位 */
        rec.b    = (uint8_t)(((cfg != NULL) &&
                              (cfg->rules.mode == (uint8_t)CFG_RULE_MANUAL))
                                 ? EC_TRIG_MANUAL : EC_TRIG_AUTO);
        rec.ts   = main_now_unix();
        rec.seq  = 0U;                      /* ★甲方案: 每台一条, 不分组★ */
        ec_action_set_code(&rec, res.code); /* ★结果码必须走访问器★ */
        s_evt_seq = (uint16_t)((s_evt_seq + 1u) % 1000u);
        rec.msg_seq = s_evt_seq;            /* 与在线路径共用同一个序号计数器 */
        (void)event_cache_push(&rec);
        log_info("[CLOUD] 4G 不在线 -> 动作结果(uid=%s gear=%u code=%d)入补报缓存\r\n",
                 uid_hex, (unsigned)gear, (int)res.code);

        /* 离线时"设备离线"这件事同样要报 —— 平台做离线告警靠它。
         * (它自己内部也会按在线/离线决定直发还是入缓存, 这里不用重复判断。) */
        if (gave_up)
        {
            main_publish_device_event(uid_hex, false);
        }
        return;
    }

    seq++;
    (void)snprintf(msg_id, sizeof(msg_id), "a%03u", (unsigned)(seq % 1000u));

    plen = cloud_proto_build_uplink(payload, (uint16_t)sizeof(payload),
                                    "event.action", msg_id, main_now_unix(), data);
    if (plen == 0u)
    {
        log_info("[CLOUD] !! event.action 信封组装失败\r\n");
        return;
    }

    tlen = cloud_profile_build_topic(CLOUD_TOPIC_EVENT, topic, (uint16_t)sizeof(topic));
    if (tlen == 0u)
    {
        log_info("[CLOUD] !! event topic 拼装失败, 动作结果丢弃\r\n");
        return;
    }

    log_info("[CLOUD] 上报 %s (%u 字节): %s\r\n", topic, (unsigned)plen, payload);

    if (g4_at_mqtt_publish_ex(topic, (const uint8_t *)payload, plen,
                              cloud_profile_topic_qos(CLOUD_TOPIC_EVENT),
                              cloud_profile_topic_retained(CLOUD_TOPIC_EVENT)) != SYS_OK)
    {
        log_info("[CLOUD] !! event.action 发布登记失败, 只记在本机\r\n");
    }

    /* ★重试用尽 ⇒ 额外报一条"设备离线"★
     * 两条都发是刻意的: event.action 给"这次动作的明细", event.device 给
     * "这台设备掉了"这个状态 —— 平台做离线告警只需要盯后者一种形状。
     * 复用液位失联那同一个函数, 平台侧处理逻辑完全一致。 */
    if (gave_up)
    {
        log_info("[CLOUD] !! %s 连续 3 条 0x03 都没成功 -> 补报设备离线\r\n", uid_hex);
        main_publish_device_event(uid_hex, false);
    }
}
/* ==================================================================
 * event.status_change：直发 / 入缓存 (v72)
 * ==================================================================
 * 状态迁移时:
 *   · 4G 在线 -> **直发** `event.status_change`；
 *   · 断线   -> 写进事件缓存, 重连后由 v70 的补报发出去。
 * ★ 两支【互斥】★ —— 都做会让平台对同一件事收到两次。
 *
 * 还有一个"其实发不出去"的第三种情况: 在线判据成立、但发布**登记失败**
 * (队列满/刚掉线)。这时**退回缓存** —— 否则这条迁移就凭空没了。
 * 判据仍只有一个(4G 在线), 只是失败后补救, 不引入第二套"能发吗"。
 */
/* ★ 第 12 条(2026-09-30): 事件 msgId 的【唯一序号】—— 在线发送与"断线入缓存"
 * 两条路【共用同一个计数器】。
 * 【为什么必须共用】以前两边各有一个 static seq: 同一个事件在线时叫 `s001`,
 * 断线入缓存、之后补报时被重新分配成 `b001` —— 平台按 msgId 做幂等/去重时对不上
 * (该去重的去不掉, 或者把同一条当成两条)。
 * 现在: 入缓存时就把号码记进记录(ec_record_t.msg_seq, 2 B/条), 补报时原样还原。 */
/* ★2026-10-08: 声明已上移到 main_on_alarm_result 之前(那里也要用)★ */

static void main_cache_status_change(uint8_t from_code, uint8_t to_code,
                                     uint8_t gear, int32_t level_mm)
{
    ec_record_t rec;

    (void)gear;             /* 档位不入记录: 渲染时由"迁移后状态"推出(见 event_cache.h) */

    (void)memset(&rec, 0, sizeof(rec));
    rec.kind     = (uint8_t)EC_KIND_STATUS_CHANGE;
    rec.a        = to_code;                 /* to   */
    rec.b        = from_code;               /* from */
    rec.ts       = main_now_unix();         /* ★ 原始发生时刻 */
    rec.aux = level_mm;
    /* ★ 第 12 条: 现在就把"这条事件的 msgId 序号"定下来(与在线路径同一个计数器),
     * 补报时按 s<序号> 原样重建 —— 不再变成另一个 b<序号>。 */
    s_evt_seq = (uint16_t)((s_evt_seq + 1u) % 1000u);
    rec.msg_seq  = s_evt_seq;
    (void)event_cache_push(&rec);
}

static void main_publish_status_change(uint8_t from_code, uint8_t to_code,
                                       uint8_t gear, int32_t level_mm)
{
    /* 一律 static: 栈只有 4 KB */
    static char           data[256];
    static char           payload[CLOUD_RESP_MAX];
    static char           topic[64];
    /* 序号不再用本函数自己的 static —— 与 main_cache_status_change() 共用
     * 文件级的 s_evt_seq(第 12 条: 在线与补报必须是同一个号)。 */
    cloud_status_change_t sc;
    char                  msg_id[20];
    uint16_t              dlen;
    uint16_t              plen;
    uint16_t              tlen;
    const char           *from = cloud_proto_state_name(from_code);
    const char           *to   = cloud_proto_state_name(to_code);

    if ((from == NULL) || (to == NULL))
    {
        /* 码值不认识: 宁可这条不发, 也不编一个状态名出来 */
        log_info("[CLOUD] !! 状态码未知(%u->%u), 不上报 status_change\r\n",
                 (unsigned)from_code, (unsigned)to_code);
        return;
    }

    sc.from     = from;
    sc.to       = to;
    sc.gear     = gear;
    sc.level_mm = level_mm;

    dlen = cloud_proto_build_status_change_data(data, (uint16_t)sizeof(data), &sc);
    if (dlen == 0u)
    {
        log_info("[CLOUD] !! status_change 的 data 段组装失败\r\n");
        return;
    }

    /* ★ 第 12 条: 与"断线入缓存"那条路共用 s_evt_seq, 同一个事件只有一个号 */
    s_evt_seq = (uint16_t)((s_evt_seq + 1u) % 1000u);
    (void)snprintf(msg_id, sizeof(msg_id), "s%03u", (unsigned)s_evt_seq);    plen = cloud_proto_build_uplink(payload, (uint16_t)sizeof(payload),
                                    "event.status_change", msg_id, main_now_unix(), data);
    if (plen == 0u)
    {
        log_info("[CLOUD] !! status_change 信封组装失败\r\n");
        return;
    }

    tlen = cloud_profile_build_topic(CLOUD_TOPIC_EVENT, topic, (uint16_t)sizeof(topic));
    if (tlen == 0u)
    {
        log_info("[CLOUD] !! event topic 拼装失败, status_change 转入缓存\r\n");
        main_cache_status_change(from_code, to_code, gear, level_mm);
        return;
    }

    log_info("[CLOUD] 上报 %s (%u 字节): %s\r\n", topic, (unsigned)plen, payload);

    if (g4_at_mqtt_publish_ex(topic, (const uint8_t *)payload, plen,
                              cloud_profile_topic_qos(CLOUD_TOPIC_EVENT),
                              cloud_profile_topic_retained(CLOUD_TOPIC_EVENT)) != SYS_OK)
    {
        /* 登记失败(刚掉线/队列满): ★ 退回缓存 ★, 否则这条迁移就凭空没了 */
        log_info("[CLOUD] !! status_change 发布登记失败, 转入缓存下次补报\r\n");
        main_cache_status_change(from_code, to_code, gear, level_mm);
    }
}

/* ==================================================================
 * ota.progress 上报 + 升级就绪后复位(文档 7.6 / 第 6 章) —— 2026-10-02
 * ==================================================================
 * 触发口径(文档: "每 10% 或 10s 一条"):
 *   · 状态变了           -> 立刻一条(idle->downloading、->verifying、->failed 都要看得见)
 *   · 进度跨过 10% 台阶   -> 一条
 *   · 距上一条满 10s      -> 一条(下载卡住时平台也能看出"还在动还是死了")
 *
 * 【为什么由 main.c 发而不是 cloud_proto】: cloud_proto 是纯协议层, 要能在 PC 上
 * 脱机单测; 真正碰 AT/MQTT 的动作按本工程惯例留在 main.c(同 sys.time_sync)。
 *
 * 【READY 之后的重启也在这里】: 协议第 6 章要求"标记启动分区 -> 重启"。写标记页
 * 是 ota_http 在 VERIFYING 里做完的, 复位由本函数倒数 3 拍(≈3s)后执行 —— 留这几
 * 秒是为了让"就绪/即将复位"的日志真的从串口发出去, 现场靠它判断走到哪一步了。
 * ================================================================== */

/* ★2026-10-05 台架验证开关: READY 之后要不要自动复位★
 *
 * 0 = 【当前默认, 台架验证用】校验通过后【不复位】, 停在 READY 只上报。
 *     为什么: 当前分区是【模式 1】(app/common/flash_map.h: FM_MAP_MODE =
 *     FM_MAP_SINGLE_RESERVED, FM_HAS_BOOTLOADER = 0) —— App 就在 0x08000000,
 *     板上【没有 bootloader】,"把暂存区搬到运行区"那件事只有 src/main_bl.c 的
 *     bl_run() 会做, 而它属于另一个 BL 构建、不在 App 里。
 *     所以复位了也只是从 0x08000000 重跑旧固件 —— 白重启一次, 还把串口刷掉,
 *     反而看不清"下载 + MD5 + 写标记"到底成没成。
 *     停在这里的意义: READY = 下载完整、MD5 比对通过、标记页已写, 这三件事都成了。
 *
 * 1 = 恢复正常行为(3s 后复位交给 bootloader 搬运)。
 *
 * ⚠【上线前必须改回 1】—— 走模式 2(双 Slot + 已烧 BL)时, 不改回来升级永远不生效。
 * ⚠ 停在 READY 的副作用: ota.query 会一直回 installing; 期间再下 ota.start 会被
 *   当"正忙"拒掉(设计如此, 主机测试 [12] 钉着), ota.abort 在 READY 也被拒(协议 5.6)。
 *   要重测一次得【断电重上】(或等以后接一个"清 READY"的本地命令)。 */
#ifndef MAIN_OTA_AUTO_REBOOT
#define MAIN_OTA_AUTO_REBOOT     0
#endif

static void main_ota_progress_poll(void)
{
    static uint8_t  s_last_state   = 0xFFu;
    static uint8_t  s_last_percent = 0xFFu;
    static uint32_t s_t_last;
    static uint16_t s_ota_seq;
    static uint8_t  s_reboot_cnt;
    static uint8_t  s_ready_announced;      /* 非自动复位时, READY 那几行只打一次 */

    static char       payload[CLOUD_RESP_MAX];
    static char       data[160];
    static char       topic[64];
    ota_http_status_t st;
    uint32_t          now = get_tick_ms();
    char              msg_id[20];
    uint16_t          plen;
    uint16_t          tlen;

    ota_http_get_status(&st);

    /* ---- 空闲: 复位节流, 且【什么都不报】----
     * ⚠ 必须【直接 return】: 早期写法只复位 s_last_state = 0xFF 就往下走, 于是
     * "IDLE(0) != 0xFF" 恒成立 -> 设备空闲时每 10s 往云端刷一条 {"state":"idle"}。
     * (现在这条判断本身已经收进 ota_http_progress_due(), 主机侧有测试盯着。) */
    if (st.state == OTA_HTTP_IDLE)
    {
        s_last_state      = 0xFFu;
        s_last_percent    = 0xFFu;
        s_reboot_cnt      = 0u;
        s_ready_announced = 0u;
        return;
    }

    /* ---- 就绪: 先起倒数, 交给 bootloader 搬运 ----
     * 【为什么倒数不放在下面的 send 分支里】: ota_http_progress_due() 对 READY
     * 只会在"首次进入"时返回 true, 放进去就只减一次, 永远数不到 0 -> 永不复位。
     * 所以倒数必须【每一拍】都推进, 与上报节流解耦。 */
    if (st.state == OTA_HTTP_READY)
    {
#if MAIN_OTA_AUTO_REBOOT
        if (s_reboot_cnt == 0u)
        {
            s_reboot_cnt = 3u;
            log_info("[OTA_HTTP] 校验通过且标记页已写, 3s 后复位交给 bootloader\r\n");
        }
        else
        {
            s_reboot_cnt--;
            if (s_reboot_cnt == 0u)
            {
                log_info("[OTA_HTTP] 复位! bootloader 将把暂存区搬到运行区\r\n");
                ota_port_reboot();          /* 不返回 */
            }
        }
#else
        /* 台架模式: 只宣告一次, 然后一直停在 READY(每一拍都走这里, 但只打一次日志) */
        if (s_ready_announced == 0u)
        {
            s_ready_announced = 1u;
            log_info("[OTA_HTTP] ================================================\r\n");
            log_info("[OTA_HTTP] ★ 校验通过: 下载完整 + MD5 比对成功 + 标记页已写 ★\r\n");
            log_info("[OTA_HTTP]   本固件 MAIN_OTA_AUTO_REBOOT=0, 【不自动复位】。\r\n");
            log_info("[OTA_HTTP]   原因: 当前是分区模式 1(FM_HAS_BOOTLOADER=0), 板上没有\r\n");
            log_info("[OTA_HTTP]   bootloader, 复位也只会从 0x08000000 重跑旧固件。\r\n");
            log_info("[OTA_HTTP]   => 台架验证到此为止【算成功】; 要真装上新固件请走\r\n");
            log_info("[OTA_HTTP]      模式 2 + 先烧 BL, 并把本宏改回 1。\r\n");
            log_info("[OTA_HTTP]   重测一次: 断电重上(READY 期间 ota.start 会被当忙拒掉)。\r\n");
            log_info("[OTA_HTTP] ================================================\r\n");
        }
#endif
        /* 不 return: 下面照常把 installing 那一条报出去 */
    }

    /* 发送判据走 ota_http_progress_due() —— 纯函数, 主机侧测得到 */
    if (!ota_http_progress_due(&st, s_last_state, s_last_percent,
                               now, s_t_last, MAIN_OTA_PROGRESS_MS))
    {
        return;
    }

    s_last_state   = (uint8_t)st.state;
    s_last_percent = st.percent;
    s_t_last       = now;

    /* 状态名走 ota_http_state_name(): 与 ota.query 的应答必须同口径 */
    if (st.version[0] != '\0')
    {
        (void)snprintf(data, sizeof(data),
                       "{\"state\":\"%s\",\"percent\":%u,\"targetVersion\":\"%s\"}",
                       ota_http_state_name(st.state), (unsigned)st.percent, st.version);
    }
    else
    {
        (void)snprintf(data, sizeof(data), "{\"state\":\"%s\",\"percent\":%u}",
                       ota_http_state_name(st.state), (unsigned)st.percent);
    }

    s_ota_seq = (uint16_t)((s_ota_seq + 1u) % 1000u);
    (void)snprintf(msg_id, sizeof(msg_id), "p%03u", (unsigned)s_ota_seq);

    plen = cloud_proto_build_uplink(payload, (uint16_t)sizeof(payload),
                                    "ota.progress", msg_id, main_now_unix(), data);
    if (plen == 0u)
    {
        log_info("[CLOUD] !! ota.progress 信封组装失败\r\n");
        return;
    }

    tlen = cloud_profile_build_topic(CLOUD_TOPIC_EVENT, topic, (uint16_t)sizeof(topic));
    if (tlen == 0u)
    {
        return;                         /* 离线: 进度丢了不致命, 平台会来 ota.query */
    }

    log_info("[CLOUD] 上报 %s (%u 字节): %s\r\n", topic, (unsigned)plen, payload);
    (void)g4_at_mqtt_publish_ex(topic, (const uint8_t *)payload, plen,
                                cloud_profile_topic_qos(CLOUD_TOPIC_EVENT),
                                cloud_profile_topic_retained(CLOUD_TOPIC_EVENT));
}

/* ==================================================================
 * sys.time_sync 的落地：把平台给的 Unix 秒写进 4G 模组时钟 (v91)
 * ==================================================================
 * 云协议层只负责解析/校验并把 Unix 秒交出来(钩子)，真正碰 AT 的在这里 ——
 * 这样协议层仍然能在 PC 上脱机单测。
 * ★ 换算用 v83 的 cloud_proto_unix_to_calendar()，时区取 +28800(北京)：
 *   写进模组的是北京时间 + "+32" 偏移，与模组自己上报的格式一致。★
 * ★ 换算刻意放这里而不是 g4_at：见 g4_at.h 里的说明（否则 test_state 链接失败）。★
 */
static bool main_time_sync(uint32_t unix_ts)
{
    /* ============================================================================
     * ★ 2026-09-30 用户拍板的口径: 【网关不接受云端下发的时钟】★
     * ----------------------------------------------------------------------------
     * 收到 sys.time_sync 时, 不拿平台的 ts 去写时钟, 而是【向 4G 模组要时间】,
     * 用它刷新网关自己的 epoch, 然后回成功。
     *
     * 为什么这么定: 网关没有 RTC 电池, 唯一可信的时间源就是 4G 网络给的(模组 AT+CCLK);
     *   平台的时间对网关来说是"另一个系统的时钟", 直接采信等于把两个时钟源混在一起。
     *
     * 旧实现(已废弃): 把平台的 ts 换算成日历后【写进模组】(g4_at_set_clock_ymdhms),
     *   写失败就回 1007 —— 于是"模组当时不可用"会让平台看到"校时失败",
     *   而实际上我们自己的时间本来就可以从模组读到。
     *
     * ⚠️ 关于"上行的 ts 从此完全跟随模组时间": 用户 2026-09-30 确认这是**期望行为**。
     *   曾经看到日志里"网关 ts 比平台 ts 快 82 秒"——那不是时钟偏差, 而是**手工测试
     *   的产物**: 平台侧的 ts 是人工提前抄好、过一会儿才发出来的, 所以它偏旧。
     *   网关的时间是正确的。(这条更正很重要: 不要把手工测试的时间误当成时钟问题。)
     * ========================================================================== */
    (void)unix_ts;                  /* 平台给的 ts 只用于日志对比, 不写时钟 */

    /* 强制下一次 refresh 真的去问模组(不然"没到陈旧阈值"就直接返回, 等于没校) */
    s_epoch_valid = false;
    main_refresh_epoch();

    if (!s_epoch_valid)
    {
        log_info("[CLOUD] sys.time_sync: 向 4G 模组要时间【没要到】(模组不可用?) -> 1007\r\n");
        return false;
    }

    log_info("[CLOUD] sys.time_sync: 已从 4G 模组取时间 -> 本机 epoch=%lu (平台 ts=%lu)\r\n",
             (unsigned long)main_now_unix(), (unsigned long)unix_ts);
    return true;
}

/* ==================================================================
 * 断线补报: 把事件缓存里的一页补发出去 (v70)
 * ==================================================================
 * 只在 4G/MQTT 在线时做。一页最多 16 条(见 cloud_proto.h), 数据段约 3 KB ——
 * **发布通路装得下**: g4_at.c 的 G4_PUB_DATA_MAX = 4352
 * (v28 踩过"发布缓冲 256 B 导致 cfg.query 永远发不出去", 所以这次先查了容量)。
 *
 * ★ 先发后删 ★：发布【登记成功】才 drop；登记失败(离线/队列满)则一条都不动, 下次再来。
 *   与 data_cache 的 peek/drop 是同一条纪律 —— 一次失败不能把事件永久丢掉。
 * ★ 缓存里"渲染不出来"的条也必须出队 ★：否则它们永远堵在队头,
 *   后面的事件全被堵死(v67 的契约)。跳过多少条会打日志 —— 可以丢, 不许静默丢。
 */
static void main_backfill_poll(void)
{
    /* ★ 必须 static: 栈只有 4 KB, 一页正文约 3 KB 放栈上会直接把栈吃掉 ★ */
    static char buf[4096];
    static char topic[64];
    uint8_t     consumed;
    uint8_t     events;
    uint8_t     skipped;
    uint8_t     i;
    uint16_t    n;
    uint16_t    tlen;

    if (!g4_at_is_online())
    {
        return;                     /* 断线期间不补报(补了也发不出去) */
    }

    n = cloud_proto_build_backfill_data(buf, (uint16_t)sizeof(buf), 16u,
                                        &consumed, &events, &skipped);
    if (consumed == 0u)
    {
        return;                     /* 缓存空, 或整页装不下(下次再来) */
    }
    if (skipped > 0u)
    {
        log_info("[CLOUD] 补报: 跳过 %u 条(渲染不出来), 本页装了 %u 条\r\n",
                 (unsigned)skipped, (unsigned)events);
    }
    if (n == 0u)
    {
        /* 全是渲染不出来的: 照样出队, 否则它们永远堵在队头 */
        for (i = 0u; i < consumed; i++)
        {
            event_cache_drop();
        }
        return;
    }

    tlen = cloud_profile_build_topic(CLOUD_TOPIC_EVENT, topic, (uint16_t)sizeof(topic));
    if (tlen == 0u)
    {
        log_info("[CLOUD] !! 补报 topic 拼装失败, 这 %u 条留着下次\r\n",
                 (unsigned)consumed);
        return;                     /* 一条都不动 */
    }

    log_info("[CLOUD] 补报一页 %u 条事件 (正文 %u 字节)\r\n", (unsigned)events, (unsigned)n);

    if (g4_at_mqtt_publish_ex(topic, (const uint8_t *)buf, n,
                              cloud_profile_topic_qos(CLOUD_TOPIC_EVENT),
                              cloud_profile_topic_retained(CLOUD_TOPIC_EVENT)) != SYS_OK)
    {
        log_info("[CLOUD] !! 补报登记失败, 这 %u 条留着下次再发\r\n", (unsigned)consumed);
        return;                     /* ★ 登记失败: 一条都不 drop ★ */
    }

    for (i = 0u; i < consumed; i++)
    {
        event_cache_drop();         /* ★ 登记成功才出队 ★ */
    }
}

#endif /* MAIN_STATUS_JSON_ENABLE */

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
#ifndef MAIN_G4_ENABLE
#define MAIN_G4_ENABLE           1
#endif

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
#ifndef MAIN_LORA_ENABLE
#define MAIN_LORA_ENABLE         1
#endif

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
 * 模式宏的唯一真相源是 app/radio/llcc68/project_config.h(经 lora_proto.h 在本文件
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
#error "主功能模式必须恰好选一个: 见 app/radio/llcc68/project_config.h 的『主功能模式(三选一)』"
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

/* ==================== 日志口手敲 AT 控制台 ====================
 * USART1 接收中断 → 环形缓冲 → console_poll 拼行 → g4_io_send_cmd 转发给 4G。
 * 便于台架上直接敲 AT+CSQ? / AT+MQTTCONN? 等命令排障。
 */
#if MAIN_AT_CONSOLE_ENABLE

#define CONSOLE_BUF_SIZE   128U

static volatile uint8_t  s_con_buf[CONSOLE_BUF_SIZE];
static volatile uint16_t s_con_wr = 0U;
static volatile uint16_t s_con_rd = 0U;
static char              s_con_line[CONSOLE_BUF_SIZE];

void USART1_IRQHandler(void)
{
    if (USART_GetIntStatus(USART1, USART_INT_RXDNE) != RESET)
    {
        uint8_t  ch   = (uint8_t)(USART_ReceiveData(USART1) & 0xFFU);
        uint16_t next = (uint16_t)((s_con_wr + 1U) % CONSOLE_BUF_SIZE);

        if (next != s_con_rd)   /* 满则丢弃, 不覆盖未处理数据 */
        {
            s_con_buf[s_con_wr] = ch;
            s_con_wr = next;
        }
        USART_ClrFlag(USART1, USART_FLAG_RXDNE);
    }
}

static void console_init(void)
{
    USART_ConfigInt(USART1, USART_INT_RXDNE, ENABLE);
    NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(5, 3, 3));
    NVIC_EnableIRQ(USART1_IRQn);
}

static void console_poll(void)
{
    static uint16_t len = 0U;

    while (s_con_rd != s_con_wr)
    {
        char c = (char)s_con_buf[s_con_rd];

        s_con_rd = (uint16_t)((s_con_rd + 1U) % CONSOLE_BUF_SIZE);

        if ((c == '\r') || (c == '\n'))
        {
            if (len > 0U)
            {
                s_con_line[len] = '\0';

#if MAIN_G4_ENABLE
                /* 以 '#' 开头 = 本地诊断命令(#pwr? / #pwrswap / #cycle / #help...),
                 * 【不转发给模组】。现场排"模组没电/电压不稳/不回话"用, 见 g4_at.c。 */
                if (s_con_line[0] == '#')
                {
                    if (!g4_at_console_cmd(s_con_line))
                    {
                        log_info("[CONSOLE] 未知本地命令 %s (敲 #help 看列表)\r\n", s_con_line);
                    }
                    len = 0U;
                    continue;
                }
#endif
                /* 其它行: 整行当 AT 命令转发给 4G 模组。
                 * (以前这里还支持"以 { 开头当云端 JSON 喂给下行处理函数",
                 *  那个台架后门已经删掉了 —— 下行只走 4G/MQTT。) */
                log_info("[CONSOLE->4G] %s\r\n", s_con_line);
                g4_io_send_cmd(s_con_line);
                len = 0U;
            }
        }
        else if (len < (CONSOLE_BUF_SIZE - 1U))
        {
            s_con_line[len] = c;
            len++;
        }
    }
}

#else  /* !MAIN_AT_CONSOLE_ENABLE */

static void console_init(void)  {}
static void console_poll(void)  {}

#endif /* MAIN_AT_CONSOLE_ENABLE */

/* ==================== 状态打印 ==================== */

static void status_report(void)
{
    uint8_t       online_cnt = 0U;
    uint8_t       total      = lora_business_get_slave_count();
    uint8_t       i;
    lora_slave_t *list = lora_business_get_slave_list();
    const g4_module_info_t *gi = g4_at_get_info();
    const g4_cereg_diag_t  *cd = g4_at_get_cereg_diag();
    char          vbat_text[16];
    char          hb_text[16];
    char          cereg_text[24];

    for (i = 0U; i < total; i++)
    {
        if (list[i].is_online)
        {
            online_cnt++;
        }
    }

    /* VBAT 是【模组自己 ADC 测的供电】, 比万用表准(万用表读的是平均值)。
     * 规格 3.4~4.2V, 典型 3.8V。 */
    if (gi->vbat_valid)
    {
        snprintf(vbat_text, sizeof(vbat_text), "%umV", (unsigned)gi->vbat_mv);
    }
    else
    {
        vbat_text[0] = '-';
        vbat_text[1] = '\0';
    }

    /* 距上次成功发布(心跳)的秒数 —— 【判断保活是不是断了就看这个数】
     * 在线时它应该始终 < 30s 来回; 一旦只涨不回头, 就是心跳发不出去了。 */
    if (g4_at_pub_age_ms() != 0xFFFFFFFFu)
    {
        snprintf(hb_text, sizeof(hb_text), "%lus",
                 (unsigned long)(g4_at_pub_age_ms() / 1000u));
    }
    else
    {
        hb_text[0] = '-';
        hb_text[1] = '\0';
    }

    /* 驻网(CEREG)只读观测: 紧凑显示。
     * 【默认模式下 stat 恒为 0xFF】—— 因为出厂固件【不轮询】AT+CEREG?(ONLINE 期间
     * 不许发 AT, 理由见 g4_at.h 那段长注释), 只靠模组主动上报。所以这一栏在默认
     * 模式下的含义是"有没有把主动上报打开"(armed), 真正的证据在:
     *   · "[REG] 驻网行(原始)" / "[REG] !! PS 去附着" 那几行日志(事件发生时才有)
     *   · [STAT2] 里的 URC行 / CGEV / 去附着 累计计数
     * 台架若把 G4_CEREG_POLL_MS 打开, 这里就会显示实际轮询到的 stat。 */
    if (cd->stat != 0xFFu)
    {
        if (cd->bad_run != 0u)
        {
            snprintf(cereg_text, sizeof(cereg_text), "%u!!连续%u次",
                     (unsigned)cd->stat, (unsigned)cd->bad_run);
        }
        else
        {
            snprintf(cereg_text, sizeof(cereg_text), "%u", (unsigned)cd->stat);
        }
    }
    else if (cd->urc_armed)
    {
        snprintf(cereg_text, sizeof(cereg_text), "armed");
    }
    else
    {
        snprintf(cereg_text, sizeof(cereg_text), "-");
    }

    log_info("[STAT] 4G=%s | HB=%s(距上次发布成功) | PA8=%s(4G电源,%s) RX累计=%lu | "
             "IMEI=%s CFUN=%d CSQ=%u(%ddBm) IP=%s VBAT=%s CEREG=%s | "
             "LoRa=%s 在线从机=%u/%u | 下发缓存=%u | tick=%lu\r\n",
             g4_at_is_online() ? "ONLINE" : "offline",
             hb_text,
             G4_PowerSwitch_GetLevel() ? "H" : "L",
             G4_PowerSwitch_GetOnHigh() ? "高电平上电" : "低电平上电",
             (unsigned long)g4_io_rx_total(),
             (gi->imei[0] != '\0') ? gi->imei : "-",
             (int)g4_at_get_cfun(),      /* 1=全功能 0=最小功能(射频关,不会搜网) -1=没读到 */
             (unsigned)gi->csq, (int)gi->rssi_dbm,
             (gi->ip[0] != '\0') ? gi->ip : "-",
             vbat_text,
             cereg_text,
#if MAIN_LORA_ENABLE
             lora_business_radio_stuck() ? "模组无应答!" : "OK",
#else
             "关闭",
#endif
             (unsigned)online_cnt, (unsigned)total,
             (unsigned)data_cache_down_get_count(),
             (unsigned long)get_tick_ms());

    /* 【链路健康计数 —— 排"收不到 / 掉线"先看这一行】
     *   丢字节   : USART2 溢出 + 环形缓冲写满而丢掉的字节数。>0 = 接收真的被压垮过,
     *              下一步就是加大缓冲 / 降日志 / 让云端放慢下发, 而不是瞎猜。
     *   收包重同步: 下行收包中途被截断(超时/长度非法)而放弃的次数。>0 = 云端下行
     *              确实丢过消息, 但【不会再把整条 AT 流带偏】—— 这是新加的兜底。
     *   日志丢   : 收包路径日志被限流丢掉的条数。用来判断"日志本身是不是瓶颈"。
     *   下行     : 收到的云端消息条数; 括号里是下行队列满丢掉的条数。
     *   puback   : broker 确认过的上行条数 —— 【这才是"云端确实收到了"的证据】,
     *              模组回 OK 只代表它把数据交给了自己的 MQTT 栈。
     *   模组报就绪: 模组报过多少次 "+MATREADY"/"RDY", 也就是它的 AT 应用层重启过
     *              多少次 —— 【这才是"模组自己重启了没有"的可靠计数】。
     *              老版本按"离上次发送是否 >3s"分档, 而注册阶段每 2s 就问一条命令,
     *              于是真重启也被算进"应答习惯"那一档, 这个数一路是 0
     *              (现场日志 78.txt: +MATREADY 明明出现了 4 次)。现在给总数 +
     *              括号里的"紧随我们发送"那一档, 两者对比着看。
     *   '>'丢失重发: 发 MQTTPUB 时模组把数据模式提示 '>' 吞掉的次数。
     *              现在不再直接断电重启, 而是先敲 AT 探活、活着就重发(见 g4_at.h)。
     *   MQTT快修  : 只重连 MQTT(MQTTDISC->MQTTCONN->MQTTSUB)而【不给模组断电】的次数。
     *              现场病灶是"一发 PUBLISH 这个会话就死": 第 1 条收下但 broker 不回
     *              puback(云端收不到), 第 2 条起回 +CME ERROR: 601; 而模组【不发
     *              conn URC】, 状态机只能靠自己发现。快修 2~5 秒, 断电要 40~60 秒,
     *              所以现在优先快修, 连续多次救不回来才升级到断电。 */
    /* ---------- LoRa 接收状态 ----------
     * 【现场要求: 网关的 LoRa 一直在接收状态】。这一栏把它变成【可验证的】:
     *   · "在收" 是直接读【芯片状态字节】得出的(不是读软件缓存) ——
     *     芯片真的在 RX 才显示"在收";
     *   · "补开" 是"发现它不在接收并重新切回 RX"的累计次数。这个数一涨, 就说明
     *     确实漏过(发射超时 / 模式漂移), 而不是我们以为它一直在收。
     * ⚠️ 只有开了 LoRa 才读芯片: 没上电的芯片读状态会卡在 BUSY。 */
    {
        const char *lora_rx;
        uint32_t    lora_rearm;

#if MAIN_LORA_ENABLE
        lora_rx    = LoraApp_IsRxActive() ? "在收" : "!!没在收";
        lora_rearm = LoraApp_RxRearmCount();
#else
        lora_rx    = "关闭";
        lora_rearm = 0u;
#endif

        log_info("[STAT2] 丢字节=%lu 收包重同步=%lu 日志丢=%lu | "
                 "下行=%lu(队列满丢 %lu) puback=%lu | "
                 "模组报就绪=%lu(其中紧随发送的 %lu) '>'丢失重发=%lu MQTT快修=%lu | "
                 "CEREG轮询=%lu 未驻网=%lu URC行=%lu CGEV=%lu(去附着 %lu) | "
                 "断电重启=%lu 退避=%lus | "
                 "LoRa RX=%s 补开=%lu\r\n",
                 (unsigned long)g4_io_rx_ovf_count(),
                 (unsigned long)g4_io_dl_abort_count(),
                 (unsigned long)g4_io_rxlog_drop_count(),
                 (unsigned long)g4_urc_pub_rx_count(),
                 (unsigned long)g4_urc_pub_drop_count(),
                 (unsigned long)g4_urc_puback_count(),
                 (unsigned long)g4_io_ready_total(),
                 (unsigned long)g4_io_ready_after_tx_count(),
                 (unsigned long)g4_at_prompt_loss_count(),
                 (unsigned long)g4_at_mqtt_fast_count(),
                 (unsigned long)cd->poll_cnt,
                 (unsigned long)cd->bad_total,
                 (unsigned long)cd->urc_line_cnt,
                 (unsigned long)cd->cgev_cnt,
                 (unsigned long)cd->cgev_detach_cnt,
                 (unsigned long)g4_at_get_rst_count(),
                 (unsigned long)(g4_at_get_backoff_ms() / 1000u),
                 lora_rx,
                 (unsigned long)lora_rearm);
    }
}

/* ==================== 复位原因 ====================
 *
 * 【为什么第一件事就要打这个】: 现场出现"模组一开始工作, 板子就自己重启"时,
 * 唯一有价值的问题是"谁把我复位的"。本工程【没有任何看门狗】(全工程没有 IWDG/WWDG),
 * 所以复位只可能是这几种:
 *   BOR  欠压复位 → 电源被拉塌了(模组上电浪涌/供电能力不足) ← 最常见
 *   POR  上电复位 → 冷启动(正常)
 *   PIN  NRST 脚被外部拉低 → 板上复位电路/别人按着
 *   SOFT 软件复位 → 有代码主动 NVIC_SystemReset(本工程只有 OTA 里有, 默认关)
 *   IWDG/WWDG     → 看门狗(本工程没开, 出现就说明配置被改了)
 *
 * 读完就把标志清掉, 这样下一次复位的"真凶"不会被这一次的残留掩盖 ——
 * 反复重启时, 每一轮打印的就是【刚刚那次】的原因。
 */
static void reset_cause_report(void)
{
    uint8_t bor  = (RCC_GetFlagStatus(RCC_FLAG_BORRSTF) != RESET) ? 1U : 0U;
    uint8_t por  = (RCC_GetFlagStatus(RCC_FLAG_PORRSTF) != RESET) ? 1U : 0U;
    uint8_t pin  = (RCC_GetFlagStatus(RCC_FLAG_PINRSTF) != RESET) ? 1U : 0U;
    uint8_t soft = (RCC_GetFlagStatus(RCC_FLAG_SFTRSTF) != RESET) ? 1U : 0U;
    uint8_t iwdg = (RCC_GetFlagStatus(RCC_FLAG_IWDGRSTF) != RESET) ? 1U : 0U;

    RCC_ClearResetFlag();

    log_info("[RST] 上次复位: BOR(欠压)=%u POR(上电)=%u PIN(NRST脚)=%u SOFT(软复位)=%u IWDG=%u\r\n",
             (unsigned)bor, (unsigned)por, (unsigned)pin, (unsigned)soft, (unsigned)iwdg);

    if ((bor != 0U) && (por == 0U))
    {
        log_info("[RST] !! 是【欠压复位】—— 电源被拉塌了, 不是代码问题。\r\n");
        log_info("[RST] !! 查: 模组上电浪涌 / 电源(电池)带载能力 / 模组 VCC 处的储能电容。\r\n");
    }
}

/* ==================== 云端 JSON 命令下行处理(P1-3) ====================
 *
 * 过渡期同一条下行通道上两种协议共存, 靠【内容嗅探】分流:
 *    以 '{' 开头          -> 云端 JSON 命令(RoadTwin), 走 cloud_proto, 回 cmdresp
 *    其它(0xAA 0x55 等)   -> 原有二进制 LoRa 帧透传, 交给 lora_business
 *
 * 【为什么用嗅探而不是按 topic 分】: "是不是 JSON" 与档位/topic 无关; 而且这样
 * 你在本地 broker 上往 rt/gw/{gwId}/cmd 发一条 JSON 就能直接验证整条链路。
 *
 * 【为什么要先拷进 NUL 结尾的缓冲】: MQTT 下发的 payload 是"字节流 + 长度",
 * 【不保证 NUL 结尾】(实测里含 0x00), 而 JSON 解析要求 NUL 结尾的 C 字符串。
 *
 * @return true = 已按云协议处理完(调用方不要再走 LoRa 透传)
 */
/* 【2026-09-27 放大: 384 -> CFG_JSON_MAX+64】
 * 原值 384 只装得下"改几个字段"的小下行; 而 cfg.set 允许"整份配置下发"
 * (实测 1203 B 默认 / 3584 B 满配), 超过 384 会在门口被丢掉 —— 而且【不回应答】,
 * 平台侧看起来就像网关死了。这里按单区配置的上限放大, 保证合法报文都进得来。
 * jsonbuf 本来就是 static, 代价只是 .bss; 改后整机 RAM 仍只用 ~26%, 很宽裕。 */
#define MAIN_CLOUD_JSON_MAX   (CFG_JSON_MAX + 64u)

/* (2026-10-08) 这里原来放过一版"字符串分片"(main_utf8_trim / main_json_escape /
 * main_publish_chunked) —— 因"必须转义引号"+"任一片丢就拼不齐"两个缺陷已删除。
 * 改为【按域分组】: 每片都是完整 JSON 对象, 零转义。见 cloud_proto 的分片接口。 */

static bool main_cloud_downlink_try(const char *topic, const uint8_t *data, uint16_t len)
{
    static char jsonbuf[MAIN_CLOUD_JSON_MAX + 1u];
    /* 【必须 static, 不能用 CLOUD_RESP_MAX(512) 放栈上】(P2-4)
     * cfg.query 的应答正文 = 完整配置对象(默认 1203 B / 满配最坏 3584 B) + 信封,
     * 512 B 装不下(会溢出 -> 整个应答被判失败); 而 CFG_JSON_MAX+256 放栈上又会
     * 爆掉 4KB 的栈(startup_n32h47x_48x.s 的 Stack_Size)。
     * 本函数只在主循环里被调用, 不会重入, static 暂存安全。 */
    static char resp[CFG_JSON_MAX + 256u];
    char        rtopic[64];
    uint16_t    rlen = 0u;
    uint16_t    tlen;
    int         rc;

    /* ==================== 上板验证用: 每条 MQTT 下行的【原始正文】(v76) ====================
     * 【为什么必须打在这里、打在最前面】
     *   lora_business.c 的 MQTT 回调把【每一条】下行消息都先喂给本函数(前置过滤器),
     *   所以在这里打印 = 把"到底收没收到、收到的是什么字节"变成串口上看得见的东西。
     *   原来的写法有两种情况在串口上【完全没有日志】:
     *     · 报文不像 JSON -> 下面的 looks_like_json 直接 return false, 一行都不打;
     *     · 云端把下行发到了上行 topic(设备没订阅) -> 压根进不来这个函数
     *       (这种只能在 broker 侧看订阅关系, 设备侧无从证明)。
     *   ★ 可打印的按文本打; 含不可打印字节的改打 hex ★
     *   (下发给 LoRa 的 payload 是原始二进制, 用 %s 会被 0x00 截断/错位)
     *   ★ 打印上限 128 字节, 免得长报文把日志刷爆 ★ */
    if ((data != NULL) && (len > 0u))
    {
        uint16_t i;
        uint16_t show = (len > 128u) ? 128u : len;
        bool     printable = true;

        for (i = 0u; i < show; i++)
        {
            /* ★ v90: `\n` `\r` `\t` 也算"可打印" ★
             * 带缩进/换行的 JSON 是最常见的情形 —— 只有走**文本分支**才看得见内容。
             * 起因(v89 复盘): 那条 259 字节的下行含 `\n`, 于是整条被判为"不可打印"、
             * 走了 hex 分支、又被 48 字节上限截断, 我只能回头手工解码才看清是什么。
             * 真正的二进制帧(含 0x00/其它控制字节)仍然走 hex。 */
            if (((data[i] < 0x20u) && (data[i] != '\n') && (data[i] != '\r') &&
                 (data[i] != '\t')) || (data[i] > 0x7Eu))
            {
                printable = false;
                break;
            }
        }

        if (printable)
        {
            /* ★ 把 `\n` `\r` `\t` 转义成可见的两个字符再打 ★
             * 为什么不直接 %s: 含换行的 JSON 会把**一条日志切成好几行**,
             * 之后没法按行检索/过滤(本工程所有日志工具都是按行看的)。
             * 转义后既看得见内容, 又保住"一条日志一行"。 */
            char     tmp[2u * 129u + 1u];
            uint16_t n = 0u;

            for (i = 0u; i < show; i++)
            {
                if (data[i] == '\n')      { tmp[n] = '\\'; n++; tmp[n] = 'n'; n++; }
                else if (data[i] == '\r') { tmp[n] = '\\'; n++; tmp[n] = 'r'; n++; }
                else if (data[i] == '\t') { tmp[n] = '\\'; n++; tmp[n] = 't'; n++; }
                else                      { tmp[n] = (char)data[i]; n++; }
            }
            tmp[n] = '\0';
            log_info("[4G DOWN] topic=%s len=%u text: %s%s\r\n",
                     (topic != NULL) ? topic : "-", (unsigned)len, tmp,
                     (len > show) ? " ...(截断)" : "");
        }
        else
        {
            char     hex[3u * 48u + 1u];
            uint16_t n    = 0u;
            uint16_t hshow = (show > 48u) ? 48u : show;

            for (i = 0u; i < hshow; i++)
            {
                (void)snprintf(&hex[n], sizeof(hex) - n, "%02X ", data[i]);
                n += 3u;
            }
            hex[n] = '\0';
            log_info("[4G DOWN] topic=%s len=%u hex: %s%s\r\n",
                     (topic != NULL) ? topic : "-", (unsigned)len, hex,
                     (len > hshow) ? " ...(截断)" : "");
        }
    }

    if (!cloud_proto_looks_like_json(data, len))
    {
        /* ★★2026-10-14 安全加固: 云端【命令 topic】上只允许 JSON ★★
         * ------------------------------------------------------------------
         * 【为什么要加这道闸】下面那个 return false 会让调用方把它【空口透传】出去
         *   (lora_business 的"最简透传"路径)。而经历 2026-10-14 那次分包 bug 时,
         *   一条 1441 字节的 rule.set 被模块拆成 plen=489/512/440 三片, 后两片
         *   不以 '{' 开头 ⇒ 就走到了这里 ⇒ 差一点被当成 LoRa 载荷发出去:
         *       [4G DOWN] ^^ 这条【不像 JSON】, 交给 LoRa 透传路径
         *       [CLOUD->LoRa] 透传 512 字节到空口 ... !! 超过空口单包上限 240, 丢弃
         *   ⇒ 那次只是【侥幸】: 两片都 >240 字节, 被后面那道长度检查拦住了。
         *     只要哪一片 <=240 字节, 它就会被【真的发到空口】, 变成一条发给所有
         *     从机的无意义广播 —— 现场表现是"设备收到莫名其妙的指令"。
         *
         * 【判据】topic 以 "/cmd" 结尾 = 云端命令通道(rt/gw/{gwId}/cmd)。
         *   这里刻意【只看后缀】, 不去拼 CLOUD_TOPIC_CMD 再比较:
         *   少一个跨模块符号依赖, 而且命令通道的命名约定就是这样的后缀。
         *
         * 【丢弃而不是转发】命令通道上的"不像 JSON"只有两种来源 ——
         *   ① 平台发错了   ② 接收侧切碎了(分包/截断)。
         *   两种都不该被送上空口。⇒ 丢弃 + 留一条日志(便于现场取证)。
         *   ⚠️ 返回 true 表示"已处理(丢弃)", 调用方【不要】再走 LoRa 透传。 */
        size_t tlen = (topic != NULL) ? strlen(topic) : 0u;

        if ((tlen >= 4u) && (strcmp(topic + (tlen - 4u), "/cmd") == 0))
        {
            log_info("[4G DOWN] !! 命令 topic 上的下行【不像 JSON】(topic=%s, %u 字节) "
                     "-> 丢弃, 【不】空口转发(避免碎片被发到 LoRa 空口)\r\n",
                     topic, (unsigned)len);
            return true;
        }

        log_info("[4G DOWN] ^^ 这条【不像 JSON】, 交给 LoRa 透传路径\r\n");
        return false;
    }

    log_info("[CLOUD] 收到 JSON 下行(topic=%s, %u 字节)\r\n",
             (topic != NULL) ? topic : "-", (unsigned)len);

    if (len > (uint16_t)MAIN_CLOUD_JSON_MAX)
    {
        log_info("[CLOUD] !! JSON 下行超长(%u > %u), 拒绝\r\n",
                 (unsigned)len, (unsigned)MAIN_CLOUD_JSON_MAX);
        return true;
    }

    (void)memcpy(jsonbuf, data, len);
    jsonbuf[len] = '\0';

    main_refresh_epoch();
    rc = cloud_proto_handle(jsonbuf, main_now_unix(),
                            resp, (uint16_t)sizeof(resp), &rlen);
    log_info("[CLOUD] 处理完成 code=%d, 应答 %u 字节\r\n", rc, (unsigned)rlen);

    if (rlen == 0u)
    {
        return true;            /* 已处理(即使没生成出应答), 不要再去透传 */
    }

    /* 【弱网关键】把应答正文也打出来。现场 4G 常入不了网 / broker 连不上, 此时
     * MQTT 发不出去, 串口是唯一能看到"我们到底回了什么"的地方 —— 没有这一行,
     * 弱网下就只能看到"应答发布登记失败", 没法判断协议层本身对不对。 */
    log_info("[CLOUD] 应答正文: %s\r\n", resp);

    tlen = cloud_profile_build_topic(CLOUD_TOPIC_CMDRESP, rtopic, (uint16_t)sizeof(rtopic));
    if (tlen == 0u)
    {
        log_info("[CLOUD] !! 应答 topic 拼装失败, 应答丢弃\r\n");
        return true;
    }

    /* ★★2026-10-08 撤回: 原来这里调的 main_publish_chunked() 已删除★★
     * 那个实现把整份应答 JSON 【塞进一个字符串】("chunk":"<JSON 文本>")，有两个缺陷:
     *   ① 必须把 JSON 里的引号转义成 \" —— 平台要反转义, 而且显示很难看;
     *   ② N 条独立发布, 任一条丢就永远拼不齐。
     * 改为【按域分组】(每片是完整 JSON 对象, 零转义) —— 见 cloud_proto 的
     * 云命令应答分片接口。小应答仍然走下面的单条路径, 格式一字不变。 */
    if (g4_at_mqtt_publish_ex(rtopic, (const uint8_t *)resp, rlen,
                              cloud_profile_topic_qos(CLOUD_TOPIC_CMDRESP),
                              false) != SYS_OK)
    {
        /* ==========================================================================
         * ★★2026-10-08: 发布失败时【必须回一条错误应答】, 不能就这么算了★★
         * --------------------------------------------------------------------------
         * 【现场(2026-10-09)】平台发 `cfg.query sections:["all"]`, 网关算出 1551 字节
         *   应答, 但模组单条发布上限是 1024(超了会把模组 AT 应用层撑重启):
         *       [G4] !! 发布被拒: 1551 字节 > 单条上限 1024 —— 不硬发...
         *       [CLOUD] !! 应答发布登记失败(离线/队列满), 这条应答丢了
         *   ⇒ 平台【什么都收不到】, 只能干等到超时, 而且不知道该怎么办(改什么命令)。
         *
         * 【现在】改回一条【小的】错误应答: cmd/msgId 原样回带、code=1011、
         *   msg 明确告诉平台"应答过大, 请按域查询"。
         *   为什么重解析一次请求: cloud_proto_handle() 内部解析完就丢了, 而
         *   cloud_proto_build_resp() 需要一个 cloud_req_t 才能回带 cmd/msgId。
         *
         * 【为什么用独立的 static 小缓冲】错误应答只有 ~180 字节, 没必要占用
         *   上面那个 CFG_JSON_MAX 级的大缓冲; static 是为了不占本来就紧的栈。
         * ======================================================================== */
        static char eresp[256];
        cloud_req_t rq;
        uint16_t    elen;

        if (cloud_proto_parse_req(jsonbuf, &rq) == 0)
        {
            elen = cloud_proto_build_resp(eresp, (uint16_t)sizeof(eresp), &rq,
                                          CLOUD_EC_RESP_TOO_BIG,
                                          "response too large; query by section",
                                          NULL, main_now_unix());
            if ((elen > 0u) &&
                (g4_at_mqtt_publish_ex(rtopic, (const uint8_t *)eresp, elen,
                                       cloud_profile_topic_qos(CLOUD_TOPIC_CMDRESP),
                                       false) == SYS_OK))
            {
                log_info("[CLOUD] 已改回【错误应答】code=%d(%u 字节), 平台不会再干等: %s\r\n",
                         (int)CLOUD_EC_RESP_TOO_BIG, (unsigned)elen, eresp);
                return true;
            }
        }

        log_info("[CLOUD] !! 应答发布登记失败(离线/队列满/超长), 连错误应答也发不出去"
                 " —— 平台只能超时重试\r\n");
    }
    return true;
}

/* ==================== 日志口云协议离线自测(#json / #status / #topic / #offline) + 射频诊断(#lora) ====================
 *
 * 【为什么需要】现场 4G 常处于弱网: 模组入不了网 / broker 连不上, 于是
 * "云端下发 -> 网关应答"这条链路根本没法验。但云协议层的全部代码(信封解析、
 * 21 条命令表、授权校验、应答组包、档位 topic 映射)其实【一行都不依赖网络】,
 * 缺的只是一个把下行字节喂进 main_cloud_downlink_try() 的入口。
 *
 * 这三条命令就是那个入口, 而且【走的就是 MQTT 下行那一个函数】, 不另开分支 ——
 * 所以串口上测通 = 真链路也通, 唯一差别只是应答最后能不能发出去(发不出去也会
 * 打印出来, 见 main_cloud_downlink_try 里的"应答正文")。
 *
 * 这一段纯属台架调试, 量产可把 MAIN_AT_CONSOLE_ENABLE 改成 0 整段去掉。
 */
#if MAIN_AT_CONSOLE_ENABLE

/* 内置样例: 选 sys.query_status —— 它在 21 条命令表里, 但处理函数目前还是占位
 * (回 1006)。正好一次把"信封解析 + 授权 + 查表 + 应答组包"四步全走到,
 * 而且不用自己敲花括号和引号(串口终端常把引号吃掉)。 */
#define MAIN_CONSOLE_JSON_DEMO \
    "{\"cmd\":\"sys.query_status\",\"msgId\":\"t1\",\"ts\":1700000000}"

static void main_console_help(void)
{
    log_info("[DIAG] #json [payload] 把下行 JSON 喂进云协议层(离线验证 P1 全链路; "
             "不带参数用内置样例)\r\n");
    log_info("[DIAG] #status 立刻组一条 status.report 并打印正文\r\n");
    log_info("[DIAG] #topic 打印当前档位的 5 个 topic / QoS / retained\r\n");
    log_info("[DIAG] #offline 打印各液位从机的上次上报/阈值/在线状态, 并立刻跑一次失联检查\r\n");
    log_info("[DIAG] #lora 射频诊断: 回读芯片实际配置(频率/同步字/模式/IRQ/统计/BUSY) + 一句话判读; "
             "只读不改, 不打断接收\r\n");
}

static bool main_console_ext(const char *line)
{
    /* ---------------- #json ---------------- */
    if (strncmp(line, "#json", 5) == 0)
    {
        const char *p = line + 5;

        while (*p == ' ') { p++; }

        if (*p == '\0')
        {
            p = MAIN_CONSOLE_JSON_DEMO;
            log_info("[TEST] #json 未带参数, 用内置样例\r\n");
        }

        log_info("[TEST] 注入下行 JSON (%u 字节): %s\r\n", (unsigned)strlen(p), p);

        /* 和 MQTT 下行【同一个函数】, 不另开分支 */
        (void)main_cloud_downlink_try("(console)",
                                      (const uint8_t *)p, (uint16_t)strlen(p));

        log_info("[TEST] 注入结束(应答能否发出去取决于 4G 是否在线, 正文见上)\r\n");
        return true;
    }

    /* ---------------- #status ---------------- */
    if (strcmp(line, "#status") == 0)
    {
#if MAIN_STATUS_JSON_ENABLE
        log_info("[TEST] 立刻组一条 status.report:\r\n");
        (void)main_publish_status_json();
#else
        log_info("[TEST] MAIN_STATUS_JSON_ENABLE=0, status.report 未编译进来\r\n");
#endif
        return true;
    }

    /* ---------------- #topic ---------------- */
    if (strcmp(line, "#topic") == 0)
    {
        const cloud_profile_t *p = cloud_profile_get();
        char    t[80];
        uint8_t i;

        if (p == NULL)
        {
            log_info("[TEST] !! cloud_profile_get() 返回空\r\n");
            return true;
        }

        log_info("[TEST] 档位=%s host=%s:%u gwId=%s prefix=%s\r\n",
                 cloud_profile_is_doc() ? "DOC(文档)" : "LOCAL(本地)",
                 p->host, (unsigned)p->port, p->gw_id, p->topic_prefix);
        log_info("[TEST] 身份映射=%s 授权校验=%s\r\n",
                 p->id_map_enable ? "开" : "关",
                 p->auth_enable   ? "开" : "关");

        for (i = 0u; i < (uint8_t)CLOUD_TOPIC_COUNT; i++)
        {
            t[0] = '\0';
            if (cloud_profile_build_topic((cloud_topic_t)i, t, (uint16_t)sizeof(t)) == 0u)
            {
                (void)snprintf(t, sizeof(t), "(拼装失败)");
            }
            log_info("[TEST]   %s -> %s [QoS%u%s]\r\n",
                     cloud_profile_topic_name((cloud_topic_t)i), t,
                     (unsigned)cloud_profile_topic_qos((cloud_topic_t)i),
                     cloud_profile_topic_retained((cloud_topic_t)i) ? " retained" : "");
        }
        return true;
    }

    /* ---------------- #offline ---------------- */
    /* 液位失联检测的"看得见"入口(2026-09-27 用户补充要求第 3 条)。
     * 现场要验这条逻辑很不方便: 阈值默认是 3 × 上报周期(60s 周期 = 180s 没上报),
     * 而且弱网下事件根本发不出去。所以这里把【判定用的每个量】都打出来, 再顺手
     * 跑一次检查 —— 与 lora_business 内部用的是同一个纯函数(sensor_offline.c),
     * 串口上看到的结论就是真实结论, 不是另算一套。
     * ⚠ 只读+检查, 不伪造事件: 真到阈值了才会真的发(发不出去就是发不出去)。 */
    if (strcmp(line, "#offline") == 0)
    {
        const config_store_t *cfg = config_store_get();
        lora_slave_t         *sl  = lora_business_get_slave_list();
        uint8_t               cnt = lora_business_get_slave_count();
        uint32_t              now = get_tick_ms();
        uint8_t               i, shown = 0u;

        log_info("[TEST] 液位失联检测: 阈值 = 上报周期(config_store) × %u\r\n",
                 (unsigned)SENSOR_OFFLINE_PERIOD_MULT);

        for (i = 0u; (i < cnt) && (sl != NULL); i++)
        {
            char     uid[CFG_UID_STR_MAX];
            uint16_t period_s;

            if (sl[i].dev_type != SLAVE_TYPE_SENSOR) { continue; }

            cfg_uid_from_bytes(sl[i].dev_uid, uid);
            period_s = sensor_offline_period_for_uid(cfg, uid);

            log_info("[TEST]   %s 周期=%us(0/查不到则用默认%us) 阈值=%lums "
                     "上次上报=%lums 距今=%lums 在线=%s 已报失联=%s\r\n",
                     uid, (unsigned)period_s, (unsigned)SENSOR_OFFLINE_DEFAULT_PERIOD_S,
                     (unsigned long)sensor_offline_threshold_ms(period_s),
                     (unsigned long)sl[i].last_report_tick,
                     (unsigned long)(uint32_t)(now - sl[i].last_report_tick),
                     sl[i].is_online ? "是" : "否",
                     sl[i].offline_reported ? "是" : "否");
            shown++;
        }

        if (shown == 0u)
        {
            log_info("[TEST]   从机表里【没有】液位从机(dev_type=SENSOR), 无从判定\r\n");
        }

        log_info("[TEST] 立刻跑一次失联检查(该发的会真的发到 event topic):\r\n");
        lora_business_check_sensor_offline();
        log_info("[TEST] 检查结束(上面没有 '失联'/'恢复' 行 = 没有跳变)\r\n");
        return true;
    }

    /* ---------------- #lora ---------------- */
    /* 射频诊断(2026-09 现场: 网关自称"在收", 但从机永远不在线, 且 [LoRa RX] 与
     * [proto] !! PARSE FAILED 一条都不出现 -> 说明 RxDone 从未触发)。
     *
     * 【为什么放在 main.c 的扩展钩子里, 而不是加进 g4_at.c】
     *   1) g4_at.c 是 4G/AT 模块, 它的 include 列表里【没有】任何射频头文件, 而且
     *      它还要被主机单测(_host_test 的 test_state)编译 —— 往那里引 radio.h /
     *      LLCC68-board.h 会平白把射频层耦合进 4G 模块, 还要给主机测试造桩。
     *   2) 真正常识上的"组合根"就是本文件: lora_business_init() 在这里调,
     *      g4_at_set_console_ext() 也在这里注册。射频诊断放在这里, 既复用了现成的
     *      扩展钩子(不动 g4_at.c 的命令表), 又能直接调 LoRa 层的接口。
     *   3) 具体实现写在 lora_proto.c 的 LoraApp_DiagPrint(): 那里正是 LoRa 配置
     *      (project_config.h 的 LORA_FRE/SF/BW/CR)与 LoraApp_* 状态所在的一层。
     * 该命令【只读】: 只发 Get 类命令 + 读寄存器, 不发任何 Set/SetRx, 不打断接收。 */
#if !MAIN_LORA_ENABLE
    if (strcmp(line, "#lora") == 0)
    {
        LoraApp_DiagPrint();
        return true;
    }
#else
    if (strcmp(line, "#lora") == 0)
    {
        log_info("[TEST] #lora 不可用: 本固件 MAIN_LORA_ENABLE=0, 没有射频\r\n");
        return true;
    }
#endif

    return false;
}
#endif /* MAIN_AT_CONSOLE_ENABLE */

/* ==================== 4G 下行"只打印 + 回执"回调(LoRa 关闭时用) ====================
 *
 * 【为什么必须有】: MQTT 下行回调是在 lora_business_init() 里注册的
 * (g4_urc_set_recv_cb(lora_mqtt_recv_cb))。MAIN_LORA_ENABLE=0 时那个 init 不跑,
 * 于是【没有任何回调注册】—— 云端下发进来会被悄悄丢掉, 现场看起来就像
 * "4G 模组收不到下发消息", 其实是收到了没人处理(日志里也看不到)。
 *
 * 【回执机制 —— 现场要求把"定时上报"换成"收到就回执"】:
 * 定时上报是"我自己按节奏发", 云端收没收到全靠运气(而且实测很多都没到);
 * 回执是"一问一答": 云端既然发了指令, 就一定会等这条回执 —— 收不到就说明链路
 * 有问题, 责任和时机都清楚。回执里带上收到的原始字节(hex), 云端还能对上
 * 到底是哪一条指令被收到了。
 */
#if MAIN_LORA_ENABLE
static void main_downlink_log_cb(const char *topic, const uint8_t *data, uint16_t len)
{
    uint16_t i;
    char     ack[192];
    int      n = 0;
    sys_err_t r;

    /* P1-3: 先让云协议层试一次(JSON 命令)。归它管就到此为止, 不再往下走。 */
    if (main_cloud_downlink_try(topic, data, len))
    {
        return;
    }

    log_info("[4G DOWN] topic=%s, %u 字节\r\n",
             (topic != NULL) ? topic : "-", (unsigned)len);

    /* 【十六进制 + ASCII 双转储】: 下发的 payload 是【原始二进制】而不是 ASCII hex
     * (实测确认: "+MQTTURC: "publish",...,20,20," 后面直接跟 20 字节二进制,
     *  其中含 0x00/0x0A, 用 %s 打印会被截断/错位)。
     * 十六进制看"字节到底是什么", ASCII 看"能不能当文本读", 一起打才不会误判。 */
    if ((data != NULL) && (len > 0u))
    {
        log_info("[4G DOWN] HEX:");
        for (i = 0u; (i < len) && (i < 64u); i++)
        {
            log_info(" %02X", (unsigned)data[i]);
        }
        log_info("\r\n");

        log_info("[4G DOWN] ASC:");
        for (i = 0u; (i < len) && (i < 64u); i++)
        {
            uint8_t c = data[i];
            log_info("%c", ((c >= 0x20u) && (c < 0x7Fu)) ? (char)c : '.');
        }
        log_info("\r\n");
    }

    /* 回执: "ACK:" + 收到的字节(hex)。云端收到这条就知道指令到底有没有送达。 */
    n = snprintf(ack, sizeof(ack), "ACK:");
    if ((n > 0) && (data != NULL))
    {
        for (i = 0u; i < len; i++)
        {
            if ((n + 3) >= (int)sizeof(ack))
            {
                break;
            }
            n += snprintf(&ack[n], sizeof(ack) - (size_t)n, "%02X", (unsigned)data[i]);
        }
    }

    r = g4_at_mqtt_publish(NULL, (uint8_t *)ack, (uint16_t)n);
    if (r == SYS_OK)
    {
        log_info("[4G] 已回执: %s\r\n", ack);
    }
    else
    {
        log_info("[4G] !! 回执登记失败(发布忙/链路不在线), 这条回执丢了\r\n");
    }
}
#endif

/* ============================================================================
 *              【最小 LoRa 收发隔离测试】  MAIN_LORA_RAW_TEST = 1
 * ============================================================================
 * 【为什么要它 —— 现场故障】
 *   网关(本板, LLCC68)收不到对端液位传感器的任何帧。已查明的事实:
 *     · 对端确在发(芯片自报 TX_DONE, 帧内容/地址/功能码全对, 20 字节);
 *     · 网关侧 "[LoRa RX]"(解析通过)与 "[proto] !! PARSE FAILED"(解析失败)
 *       【两条打印一条都不出现】—— 这两条分别覆盖了"解析通过/解析失败"两种情形,
 *       两条都没有, 只能说明【射频层的 RxDone 从未触发】;
 *     · #lora 显示: 芯片在 RX、BUSY 正常、等 BUSY 超时 0 次、GetErrors=0、
 *       GetStats 收包=0, 而 PREAMBLE_DETECTED 在涨、SYNCWORD_VALID/RX_DONE 恒 0。
 *   本测试的目的只有一个: 把"射频本身有问题"和"上层业务/协议有问题"彻底分开。
 *
 * 【它做了什么 / 没做什么】
 *   起: 1ms 时基(bsp_tim) / 日志口(log) / 复位原因 / 射频初始化(LoraApp_Init)
 *   不起: 4G(AT/URC/MQTT/发布) · param_save(Flash 参数) · lora_business(从机表/
 *         ACK/重传/失联) · cloud_proto/config_store/sensor_offline · 心跳 ·
 *         RS232/RS485 · OTA · USB · 看门狗 —— 射频不受任何其它东西干扰。
 *   4G 模组: 显式【不上电】(PA8 驱到断电电平, 不悬空) —— 少一个干扰源, 同时避免
 *            PA8 悬空把模组置于"半通"的不确定状态(见 bsp_gpio.h 的说明)。
 *
 * 【三条关键要求怎么落实的】
 *   a) "绕过解析直接打原始字节": 挂在【radio.c 的 RadioIrqProcess()】里 —— 那是
 *      拿到芯片 IRQ 原始状态字、且【早于 RadioEvents 回调】的一层([RAW-IRQ] /
 *      [RAW-RX], 见 radio.c 里 LoraRawTestDumpIrq/DumpBytes 的说明)。
 *      lora_proto.c 的 "[LoRa RX]" 在驱动回调层也会打一份, 两层互为对照。
 *   b) 每轮把射频【累计计数】打出来: g_irqStat(radio.c) + GetStats(芯片回读) +
 *      GetRssiInst + Busy/GetErrors —— 全部走 #lora 已有的只读接口, 不另写一份
 *      回读代码, 也不新增任何射频命令。
 *   c) 发射参数与生产一致: 复用 LoraApp_Init()(lora_proto_init 内部调的就是它,
 *      参数全取 project_config.h 的 LORA_*)与 lora_proto_send_*() 生产发送路径,
 *      【不另配一套】。更不碰同步字/功率。
 *
 * 【★怎么切回正常固件★】
 *   把 app/radio/llcc68/project_config.h 里 MAIN_LORA_RAW_TEST 由 1 改成 0
 *   (或在 Keil/IDE 的 C/C++ -> Define 里定义 MAIN_LORA_RAW_TEST=0 覆盖), 重新编译。
 *   开关=0 时本段代码(#if 之内)整体不参与编译, main() 走 #else 分支 ——
 *   那就是加这个开关之前的正常固件, Program Size 逐项一致。
 * ============================================================================ */
#if MAIN_LORA_RAW_TEST

#define RAWTEST_TX_PERIOD_MS      5000U   /* 每 5 秒一轮发射(两条帧) */
#define RAWTEST_RX_SUPERVISE_MS   1000U   /* 每 1 秒核对一次"芯片还在不在收" */
#define RAWTEST_PROBE_LEN         20U     /* "全 0xAA 原始探针"的字节数 */

static uint32_t     s_rawtest_round;                       /* 第几轮(只增) */
static uint8_t      s_rawtest_probe[RAWTEST_PROBE_LEN];     /* 原始探针缓冲 */
static lora_frame_t s_rawtest_frame;                        /* 合法业务帧(502B, 必须 static) */

/**
 * @brief 打印一轮"射频到底有没有新动静"的累计计数 + 芯片回读(全部只读)
 * @param tag 这一行是什么时刻打的(发射前/发射后/开机)
 *
 * 【为什么必须用累计计数, 不能用实时 GetIrqStatus】
 *   主循环每处理一次中断就把 IRQ status 清一次(radio.c RadioIrqProcess), 所以
 *   事后读 GetIrqStatus 永远接近 0 —— "RxDone 到底触发过没有"用那个寄存器回答不了。
 *   g_irqStat(radio.c, #lora 那版加的)是只增不减的软件计数, 才是判据。
 *
 * 【回读接口全部复用 #lora 已有的那几个, 没有另写一份 SPI 回读】
 *   RadioIrqStat() / LLCC68GetStats() / LLCC68GetRssiInst() / LLCC68GetStatus() /
 *   LLCC68GetDeviceErrors() / LLCC68BusyLevel() / LLCC68BusyTimeoutCount() /
 *   LLCC68IsStuck() / LoraApp_RxRearmCount()。
 *   (没有把 LoraApp_DiagPrint() 里那段打印抽成公共函数: 那会改动正常固件的目标码,
 *    而"开关=0 必须与基线逐字节等价"是本次的硬要求 —— 所以两边各自调用同一组
 *    只读接口, 而不是复制 SPI 回读实现。)
 */
static void rawtest_print_counters(const char *tag)
{
    const RadioIrqStat_t *irq = RadioIrqStat();
    static LLCC68Stats_t  st;          /* GetStats 结果(static: 不占栈) */
    RadioStatus_t         status;
    RadioError_t          err;
    int8_t                rssi;

    LLCC68GetStats(&st);
    status = LLCC68GetStatus();
    err    = LLCC68GetDeviceErrors();
    rssi   = LLCC68GetRssiInst();

    /* --- IRQ 累计(radio.c 的只增计数): 一眼看出"有没有新动静" --- */
    log_info("[RAWCNT] %s IRQ累计: RX_DONE=%lu(其中CRC错=%lu) PREAMBLE=%lu SYNC=%lu "
             "HDR_VALID=%lu HDR_ERR=%lu CRC_ERR=%lu TIMEOUT=%lu TX_DONE=%lu IRQ处理=%lu\r\n",
             tag,
             (unsigned long)irq->RxDone,        (unsigned long)irq->RxDoneCrcErr,
             (unsigned long)irq->PreambleDetected, (unsigned long)irq->SyncWordValid,
             (unsigned long)irq->HeaderValid,   (unsigned long)irq->HeaderError,
             (unsigned long)irq->CrcError,      (unsigned long)irq->RxTxTimeout,
             (unsigned long)irq->TxDone,        (unsigned long)irq->IrqProcessed);

    /* --- 芯片自带收包统计【回读 GetStats】: 全 0 = 一个包都没解调出来 --- */
    log_info("[RAWCNT] %s 芯片GetStats【回读】: 收包=%u CRC过=%u 长度/头错=%u "
             "(原始 %02X %02X %02X %02X %02X)\r\n",
             tag,
             (unsigned)st.NbPacketReceived, (unsigned)st.NbPacketCrcOk,
             (unsigned)st.NbPacketLengthError,
             (unsigned)st.Raw[0], (unsigned)st.Raw[1], (unsigned)st.Raw[2],
             (unsigned)st.Raw[3], (unsigned)st.Raw[4]);

    /* --- 模式 / 瞬时 RSSI【回读】: 芯片模式 5=RX(在收) --- */
    log_info("[RAWCNT] %s 瞬时RSSI=%d dBm | 芯片模式=%u(2=STDBY 5=RX 6=TX) | "
             "软件缓存mode=%u | RxContinuous=%s | 接收补开=%lu 次\r\n",
             tag, (int)rssi, (unsigned)status.Fields.ChipMode,
             (unsigned)LLCC68GetOperatingMode(),
             RadioGetRxContinuous() ? "真(无限期)" : "假(有限超时)",
             (unsigned long)LoraApp_RxRearmCount());

    /* --- BUSY / 芯片故障标志【回读】: BUSY 卡高会让上面的配置根本没写进芯片 --- */
    log_info("[RAWCNT] %s BUSY电平=%u 卡高=%s 等BUSY超时累计=%lu 次 | "
             "GetErrors=0x%04X(PLL未锁=%u XOSC未起=%u PA_ramp失败=%u)\r\n",
             tag,
             (unsigned)LLCC68BusyLevel(),
             (LLCC68IsStuck() != 0u) ? "是" : "否",
             (unsigned long)LLCC68BusyTimeoutCount(),
             (unsigned)err.Value,
             (unsigned)err.Fields.PllLock, (unsigned)err.Fields.XoscStart,
             (unsigned)err.Fields.PaRamp);
}

/**
 * @brief ① 发一条【全 0xAA 的原始字节】—— 内容无关的探针
 *
 * 【为什么发它】它与本工程协议格式完全无关:
 *   · 对端只要在射频/驱动层打字节, 就一定能看到它(长度 20, 内容全 AA, 一眼能认);
 *   · 若对端只在应用层打日志, 它会按"帧头不是 AA55"被丢掉 —— 这个"丢"本身也有
 *     信息量: "看到了字节但不认" = 射频通了, 问题在帧格式/协议;
 *                 "连字节都没有"  = 问题在射频(频率/带宽/SF/同步字/天线/前端)。
 * 走的仍然是【生产发送路径】lora_proto_send_raw(): 发完照样等 TX_DONE, 等不到
 * 会明确报"没发出去"(而不是静默返回)。
 */
static void rawtest_send_probe(void)
{
    /* ⚠️ 原本这里有一段"运行时把探针填成全 0xAA"的循环, 现在改成直接给下面那个
     *    常量帧(内容可编辑, 见 v99 改动)。所以 i 和 RAWTEST_PROBE_LEN 暂时用不到了 ——
     *    保留在注释里: 一是本工程要求【零警告】(留着声明会报 unused variable),
     *    二是以后想改回"运行时填充"时可以直接照抄。 */
    // uint16_t i;
    // for (i = 0U; i < RAWTEST_PROBE_LEN; i++)
    // {
    //     s_rawtest_probe[i] = 0xAAU;
    // }

    bool ok;

    static const uint8_t s_rawtest_probe[] = {
        0xAA, 0x55,       /* 帧头 */
        0x01,             /* 协议版本 */
        0x11, 0x00, 0x01, /* 网关地址：类型1 环境1 编号1 */
        0x31, 0x00, 0x01, /* 设备地址：类型3 环境1 编号1 */
        0x03,             /* 功能码：0x03 设置声光报警器 */
        0x1B,             /* 数据长度 = 27 字节 */
        /* ---- data 27 字节 ---- */
        0x01, /*  0 总电源：1 开 */
        0x05, /*  1 灯光模式：5 爆闪 */
        0x50, /*  2 功放音量：80 */
        0x01, /*  3 LED开关机：1 开 */
        0x01, /*  4 LED版式：1 全屏 */
        0x01, /*  5 显示模式：1 滚动 */
        0x0F, /*  6 屏幕亮度：15 */
        0x14, /*  7 滚动速度：20 */
        0x12, /*  8 文本长度 = 18 */
        /* 文本："水深危险，禁止通行" (GB2312) */
        0xCB, 0xAE, 0xC9, 0xEE, 0xCE, 0xA3, 0xCF, 0xD5, 0xA3, 0xAC,
        0xBD, 0xFB, 0xD6, 0xB9, 0xCD, 0xA8, 0xD0, 0xD0,
        /* ---- CRC16 (Modbus) 低字节在前 ---- */
        0x5D, 0x17};

    // log_info("[RAWTEST] ① 发【全 0xAA 原始探针】%u 字节(与协议格式无关)...\r\n", (unsigned)RAWTEST_PROBE_LEN);

    /* 【2026-09-29 移除】这里原来有一段临时的 utf8_to_hexstr() 试调用:
     *     char buf[256]; utf8_to_hexstr(text, buf, sizeof(buf));
     *  它有两个问题: ① 256 传进 uint8_t 形参被截成 0 -> 函数立刻返回 -1, 那行
     *  log 永远不会打印; ② 在 radio 测试函数里做文本转码本来也没用。
     *  真正的"云端文本 -> 三字节组帧"已经按用户第 6 条做成 utils 的
     *  utf8_pack_3byte()(见 utils.h), 用到它的地方在 dev.ctrl/联动下发那条链上。 */

    ok = lora_proto_send_raw(s_rawtest_probe, (uint8_t)sizeof(s_rawtest_probe));

    log_info("[RAWTEST] ① 发射结果: %s\r\n",
             ok ? "TX_DONE 成功(空口上确实发完了)"
                : "【等 TX_DONE 超时 —— 这一条没发出去】");
}

/**
 * @brief ② 发一条【合法业务帧】(用生产的组帧函数拼, 不手算 CRC)
 *
 * 帧内容与 lora_proto.h 文件头里那个 20 字节参考帧【逐字节一致】:
 *   AA 55 | 01 | 11 00 01 | 21 00 01 | 02 | 07 | 04 D2 64 00 00 00 00 | FF A8
 *   └头┘   ver  └─网关──┘  └─液位──┘  fc  dlen 液位1234 电量100% 月日时分   └CRC┘
 *                    110001        210001
 *
 * 【为什么用这一帧】
 *   1) 它是协议文档里就有的实例, 不是我临时编的 —— 而且 CRC 由【生产的组帧函数
 *      lora_proto_build_frame()】算出来, 打完还会跟文档值 FF A8 对照一次:
 *      这一步顺带自检了"生产组帧路径(含 CRC16)本身对不对"。组帧若错, 对端收到
 *      也必然当坏帧丢掉 —— 那就与射频无关了, 必须先修那里。
 *   2) dev=210001 + func=0x02(液位上报)用的就是液位传感器自己的地址与功能码,
 *      对端的"按首字节识别设备类型/环境ID + 查功能码"这条逻辑能原样走一遍。
 *   3) 与 ① 的 0xAA 探针形成对照: 对端"只认业务帧、不理原始字节"时, 这一条就是
 *      证明射频 TX 路径可用的那一半。
 *   ⚠ 口径说明(如实写清): 网关正常是【不会】主动发 0x02 的(0x02 是传感器上行)。
 *     这里发它纯粹是为了让对端一眼认出"这是一条格式完全正确的帧", 便于区分它与噪声。
 *     若现场更想让对端走"下行指令"那条路, 把下面的 func/data 改成 0x01(设置液位参数)
 *     或 0x03(设置声光)即可 —— 组帧/CRC 都由生产函数负责, 改这两个字段不会算错 CRC。
 */
static void rawtest_send_legal_frame(void)
{
    bool ok;

    (void)memset(&s_rawtest_frame, 0, sizeof(s_rawtest_frame));

    s_rawtest_frame.gw_uid[0]  = 0x11U;    /* 110001 = 本网关(param_save 的出厂默认 gw_uid) */
    s_rawtest_frame.gw_uid[1]  = 0x00U;
    s_rawtest_frame.gw_uid[2]  = 0x01U;
    s_rawtest_frame.dev_uid[0] = 0x21U;    /* 210001 = 液位传感器(lora_business.c 固定从机表) */
    s_rawtest_frame.dev_uid[1] = 0x00U;
    s_rawtest_frame.dev_uid[2] = 0x01U;
    s_rawtest_frame.func       = LORA_FUNC_SENSOR_REPORT;   /* 0x02 */
    s_rawtest_frame.data_len   = 7U;
    s_rawtest_frame.data[0]    = 0x04U;    /* 液位 大端 = 0x04D2 = 1234 */
    s_rawtest_frame.data[1]    = 0xD2U;
    s_rawtest_frame.data[2]    = 0x64U;    /* 电量 = 100% */
    s_rawtest_frame.data[3]    = 0x00U;    /* 月(测试帧置 0) */
    s_rawtest_frame.data[4]    = 0x00U;    /* 日 */
    s_rawtest_frame.data[5]    = 0x00U;    /* 时 */
    s_rawtest_frame.data[6]    = 0x00U;    /* 分 */

    if (!lora_proto_build_frame(&s_rawtest_frame, NULL))
    {
        log_info("[RAWTEST] ② !! 组帧失败(lora_proto_build_frame 返回 false), 这一条跳过\r\n");
        return;
    }

    /* 组帧自检: 与 lora_proto.h 的参考帧逐字节对照 */
    if ((s_rawtest_frame.raw_len == 20U) &&
        (s_rawtest_frame.raw[18] == 0xFFU) && (s_rawtest_frame.raw[19] == 0xA8U))
    {
        log_info("[RAWTEST] ② 组帧自检: len=%u crc=%02X %02X —— 与文档参考帧【一致】(AA 55 01 11 00 01 21 00 01 02 07 04 D2 64 00 00 00 00 FF A8)\r\n",
                 (unsigned)s_rawtest_frame.raw_len,
                 (unsigned)s_rawtest_frame.raw[18], (unsigned)s_rawtest_frame.raw[19]);
    }
    else
    {
        log_info("[RAWTEST] ② !! 组帧自检【不一致】: len=%u crc=%02X %02X (文档参考帧应为 len=20 crc=FF A8) —— 发出去的帧本身就是坏帧, 对端会丢它, 与射频无关\r\n",
                 (unsigned)s_rawtest_frame.raw_len,
                 (unsigned)s_rawtest_frame.raw[18], (unsigned)s_rawtest_frame.raw[19]);
    }

    log_info("[RAWTEST] ② 发【合法业务帧】func=0x%02X dev=%02X%02X%02X len=%u(整帧字节由发送函数自己打 [TX] 那一行)\r\n",
             (unsigned)s_rawtest_frame.func,
             s_rawtest_frame.dev_uid[0], s_rawtest_frame.dev_uid[1], s_rawtest_frame.dev_uid[2],
             (unsigned)s_rawtest_frame.raw_len);

    ok = lora_proto_send_frame(&s_rawtest_frame);

    log_info("[RAWTEST] ② 发射结果: %s\r\n",
             ok ? "TX_DONE 成功(空口上确实发完了)"
                : "【等 TX_DONE 超时 —— 这一条没发出去】");
}

/**
 * @brief 一轮: 先打"发射前"累计计数 → 发两条帧 → 再打"发射后"累计计数
 *
 * 【发射前后都要打】这样两个数字一减就是"这一轮到底发生了什么":
 *   发射前 RX_DONE 没变、发射后 TX_DONE +2 = 发射路径正常而空口上没有对端的帧;
 *   或者 PREAMBLE 在涨而 SYNC 不动 = 空口参数层面就对不上。
 */
static void rawtest_do_round(void)
{
    s_rawtest_round++;

    log_info("\r\n[RAWTEST] ================ 第 %lu 轮 (tick=%lums) ================\r\n",
             (unsigned long)s_rawtest_round, (unsigned long)get_tick_ms());

    rawtest_print_counters("发射前");

    rawtest_send_probe();
    rawtest_send_legal_frame();

    rawtest_print_counters("发射后");
}

/**
 * @brief 【最小 LoRa 收发隔离测试】主流程 —— 只起 时基/日志/射频, 之后【永不返回】
 *
 * 【时基/周期】5 秒周期用 get_tick_ms() 判断, 主循环每 1ms 与 1ms 时基对齐;
 *   接收期间【每毫秒】都在处理射频中断, 不会因为"等周期"而漏掉接收。
 *
 * 【关于阻塞 —— 如实说明】
 *   发射这一步是【会阻塞】的, 而且这是故意的: 生产发送路径
 *   LoraApp_SendAsync() 内部有 LORA_TX_SETTLE_MS(500ms) 的收发切换等待,
 *   之后 lora_proto_send_*() 还要等 TX_DONE(最多 LORA_TX_DONE_TIMEOUT_MS=1000ms)。
 *   本次测试的立论就是"用生产那一条路径去测", 所以不能把这些等待去掉或改成非阻塞
 *   (改了就不是现状了)。两条帧最坏约 3 秒, 5 秒周期内的其余时间完全在收 ——
 *   对"收不到帧"这个现象来说, 阻塞只影响我们自己发射的那一小段, 可以接受。
 *   除发射之外, 主循环里【没有任何阻塞延时】。
 */
/* ---------- 【模式③: 可编辑的测试报文】----------
 * ★ 这里放的是【整帧】, 原样发给射频 —— 不组帧、不自动算 CRC ★
 *   所以: 你改了 payload, 就必须【自己重算 CRC 并改最后两个字节】,
 *   否则对端会因 CRC 不过而整帧丢弃(现象: 对端毫无反应)。
 *   CRC 规则(已用三帧现场数据复算验证过):
 *       crc16_modbus, 覆盖【版本字节 .. data 末尾】, 结果【低字节在前】。
 *   下面这 40 字节是现场真实用过的声光帧(func=0x03, 文字"水深危险，禁止通行"),
 *   其 CRC = 5D 17 已按该规则复算吻合 ✓ —— 可直接拿它当模板改。
 *
 * 【改法示例】只想换文字: 保持总长度不变(改 dlen 与 CRC 即可); 若总长变了,
 *   记得同时改 dlen(第 11 个字节)与 CRC(最后两字节)。
 */
static const uint8_t s_rawtest_user_frame1[] =
{
    0xAA, 0x55,                     /* 0-1   帧头 */
    0x01,                           /* 2     版本 */
    0x11, 0x00, 0x01,               /* 3-5   网关地址(发出方) */
    0x31, 0x00, 0x01,               /* 6-8   设备地址(接收方) */
    0x03,                           /* 9     功能码(0x03=设置声光) */
    0x1B,                           /* 10    数据长度 = 27 */
    0x01, 0x05, 0x50, 0x01, 0x01, 0x01, 0x0F, 0x14, 0x12,       /* 11-19 声光参数 9B */
    0xCB, 0xAE, 0xC9, 0xEE, 0xCE, 0xA3, 0xCF, 0xD5, 0xA3, 0xAC, /* 20-29 "水深危险，" */
    0xBD, 0xFB, 0xD6, 0xB9, 0xCD, 0xA8, 0xD0, 0xD0,             /* 30-37 "禁止通行"   */
    0x5D, 0x17                      /* 38-39 CRC16(低字节在前) —— 改了上面就必须重算! */
};

static const uint8_t s_rawtest_user_frame[] =
{
		0xAA, 0x55,                     /* 0-1   帧头 */
    0x01,                           /* 2     版本 */
    0x11, 0x00, 0x01,               /* 3-5   网关地址(发出方) */
    0x31, 0x00, 0x01,               /* 6-8   设备地址(接收方) */
    0x05,                           /* 9     功能码(0x05=解绑设备) */
		0x00,
		0xE9,0x25
	
};

static void rawtest_send_user_frame(void)
{
    uint8_t i;

    rawtest_print_counters("发射前");

    log_info("[RAWTEST] 发整帧 %u 字节(原样发出, 不自动算 CRC): ",
             (unsigned)sizeof(s_rawtest_user_frame));
    for (i = 0u; i < (uint8_t)sizeof(s_rawtest_user_frame); i++)
    {
        log_info("%02X ", s_rawtest_user_frame[i]);
    }
    log_info("\r\n");

    /* lora_proto_send_raw 内部已阻塞等到 TX_DONE(成功才返回 true), 并消费掉标志;
     * 这里不能再等第二次 —— 第二次等的是一个永不会来的新 TX_DONE, 必超时。 */
    if (!lora_proto_send_raw(s_rawtest_user_frame, (uint8_t)sizeof(s_rawtest_user_frame)))
    {
        log_info("[RAWTEST] !! 发送失败(协议层拒绝或等 TX_DONE 超时)\r\n");
        return;
    }
    log_info("[RAWTEST] TX_DONE 正常\r\n");

    rawtest_print_counters("发射后");
}

static void rawtest_run(void)
{
    uint32_t t_tx;
    uint32_t t_sup;
    uint32_t now;

    /* ---------- 1. 只做必要的底层初始化: 时基 + 日志 ---------- */
    bsp_tim1_init_1ms();    /* 必须最先: delay_ms / get_tick_ms 都依赖 ATIM1 时基 */
    log_init();             /* 日志口 USART1(PA9-TX/PA10-RX, 115200) —— 全部证据从这里出来 */

    log_info("\r\n==== 【最小 LoRa 收发隔离测试】模式=%s  %s %s ====\r\n",
             MAIN_MODE_NAME, __DATE__, __TIME__);
    log_info("[RAWTEST] 本固件【只跑射频】: 4G / 业务层 / 协议层 / 参数Flash / 心跳 全部让开\r\n");
    log_info("[RAWTEST] 节拍: 每 %lums 发两条(① 全 0xAA 探针  ② 合法业务帧), 其余时间一直连续接收; "
             "收到字节在【射频 IRQ 层】原样打印([RAW-IRQ]/[RAW-RX]), 不经过任何解析\r\n",
             (unsigned long)RAWTEST_TX_PERIOD_MS);
    log_info("[RAWTEST] 生产射频参数(project_config.h, 本测试【未改任何一项】): "
             "LORA_FRE=%luHz | SF=%u | BW代码=%u(0=125k 1=250k 2=500k) | CR=4/%u | 前导码=%u | "
             "显式头=%s | CRC=%s | 功率=%ddBm | IQ反转=%s | 同步字: 【不额外设置】(保持芯片现状)\r\n",
             (unsigned long)LORA_FRE,
             (unsigned)LORA_SPREADING_FACTOR,
             (unsigned)LORA_BANDWIDTH,
             (unsigned)((unsigned)LORA_CODINGRATE + 4U),
             (unsigned)LORA_PREAMBLE_LENGTH,
             (LORA_FIX_LENGTH_PAYLOAD_ON) ? "否(定长)" : "是(变长)",
             "开",
             (int)LORA_TX_OUTPUT_POWER,
             (LORA_IQ_INVERSION_ON) ? "是" : "否");

    /* 上一轮是谁把我复位的 —— 欠压复位(BOR)本身就是"射频收不到"的重大嫌疑
     * (供电塌陷 -> BUSY 卡高 -> 配置没写进芯片)。这条与射频无关, 但必须看得见。 */
    reset_cause_report();

    /* ---------- 2. 4G 模组【不上电】----------
     * 少一个干扰源(4G 上电浪涌/发射谐波), 同时保证 PA8 被驱到确定电平而不是悬空
     * 半通(见 bsp_gpio.h 的 G4_PowerSwitch_InitNoPower 说明)。
     * 【注意】这里不调用任何 4G 初始化: g4_at_init / g4_urc / g4_io 一行都不跑。 */
    log_info("[RAWTEST] 4G 模组: 不上电(PA8 驱到断电电平, 不悬空), 也不跑任何 4G 代码\r\n");
    G4_PowerSwitch_InitNoPower();

    /* ---------- 3. 射频初始化 —— 【就是生产那一套】----------
     * LoraApp_Init() 正是 lora_proto_init() 内部调用的同一个函数:
     *   Radio.Init(生产事件回调) + Radio.SetTxConfig/SetRxConfig/SetChannel,
     *   参数全部取 project_config.h 的 LORA_*。
     * 所以"发射参数与生产一致"不是靠我抄一遍保证的, 而是【同一个函数 + 同一份宏】。
     * ⚠ 本测试【绝不】调 LLCC68SetSyncWord / RadioSetPublicNetwork: 同步字保持现状。 */
    log_info("[RAWTEST] 射频初始化: LoraApp_Init() (生产初始化函数, 未另配参数)...\r\n");
    LoraApp_Init();

    log_info("[RAWTEST] 射频自检: BUSY电平=%u 卡高=%s | 等BUSY超时累计=%lu 次 | 芯片状态=0x%02X 模式=%u(%s)\r\n",
             (unsigned)LLCC68BusyLevel(),
             (LLCC68IsStuck() != 0u) ? "是(模组不应答, 配置可能根本没写进芯片!)" : "否",
             (unsigned long)LLCC68BusyTimeoutCount(),
             (unsigned)LLCC68GetStatus().Value,
             (unsigned)LLCC68GetStatus().Fields.ChipMode,
             (LLCC68GetStatus().Fields.ChipMode == 5U) ? "RX=在收" : "不是 RX");

    LoraApp_StartRxContinuous();    /* 先进入连续接收, 再按 5 秒周期发 */
    log_info("[RAWTEST] 已进入【连续接收】(Radio.Rx(0) + RxContinuous=true): 网关常态就该一直在收\r\n");

    rawtest_print_counters("开机");

    t_tx  = get_tick_ms();
    t_sup = t_tx;

    /* ---------- 4. 测试主循环(永不返回) ---------- */
    for (;;)
    {
        now = get_tick_ms();

        /* (A) 处理射频中断 —— 整条接收链路上唯一的驱动点。
         *     收到帧时, radio.c 的 [RAW-IRQ]/[RAW-RX] 与 lora_proto.c 的 [LoRa RX]
         *     都会在【这一句里】立刻打印出来, 完全不经过任何帧解析。
         *     (DIO1 的 EXTI4 中断本体只置一个 IrqFired 标志, 真正的处理在这里做,
         *      所以这里的打印是【主循环上下文】, 不是 ISR 上下文。) */
        LoraApp_ProcessIrq();

        /* (B) 每 1s: 接收守护 —— 与生产 lora_proto_poll() 里那条腿用同一判据
         *     (读【芯片真实状态字节】而不是软件缓存), 不在 RX 就补开并计数。 */
        if ((uint32_t)(now - t_sup) >= RAWTEST_RX_SUPERVISE_MS)
        {
            t_sup = now;

            if (!LoraApp_IsRxActive())
            {
                LoraApp_EnsureRxContinuous("[RAWTEST] 守护: 芯片不在接收(RX)模式");
            }
        }

        /* (C) 每 5s: 发射【一个整帧】(可编辑的 s_rawtest_frame, 原样发出) + 发射前后各打一次计数 */
        if ((uint32_t)(now - t_tx) >= RAWTEST_TX_PERIOD_MS)
        {
            rawtest_send_user_frame();

            /* 从"这一轮发射结束"起算 5 秒 —— 保证每两轮之间有整 5 秒是纯粹在收的 */
            t_tx = get_tick_ms();
        }

        /* (D) 与 1ms 时基对齐, 避免空转(射频中断仍可抢占) */
        while (get_tick_ms() == now)
        {
        }
    }
}

#endif /* MAIN_LORA_RAW_TEST */

/* ==================================================================
 * 模式②: 从机配对模式  (MAIN_MODE_PAIRING=1)
 * ==================================================================
 * 【场景】从机(液位/声光)出厂后还不知道网关是谁, 于是用【未配对网关地址 FF FF FF】
 *   发一条配对请求; 网关收到后回一条 ACK, 从机从 ACK 里学到网关地址, 配对完成。
 *
 * 【现场实测的两帧 —— 本实现的依据】(CRC 已用 crc16_modbus 复算验证过)
 *   请求:  AA 55 01 FF FF FF 21 00 01 04 00 BC 63
 *          └头┘ ver └─gw─┘  └─dev─┘ fc dl  └CRC┘      gw=FF FF FF 表示"还没配对"
 *   应答:  AA 55 01 11 00 01 21 00 01 80 00 4A 76
 *          └头┘ ver └─gw─┘  └─dev─┘ fc dl  └CRC┘      fc=0x80(ACK_OK), dlen=0
 *   -> 规则: ACK = 把请求的 gw_uid 换成本网关地址、func 改 0x80、dlen=0, 重算 CRC。
 *
 * 【本模式只跑射频】4G 不上电、不跑云协议层/事件缓存/水位评估 —— 配对期不需要它们。
 *
 * ★ 下面这个"本网关地址"目前是【编译期常量】★
 *   它必须与配置里的网关地址一致(样本是 11 00 01)。等你确认"ACK 里的 gw_uid 该从
 *   配置的哪个字段取"(system.gwId 是字符串, 要按 hex 解析成 3 字节), 我再接成配置来源。
 */
#if MAIN_MODE_PAIRING

#ifndef PAIRING_GW_UID_HEX
/* 本网关的 3 字节地址(十六进制)。改这里即可换网关地址。 */
#define PAIRING_GW_UID_HEX       0x110001u
#endif

#define PAIRING_FUNC_REQUEST     0x04u     /* 从机请求配对的功能码(现场实测) */

static void pairing_on_frame(const lora_frame_t *frame, int8_t rssi, int8_t snr)
{
    lora_frame_t ack;

    if (frame == NULL)
    {
        return;
    }

    log_info("[PAIR] rx gw=%02X%02X%02X dev=%02X%02X%02X func=0x%02X dlen=%u rssi=%d snr=%d\r\n",
             frame->gw_uid[0], frame->gw_uid[1], frame->gw_uid[2],
             frame->dev_uid[0], frame->dev_uid[1], frame->dev_uid[2],
             (unsigned)frame->func, (unsigned)frame->data_len, (int)rssi, (int)snr);

    /* === 阶段1: 从机发 0x04 配对请求(gw_uid=FFFFFF) -> 网关回 0x80 ACK 告知本网关地址 ===
     * 从机收到 ACK 后会把内部网关值从 FFFFFF 改成本网关地址, 然后一直回 0x80 ACK。 */
    if (frame->func == PAIRING_FUNC_REQUEST)
    {
        /* 组 ACK: gw_uid 换成本网关, func=0x80(ACK_OK), dlen=0。CRC 由协议层算。
         * 注意 data_len=0, 所以 data[] 里的内容不参与组帧, 不必清零。 */
        ack.gw_uid[0] = (uint8_t)((PAIRING_GW_UID_HEX >> 16) & 0xFFu);
        ack.gw_uid[1] = (uint8_t)((PAIRING_GW_UID_HEX >> 8) & 0xFFu);
        ack.gw_uid[2] = (uint8_t)(PAIRING_GW_UID_HEX & 0xFFu);
        ack.dev_uid[0] = frame->dev_uid[0];
        ack.dev_uid[1] = frame->dev_uid[1];
        ack.dev_uid[2] = frame->dev_uid[2];
        ack.func       = (uint8_t)LORA_FUNC_RESP_OK;
        ack.data_len   = 0u;

        log_info("[PAIR] 收到配对请求 dev=%02X%02X%02X -> 回 ACK(本网关 %02X%02X%02X), 等从机确认\r\n",
                 frame->dev_uid[0], frame->dev_uid[1], frame->dev_uid[2],
                 ack.gw_uid[0], ack.gw_uid[1], ack.gw_uid[2]);

        /* lora_proto_send_frame 内部已阻塞等到 TX_DONE(成功才返回 true),
         * 这里不能再等第二次 —— 第二次必超时(不会有新 TX_DONE)。 */
        if (!lora_proto_send_frame(&ack))
        {
            log_info("[PAIR] !! ACK 发送失败(协议层拒绝或等 TX_DONE 超时)\r\n");
            return;
        }
        /* 不再在此判定"配对完成": 要等从机回 0x80 ACK(阶段2)才算双向握手完成 */
        return;
    }

    /* === 阶段2: 从机改完网关值后回 0x80 ACK -> 网关写本地从机列表(配对确认完成) ===
     * 这是"加上的部分": 之前网关收到 0x04 回一次 ACK 就算完, 现在要等从机再回
     * 0x80 确认, 才把从机写进 config_store(本地从机列表持久化)。
     * 上报云端在 PAIRING 模式不跑(4G 没上电), 不影响后续流程 —— 切回 CLOUD 模式
     * 后从机已在 config_store 里, 直接可用。 */
    if (frame->func == LORA_FUNC_RESP_OK)
    {
        /* dev_type = uid[0]高4位: 0x02=液位(sensor), 0x03=声光(alarm) */
        bool is_sensor = ((uint8_t)(frame->dev_uid[0] >> 4) == 0x02U);
        char uid_hex[CFG_UID_STR_MAX];
        cfg_result_t r;

        (void)cfg_uid_from_bytes(frame->dev_uid, uid_hex);  /* 3字节转hex, 不会失败 */

        if (!config_store_can_add(is_sensor))
        {
            log_info("[PAIR] !! 收到 %s 的确认 ACK, 但从机表已满/已存在, 不写入\r\n",
                     uid_hex);
            return;
        }
        /* dev_type: 联动设备类型(1声光/2LED/3一体); is_sensor=true 时该参数被忽略传0。
         * now_unix 传 0: PAIRING 模式未初始化时间源(RTC/4G), 绑定记录时间戳为 0
         * 不影响设备列表写入; 切回 CLOUD 模式后 cloud_proto 会用真实时间。 */
        r = config_store_bind_device(uid_hex, "", "", is_sensor,
                                      is_sensor ? 0U : 1U, 0U);
        if (r == CFG_OK)
        {
            log_info("[PAIR] ★ 双向握手完成: %s(%s) 已写入本地从机列表 ★\r\n",
                     uid_hex, is_sensor ? "液位" : "声光");
        }
        else
        {
            log_info("[PAIR] !! %s 写入失败(code=%d)\r\n", uid_hex, (int)r);
        }
        return;
    }

    log_info("[PAIR] 非配对请求/确认(func=0x%02X), 忽略\r\n", (unsigned)frame->func);
}

static void pairing_run(void)
{
    bsp_tim1_init_1ms();    /* 必须最先: delay_ms / get_tick_ms 都依赖 ATIM1 时基 */
    log_init();             /* 日志口 USART1(PA9-TX/PA10-RX, 115200) */

    log_info("\r\n==== 【从机配对模式】模式=%s  %s %s ====\r\n",
             MAIN_MODE_NAME, __DATE__, __TIME__);
    log_info("[PAIR] 本固件只跑射频: 4G 不上电 / 不跑云协议层 / 不跑水位评估\r\n");
    log_info("[PAIR] 本网关地址(ACK 里的 gw_uid) = %02X %02X %02X\r\n",
             (unsigned)((PAIRING_GW_UID_HEX >> 16) & 0xFFu),
             (unsigned)((PAIRING_GW_UID_HEX >> 8) & 0xFFu),
             (unsigned)(PAIRING_GW_UID_HEX & 0xFFu));
    log_info("[PAIR] 双向握手: 从机发 0x%02X(gw=FFFFFF) -> 网关回 0x80 ACK(告知本网关地址)\r\n",
             (unsigned)PAIRING_FUNC_REQUEST);
    log_info("[PAIR]              -> 从机改网关值后回 0x80 ACK -> 网关写本地从机列表\r\n");

    reset_cause_report();   /* 上电复位原因(欠压 BOR 是"收不到"的重大嫌疑) */

    /* 4G 模组不上电: 少一个干扰源, 且 PA8 被驱到确定电平而不是悬空 */
    G4_PowerSwitch_InitNoPower();
    log_info("[PAIR] 4G 模组: 不上电(PA8 驱到断电电平), 也不跑任何 4G 代码\r\n");

    /* 射频初始化(生产同一套): Radio.Init + 持续接收 + 注册帧回调 */
    lora_proto_init(pairing_on_frame);
    log_info("[PAIR] 射频已进入持续接收, 等待配对请求…\r\n");

    for (;;)
    {
        lora_proto_poll();      /* 处理 IRQ + 解析 + 分发(含"回到接收"守护) */
    }
}

#endif /* MAIN_MODE_PAIRING */

/* ==================== main ==================== */
int main(void)
{
#if (MAIN_MODE_LORA_TEST != 0) || (MAIN_MODE_PAIRING != 0)
    /* ★ 2026-09-28: 把"4G 断电"提到【main 的第一件事】★
     * 原因(见 bsp_gpio.h:155): MCU 复位后 PA8 是【输入悬空】, 而本板【悬空 = 给模组上电】
     *   (现场已确认 PA8 低=上电 / 高=断电)。所以等到 rawtest_run()/pairing_run() 里
     *   再调 G4_PowerSwitch_InitNoPower() 其实已经晚了 —— 前面 bsp_tim1_init_1ms()、
     *   log_init() 与好几行日志(115200bps, 每行毫秒级)的这段时间里, 模组早已被通电启动。
     * 纯软件只能把这个窗口缩到最短; 要彻底消除需在 PA8 上加【上拉电阻】(高=断电)。
     * 各模式内原来那次调用保留(重复无害, 也保证单独调用该模式函数时同样安全)。 */
    G4_PowerSwitch_InitNoPower();
#endif

#if MAIN_MODE_PAIRING
    /* ---- 【从机配对模式】只起 时基/日志/射频, 然后进配对循环(永不返回) ---- */
    pairing_run();
#elif MAIN_MODE_LORA_TEST
    /* ---- 【LoRa 收发测试模式】只起 时基/日志/射频, 然后进测试循环(永不返回) ----
     * ⚠️ 这里用【新宏 MAIN_MODE_LORA_TEST】而不是老宏 MAIN_LORA_RAW_TEST:
     *    两者虽然由 project_config.h 映射成一致, 但分发处统一用新宏才不会再出现
     *    "上面的 #if 看新宏、下面的 #elif 看老宏"那种各看各的坑(v100 就是这么错的)。
     * 正常固件的主流程【一行都没删】, 它整体在 #else 里: 开关=0 时逐字参与编译,
     * 开关=1 时完全不参与(4G 初始化 / param_save / 业务轮询 / 心跳 一次都不跑)。 */
    rawtest_run();
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
    console_init();

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

    reset_cause_report();   /* 上一轮到底是谁把我复位的 */

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
     *   - 业务层在 0x04 绑定成功/失败、0x05 解绑完成时回调 main_publish_bind_event,
     *     由它组 event.device{bound/bind_failed/unbound} 发到 event topic。
     * 这两条注册都要在 lora_business_init() 之后(网关 UID 已读出)、
     * 在云端命令处理之前完成。
     * ⚠️【别删】这两行 + main_publish_bind_event(): 少任何一个, sen.bind/dev.bind
     *    都会恒回 1006(云端再也绑不上设备) —— 2026-09-29 误删过一次。 */
    cloud_proto_set_pair_hook(lora_business_open_bind_window);
    lora_business_set_bind_event_cb(main_publish_bind_event);
    /* ★2026-10-08 新增★ 声光(0x03)执行结果回调 -> event.action(+ 离线时补 event.device)
     * 声光设备对 0x03 回 0x80(收到了) -> 0x82(成功)/0x83(失败), 见 lora_proto.h。
     * ⚠️【别删】少了这一行, 声光状态机照样跑完 15 秒 x3 条的重试, 但结果
     *   一个字都发不出去 —— 平台永远不知道灯亮没亮。 */
    lora_business_set_alarm_result_cb(main_on_alarm_result);
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
                                 ? cfg->system.gw_id : "110001";

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
    g4_at_set_link_watchdog(5U * main_cfg_period_ms(config_store_get()->network.status_interval_s,
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
    lora_business_set_dl_prefilter(main_cloud_downlink_try);

    /* 把 sys.query_status 要的运行时状态接上(v40): 协议层不 include 4G/LoRa 的模块,
     * 由这里提供一个回调 —— 这样协议层仍然能在 PC 上脱机单测。
     * ⚠ 不注册的话 sys.query_status 会明确回 1006(而不是拿一堆 0 冒充状态)。 */
    cloud_proto_set_status_provider(main_fill_status);

    /* sys.time_sync 的写时钟钩子(v91)。★ 不注册的话这条命令会明确回 1006 ★ ——
     * 此前正是如此（自 v49 实现该命令起一直没接上，见 v82 说明）。 */
    cloud_proto_set_time_sync_hook(main_time_sync);

    /* sen.query / dev.query 的 lastTs 需要"收到那一帧时的 Unix 秒"(v53)。
     * 用钩子注入 —— last_value 模块要能在 PC 上脱机单测, 不能直接调 main_now_unix()。
     * ⚠ 不注册的话 lastTs 在应答里【根本不出现】, 不拿 0 冒充 1970。 */
    last_value_set_now_hook(main_now_unix);
#if MAIN_STATUS_JSON_ENABLE
    /* 液位失联/恢复事件: lora_business 只负责"什么时候该报"(判定+跳变去重),
     * 真正的组 JSON + 发 event topic 在本文件的 main_publish_device_event()。
     * 这样 LoRa 业务层不用依赖云协议层, 主机测试也就不用为它造 4G 桩。 */
    lora_business_set_sensor_event_cb(main_publish_device_event);
    log_info("[BOOT] 液位失联事件回调已挂载(event.device -> event topic)\r\n");
#endif
    log_info("[BOOT] MQTT 下行回调 = lora_mqtt_recv_cb, 前置过滤器 = 云协议层(JSON)\r\n");
#else
    /* LoRa 关闭时也必须注册下行回调, 否则云端下发的消息会被静默丢掉,
     * 现场会误判成"模组收不到下发"(见上面 main_downlink_log_cb 的说明)。 */
    g4_at_set_recv_cb(main_downlink_log_cb);
    log_info("[BOOT] LoRa 已关闭 -> MQTT 下行回调 = 只打印(JSON 命令仍走云协议层)\r\n");
#endif

#if MAIN_AT_CONSOLE_ENABLE
    /* 把云协议离线自测命令挂到日志口(#json / #status / #topic / #offline) + 射频诊断(#lora)。
     * 弱网/离线时靠它验证协议层; 射频收不到时靠 #lora 看芯片里的实际配置。 */
    g4_at_set_console_ext(main_console_ext, main_console_help);
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

#if MAIN_OTA_ENABLE
    ota_init();
    log_info("[BOOT] OTA(MQTT 分包版)也已初始化 —— 但与 cloud_proto 无连接线\r\n");
#endif

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
    /* ⚠️ 这里【不】加 #if MAIN_OTA_ENABLE —— 踩过一次:
     *   MAIN_OTA_ENABLE 是"旧的 MQTT 分包版 ota.c"的开关, 当前 = 0。
     *   A/B 回滚对应的是 OTA over HTTP 那条线, 与它无关。
     *   2026-10-11 第一版就是错包在它里面 ⇒ 整段被编译掉 ⇒ 上板看不到 [OTAHL]。
     *   ⇒ 保持无条件编译: 模式 1 下只记录日志, 零副作用(见下面的 (乙) 说明)。 */
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
        if (cloud_proto_reboot_due(main_now_unix()))
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
            console_poll();
        }

        /* (D) 每 1s: 状态上报 + LED 心跳 */
        if ((uint32_t)(now - t_status) >= TICK_STATUS_MS)
        {
            t_status = now;

            status_report();

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
                    s_last_ld       = ld;
                    s_last_ld_valid = true;
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
                main_failsafe_step(&ld);

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
                         * 相关函数见上面 main_publish_status_change / main_cache_status_change。
                         * 之前这里只有"入缓存"一支; 现在在线时也真的把 status_change 发出去了。 */
                        if (g4_at_is_online())
                        {
                            main_publish_status_change((uint8_t)ld.prev_state, (uint8_t)ld.state,
                                                       ld.gear, ld.level_mm);
                        }
                        else
                        {
                            main_cache_status_change((uint8_t)ld.prev_state, (uint8_t)ld.state,
                                                     ld.gear, ld.level_mm);
                        }

                        /* 事件【已生成】(直发或入缓存都算) -> 记下基准, 供下一次去抖比较。
                         * ★ 不在这里留"失败重试"的口子: 入缓存那条本来就是断线补报的入口;
                         *   直发失败也只影响这一条, 下一个够格的样本会再发一条新事件。 */
                        level_supervisor_mark_event(ld.level_mm, get_tick_ms());
                    }
                }

                /* ---- (D3) 水位遥测上行(第 3 条): 周期 or 变化量触发 ---- */
                main_telemetry_step();

                /* ---- (D4) manual 模式窗口(第 6 条): 到期自动回归 auto ---- */
                main_manual_step();
            }

            /* ---- 断线补报: 每次最多补一页(16 条), 缓存空了立刻返回 (v70) ---- */
            main_backfill_poll();

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
#endif

#if MAIN_HEARTBEAT_LED
            GPIO_TogglePin(GPIOA, GPIO_PIN_8);
#endif

#if MAIN_OTA_ENABLE
            ota_poll();
#endif

            /* OTA over HTTP 的进度上报 + 就绪后复位。
             * 放在 1s 块里是【刻意的】: 文档 7.6 要求 ota.progress 每 10% 或 10s
             * 一条, 状态机本身的推进在 5ms 块里(见上面 (B)), 两件事分开。
             * 本函数只读状态 + 发 MQTT, 不发 AT、不阻塞; 空闲时第一件事就是 return。
             * 刻意【不】加 #if MAIN_G4_ENABLE: 那样在 4G 关掉时它就成了没人调的
             * static 函数, 编译器会报 unused-function。 */
            main_ota_progress_poll();
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
            uint32_t hb_period_ms = main_cfg_period_ms(config_store_get()->network.status_interval_s,
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
            hb_res = main_publish_status_json();
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
