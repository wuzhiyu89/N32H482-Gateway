/**
 * @file  app_modes.c
 * @brief 两种"只跑射频"的工程模式：LoRa 收发测试 / 从机配对
 *
 * 它们不是产品功能，而是现场排障与产线配对用的固件形态：
 *   · MAIN_MODE_LORA_TEST：每 5 s 发一帧，其余时间连续收（4G 不上电）
 *   · MAIN_MODE_PAIRING  ：收从机的 0x04 配对请求，回 0x80 ACK
 *
 * 两者都【不进主循环】，由 main() 直接调用后永不返回。
 * 选择哪个模式见 app/drivers/lora/project_config.h（三选一，编译期）。
 */
#include "n32h47x_48x.h"
#include <string.h>
#include <stdio.h>

#include "log.h"
#include "bsp_tim.h"
#include "bsp_gpio.h"
#include "app_core.h"
#include "app_state.h"
#include "param_save.h"
#include "project_config.h"
#include "lora_proto.h"
#include "lora_business.h"

#if MAIN_LORA_RAW_TEST
#include "radio.h"
#include "llcc68-board.h"
#include "llcc68.h"
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
 *   把 app/drivers/lora/project_config.h 里 MAIN_LORA_RAW_TEST 由 1 改成 0
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

static void app_mode_rawtest_run(void)
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
    app_report_reset_cause();

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

static void app_mode_pairing_run(void)
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

    app_report_reset_cause();   /* 上电复位原因(欠压 BOR 是"收不到"的重大嫌疑) */

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
