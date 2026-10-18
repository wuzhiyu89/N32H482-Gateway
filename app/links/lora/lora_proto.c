/**
 * \file  lora_proto.c
 * \brief LoRa 自定义二进制帧协议实现
 *
 * 复用:
 *   - utils.c 的 crc16_modbus (Modbus CRC16, 0xA001, 0xFFFF)
 *   - radio.c / llcc68.c 的底层收发
 *
 * 架构:
 *   LoRa 是整包投递(RxDone 回调每次给完整一帧), 不存在 TCP 分包问题。
 *   用简单的"待解析标志 + 双缓冲 + 中断保护"代替环形缓冲:
 *     RadioRxDoneCb (中断)  → 填 g_rx_buf/g_rx_len, g_rx_ready=true
 *     lora_proto_poll (主循环) → 读 g_rx_ready → parse_frame_impl → 分发回调
 *   双缓冲避免 RxDone 覆盖 poll 正在解析的数据。
 *
 * CRC 说明:
 *   协议规定 CRC 覆盖 "从协议版本到 data 末尾"，跳过帧头 AA 55 和末尾 CRC 两字节。
 *   直接对 &raw[2] 起 (raw_len - 4) 字节算 crc16_modbus, 手动填末尾两字节。
 */
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "lora_proto.h"
#include "radio.h"
#include "LLCC68-board.h"
#include "bsp_tim.h"        /* delay_ms(): ATIM1 时基, 发送前的收发切换等待 */
#include "utils.h"
#include "log.h"

/* ==================== 接收缓冲(单缓冲 + 中断写入标记) ====================
 * LoRa 帧率极低(传感器几分钟才上报一帧), 覆盖概率可忽略, 用单缓冲就够了。
 * 中断 RadioRxDoneCb 写 g_rx_buf, 置 g_rx_write_idx;
 * poll 读 g_rx_write_idx, 清 g_rx_ready, 读 g_rx_buf[write_idx]。 */
static uint8_t  g_rx_buf[LORA_FRAME_RAW_MAX];
static uint8_t  g_rx_len;
static int8_t   g_rx_rssi;
static int8_t   g_rx_snr;
static volatile uint8_t g_rx_ready;        /* 中断置位, poll 清位 */
static uint8_t  g_rx_buf_snapshot[LORA_FRAME_RAW_MAX]; /* 快照缓冲,避免中断覆盖 */
static volatile uint8_t g_need_restart_rx; /* TxDone 中断置位, poll 里重启 RX 窗口 */

/* ==================== 【#lora 诊断用】应用层累计计数(只增不减) ====================
 * 芯片侧的 IRQ 计数在 radio.c(g_irqStat), 这里两个是【应用层】的:
 *   g_stat_rxdone_cb  RxDone 回调真的被调用了几次 -> 帧确实到了协议层;
 *   g_stat_parse_fail 到了协议层但解析(帧头/版本/长度/CRC)失败了几次。
 * 三个数字放在一起就能分清"芯片没收到"和"收到了但被上层丢掉"。 */
static uint32_t g_stat_rxdone_cb  = 0U;
static uint32_t g_stat_parse_fail = 0U;

/* ==================== Radio 底层事件回调 ==================== */
static LoraFrameCallback_t g_frame_cb = NULL;

/* 阻塞模式临时状态变量(radio 原有的 LoraApp_* API 需要) */
static bool     g_txDoneFlag = false;
static bool     g_txTimeoutFlag = false;   /* 发射超时(芯片放弃发射), 不能当成"发完了" */
static bool     g_rxDoneFlag = false;
static uint8_t  g_rxTempBuf[LORA_FRAME_RAW_MAX];
static uint8_t  g_rxTempLen;
static int8_t   g_rxTempRssi;
static int8_t   g_rxTempSnr;

/* ---------- Radio 底层回调 ---------- */
static void RadioTxDoneCb(void)
{
    g_txDoneFlag = true;

    /* TX 完成后 radio 会自动切回 Standby, 需重启 RX 窗口让设备能收到下行 ACK/新上行 */
    /* 注意: 不能在中断里直接调 Radio.Rx (可能阻塞/耗时), 置 flag 让 poll 处理 */
    g_need_restart_rx = 1U;
}

static void RadioRxDoneCb(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    g_rxDoneFlag = true;
    g_stat_rxdone_cb++;     /* 【#lora】只加计数: 帧确实从芯片递到协议层了 */

    /* 中断上下文: 快拷贝 + 置位, 不做解析/CRC */
    uint16_t copy_len = (size > LORA_FRAME_RAW_MAX) ? LORA_FRAME_RAW_MAX : size;
    memcpy(g_rx_buf, payload, copy_len);
    g_rx_len  = (uint8_t)copy_len;
    g_rx_rssi = (int8_t)rssi;
    g_rx_snr  = snr;
    g_rx_ready = 1U;   /* 告诉主循环有新帧待解析 */

    log_info("[LoRa RX] size=%d, rssi=%d, snr=%d, data: ", size, (int)rssi, (int)snr);
    for (uint16_t i = 0; i < copy_len && i < 64; i++) {
        log_info("%02X ", payload[i]);
    }
    log_info("\r\n");

    /* 给老的阻塞 API 也填一份(兼容) */
    memcpy(g_rxTempBuf, payload, copy_len);
    g_rxTempLen   = (uint8_t)copy_len;
    g_rxTempRssi  = (int8_t)rssi;
    g_rxTempSnr   = snr;
}

/* 发射超时: 芯片把这次发射放弃了(PA ramp 失败 / PLL 未锁定 / 供电塌陷...)。
 * 【绝不能置 g_txDoneFlag】—— 否则 "等 TX_DONE" 会把"根本没发出去"误判成"发成功",
 * 日志上就会永远看不到真正的原因。
 *
 * ★这里必须补"回到接收"★: 芯片放弃发射后会落到 Fallback 模式(radio.c 里置
 * MODE_STDBY_RC), 也就是【不再接收】。TxDone 那条路会置 g_need_restart_rx,
 * 这条路原来没有 —— 少这一句, 一次发射超时就能让网关"此后一直收不到"
 * (只有下一次成功发射才会把它带回 RX)。这就是"LoRa 没有一直处于接收状态"
 * 的一个真实来源。 */
static void RadioTxTimeoutCb(void)
{
    g_txTimeoutFlag   = true;
    g_need_restart_rx = 1U;
}

/* 接收窗口超时: 说明芯片已经离开 RX, 同样必须重新开连续接收。
 * 注意 g_rxDoneFlag 只给老的阻塞 API(LoraApp_ReceiveBlock)用, poll 看的是
 * g_rx_ready —— 所以这里【绝不能】置 g_rx_ready, 否则会把"超时"当成"收到一帧"
 * 去解析缓冲区里的旧数据。 */
static void RadioRxTimeoutCb(void)
{
    g_rxDoneFlag      = true;
    g_need_restart_rx = 1U;
}

/* CRC 错 / 接收异常 —— ★★2026-10-10 修: 这里【不再】请求重启接收★★
 *
 * 【原来错在哪】老代码无条件 `g_need_restart_rx = 1U;`, 注释还给了一个错误的代价
 *   估计: "这里是双保险, 代价只有一条 SPI 命令"。
 *   实际上代价【不是一条 SPI 命令】—— 是 LoraApp_EnsureRxContinuous() -> RadioRx(0)
 *   -> LLCC68SetRx() 的【一次 STDBY->RX 模式切换】, 切换期间射频是聋的。
 *
 * 【为什么本来就不需要补】已核对 radio.c 的 CRC 错分支(IRQ_CRC_ERR, 约 1683 行):
 *   那条路【一个模式都不改】, 芯片留在连续接收。RxContinuous==true 时的 SetRx()
 *   就是 0xFFFFFF(永不超时), 所以一个 CRC 错的包之后芯片【立刻回到监听】,
 *   根本不需要任何人重设。
 *   ⇒ 原来的"双保险"是【纯亏】: 每来一个噪声包就白聋一次。
 *
 * 【现场证据】2026-10-10 日志: [STAT2] 的 `补开` 在 4.5 分钟内涨了 6 次, 而当时
 *   `在线从机=0/0`、`下发缓存=0` —— 【网关根本没发射, 无从机】⇒ 不可能是
 *   TxDone/TxTimeout; 只能是 RxError(空口噪声导致 CRC 错)或 RxTimeout。
 *   而 78.txt 里确实有 7 字节垃圾帧(FE F0 FE F8 7E 3F FF)在刷。
 *   ⇒ 链条: 噪声 -> CRC 错 -> 本回调 -> 补开 -> 真空期 -> 丢真帧
 *     ⇒ 配对成功率 2/20、上报丢帧。
 *
 * 【工程自己的验收标准】(LoRa调试问题与解决复盘.md §C1):
 *   "`补开` 这个数应该长期停在 0 或极小; 它明显增长 = 确实漏过接收。"
 *
 * 【去掉之后万一真掉出 RX 怎么办】有兜底, 不影响安全:
 *   lora_proto.c 的 poll 里有【守护检查: 芯片不在接收(RX)模式】, 那条会把它拉回来
 *   (见同文件下面的 LoraApp_EnsureRxContinuous("守护检查: ..."))。
 *   另外 TxDone / TxTimeout / RxTimeout 三条【仍然】会请求重启 —— 那三条才是
 *   真的会离开 RX 的场景, 保持不动。
 *
 * ⚠️ 如果以后把产品改成【非连续接收】(RxContinuous=false), 这条要改回置位:
 *   那种模式下 CRC 错后确实需要重开窗口。当前产品是全程连续接收(RadioRx(0))。 */
static void RadioRxErrorCb(void)
{
    /* 有意留空: 连续接收下芯片还在监听, 重设只会制造一段聋的真空期。 */
}

static RadioEvents_t g_radioEvents =
{
    .TxDone            = RadioTxDoneCb,
    .RxDone            = RadioRxDoneCb,
    .TxTimeout         = RadioTxTimeoutCb,
    .RxTimeout         = RadioRxTimeoutCb,
    .RxError           = RadioRxErrorCb,
    .CadDone           = NULL,
    .FhssChangeChannel = NULL,
    .GnssDone          = NULL,
    .WifiDone          = NULL
};

/* ==================== Radio 底层 API (与原版 lora_proto.c 一致, 不改) ==================== */

uint8_t LoraApp_Init(void)
{
    Radio.Init(&g_radioEvents);
    Radio.SetModem(MODEM_LORA);
    Radio.SetTxConfig(
        MODEM_LORA,
        LORA_TX_OUTPUT_POWER,
        0U,
        LORA_BANDWIDTH,
        LORA_SPREADING_FACTOR,
        LORA_CODINGRATE,
        LORA_PREAMBLE_LENGTH,
        LORA_FIX_LENGTH_PAYLOAD_ON,
        true,
        false,
        0U,
        LORA_IQ_INVERSION_ON,
        LORA_RX_TIMEOUT_VALUE
    );
    Radio.SetRxConfig(
        MODEM_LORA,
        LORA_BANDWIDTH,
        LORA_SPREADING_FACTOR,
        LORA_CODINGRATE,
        0U,
        LORA_PREAMBLE_LENGTH,
        LORA_LLCC68_SYMBOL_TIMEOUT,
        LORA_FIX_LENGTH_PAYLOAD_ON,
        0U,
        true,
        false,
        0U,
        LORA_IQ_INVERSION_ON,
        true
    );
    Radio.SetChannel(LORA_FRE);
    return 0U;
}

void LoraApp_SetRxCallback(LoraRxCallback_t cb)
{
    (void)cb;   /* 已改成通过 LoraFrameCallback_t 投递, 此函数保留兼容 */
}

uint8_t LoraApp_SendAsync(uint8_t *buf, uint8_t len, uint32_t msTimeout)
{
    (void)msTimeout;
    if ((buf == NULL) || (len == 0U) || (len > 255U))
    {
        return 1U;
    }

    /* 【模式切换等待】发射前留空档:
     * 刚收完一包时本机接收链路还在上电, 立刻切 TX 前导码容易发歪;
     * 对方也正好利用这段时间从 TX 切回 RX, 两边错开就不会互相打断。
     *
     * 用本工程 app/bsp 的 delay_ms()(ATIM1 1ms 时基, 和主状态机的 get_tick_ms 同源)。
     * 原先用 LLCC68DelayMs()(SysTick 轮询 COUNTFLAG + 依赖 SystemCoreClock),
     * 实测出现过"该等却没等"的情况, 换成已验证可靠的 delay_ms。 */
    delay_ms(LORA_TX_SETTLE_MS);

    /* 清掉上一包的 TxDone/TxTimeout 标志, 否则调用方 LoraApp_WaitTxDone()
     * 会立刻返回"已完成"(上一轮的残旗) */
    g_txDoneFlag    = false;
    g_txTimeoutFlag = false;

    Radio.Standby();
    Radio.Send(buf, len);
    return 0U;
}

void LoraApp_StartRxContinuous(void)
{
    Radio.Rx(0U);
}

/* ==================== "网关一直在接收"的保障 ====================
 *
 * 【常态是什么】LoraApp_Init() 里 Radio.SetRxConfig(..., rxContinuous = true)
 * 已经把 radio.c 的 RxContinuous 置真, 所以 Radio.Rx(0) 走的是
 * LLCC68SetRx(0xFFFFFF) —— 真正【无限期】的连续接收。这部分本来就是对的。
 *
 * 【那为什么还需要这一段】因为下面这些路径会让芯片离开 RX, 而且【没人知道】:
 *   1) 每次发射之后: 芯片落回 STDBY(radio.c TxDone 分支置 MODE_STDBY_RC),
 *      必须重新 Radio.Rx(0);
 *   2) 发射超时(TxTimeout): 同样落到 STDBY —— 而这条路径原来【不会】请求重启 RX,
 *      一次发射超时就能让网关此后一直收不到(见 RadioTxTimeoutCb 的注释);
 *   3) 接收窗口超时 / 接收错误: 也可能已经离开 RX;
 *   4) 任何外因(供电抖动、SPI 误码)造成的模式漂移 —— 这时软件缓存
 *      (LLCC68GetOperatingMode)与芯片实际状态会不一致, 光看缓存发现不了。
 *
 * 【三层保障】
 *   L1 每次发射后【无条件】补一次(不依赖中断标志) —— 两条发送路径都补;
 *   L2 TxTimeout / RxTimeout / RxError 中断里置"需要重启"标志, poll 里补;
 *   L3 poll 里每 LORA_RX_SUPERVISE_MS 用【芯片真实状态字节】核对一次
 *      (LLCC68GetStatus().ChipMode == 5 就是 RX), 不在 RX 就补并报出来。
 *
 * 这样"网关一直在接收"就不再是一个假设, 而是一个每秒被验证一次的事实,
 * 并且补开次数会记在 [STAT2] 的 "LoRa补RX" 上 —— 它一旦增长就说明真的漏过。
 */
#define LORA_RX_SUPERVISE_MS   1000U
#define LORA_CHIP_MODE_RX      5U      /* LLCC68 状态字节 bits6:4: 5 = RX */

static uint32_t g_rx_rearm_cnt;        /* 累计"发现不在接收 -> 补开"的次数 */
static uint32_t g_rx_supervise_tick;

/**
 * \brief 立刻切回连续接收(幂等)
 * \param why 非 NULL 时说明这是"修复", 会打日志并计数; 正常补位传 NULL
 */
void LoraApp_EnsureRxContinuous(const char *why)
{
    g_need_restart_rx = 0U;     /* 我们自己这就去开, 清掉挂起的请求 */
    Radio.Rx(0U);               /* timeout=0 + RxContinuous=true -> 无限期接收 */
    g_rx_supervise_tick = get_tick_ms();   /* 刚切过来: 让守护检查从这一刻重新计时,
                                            * 免得紧接着又读一次状态误判成"没在收" */

    if (why != NULL)
    {
        if (g_rx_rearm_cnt < 0xFFFFFFFFu) { g_rx_rearm_cnt++; }
        log_info("[LoRa] !! %s —— 已重新切回【连续接收】(累计补开 %lu 次)\r\n",
                 why, (unsigned long)g_rx_rearm_cnt);
    }
}

/** \brief 芯片现在真的在接收吗(读芯片状态字节, 不是读软件缓存) */
bool LoraApp_IsRxActive(void)
{
    return (LLCC68GetStatus().Fields.ChipMode == LORA_CHIP_MODE_RX);
}

/** \brief 累计"发现不在接收并补开"的次数(诊断用; 只增不减) */
uint32_t LoraApp_RxRearmCount(void)
{
    return g_rx_rearm_cnt;
}

void LoraApp_StopRx(void)
{
    Radio.Standby();
}

void LoraApp_ProcessIrq(void)
{
    Radio.IrqProcess();
}

/**
 * \brief 阻塞等这一包真的发完(TX_DONE)
 * \param timeout_ms 超时上限
 * \return true = 收到 TX_DONE(空口上确实发完了); false = 超时(芯片没发出去)
 *
 * 【为什么必须有这个函数】
 *   Radio.Send() 只是把 SET_TX 命令写进芯片就返回了, 发射是芯片自己异步做的
 *   (13 字节 @SF8/BW500 空中约 20ms)。如果调用方在 Send() 之后紧接着调
 *   Radio.Standby() / Radio.Rx(), 芯片会【当场中止这次发射】——
 *   现象就是串口打了 "submitted ok", 空口上却什么都没发出去。
 *   所以"发完"必须用 TX_DONE 作判据, 不能靠"Send() 返回了"来推断。
 *
 *   时间基准用 bsp_tim 的 get_tick_ms()/delay_ms()(ATIM1 1ms 时基,
 *   和主状态机同源), 不用 LLCC68DelayMs(): 后者依赖 SysTick/SystemCoreClock,
 *   本工程实测过"该等却没等"的问题。
 */
bool LoraApp_WaitTxDone(uint32_t timeout_ms)
{
    uint32_t t0 = get_tick_ms();

    while ((uint32_t)(get_tick_ms() - t0) < timeout_ms)
    {
        Radio.IrqProcess();

        /* 芯片自己放弃了这次发射 -> 立刻返回失败, 不要等到超时 */
        if (g_txTimeoutFlag)
        {
            g_txTimeoutFlag = false;
            g_txDoneFlag    = false;
            return false;
        }

        if (g_txDoneFlag)
        {
            g_txDoneFlag = false;
            return true;
        }
        delay_ms(1U);
    }

    /* 超时也别忘了问一次, 边界上可能刚好完成 */
    Radio.IrqProcess();
    if (g_txDoneFlag)
    {
        g_txDoneFlag = false;
        return true;
    }
    g_txTimeoutFlag = false;
    return false;
}

/* 阻塞测试接口, 仅调试用 */
bool LoraApp_SendBlock(uint8_t *buf, uint8_t len, uint32_t waitMs)
{
    if (LoraApp_SendAsync(buf, len, waitMs) != 0U)
    {
        return false;
    }
    return LoraApp_WaitTxDone(waitMs);
}

bool LoraApp_ReceiveBlock(uint8_t *rxBuf, uint8_t maxLen, uint8_t *outLen,
                          uint32_t waitMs, int8_t *rssiOut, int8_t *snrOut)
{
    if ((rxBuf == NULL) || (outLen == NULL) || (rssiOut == NULL) || (snrOut == NULL))
    {
        if (outLen != NULL) *outLen = 0U;
        return false;
    }
    g_rxDoneFlag = false;
    *outLen = 0U;
    Radio.Rx(waitMs);
    uint32_t tick = 0U;
    while (g_rxDoneFlag == false)
    {
        Radio.IrqProcess();
        LLCC68DelayMs(1U);
        tick++;
        if (tick >= waitMs) return false;
    }
    if (g_rxTempLen > maxLen)
    {
        *outLen = 0U;
        return false;
    }
    memcpy(rxBuf, g_rxTempBuf, g_rxTempLen);
    *outLen   = g_rxTempLen;
    *rssiOut  = g_rxTempRssi;
    *snrOut   = g_rxTempSnr;
    return true;
}

/* ==================== 协议层核心 ==================== */

/* 帧总长计算: 帧头(2) + ver(1) + gw(3) + dev(3) + func(1) + data_len(1) + data(N) + crc(2) */
#define LORA_FRAME_LEN(data_len)  (LORA_FRAME_OVERHEAD + (data_len))

/**
 * \brief 解析一帧原始字节流(内部共享实现, 被所有 parse 调用)
 *
 * 长度/CRC 都是严格的: raw_len 必须恰好等于 14 + data_len, CRC16 必须对上。
 * 不够长/超长/CRC 错一律丢弃。
 *
 * 历史坑(2026-xx 定位): 曾出现"网关只收到 20 字节、dlen=7"的现象, 一度以为是
 * 空口丢了最后一个字节并加了长度容错。实际根因在设备端 app_sensor.c:
 *   SENSOR_REPORT_FRAME_MAX_LEN 被写成 20(2+1+3+3+1+1+7+2 实际=21),
 *   frame_buf 只有 20 字节却要写 21 字节, CRC 高字节越界写并踩坏紧邻的局部变量,
 *   导致 *out_len 变成 20、旧固件还打印 "tx report len=20"。
 *   设备端把长度改成 21 后, 这里恢复严格校验即可。
 */
static bool parse_frame_impl(const uint8_t *raw, uint16_t raw_len, lora_frame_t *out)
{
    uint16_t expected;

    if ((raw == NULL) || (out == NULL)) return false;

    /* 1. 最小长度检查 */
    if (raw_len < LORA_FRAME_HDR_SIZE) return false;
    /* 2. 帧头检查 */
    if ((raw[0] != LORA_FRAME_HEAD0) || (raw[1] != LORA_FRAME_HEAD1)) return false;
    /* 3. 协议版本检查 */
    if (raw[2] != LORA_PROTO_VER) return false;

    /* 4. data_len 合法性 */
    {
        /* ★2026-10-18: 偏移由宏算(原写死 raw[10]) —— UID 3->13 后字段整体后移 */
        uint8_t data_len = raw[LORA_OFF_DLEN];
        if (data_len > LORA_FRAME_DATA_MAX) return false;
        expected      = LORA_FRAME_LEN(data_len);
        out->data_len = data_len;
    }

    /* 5. 完整帧总长必须匹配 */
    if (raw_len != expected) return false;

    /* 6. CRC16 校验: 覆盖从 ver 到 data 末尾 (跳过帧头 2 + CRC 2) */
    {
        uint16_t crc_calc = crc16_modbus(&raw[2], expected - 4U);
        uint16_t crc_recv = (uint16_t)raw[expected - 2U] | ((uint16_t)raw[expected - 1U] << 8U);
        if (crc_calc != crc_recv) return false;
    }

    /* 7. 字段拆包 ★2026-10-18: gw/dev 各 13 字节, 用宏偏移 + 循环 ★ */
    {
        uint8_t i;
        for (i = 0u; i < LORA_UID_BYTES; i++)
        {
            out->gw_uid[i]  = raw[LORA_OFF_GW_UID  + i];
            out->dev_uid[i] = raw[LORA_OFF_DEV_UID + i];
        }
    }
    out->func       = raw[LORA_OFF_FUNC];
    if (out->data_len > 0U)
    {
        memcpy(out->data, &raw[LORA_OFF_DATA], out->data_len);
    }
    memcpy(out->raw, raw, expected);
    out->raw_len = (uint8_t)expected;

    return true;
}

/* ---------- 对外: 直接字节流版(测试/已知完整帧场景) ---------- */
bool lora_proto_parse_raw(const uint8_t *raw, uint8_t raw_len, lora_frame_t *out)
{
    return parse_frame_impl(raw, raw_len, out);
}

/* ---------- 对外: 组帧 ---------- */
bool lora_proto_build_frame(lora_frame_t *frame, uint8_t *raw_out)
{
    if (frame == NULL) return false;
    if (frame->data_len > LORA_FRAME_DATA_MAX) return false;

    uint8_t *r = frame->raw;

    r[0]  = LORA_FRAME_HEAD0;
    r[1]  = LORA_FRAME_HEAD1;
    r[2]  = LORA_PROTO_VER;
    /* ★2026-10-18: gw/dev 各 13 字节, 用宏偏移 + 循环(原写死 r[3..8]) */
    {
        uint8_t i;
        for (i = 0u; i < LORA_UID_BYTES; i++)
        {
            r[LORA_OFF_GW_UID  + i] = frame->gw_uid[i];
            r[LORA_OFF_DEV_UID + i] = frame->dev_uid[i];
        }
    }
    r[LORA_OFF_FUNC] = frame->func;
    r[LORA_OFF_DLEN] = frame->data_len;

    if (frame->data_len > 0U)
    {
        memcpy(&r[LORA_OFF_DATA], frame->data, frame->data_len);
    }

    uint8_t raw_len = (uint8_t)LORA_FRAME_LEN(frame->data_len);

    /* CRC16: 覆盖 &r[2] 起 (raw_len - 4) 字节, 结果低字节在前填末尾 */
    uint16_t crc = crc16_modbus(&r[2], raw_len - 4U);
    r[raw_len - 2U] = (uint8_t)(crc & 0x00FFU);
    r[raw_len - 1U] = (uint8_t)((crc >> 8U) & 0x00FFU);

    frame->raw_len = raw_len;

    if (raw_out != NULL)
    {
        memcpy(raw_out, r, raw_len);
    }
    return true;
}

/* ---------- 对外: 发送 ----------
 *
 * 【这里是网关所有下发的唯一出口】, 所以"到底有没有发出去"必须在这里自证:
 *   1) 先把要发的整帧字节打出来(和设备端 [rx] 日志逐字节对照);
 *   2) 发完等 TX_DONE, 把实测耗时和芯片模式打出来;
 *   3) 等不到就明确报"没发出去", 而不是静默返回 false。
 *
 * 为什么要等 TX_DONE: LoraApp_SendAsync() 只是把 SET_TX 命令写进芯片就返回了,
 * 发射是芯片异步做的(13 字节 @SF8/BW500 空中约 20ms)。不等就切模式的那一方
 * (Standby/SetRx) 会当场把这一包掐断在 PA ramp 阶段 —— 日志上"已发出"照打,
 * 空口上却一个字节都没出去。 */
bool lora_proto_send_frame(lora_frame_t *frame)
{
    uint32_t t0;
    uint16_t i;

    if (!lora_proto_build_frame(frame, NULL))
    {
        log_info("[TX] !! build FAILED (dlen=%u)\r\n",
                 (unsigned)((frame != NULL) ? frame->data_len : 0U));
        return false;
    }

    /* 打整帧: 有了这行, "发没发出去"和"发的是什么"都不需要猜 */
    log_info("[TX] func=0x%02X dev=%02X%02X%02X dlen=%u len=%u tick=%lu:",
             (unsigned)frame->func,
             frame->dev_uid[0], frame->dev_uid[1], frame->dev_uid[2],
             (unsigned)frame->data_len, (unsigned)frame->raw_len,
             (unsigned long)get_tick_ms());
    for (i = 0U; i < frame->raw_len; i++)
    {
        log_info(" %02X", frame->raw[i]);
    }
    log_info("\r\n");

    t0 = get_tick_ms();

    if (LoraApp_SendAsync(frame->raw, frame->raw_len, 0U) != 0U)
    {
        log_info("[TX] !! LoraApp_SendAsync 拒绝发送(len=%u)\r\n",
                 (unsigned)frame->raw_len);
        return false;
    }

    /* 【必须等】理由见上面函数头注释 */
    if (LoraApp_WaitTxDone(LORA_TX_DONE_TIMEOUT_MS))
    {
        log_info("[TX] 发送完成 TX_DONE (耗时=%lums, mode=%u) mode:2=STDBY 6=TX\r\n",
                 (unsigned long)(get_tick_ms() - t0),
                 (unsigned)LLCC68GetOperatingMode());
        /* 【发完立刻回到接收】: 不依赖 TxDone 中断里的标志 ——
         * 漏一次就是"此后一直收不到", 而补一次只是一条 SPI 命令。 */
        LoraApp_EnsureRxContinuous(NULL);
        return true;
    }

    /* 走到这里说明芯片压根没发射完 —— 查供电(PA ramp)/天线/射频配置。
     * 再读一次 IRQ 状态做判据:
     *   irq bit0(TX_DONE)=1 却等了超时 -> DIO1/EXTI 中断通路没进来(硬件/引脚问题)
     *   irq=0x0000                      -> 芯片自己中止了这次发射(PA ramp/PLL/供电) */
    log_info("[TX] !! 等 TX_DONE 超时 %ums (mode=%u, irq=0x%04X) 芯片没发出去\r\n",
             (unsigned)LORA_TX_DONE_TIMEOUT_MS,
             (unsigned)LLCC68GetOperatingMode(),
             (unsigned)LLCC68GetIrqStatus());
    /* 发失败【更要】补: 芯片这时停在 STDBY, 不补就真的不在接收了 */
    LoraApp_EnsureRxContinuous(NULL);
    return false;
}

/* ---------- 对外: 原样透传发送(不组帧) ----------
 *
 * 【用途】"云端下发什么, 空口就发什么"的最简版本:
 *   下行字节一个不改地发出去, 网关【不做帧头/版本/长度/CRC 任何校验】, 也不解释
 *   func、不查白名单、不补 0x01 那 6 字节校时。对端收到什么就是什么。
 *
 * 【为什么不能复用 lora_proto_send_frame()】它会重新组帧(加 AA 55 + 版本 + 重算
 *   CRC), 那样发出去的就不是云端原来那串字节了 —— 透传场景下必须逐字节一致。
 *
 * 【仍然必须等 TX_DONE】原因和 lora_proto_send_frame() 完全一样:
 *   LoraApp_SendAsync() 只是把 SET_TX 命令写进芯片就返回, 发射由芯片异步完成。
 *   不等就切模式(Standby/Rx)会把这一包当场掐断 —— 日志打"已发出", 空口是空的。 */
bool lora_proto_send_raw(const uint8_t *data, uint8_t len)
{
    uint32_t t0;
    uint16_t i;

    if ((data == NULL) || (len == 0U))
    {
        log_info("[TX-RAW] !! 空载荷, 不发\r\n");
        return false;
    }

    /* 发之前先把要发的字节完整打出来: "到底发了什么"不用猜 */
    log_info("[TX-RAW] 透传 %u 字节 tick=%lu:",
             (unsigned)len, (unsigned long)get_tick_ms());
    for (i = 0U; i < len; i++)
    {
        log_info(" %02X", (unsigned)data[i]);
    }
    log_info("\r\n");

    t0 = get_tick_ms();

    /* LoraApp_SendAsync 只读这段缓冲, 这里的去 const 是安全的 */
    if (LoraApp_SendAsync((uint8_t *)data, len, 0U) != 0U)
    {
        log_info("[TX-RAW] !! LoraApp_SendAsync 拒绝发送(len=%u)\r\n", (unsigned)len);
        return false;
    }

    if (LoraApp_WaitTxDone(LORA_TX_DONE_TIMEOUT_MS))
    {
        log_info("[TX-RAW] 发送完成 TX_DONE (耗时=%lums, mode=%u)\r\n",
                 (unsigned long)(get_tick_ms() - t0),
                 (unsigned)LLCC68GetOperatingMode());
        /* 透传同样: 发完立刻回到接收(见 lora_proto_send_frame 的同款说明) */
        LoraApp_EnsureRxContinuous(NULL);
        return true;
    }

    /* 没等到 TX_DONE = 芯片压根没发射完: 查供电(PA ramp)/天线/BUSY/中断通路 */
    log_info("[TX-RAW] !! 等 TX_DONE 超时 %ums (mode=%u, irq=0x%04X) 芯片没发出去\r\n",
             (unsigned)LORA_TX_DONE_TIMEOUT_MS,
             (unsigned)LLCC68GetOperatingMode(),
             (unsigned)LLCC68GetIrqStatus());
    LoraApp_EnsureRxContinuous(NULL);
    return false;
}

/* ---------- 对外: init + poll + 连发辅助 ---------- */

/**
 * \brief 撤销"TX 完成后重进接收"的挂起请求
 * \note  连发多帧(且要求间隔期不切回接收)时用: TxDone 中断会置
 *        g_need_restart_rx, lora_proto_poll() 会据此把芯片切回 RX。
 *        连发期间每发完一帧调一次本函数把它清掉, 芯片就留在发送侧。
 */
void lora_proto_cancel_rx_restart(void)
{
    g_need_restart_rx = 0U;
}

void lora_proto_init(LoraFrameCallback_t cb)
{
    g_frame_cb  = cb;
    g_rx_ready  = 0U;
    g_need_restart_rx = 0U;

    /* 注册探针: 确认回调真的登记进来了(cb 不能是 NULL, 否则收到帧也只是静默丢弃) */
    log_info("[proto] init: frame_cb=%p\r\n", (void *)g_frame_cb);

    /* 初始化 Radio 底层 (只做一次) */
    LoraApp_Init(); 

    /* 立即开启持续 RX —— 网关常态应一直在监听设备上报 */
    LoraApp_StartRxContinuous();    
}

void lora_proto_poll(void)
{
    /* 1. 处理 radio 中断(把 RxDone/TxDone 事件转为标志) */
    LoraApp_ProcessIrq();

    /* 2. 有"需要重启接收"的挂起请求就补上。
     *    来源: TX 完成 / 发射超时 / 接收窗口超时 / 接收错误 —— 这些都会让芯片离开 RX。
     *    (TxDoneCb 等中断里只置 flag, 不在中断里直接调 Radio.Rx) */
    if (g_need_restart_rx != 0U)
    {
        LoraApp_EnsureRxContinuous("收到【需要重启接收】的请求(TX完成/发射超时/接收异常)");
    }

    /* 2.5 【接收守护】: 定期用【芯片真实状态字节】核对"到底还在不在接收"。
     *     软件缓存 LLCC68GetOperatingMode() 可能与芯片实际状态不一致 ——
     *     发射超时、供电抖动、SPI 误码都可能让它悄悄离开 RX 而【没有任何人知道】。
     *     一条 SPI 命令很便宜, 1 秒查一次就够; 换来的是"网关一直在收"这句话
     *     每秒被验证一次, 而不是一个假设。补开次数记在 [STAT2] 的 "LoRa补RX"。 */
    {
        uint32_t now = get_tick_ms();

        if ((uint32_t)(now - g_rx_supervise_tick) >= LORA_RX_SUPERVISE_MS)
        {
            g_rx_supervise_tick = now;

            if (!LoraApp_IsRxActive())
            {
                LoraApp_EnsureRxContinuous("守护检查: 芯片不在接收(RX)模式");
            }
        }
    }

    /* 3. 待解析帧就绪就尝试解析 + 分发
     *    先清标志(单缓冲, 即使刚清完又有新帧到也只是覆盖, LoRa 帧率极低可接受) */
    if (g_rx_ready != 0U)
    {
        lora_frame_t frame;
        uint8_t  len = g_rx_len;

        g_rx_ready = 0U;

        if (parse_frame_impl(g_rx_buf, len, &frame))
        {
            if (g_frame_cb != NULL)
            {
                g_frame_cb(&frame, g_rx_rssi, g_rx_snr);
            }
            else
            {
                /* 只有真出问题时才打, 不占正常流程的日志 */
                log_info("[proto] !! frame_cb is NULL, frame dropped\r\n");
            }
        }
        else
        {
            g_stat_parse_fail++;    /* 【#lora】帧到了但没解析过(帧头/版本/长度/CRC) */
            /* 解析失败诊断: 只在失败时打一次, 带关键字段便于定位是长度还是 CRC 不过 */
            log_info("[proto] !! PARSE FAILED len=%u hdr=%02X%02X ver=%02X dlen=%u\r\n",
                     (unsigned)len,
                     (unsigned)g_rx_buf[0], (unsigned)g_rx_buf[1],
                     (unsigned)((len > 2U) ? g_rx_buf[2] : 0U),
                     (unsigned)((len > LORA_OFF_DLEN) ? g_rx_buf[LORA_OFF_DLEN] : 0U));
        }
    }
}

/* ---------- #lora 诊断用的小工具(不改变任何射频行为) ---------- */

/* 芯片 GetStatus 的 ChipMode 字段含义(见 DS_SX1261-2: GetStatus) */
#define LORA_DIAG_CHIPMODE_RX        5u   /* 5 = RX */

static const char *lora_diag_chipmode_text(uint8_t mode)
{
    switch (mode)
    {
        case 2u: return "STDBY_RC(待机)";
        case 3u: return "STDBY_XOSC(待机)";
        case 4u: return "FS(频率合成)";
        case 5u: return "RX(接收中)";
        case 6u: return "TX(发射中)";
        default: return "未知/保留";
    }
}

/* LoRa 带宽枚举 -> kHz(只用于打印) */
static uint32_t lora_diag_bw_khz(RadioLoRaBandwidths_t bw)
{
    switch (bw)
    {
        case LORA_BW_500: return 500u;
        case LORA_BW_250: return 250u;
        case LORA_BW_125: return 125u;
        case LORA_BW_062: return 62u;
        case LORA_BW_041: return 41u;
        case LORA_BW_031: return 31u;
        case LORA_BW_020: return 20u;
        case LORA_BW_015: return 15u;
        case LORA_BW_010: return 10u;
        case LORA_BW_007: return 7u;
        default:          return 0u;
    }
}

/* ============================================================================
 *                    射频诊断命令 #lora (2026-09 现场排障用)
 * ============================================================================
 *
 * 【为什么需要它 —— 这次现场故障的本质】
 *
 *   网关串口自诊断说"[STAT2] ... LoRa RX=在收", 但从机表 在线从机=0/2, 而且
 *   [LoRa RX] 和 [proto] !! PARSE FAILED 这两条【一条都不打印】。
 *   这两条各覆盖了"解析通过"和"解析失败"两种情形 —— 两条都没有, 只能说明
 *   一件事: 【射频层的 RxDone 中断从来没有触发过】。
 *   那就不是帧格式问题(帧格式再错, 芯片也会先报 Preamble/SyncWord/RxDone),
 *   而是频率/带宽/扩频因子/同步字/包类型对不上, 或者射频压根没真正进入 RX,
 *   再或者 BUSY 卡高导致配置根本没写进芯片、又或者天线/前端。
 *
 *   所以这条命令的目标只有一个: 把【芯片里实际是什么配置】读出来, 而不是读代码里
 *   "打算配成什么"。这两者不一致, 正是这类"两边都不报错的哑失败"的典型成因。
 *
 * 【纪律: 绝对只读】
 *   · 只发 Get 类命令(GetStatus / GetIrqStatus / GetStats / GetErrors / GetRssiInst)
 *     和 ReadRegister / ReadCommand; 【绝不发】Set / Calibrate / SetRx / SetStandby /
 *     ResetStats 之类会改状态、改模式、清计数的命令 —— 那会把唯一的证据毁掉。
 *   · 因此本命令【不会打断正在进行的接收】: 芯片一直留在 RX, 只是被多读了几次状态。
 *     接收期间读 GetRssiInst/GetStatus 是芯片允许的用法。
 *     (唯一的理论例外: 读命令前面的 LLCC68CheckDeviceReady() 只在驱动认为芯片处于
 *      SLEEP/RX_DC 时才会发 Wakeup —— 本工程常态是 RX, 不会走到那条路。)
 *   · 缓冲一律 static —— 本工程栈只有 4KB(startup_n32h47x_48x.s 的 Stack_Size)。
 *
 * 【能回读的 / 只能报固件写入值的 / 根本拿不到的】
 *   能回读(真·从芯片读出来):
 *     频率(寄存器 0x0880)、LoRa 同步字(0x0740)、payload 长度寄存器(0x0702)、
 *     芯片版本串(0x0320)、接收增益(0x08AC)、TxClamp(0x08D8)、OCP(0x08E7)、
 *     GetStatus(芯片模式)、GetIrqStatus、GetStats(收包/CRC/长度错统计)、
 *     GetErrors(PLL/XOSC/PA ramp 等故障标志)、GetRssiInst(瞬时 RSSI)、BUSY 引脚电平。
 *   只能报【固件写入值】(LLCC68 用命令写入, 没有对应回读命令):
 *     SF、带宽、编码率 CR、LDRO、前导码长度、显式/隐式报头、CRC 开关、IQ 反转、
 *     包类型(驱动缓存)、发射功率、RxContinuous 标志。
 *   根本拿不到:
 *     "芯片内部当前生效的 SF/BW/CR"没有任何寄存器可读; 驱动也没有提供任何 getter
 *     (llcc68.c 里只有 SetModulationParams/SetPacketParams)。这一类就是拿不到,
 *     不编造 —— 改用"回读频率/同步字是否与固件写入值一致"+ "GetStats 收包计数"
 *     来交叉验证配置到底有没有生效。
 */
void LoraApp_DiagPrint(void)
{
    static uint8_t        rb[8];      /* 寄存器回读缓冲(static: 不放栈上) */
    static LLCC68Stats_t  st;         /* GetStats 结果(static) */
    static uint8_t        ver[6];     /* 0x0320 版本串(static) */
    RadioStatus_t         status;
    RadioError_t          err;
    const RadioIrqStat_t *irq;
    const ModulationParams_t *mp;
    const PacketParams_t     *pp;
    uint32_t              pllSteps;
    uint32_t              freqHz;
    uint16_t              syncWord;
    uint16_t              irqNow;
    uint8_t               chipMode;
    uint8_t               payloadReg;
    uint8_t               rxGainReg;
    uint8_t               txClampReg;
    uint8_t               ocpReg;
    uint32_t              busyTo;
    bool                  busyStuck;
    bool                  freqMismatch;
    bool                  rxContinuous;
    uint8_t               warn = 0u;
    int8_t                rssiNow;

    log_info("\r\n[LORA] ===== 射频诊断 #lora (只读: 只发 Get 类命令 + 读寄存器, 不改任何射频配置) =====\r\n");
    log_info("[LORA] 标注约定: 【回读】= 从芯片真读出来的; 【固件写入值】= 当初写下去的数(芯片没有回读命令)。\r\n");
    log_info("[LORA] 两者不一致 = 配置没写进去/被改写 —— 这正是本次要查的东西。\r\n");

    /* ---------------- 1. 芯片型号 + BUSY ---------------- */
    LLCC68ReadRegisters(0x0320, ver, 6);
    log_info("[LORA] 芯片版本串【回读 0x0320】= %02X %02X %02X %02X %02X %02X (\"SX1261\" = 53 58 31 32 36 31)\r\n",
             (unsigned)ver[0], (unsigned)ver[1], (unsigned)ver[2],
             (unsigned)ver[3], (unsigned)ver[4], (unsigned)ver[5]);
    log_info("[LORA] 驱动判定的型号 = %s (决定 PA 分支与功率上限)\r\n",
             (LLCC68GetDeviceId() == SX1261) ? "SX1261 -> PA 用 deviceSel=1, 功率上限 +14dBm"
                                             : "SX1262/LLCC68 -> PA 用 deviceSel=0, 上限 +22dBm");

    busyStuck = (LLCC68IsStuck() != 0u);
    busyTo    = LLCC68BusyTimeoutCount();
    log_info("[LORA] BUSY(PB3) 当前电平 = %u (%s) | 是否已被判定不应答 = %s | 等 BUSY 超时【累计】= %lu 次\r\n",
             (unsigned)LLCC68BusyLevel(),
             (LLCC68BusyLevel() != 0u) ? "高 = 忙/模组没应答" : "低 = 正常",
             busyStuck ? "是" : "否",
             (unsigned long)busyTo);
    if (busyTo != 0u)
    {
        log_info("[LORA] !! 等 BUSY 超时累计 %lu 次: 意味着有 SPI 命令是在 BUSY 还高的时候发出去的 ——\r\n",
                 (unsigned long)busyTo);
        log_info("[LORA] !!    那一次的写入/读取结果不可信, 芯片里的配置可能不是固件以为的那个。\r\n");
        ++warn;
    }

    /* ---------------- 2. 频率(能回读) ---------------- */
    LLCC68ReadRegisters(0x0880, rb, 4);   /* RegRfFreq, 32bit, 高位在前 */
    pllSteps = ((uint32_t)rb[0] << 24) | ((uint32_t)rb[1] << 16) |
               ((uint32_t)rb[2] << 8)  | (uint32_t)rb[3];
    /* PLL 步进 = 32MHz / 2^25, 即 Hz = step * 32e6 / 2^25 = step * 15625 / 16384(精确等值) */
    freqHz   = (uint32_t)(((uint64_t)pllSteps * 15625ULL) >> 14);
    freqMismatch = (freqHz != (uint32_t)LORA_FRE);

    log_info("[LORA] 频率【回读 0x0880】= %02X %02X %02X %02X (PLL 步进 %lu) -> %lu Hz (%lu.%03lu MHz)\r\n",
             (unsigned)rb[0], (unsigned)rb[1], (unsigned)rb[2], (unsigned)rb[3],
             (unsigned long)pllSteps,
             (unsigned long)freqHz,
             (unsigned long)(freqHz / 1000000u),
             (unsigned long)((freqHz % 1000000u) / 1000u));
    log_info("[LORA] 频率【固件写入值 LORA_FRE】= %lu Hz (%lu.%03lu MHz)  %s\r\n",
             (unsigned long)LORA_FRE,
             (unsigned long)((uint32_t)LORA_FRE / 1000000u),
             (unsigned long)(((uint32_t)LORA_FRE % 1000000u) / 1000u),
             freqMismatch ? "★ 与回读不一致!" : "<= 与回读一致");

    /* ---------------- 3. LoRa 同步字(能回读) ---------------- */
    LLCC68ReadRegisters(REG_LR_SYNCWORD, rb, 2);   /* 0x0740, 2 字节 */
    syncWord = ((uint16_t)rb[0] << 8) | (uint16_t)rb[1];
    log_info("[LORA] LoRa 同步字【回读 0x0740】= 0x%04X %s\r\n",
             (unsigned)syncWord,
             (syncWord == (uint16_t)LORA_MAC_PUBLIC_SYNCWORD)  ? "(公有网络 0x3444)" :
             ((syncWord == (uint16_t)LORA_MAC_PRIVATE_SYNCWORD) ? "(私有网络 0x1424 = 芯片复位默认)" :
                                                                  "(既不是公有也不是私有默认值)"));
    log_info("[LORA]   注: 本固件的 LoRa 收发路径【从未】写过 LoRa 同步字(radio.c 只在 GFSK 分支写 0x06C0,\r\n");
    log_info("[LORA]       LoRa 分支没有调 SetPublicNetwork), 所以它应该是芯片复位默认 0x1424。\r\n");
    log_info("[LORA]       对端(液位传感器)必须用同一个同步字, 差一位就永远收不到。\r\n");

    /* ---------------- 4. 包类型 + 调制/包头参数(无法回读, 只报固件写入值) ---------------- */
    mp = &LLCC68.ModulationParams;
    pp = &LLCC68.PacketParams;

    log_info("[LORA] 包类型【固件写入值】= %s (LLCC68 没有回读包类型的寄存器; GetPacketType 读的是驱动缓存)\r\n",
             (LLCC68GetPacketType() == PACKET_TYPE_LORA) ? "LoRa" :
             ((LLCC68GetPacketType() == PACKET_TYPE_GFSK) ? "GFSK" : "NONE(未设置)"));

    log_info("[LORA] ---- 以下 5 行: SF/BW/CR/LDRO/前导码/报头/CRC 全是【固件写入值】 ----\r\n");
    log_info("[LORA] ---- LLCC68 用 SET_MODULATIONPARAMS / SET_PACKETPARAMS 写入, 芯片【没有回读命令】, ----\r\n");
    log_info("[LORA] ---- 所以这些数只能证明 固件打算配成这样, 证明不了芯片里就是这个。 ----\r\n");
    log_info("[LORA] SF=SF%u BW=%lu kHz CR=4/%u LDRO=%u 前导码=%u 符号\r\n",
             (unsigned)mp->Params.LoRa.SpreadingFactor,
             (unsigned long)lora_diag_bw_khz(mp->Params.LoRa.Bandwidth),
             (unsigned)((unsigned)mp->Params.LoRa.CodingRate + 4u),
             (unsigned)mp->Params.LoRa.LowDatarateOptimize,
             (unsigned)pp->Params.LoRa.PreambleLength);
    log_info("[LORA] 报头=%s payload 长度=%u CRC=%s IQ=%s\r\n",
             (pp->Params.LoRa.HeaderType == LORA_PACKET_FIXED_LENGTH) ? "隐式(implicit, 不带头部)" : "显式(explicit, 带头部)",
             (unsigned)pp->Params.LoRa.PayloadLength,
             (pp->Params.LoRa.CrcMode == LORA_CRC_ON) ? "开" : "关",
             (pp->Params.LoRa.InvertIQ == LORA_IQ_INVERTED) ? "反转" : "正常");

    payloadReg = LLCC68ReadRegister(REG_LR_PAYLOADLENGTH);   /* 0x0702 */
    log_info("[LORA] payload 长度寄存器【回读 0x0702】= %u 字节 (隐式头模式下芯片真正用的就是这个值;\r\n",
             (unsigned)payloadReg);
    log_info("[LORA]       显式头/变长模式下它不决定收包长度, 只作参考)\r\n");

    /* ---------------- 5. 发射功率 + PA 分支证据(能回读) ---------------- */
    txClampReg = LLCC68ReadRegister(0x08D8);   /* RegTxClampConfig */
    ocpReg     = LLCC68ReadRegister(0x08E7);   /* RegOcp */
    log_info("[LORA] 发射功率【固件写入值】= %d dBm (project_config.h: LORA_TX_OUTPUT_POWER;\r\n",
             (int)LORA_TX_OUTPUT_POWER);
    log_info("[LORA]       SET_TXPARAMS 命令本身没有回读, 只能看下面这组 分支证据 是否与型号匹配\r\n");
    log_info("[LORA] PA 分支证据【回读】: 0x08D8(TxClamp)=0x%02X 0x08E7(OCP)=0x%02X\r\n",
             (unsigned)txClampReg, (unsigned)ocpReg);

    /* ---------------- 6. 当前工作模式(能回读) ---------------- */
    status       = LLCC68GetStatus();
    chipMode     = (uint8_t)status.Fields.ChipMode;
    rxContinuous = RadioGetRxContinuous();
    rxGainReg    = LLCC68ReadRegister(REG_RX_GAIN);   /* 0x08AC */

    log_info("[LORA] 当前模式【回读 GetStatus】= 原始字节 0x%02X -> chipMode=%u(%s) cmdStatus=%u\r\n",
             (unsigned)status.Value, (unsigned)chipMode,
             lora_diag_chipmode_text(chipMode), (unsigned)status.Fields.CmdStatus);
    log_info("[LORA] 软件缓存 LLCC68GetOperatingMode()=%u (仅固件自记, 可能与芯片不一致) | RxContinuous【固件写入值】=%s\r\n",
             (unsigned)LLCC68GetOperatingMode(),
             rxContinuous ? "真 -> SetRx(0xFFFFFF) = 无限期连续接收" : "假 -> SetRx(有限超时), 超时后就不在接收了");
    log_info("[LORA] 接收增益寄存器【回读 0x08AC】= 0x%02X (%s)\r\n",
             (unsigned)rxGainReg,
             (rxGainReg == 0x96u) ? "boosted 高增益" :
             ((rxGainReg == 0x94u) ? "普通增益(本工程走 Radio.Rx, 即普通)" : "其它值"));

    rssiNow = LLCC68GetRssiInst();
    log_info("[LORA] 瞬时 RSSI【回读 GetRssiInst】= %d dBm%s\r\n",
             (int)rssiNow,
             (chipMode == LORA_DIAG_CHIPMODE_RX) ? " (RX 模式下有效; 对端发射时它会明显抬起来)"
                                                 : " ★芯片不在 RX, 这个值没有意义");

    /* ---------------- 7. IRQ 实时 + 累计(现场最关键的一条) ---------------- */
    irqNow = LLCC68GetIrqStatus();     /* 只读, 不会清 IRQ */
    log_info("[LORA] IRQ status【回读 GetIrqStatus(本命令不清它)】= 0x%04X\r\n", (unsigned)irqNow);
    log_info("[LORA]   TX_DONE=%u RX_DONE=%u PREAMBLE_DETECTED=%u SYNCWORD_VALID=%u HEADER_VALID=%u\r\n",
             (unsigned)((irqNow & IRQ_TX_DONE) ? 1u : 0u),
             (unsigned)((irqNow & IRQ_RX_DONE) ? 1u : 0u),
             (unsigned)((irqNow & IRQ_PREAMBLE_DETECTED) ? 1u : 0u),
             (unsigned)((irqNow & IRQ_SYNCWORD_VALID) ? 1u : 0u),
             (unsigned)((irqNow & IRQ_HEADER_VALID) ? 1u : 0u));
    log_info("[LORA]   HEADER_ERROR=%u CRC_ERROR=%u CAD_DONE=%u CAD_DETECTED=%u RX_TX_TIMEOUT=%u\r\n",
             (unsigned)((irqNow & IRQ_HEADER_ERROR) ? 1u : 0u),
             (unsigned)((irqNow & IRQ_CRC_ERROR) ? 1u : 0u),
             (unsigned)((irqNow & IRQ_CAD_DONE) ? 1u : 0u),
             (unsigned)((irqNow & IRQ_CAD_ACTIVITY_DETECTED) ? 1u : 0u),
             (unsigned)((irqNow & IRQ_RX_TX_TIMEOUT) ? 1u : 0u));
    log_info("[LORA]   ★ 这一行【几乎必然为 0】: 主循环的 RadioIrqProcess() 每处理一次就把 IRQ status 清一次。\r\n");
    log_info("[LORA]     所以 RxDone 到底有没有触发过, 不能看它, 要看下面这组【累计计数】。\r\n");

    irq = RadioIrqStat();
    log_info("[LORA] IRQ 累计计数(本工程新增的软件计数, 不是芯片回读; 自本次上电起):\r\n");
    log_info("[LORA]   RX_DONE=%lu (其中 CRC 错=%lu) | PREAMBLE_DETECTED=%lu | SYNCWORD_VALID=%lu\r\n",
             (unsigned long)irq->RxDone, (unsigned long)irq->RxDoneCrcErr,
             (unsigned long)irq->PreambleDetected, (unsigned long)irq->SyncWordValid);
    log_info("[LORA]   HEADER_VALID=%lu | HEADER_ERROR=%lu | CRC_ERROR=%lu | RX_TX_TIMEOUT=%lu | TX_DONE=%lu\r\n",
             (unsigned long)irq->HeaderValid, (unsigned long)irq->HeaderError,
             (unsigned long)irq->CrcError, (unsigned long)irq->RxTxTimeout,
             (unsigned long)irq->TxDone);
    log_info("[LORA]   IRQ 处理次数=%lu | 应用层 RxDone 回调=%lu 次 | 帧解析失败=%lu 次 | 接收补开=%lu 次\r\n",
             (unsigned long)irq->IrqProcessed,
             (unsigned long)g_stat_rxdone_cb,
             (unsigned long)g_stat_parse_fail,
             (unsigned long)LoraApp_RxRearmCount());

    /* ---------------- 8. 芯片自带收包统计(能回读, 且不会被本命令清零) ---------------- */
    LLCC68GetStats(&st);
    log_info("[LORA] 芯片收包统计【回读 GetStats 0x10】= 收包 %u | CRC 过 %u | 长度/头错 %u\r\n",
             (unsigned)st.NbPacketReceived, (unsigned)st.NbPacketCrcOk,
             (unsigned)st.NbPacketLengthError);
    log_info("[LORA]   原始 5 字节 = %02X %02X %02X %02X %02X\r\n",
             (unsigned)st.Raw[0], (unsigned)st.Raw[1], (unsigned)st.Raw[2],
             (unsigned)st.Raw[3], (unsigned)st.Raw[4]);
    log_info("[LORA]   本命令【不】发 ResetStats, 所以这是芯片自复位以来的累计值: 全 0 = 一个包都没解调出来。\r\n");

    /* ---------------- 9. 芯片故障标志(能回读) ---------------- */
    err = LLCC68GetDeviceErrors();
    log_info("[LORA] 芯片错误标志【回读 GetErrors 0x17】= 0x%04X: PLL未锁=%u XOSC未起振=%u PA_ramp失败=%u\r\n",
             (unsigned)err.Value,
             (unsigned)err.Fields.PllLock, (unsigned)err.Fields.XoscStart,
             (unsigned)err.Fields.PaRamp);
    log_info("[LORA]   映像校准失败=%u ADC校准失败=%u PLL校准失败=%u RC13M校准失败=%u RC64K校准失败=%u\r\n",
             (unsigned)err.Fields.ImgCalib, (unsigned)err.Fields.AdcCalib,
             (unsigned)err.Fields.PllCalib, (unsigned)err.Fields.Rc13mCalib,
             (unsigned)err.Fields.Rc64kCalib);

    /* ---------------- 10. 一句话判读(照 #clk 的风格) ---------------- */
    log_info("[LORA] ---- 判读 ----\r\n");
    log_info("[LORA] 射频%s | BUSY=%s(等超时%lu次) | 芯片模式=%s | 回读频率=%lu.%03luMHz | 固件写入: SF%u BW%lukHz CR4/%u\r\n",
             (chipMode == LORA_DIAG_CHIPMODE_RX) ? "在 RX 连续接收" : "【不在 RX】",
             (LLCC68BusyLevel() != 0u) ? "高" : "低",
             (unsigned long)busyTo,
             lora_diag_chipmode_text(chipMode),
             (unsigned long)(freqHz / 1000000u),
             (unsigned long)((freqHz % 1000000u) / 1000u),
             (unsigned)mp->Params.LoRa.SpreadingFactor,
             (unsigned long)lora_diag_bw_khz(mp->Params.LoRa.Bandwidth),
             (unsigned)((unsigned)mp->Params.LoRa.CodingRate + 4u));

    if (busyStuck)
    {
        log_info("[LORA] !! BUSY(PB3) 卡在高: LoRa 模组不应答 —— 射频配置很可能【根本没写进芯片】。\r\n");
        log_info("[LORA] !!    先查: 模组供电 / EN(PB5) / BUSY 接线 / 有没有焊好; 此时下面所有回读值都不可信。\r\n");
        ++warn;
    }
    if (busyTo != 0u)
    {
        log_info("[LORA] !! 等 BUSY 超时累计 %lu 次: 有配置命令没等成功就发出去了, 芯片里的参数可能是旧值/默认值。\r\n",
                 (unsigned long)busyTo);
        ++warn;
    }
    if (chipMode != LORA_DIAG_CHIPMODE_RX)
    {
        log_info("[LORA] !! 芯片当前不在 RX(在 %s): 现在这一刻收不到任何东西。\r\n",
                 lora_diag_chipmode_text(chipMode));
        ++warn;
    }
    if (!rxContinuous)
    {
        log_info("[LORA] !! RxContinuous=假: 用的是有限接收超时, 超时后芯片就离开 RX 了。\r\n");
        ++warn;
    }
    if (freqMismatch && (busyStuck == false) && (busyTo == 0u))
    {
        log_info("[LORA] !! 回读频率与固件写入值不一致: SET_RFFREQUENCY 没有生效(或被谁改写)。\r\n");
        ++warn;
    }
    if (((syncWord != (uint16_t)LORA_MAC_PRIVATE_SYNCWORD) &&
         (syncWord != (uint16_t)LORA_MAC_PUBLIC_SYNCWORD)) &&
        (busyStuck == false) && (busyTo == 0u))
    {
        log_info("[LORA] !! LoRa 同步字是自定义值 0x%04X: 必须与对端逐位一致, 否则永远收不到。\r\n",
                 (unsigned)syncWord);
        ++warn;
    }

    /* 【最关键的一条判据】三种"链路特征位"全为 0 = 芯片一个包都没解调出来 */
    if ((irq->RxDone == 0u) && (irq->PreambleDetected == 0u) && (irq->SyncWordValid == 0u))
    {
        log_info("[LORA] !! 开机以来 RX_DONE / PREAMBLE_DETECTED / SYNCWORD_VALID 全为 0:\r\n");
        log_info("[LORA] !!    芯片【一个包都没解调出来】 => 问题在射频层(频率/带宽/SF/编码率/同步字/天线),\r\n");
        log_info("[LORA] !!    不是帧格式问题 —— 帧格式再错, 芯片也会先报 PREAMBLE/SYNCWORD/RX_DONE。\r\n");
        log_info("[LORA] !!    下一步: 把对端的频率/带宽/SF/CR/同步字/报头类型/前导码逐项与本机核对。\r\n");
        if (st.NbPacketReceived == 0u)
        {
            log_info("[LORA] !!    芯片 GetStats 收包计数也是 0, 与上面的结论一致。\r\n");
        }
        else
        {
            log_info("[LORA] !!    但 GetStats 收包计数=%u 非 0: 芯片其实解调出过包 -> 重点查 IRQ 通路/驱动集成。\r\n",
                     (unsigned)st.NbPacketReceived);
        }
        ++warn;
    }
    else if ((irq->RxDone > 0u) || (g_stat_rxdone_cb > 0u))
    {
        if (g_stat_rxdone_cb == 0u)
        {
            log_info("[LORA] !! IRQ 里有 RX_DONE 但应用层 RxDone 回调一次都没进来: 查 DIO1/EXTI 中断通路与回调注册。\r\n");
            ++warn;
        }
        else if (g_stat_parse_fail > 0u)
        {
            log_info("[LORA] !! 收到过帧(%lu 次)但解析失败 %lu 次: 看 [proto] !! PARSE FAILED 日志(帧头/版本/长度/CRC)。\r\n",
                     (unsigned long)g_stat_rxdone_cb, (unsigned long)g_stat_parse_fail);
            ++warn;
        }
    }

    if (warn == 0u)
    {
        log_info("[LORA] 判读: 射频在 RX 连续接收 / BUSY 正常 / 频率 %lu.%03luMHz SF%u BW%lukHz CR4/%u ——\r\n",
                 (unsigned long)(freqHz / 1000000u),
                 (unsigned long)((freqHz % 1000000u) / 1000u),
                 (unsigned)mp->Params.LoRa.SpreadingFactor,
                 (unsigned long)lora_diag_bw_khz(mp->Params.LoRa.Bandwidth),
                 (unsigned)((unsigned)mp->Params.LoRa.CodingRate + 4u));
        log_info("[LORA]       本机射频侧没发现异常; 若仍收不到, 重点查天线/前端 + 对端是否逐项一致。\r\n");
    }
    else
    {
        log_info("[LORA] 判读: 上面带 !! 的 %u 条要逐条先解决; 解决后重新敲 #lora 看累计计数有没有增长。\r\n",
                 (unsigned)warn);
        log_info("[LORA]       (累计计数只增不减, 所以 再敲一次数字没变 本身就说明射频侧没有新动静)\r\n");
    }
    log_info("[LORA] ===== 诊断结束(本次没有发送任何 Set/Calibrate/SetRx 命令, 接收没有被中断) =====\r\n\r\n");
}

