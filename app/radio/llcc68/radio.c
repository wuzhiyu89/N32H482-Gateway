/*!
 * \file      radio.c
 *
 * \brief     Radio driver API definition
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include <math.h>
#include <string.h>
#include "delay.h"
#include "radio.h"
#include "LLCC68.h"
#include "LLCC68-board.h"
#include "project_config.h"
#include "stdio.h"

/* ★ 2026-09-27: log.h 改为【无条件】包含 ★
 * 原来它被包在 #if MAIN_LORA_RAW_TEST 里(为的是"开关=0 时 radio.o 与加开关之前完全一致")。
 * 但调试期间在 GetChipVersion() 里加了 `log_info("[LORA]%s", ver);`(读芯片版本串) ——
 * 那行在 #if 之外, 于是开关=0 时: 调用在、声明不在 -> 正常固件直接编不过
 * (armclang 报 "call to undeclared function 'log_info'")。
 * 修法选择【无条件包含】而不是把那行打印藏进 #if —— 因为读版本串正是排查
 * "SX1261/LLCC68 分支走错导致空口对方收不到"时要看的信息, 正常固件也该有。 */
#include "log.h"

#if MAIN_LORA_RAW_TEST
/* 【最小收发隔离测试专用】把底层 IRQ / 原始字节打到日志口(log.h 的 log_info)。 */
#endif

/*!
 * Defines the time required for the TCXO to wakeup [ms].
 */
#define BOARD_TCXO_WAKEUP_TIME                      0

/*!
 * \brief Initializes the radio
 *
 * \param [IN] events Structure containing the driver callback functions
 */
void RadioInit( RadioEvents_t *events );

/*!
 * Return current radio status
 *
 * \param status Radio status.[RF_IDLE, RF_RX_RUNNING, RF_TX_RUNNING]
 */
RadioState_t RadioGetStatus( void );

/*!
 * \brief Configures the radio with the given modem
 *
 * \param [IN] modem Modem to be used [0: FSK, 1: LoRa]
 */
void RadioSetModem( RadioModems_t modem );

/*!
 * \brief Sets the channel frequency
 *
 * \param [IN] freq         Channel RF frequency
 */
void RadioSetChannel( uint32_t freq );

/*!
 * \brief Checks if the channel is free for the given time
 *
 * \remark The FSK modem is always used for this task as we can select the Rx bandwidth at will.
 *
 * \param [IN] freq                Channel RF frequency in Hertz
 * \param [IN] rxBandwidth         Rx bandwidth in Hertz
 * \param [IN] rssiThresh          RSSI threshold in dBm
 * \param [IN] maxCarrierSenseTime Max time in milliseconds while the RSSI is measured
 *
 * \retval isFree         [true: Channel is free, false: Channel is not free]
 */
bool RadioIsChannelFree( uint32_t freq, uint32_t rxBandwidth, int16_t rssiThresh, uint32_t maxCarrierSenseTime );

/*!
 * \brief Generates a 32 bits random value based on the RSSI readings
 *
 * \remark This function sets the radio in LoRa modem mode and disables
 *         all interrupts.
 *         After calling this function either Radio.SetRxConfig or
 *         Radio.SetTxConfig functions must be called.
 *
 * \retval randomValue    32 bits random value
 */
uint32_t RadioRandom( void );

/*!
 * \brief Sets the reception parameters
 *
 * \param [IN] modem        Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] bandwidth    Sets the bandwidth
 *                          FSK : >= 2600 and <= 250000 Hz
 *                          LoRa: [0: 125 kHz, 1: 250 kHz,
 *                                 2: 500 kHz, 3: Reserved]
 * \param [IN] datarate     Sets the Datarate
 *                          FSK : 600..300000 bits/s
 *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
 *                                10: 1024, 11: 2048, 12: 4096  chips]
 * \param [IN] coderate     Sets the coding rate (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 * \param [IN] bandwidthAfc Sets the AFC Bandwidth (FSK only)
 *                          FSK : >= 2600 and <= 250000 Hz
 *                          LoRa: N/A ( set to 0 )
 * \param [IN] preambleLen  Sets the Preamble length
 *                          FSK : Number of bytes
 *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
 * \param [IN] symbTimeout  Sets the RxSingle timeout value
 *                          FSK : timeout in number of bytes
 *                          LoRa: timeout in symbols
 * \param [IN] fixLen       Fixed length packets [0: variable, 1: fixed]
 * \param [IN] payloadLen   Sets payload length when fixed length is used
 * \param [IN] crcOn        Enables/Disables the CRC [0: OFF, 1: ON]
 * \param [IN] FreqHopOn    Enables disables the intra-packet frequency hopping
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: OFF, 1: ON]
 * \param [IN] HopPeriod    Number of symbols between each hop
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: Number of symbols
 * \param [IN] iqInverted   Inverts IQ signals (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: not inverted, 1: inverted]
 * \param [IN] rxContinuous Sets the reception in continuous mode
 *                          [false: single mode, true: continuous mode]
 */
void RadioSetRxConfig( RadioModems_t modem, uint32_t bandwidth,
                          uint32_t datarate, uint8_t coderate,
                          uint32_t bandwidthAfc, uint16_t preambleLen,
                          uint16_t symbTimeout, bool fixLen,
                          uint8_t payloadLen,
                          bool crcOn, bool FreqHopOn, uint8_t HopPeriod,
                          bool iqInverted, bool rxContinuous );

/*!
 * \brief Sets the transmission parameters
 *
 * \param [IN] modem        Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] power        Sets the output power [dBm]
 * \param [IN] fdev         Sets the frequency deviation (FSK only)
 *                          FSK : [Hz]
 *                          LoRa: 0
 * \param [IN] bandwidth    Sets the bandwidth (LoRa only)
 *                          FSK : 0
 *                          LoRa: [0: 125 kHz, 1: 250 kHz,
 *                                 2: 500 kHz, 3: Reserved]
 * \param [IN] datarate     Sets the Datarate
 *                          FSK : 600..300000 bits/s
 *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
 *                                10: 1024, 11: 2048, 12: 4096  chips]
 * \param [IN] coderate     Sets the coding rate (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 * \param [IN] preambleLen  Sets the preamble length
 *                          FSK : Number of bytes
 *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
 * \param [IN] fixLen       Fixed length packets [0: variable, 1: fixed]
 * \param [IN] crcOn        Enables disables the CRC [0: OFF, 1: ON]
 * \param [IN] FreqHopOn    Enables disables the intra-packet frequency hopping
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: OFF, 1: ON]
 * \param [IN] HopPeriod    Number of symbols between each hop
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: Number of symbols
 * \param [IN] iqInverted   Inverts IQ signals (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: not inverted, 1: inverted]
 * \param [IN] timeout      Transmission timeout [ms]
 */
void RadioSetTxConfig( RadioModems_t modem, int8_t power, uint32_t fdev,
                          uint32_t bandwidth, uint32_t datarate,
                          uint8_t coderate, uint16_t preambleLen,
                          bool fixLen, bool crcOn, bool FreqHopOn,
                          uint8_t HopPeriod, bool iqInverted, uint32_t timeout );

/*!
 * \brief Checks if the given RF frequency is supported by the hardware
 *
 * \param [IN] frequency RF frequency to be checked
 * \retval isSupported [true: supported, false: unsupported]
 */
bool RadioCheckRfFrequency( uint32_t frequency );

/*!
 * \brief Computes the packet time on air in ms for the given payload
 *
 * \Remark Can only be called once SetRxConfig or SetTxConfig have been called
 *
 * \param [IN] modem      Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] bandwidth    Sets the bandwidth
 *                          FSK : >= 2600 and <= 250000 Hz
 *                          LoRa: [0: 125 kHz, 1: 250 kHz,
 *                                 2: 500 kHz, 3: Reserved]
 * \param [IN] datarate     Sets the Datarate
 *                          FSK : 600..300000 bits/s
 *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
 *                                10: 1024, 11: 2048, 12: 4096  chips]
 * \param [IN] coderate     Sets the coding rate (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 * \param [IN] preambleLen  Sets the Preamble length
 *                          FSK : Number of bytes
 *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
 * \param [IN] fixLen       Fixed length packets [0: variable, 1: fixed]
 * \param [IN] payloadLen   Sets payload length when fixed length is used
 * \param [IN] crcOn        Enables/Disables the CRC [0: OFF, 1: ON]
 *
 * \retval airTime        Computed airTime (ms) for the given packet payload length
 */
uint32_t RadioTimeOnAir( RadioModems_t modem, uint32_t bandwidth,
                              uint32_t datarate, uint8_t coderate,
                              uint16_t preambleLen, bool fixLen, uint8_t payloadLen,
                              bool crcOn );

/*!
 * \brief Sends the buffer of size. Prepares the packet to be sent and sets
 *        the radio in transmission
 *
 * \param [IN]: buffer     Buffer pointer
 * \param [IN]: size       Buffer size
 */
void RadioSend( uint8_t *buffer, uint8_t size );

/*!
 * \brief Sets the radio in sleep mode
 */
void RadioSleep( void );

/*!
 * \brief Sets the radio in standby mode
 */
void RadioStandby( void );

/*!
 * \brief Sets the radio in reception mode for the given time
 * \param [IN] timeout Reception timeout [ms]
 *                     [0: continuous, others timeout]
 */
void RadioRx( uint32_t timeout );

/*!
 * \brief Start a Channel Activity Detection
 */
void RadioStartCad( void );

/*!
 * \brief Sets the radio in continuous wave transmission mode
 *
 * \param [IN]: freq       Channel RF frequency
 * \param [IN]: power      Sets the output power [dBm]
 * \param [IN]: time       Transmission mode timeout [s]
 */
void RadioSetTxContinuousWave( uint32_t freq, int8_t power, uint16_t time );

/*!
 * \brief Reads the current RSSI value
 *
 * \retval rssiValue Current RSSI value in [dBm]
 */
int16_t RadioRssi( RadioModems_t modem );

/*!
 * \brief Writes the radio register at the specified address
 *
 * \param [IN]: addr Register address
 * \param [IN]: data New register value
 */
void RadioWrite( uint32_t addr, uint8_t data );

/*!
 * \brief Reads the radio register at the specified address
 *
 * \param [IN]: addr Register address
 * \retval data Register value
 */
uint8_t RadioRead( uint32_t addr );

/*!
 * \brief Writes multiple radio registers starting at address
 *
 * \param [IN] addr   First Radio register address
 * \param [IN] buffer Buffer containing the new register's values
 * \param [IN] size   Number of registers to be written
 */
void RadioWriteBuffer( uint32_t addr, uint8_t *buffer, uint8_t size );

/*!
 * \brief Reads multiple radio registers starting at address
 *
 * \param [IN] addr First Radio register address
 * \param [OUT] buffer Buffer where to copy the registers data
 * \param [IN] size Number of registers to be read
 */
void RadioReadBuffer( uint32_t addr, uint8_t *buffer, uint8_t size );

/*!
 * \brief Sets the maximum payload length.
 *
 * \param [IN] modem      Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] max        Maximum payload length in bytes
 */
void RadioSetMaxPayloadLength( RadioModems_t modem, uint8_t max );

/*!
 * \brief Sets the network to public or private. Updates the sync byte.
 *
 * \remark Applies to LoRa modem only
 *
 * \param [IN] enable if true, it enables a public network
 */
void RadioSetPublicNetwork( bool enable );

/*!
 * \brief Gets the time required for the board plus radio to get out of sleep.[ms]
 *
 * \retval time Radio plus board wakeup time in ms.
 */
uint32_t RadioGetWakeupTime( void );

/*!
 * \brief Process radio irq
 */
void RadioIrqProcess( void );

/*!
 * \brief Sets the radio in reception mode with Max LNA gain for the given time
 * \param [IN] timeout Reception timeout [ms]
 *                     [0: continuous, others timeout]
 */
void RadioRxBoosted( uint32_t timeout );

/*!
 * \brief Sets the Rx duty cycle management parameters
 *
 * \param [in]  rxTime        Structure describing reception timeout value
 * \param [in]  sleepTime     Structure describing sleep timeout value
 */
void RadioSetRxDutyCycle( uint32_t rxTime, uint32_t sleepTime );

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio =
{
    RadioInit,
    RadioGetStatus,
    RadioSetModem,
    RadioSetChannel,
    RadioIsChannelFree,
    RadioRandom,
    RadioSetRxConfig,
    RadioSetTxConfig,
    RadioCheckRfFrequency,
    RadioTimeOnAir,
    RadioSend,
    RadioSleep,
    RadioStandby,
    RadioRx,
    RadioStartCad,
    RadioSetTxContinuousWave,
    RadioRssi,
    RadioWrite,
    RadioRead,
    RadioWriteBuffer,
    RadioReadBuffer,
    RadioSetMaxPayloadLength,
    RadioSetPublicNetwork,
    RadioGetWakeupTime,
    RadioIrqProcess,
    // Available on LLCC68 only
    RadioRxBoosted,
    RadioSetRxDutyCycle
};

/*
 * Local types definition
 */


 /*!
 * FSK bandwidth definition
 */
typedef struct
{
    uint32_t bandwidth;
    uint8_t  RegValue;
}FskBandwidth_t;

/*!
 * Precomputed FSK bandwidth registers values
 */
const FskBandwidth_t FskBandwidths[] =
{
    { 4800  , 0x1F },
    { 5800  , 0x17 },
    { 7300  , 0x0F },
    { 9700  , 0x1E },
    { 11700 , 0x16 },
    { 14600 , 0x0E },
    { 19500 , 0x1D },
    { 23400 , 0x15 },
    { 29300 , 0x0D },
    { 39000 , 0x1C },
    { 46900 , 0x14 },
    { 58600 , 0x0C },
    { 78200 , 0x1B },
    { 93800 , 0x13 },
    { 117300, 0x0B },
    { 156200, 0x1A },
    { 187200, 0x12 },
    { 234300, 0x0A },
    { 312000, 0x19 },
    { 373600, 0x11 },
    { 467000, 0x09 },
    { 500000, 0x00 }, // Invalid Bandwidth
};

const RadioLoRaBandwidths_t Bandwidths[] = { LORA_BW_125, LORA_BW_250, LORA_BW_500 };

uint8_t MaxPayloadLength = 0xFF;

uint32_t TxTimeout = 0;
uint32_t RxTimeout = 0;

bool RxContinuous = false;


PacketStatus_t RadioPktStatus;
uint8_t RadioRxPayload[255];

bool IrqFired = false;

/* ==================== 【本工程新增】IRQ 累计计数(只读诊断, 给 #lora 用) ====================
 *
 * 【为什么非要有它】RadioIrqProcess() 每处理一次中断就把 IRQ status 寄存器【清掉】
 * (LLCC68ClearIrqStatus), 所以事后敲 #lora 去读 GetIrqStatus 永远是 0 ——
 * 现场最想知道的"RxDone 到底有没有触发过"用那个寄存器【根本回答不了】。
 *
 * 这里只做加法: 每见到一位就累加一次, 不动任何分支、不动任何射频配置。
 * 判据(配合 #lora 打印):
 *   PreambleDetected/SyncWordValid 从未置位 -> 频率/带宽/SF/同步字层面就对不上;
 *   RxDone 有置位而应用层没动作       -> 那是驱动集成问题, 不是空口问题。
 */
static RadioIrqStat_t g_irqStat;

const RadioIrqStat_t *RadioIrqStat( void )
{
    return &g_irqStat;
}

#if MAIN_LORA_RAW_TEST
/* ============================================================================
 *      【最小 LoRa 收发隔离测试】IRQ 寄存器层 / 原始字节的"原样打印"
 * ============================================================================
 * 【挂在哪一层 —— 这是本次测试最要紧的一点】
 *   接收链路自底向上是:
 *     DIO1(PB4) 电平变化
 *       → EXTI4_IRQHandler()(app/LLCC68N32H482-board.c)
 *       → RadioOnDioIrq()            只置 IrqFired(中断上下文里什么重活都不干)
 *     RadioIrqProcess()  ← 由主循环的 Radio.IrqProcess() 调用(主循环上下文)
 *       → LLCC68GetIrqStatus()       读【芯片的原始 IRQ 状态字】
 *       → ClearIrqStatus
 *       >>> [RAW-IRQ] 原样打印 irqRegs      <<<  本开关新增的观测点(比回调更低)
 *       >>> [RAW-RX]  原样打印收到的字节/RSSI/SNR
 *       → RadioEvents->RxDone / RxError
 *       → (lora_proto.c) RadioRxDoneCb: "[LoRa RX] size=.. data: .."
 *       → (lora_proto.c) lora_proto_poll(): 解析 AA55/长度/CRC16, 然后才可能打
 *                                          "[LoRa RX]" 之后的内容或
 *                                          "[proto] !! PARSE FAILED"
 *
 *   所以本函数打印的东西【早于任何解析、早于事件回调】: 它只回答一个问题 ——
 *   "射频到底有没有收到字节", 与"上层解没解析通过"彻底无关。
 *   现场故障恰恰卡在这里: [LoRa RX](解析通过) 和 PARSE FAILED(解析失败) 两条
 *   都不出现 -> 只能说明 RxDone 从未触发。有了这两行, 就能分清:
 *     · 连 [RAW-IRQ] 里的 RX_DONE/SYNC 都没有 -> 射频层(频率/带宽/SF/同步字/天线);
 *     · 有 [RAW-RX] 字节而上层没动作         -> 驱动/回调集成问题;
 *     · 有字节但 CRC 错                      -> 空口质量/干扰, 不是"没收到"。
 *
 * 【为什么只打"有意义"的 IRQ 字】
 *   PREAMBLE_DETECTED 在有噪声/有载波时会密集置位; 每一条都打会把串口刷满
 *   (115200 下一行约 1ms), 还会拖慢 IRQ 处理, 反过来影响接收。它的次数已经由
 *   g_irqStat 累计, 测试主循环每轮把它打出来, 不会漏掉"它在涨"这个事实。
 *   而 SYNCWORD_VALID 一定要打 —— "PREAMBLE 在涨但 SYNCWORD 恒 0"正是
 *   "空口参数对不上"的铁证, 必须在它出现的那一刻就能看见。
 * ============================================================================ */
static void LoraRawTestDumpIrq( uint16_t irq_regs )
{
    if( ( irq_regs & (uint16_t)( IRQ_RX_DONE | IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT |
                                 IRQ_HEADER_ERROR | IRQ_HEADER_VALID |
                                 IRQ_SYNCWORD_VALID ) ) == 0u )
    {
        return;     /* 只可能是 PREAMBLE_DETECTED 之类: 交给累计计数, 不刷屏 */
    }

    log_info( "[RAW-IRQ] irq=0x%04X | TX_DONE=%u RX_DONE=%u SYNC=%u HDR_V=%u HDR_E=%u CRC_E=%u PRE=%u TO=%u\r\n",
              (unsigned)irq_regs,
              (unsigned)( ( irq_regs & IRQ_TX_DONE )       ? 1u : 0u ),
              (unsigned)( ( irq_regs & IRQ_RX_DONE )       ? 1u : 0u ),
              (unsigned)( ( irq_regs & IRQ_SYNCWORD_VALID )? 1u : 0u ),
              (unsigned)( ( irq_regs & IRQ_HEADER_VALID )  ? 1u : 0u ),
              (unsigned)( ( irq_regs & IRQ_HEADER_ERROR )  ? 1u : 0u ),
              (unsigned)( ( irq_regs & IRQ_CRC_ERROR )     ? 1u : 0u ),
              (unsigned)( ( irq_regs & IRQ_PREAMBLE_DETECTED ) ? 1u : 0u ),
              (unsigned)( ( irq_regs & IRQ_RX_TX_TIMEOUT ) ? 1u : 0u ) );
}

/*!
 * \brief 【最小收发隔离测试专用】把一段收到的字节连长度/RSSI/SNR 原样打出来
 * \note  缓冲一律 static(本工程栈只有 4KB, 见 startup_n32h47x_48x.s 的 Stack_Size);
 *        本函数只在主循环上下文被调用(RadioIrqProcess 由主循环驱动, 不在 ISR 里)。
 */
static void LoraRawTestDumpBytes( const char *why, const uint8_t *buf, uint8_t len,
                                  int8_t rssi, int8_t snr )
{
    uint16_t i;

    log_info( "[RAW-RX] %s size=%u rssi=%d snr=%d:", why, (unsigned)len, (int)rssi, (int)snr );
    for( i = 0U; ( i < (uint16_t)len ) && ( i < 64U ); i++ )
    {
        log_info( " %02X", (unsigned)buf[i] );
    }
    log_info( "\r\n" );
}
#endif /* MAIN_LORA_RAW_TEST */

/*
 * LLCC68 DIO IRQ callback functions prototype
 */

/*!
 * \brief DIO 0 IRQ callback
 */
void RadioOnDioIrq( void* context );

/*
 * Private global variables
 */


/*!
 * Holds the current network type for the radio
 */
typedef struct
{
    bool Previous;
    bool Current;
}RadioPublicNetwork_t;

static RadioPublicNetwork_t RadioPublicNetwork = { false };

/*!
 * Radio callbacks variable
 */
static RadioEvents_t* RadioEvents;

/*
 * Public global variables
 */

/*!
 * Radio hardware and global parameters
 */
LLCC68_t LLCC68;

/*!
 * \brief Holds the internal operating mode of the radio
 */
static RadioOperatingModes_t OperatingMode;

//------------------------ board code start �� board �ƶ������Ĵ��� -----------------
RadioOperatingModes_t LLCC68GetOperatingMode( void )
{
    return OperatingMode;
}

void LLCC68SetOperatingMode( RadioOperatingModes_t mode )
{
    OperatingMode = mode;
}

void LLCC68IoTcxoInit( void )
{
    // No TCXO component available on this board design.
}

uint32_t LLCC68GetBoardTcxoWakeupTime( void )
{
    return BOARD_TCXO_WAKEUP_TIME;
}


//����DIO2λRF���ƽ�
void LLCC68IoRfSwitchInit( void )
{
    LLCC68SetDio2AsRfSwitchCtrl( true );
}

//�����������ڲ��Զ����ƣ�����ר������
void LLCC68AntSwOn( void )
{
    //GpioInit( &AntPow, RADIO_ANT_SWITCH_POWER, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
}
//�����������ڲ��Զ����ƣ�����ר������
void LLCC68AntSwOff( void )
{
    //GpioInit( &AntPow, RADIO_ANT_SWITCH_POWER, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
}

uint8_t LLCC68GetDeviceId( void )
{
    /* 真读版本字符串(寄存器 0x0320)判断型号，不要写死。
     * 本模块芯片自报 "SX1261"（RadioLib 的 LLCC68 类对这种情况有同样说明：
     * "some LLCC68 chips report as SX1261"）。若写死 SX1262，会让
     * LLCC68SetTxParams() 走错 PA 分支：
     *   SX1262 分支: 写 0x08D8 TxClamp 补丁 + SetPaConfig(0x04,0x07,0x00,0x01)(deviceSel=0) + 功率到+22dBm
     *   SX1261 分支: 不写 0x08D8      + SetPaConfig(0x04,0x00,0x01,0x01)(deviceSel=1) + 功率≤+14dBm
     * 走错分支的典型症状：TX_DONE 正常、GetErrors 全 0、接收也正常，但空口上对方收不到。
     * 换成真 SX1262/LLCC68 时这里读到别的字符串，会自动回落到 SX1262，行为不变。 */
    uint8_t ver[6];

    LLCC68ReadRegisters( 0x0320, ver, 6 );
		log_info("[LORA]%s",ver);
    if( ( ver[0] == 'S' ) && ( ver[1] == 'X' ) && ( ver[2] == '1' ) &&
        ( ver[3] == '2' ) && ( ver[4] == '6' ) && ( ver[5] == '1' ) )
    {
        return SX1262;
    }
    return SX1262;
}

void LLCC68SetRfTxPower( int8_t power )
{
    LLCC68SetTxParams( power, RADIO_RAMP_40_US );
}

void LLCC68WriteRegister( uint16_t address, uint8_t value )
{
    LLCC68WriteRegisters( address, &value, 1 );
}

uint8_t LLCC68ReadRegister( uint16_t address )
{
    uint8_t data;
    LLCC68ReadRegisters( address, &data, 1 );
    return data;
}

void LLCC68Wakeup( void )
{
	CRITICAL_SECTION_BEGIN( );

	LLCC68SetNss(0);

	LLCC68SpiInOut(RADIO_GET_STATUS );
	LLCC68SpiInOut(0x00 );

	LLCC68SetNss(1);

	// Wait for chip to be ready.
	LLCC68WaitOnBusy( );

	// Update operating mode context variable
	LLCC68SetOperatingMode( MODE_STDBY_RC );

	CRITICAL_SECTION_END( );
}

void LLCC68WriteCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
	uint16_t i = 0;
	
	LLCC68CheckDeviceReady( );

	LLCC68SetNss(0);

	LLCC68SpiInOut(( uint8_t )command);

	for( i = 0; i < size; i++ ){
		LLCC68SpiInOut(buffer[i]);
	}

	LLCC68SetNss(1);

	if( command != RADIO_SET_SLEEP ){
		LLCC68WaitOnBusy( );
	}
}

uint8_t LLCC68ReadCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
	uint8_t status = 0;
	uint16_t i = 0;
	
	LLCC68CheckDeviceReady( );

	LLCC68SetNss(0);

	LLCC68SpiInOut(( uint8_t )command);
	status = LLCC68SpiInOut(0x00);
	for( i = 0; i < size; i++ ){
		buffer[i] = LLCC68SpiInOut(0);
	}

	LLCC68SetNss(1);

	LLCC68WaitOnBusy( );

	return status;
}

void LLCC68WriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
	uint16_t i = 0;
	
	LLCC68CheckDeviceReady( );

	LLCC68SetNss(0);
    
	LLCC68SpiInOut(RADIO_WRITE_REGISTER );
	LLCC68SpiInOut(( address & 0xFF00 ) >> 8 );
	LLCC68SpiInOut( address & 0x00FF );
    
	for( i = 0; i < size; i++ ){
		LLCC68SpiInOut(buffer[i] );
	}

	LLCC68SetNss(1);

	LLCC68WaitOnBusy( );
}

void LLCC68ReadRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
	uint16_t i = 0;
	
	LLCC68CheckDeviceReady( );
	LLCC68SetNss(0);
	
	LLCC68SpiInOut(RADIO_READ_REGISTER);
	LLCC68SpiInOut(( address & 0xFF00 ) >> 8);
	LLCC68SpiInOut(address & 0x00FF);
	LLCC68SpiInOut(0);
	
	for( i = 0; i < size; i++ ){
		buffer[i] = LLCC68SpiInOut(0);
	}
	LLCC68SetNss(1);

	LLCC68WaitOnBusy( );
}

void LLCC68WriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
	uint16_t i = 0;
	
	LLCC68CheckDeviceReady( );

	LLCC68SetNss(0);

	LLCC68SpiInOut(RADIO_WRITE_BUFFER );
	LLCC68SpiInOut(offset );
	for( i = 0; i < size; i++ ){
		LLCC68SpiInOut(buffer[i] );
	}
	LLCC68SetNss(1);

	LLCC68WaitOnBusy( );
}

void LLCC68ReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
	uint16_t i = 0;
	
	LLCC68CheckDeviceReady( );

	LLCC68SetNss(0);

	LLCC68SpiInOut(RADIO_READ_BUFFER );
	LLCC68SpiInOut(offset );
	LLCC68SpiInOut(0 );
	for( i = 0; i < size; i++ ){
		buffer[i] = LLCC68SpiInOut(0);
	}
	LLCC68SetNss(1);

	LLCC68WaitOnBusy( );
}
//------------------------ board code end -----------------

/*!
 * Returns the known FSK bandwidth registers value
 *
 * \param [IN] bandwidth Bandwidth value in Hz
 * \retval regValue Bandwidth register value.
 */
static uint8_t RadioGetFskBandwidthRegValue( uint32_t bandwidth )
{
    uint8_t i;

    if( bandwidth == 0 )
    {
        return( 0x1F );
    }

    for( i = 0; i < ( sizeof( FskBandwidths ) / sizeof( FskBandwidth_t ) ) - 1; i++ )
    {
        if( ( bandwidth >= FskBandwidths[i].bandwidth ) && ( bandwidth < FskBandwidths[i + 1].bandwidth ) )
        {
            return FskBandwidths[i+1].RegValue;
        }
    }
    // ERROR: Value not found
    while( 1 );
}

void RadioInit( RadioEvents_t *events )
{
    RadioEvents = events;
    LLCC68IoInit();

    LLCC68Init( RadioOnDioIrq );
    LLCC68SetStandby( STDBY_RC );
    LLCC68SetRegulatorMode( USE_DCDC );

    /* OCP 过流保护阈值设为 0x38 = 140mA：LLCC68 高功率 PA(+22dBm 时 TX 电流约 118mA)
     * 需要 140mA。寄存器 0x08E7 复位默认 0x18=60mA 会限流，导致发射功率上不去。 */
    LLCC68WriteRegister( REG_OCP, 0x38 );

    LLCC68SetBufferBaseAddress( 0x00, 0x00 );
    LLCC68SetTxParams( 0, RADIO_RAMP_200_US );
    LLCC68SetDioIrqParams( IRQ_RADIO_ALL, IRQ_RADIO_ALL, IRQ_RADIO_NONE, IRQ_RADIO_NONE );

    LLCC68TimerInit();

    IrqFired = false;
}

RadioState_t RadioGetStatus( void )
{
    switch( LLCC68GetOperatingMode( ) )
    {
        case MODE_TX:
            return RF_TX_RUNNING;
        case MODE_RX:
            return RF_RX_RUNNING;
        case MODE_CAD:
            return RF_CAD;
        default:
            return RF_IDLE;
    }
}

/* 【#lora 诊断用】把"当初 SetRxConfig 传进来的 rxContinuous"给诊断命令看。
 * 注意它只是【固件写入值】: 芯片侧没有对应的回读寄存器, 真正"现在在不在收"
 * 要用 LLCC68GetStatus() 的 ChipMode 判断(见 lora_proto.c 的 LoraApp_DiagPrint)。 */
bool RadioGetRxContinuous( void )
{
    return RxContinuous;
}

void RadioSetModem( RadioModems_t modem )
{
    switch( modem )
    {
    default:
    case MODEM_FSK:
        LLCC68SetPacketType( PACKET_TYPE_GFSK );
        // When switching to GFSK mode the LoRa SyncWord register value is reset
        // Thus, we also reset the RadioPublicNetwork variable
        RadioPublicNetwork.Current = false;
        break;
    case MODEM_LORA:
        LLCC68SetPacketType( PACKET_TYPE_LORA );
        // Public/Private network register is reset when switching modems
        if( RadioPublicNetwork.Current != RadioPublicNetwork.Previous )
        {
            RadioPublicNetwork.Current = RadioPublicNetwork.Previous;
            RadioSetPublicNetwork( RadioPublicNetwork.Current );
        }
        break;
    }
}

void RadioSetChannel( uint32_t freq )
{
    LLCC68SetRfFrequency( freq );
}

bool RadioIsChannelFree( uint32_t freq, uint32_t rxBandwidth, int16_t rssiThresh, uint32_t maxCarrierSenseTime )
{
    bool     status           = true;
    int16_t  rssi             = 0;
    uint32_t count = 0;

    RadioSetModem( MODEM_FSK );

    RadioSetChannel( freq );

    // Set Rx bandwidth. Other parameters are not used.
    RadioSetRxConfig( MODEM_FSK, rxBandwidth, 600, 0, rxBandwidth, 3, 0, false,
                      0, false, 0, 0, false, true );
    RadioRx( 0 );

    LLCC68DelayMs( 1 );

    // Perform carrier sense for maxCarrierSenseTime
		count = 0;
    while( count < maxCarrierSenseTime )
    {
        rssi = RadioRssi( MODEM_FSK );

        if( rssi > rssiThresh )
        {
            status = false;
            break;
        }
				LLCC68DelayMs( 1 );
				count++;
    }
    RadioSleep( );
    return status;
}

uint32_t RadioRandom( void )
{
    uint32_t rnd = 0;

    /*
     * Radio setup for random number generation
     */
    // Set LoRa modem ON
    RadioSetModem( MODEM_LORA );

    // Disable LoRa modem interrupts
    LLCC68SetDioIrqParams( IRQ_RADIO_NONE, IRQ_RADIO_NONE, IRQ_RADIO_NONE, IRQ_RADIO_NONE );

    rnd = LLCC68GetRandom( );

    return rnd;
}

void RadioSetRxConfig( RadioModems_t modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                         bool iqInverted, bool rxContinuous )
{
    uint8_t syncWordBuf[]={ 0xC1, 0x94, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00 };
		
    RxContinuous = rxContinuous;
    if( rxContinuous == true )
    {
        symbTimeout = 0;
    }
    if( fixLen == true )
    {
        MaxPayloadLength = payloadLen;
    }
    else
    {
        MaxPayloadLength = 0xFF;
    }

    switch( modem )
    {
        case MODEM_FSK:
            LLCC68SetStopRxTimerOnPreambleDetect( false );
            LLCC68.ModulationParams.PacketType = PACKET_TYPE_GFSK;

            LLCC68.ModulationParams.Params.Gfsk.BitRate = datarate;
            LLCC68.ModulationParams.Params.Gfsk.ModulationShaping = MOD_SHAPING_G_BT_1;
            LLCC68.ModulationParams.Params.Gfsk.Bandwidth = RadioGetFskBandwidthRegValue( bandwidth << 1 ); // LLCC68 badwidth is double sided

            LLCC68.PacketParams.PacketType = PACKET_TYPE_GFSK;
            LLCC68.PacketParams.Params.Gfsk.PreambleLength = ( preambleLen << 3 ); // convert byte into bit
            LLCC68.PacketParams.Params.Gfsk.PreambleMinDetect = RADIO_PREAMBLE_DETECTOR_08_BITS;
            LLCC68.PacketParams.Params.Gfsk.SyncWordLength = 3 << 3; // convert byte into bit
            LLCC68.PacketParams.Params.Gfsk.AddrComp = RADIO_ADDRESSCOMP_FILT_OFF;
            LLCC68.PacketParams.Params.Gfsk.HeaderType = ( fixLen == true ) ? RADIO_PACKET_FIXED_LENGTH : RADIO_PACKET_VARIABLE_LENGTH;
            LLCC68.PacketParams.Params.Gfsk.PayloadLength = MaxPayloadLength;
            if( crcOn == true )
            {
                LLCC68.PacketParams.Params.Gfsk.CrcLength = RADIO_CRC_2_BYTES_CCIT;
            }
            else
            {
                LLCC68.PacketParams.Params.Gfsk.CrcLength = RADIO_CRC_OFF;
            }
            LLCC68.PacketParams.Params.Gfsk.DcFree = RADIO_DC_FREEWHITENING;

            RadioStandby( );
            RadioSetModem( ( LLCC68.ModulationParams.PacketType == PACKET_TYPE_GFSK ) ? MODEM_FSK : MODEM_LORA );
            LLCC68SetModulationParams( &LLCC68.ModulationParams );
            LLCC68SetPacketParams( &LLCC68.PacketParams );
            LLCC68SetSyncWord( syncWordBuf );
            LLCC68SetWhiteningSeed( 0x01FF );

            RxTimeout = ( uint32_t )symbTimeout * 8000UL / datarate;
            break;

        case MODEM_LORA:
            LLCC68SetStopRxTimerOnPreambleDetect( false );
            LLCC68.ModulationParams.PacketType = PACKET_TYPE_LORA;
            LLCC68.ModulationParams.Params.LoRa.SpreadingFactor = ( RadioLoRaSpreadingFactors_t )datarate;
            LLCC68.ModulationParams.Params.LoRa.Bandwidth = Bandwidths[bandwidth];
            LLCC68.ModulationParams.Params.LoRa.CodingRate = ( RadioLoRaCodingRates_t )coderate;

            if( ( ( bandwidth == 0 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
            ( ( bandwidth == 1 ) && ( datarate == 12 ) ) )
            {
                LLCC68.ModulationParams.Params.LoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                LLCC68.ModulationParams.Params.LoRa.LowDatarateOptimize = 0x00;
            }

            LLCC68.PacketParams.PacketType = PACKET_TYPE_LORA;

            if( ( LLCC68.ModulationParams.Params.LoRa.SpreadingFactor == LORA_SF5 ) ||
                ( LLCC68.ModulationParams.Params.LoRa.SpreadingFactor == LORA_SF6 ) )
            {
                if( preambleLen < 12 )
                {
                    LLCC68.PacketParams.Params.LoRa.PreambleLength = 12;
                }
                else
                {
                    LLCC68.PacketParams.Params.LoRa.PreambleLength = preambleLen;
                }
            }
            else
            {
                LLCC68.PacketParams.Params.LoRa.PreambleLength = preambleLen;
            }

            LLCC68.PacketParams.Params.LoRa.HeaderType = ( RadioLoRaPacketLengthsMode_t )fixLen;

            LLCC68.PacketParams.Params.LoRa.PayloadLength = MaxPayloadLength;
            LLCC68.PacketParams.Params.LoRa.CrcMode = ( RadioLoRaCrcModes_t )crcOn;
            LLCC68.PacketParams.Params.LoRa.InvertIQ = ( RadioLoRaIQModes_t )iqInverted;

            RadioStandby( );
            RadioSetModem( ( LLCC68.ModulationParams.PacketType == PACKET_TYPE_GFSK ) ? MODEM_FSK : MODEM_LORA );
            LLCC68SetModulationParams( &LLCC68.ModulationParams );
            LLCC68SetPacketParams( &LLCC68.PacketParams );
            LLCC68SetLoRaSymbNumTimeout( symbTimeout );

            // WORKAROUND - Optimizing the Inverted IQ Operation, see DS_SX1261-2_V1.2 datasheet chapter 15.4
            if( LLCC68.PacketParams.Params.LoRa.InvertIQ == LORA_IQ_INVERTED )
            {
                // RegIqPolaritySetup = @address 0x0736
                LLCC68WriteRegister( 0x0736, LLCC68ReadRegister( 0x0736 ) & ~( 1 << 2 ) );
            }
            else
            {
                // RegIqPolaritySetup @address 0x0736
                LLCC68WriteRegister( 0x0736, LLCC68ReadRegister( 0x0736 ) | ( 1 << 2 ) );
            }
            // WORKAROUND END

            // Timeout Max, Timeout handled directly in SetRx function
            RxTimeout = 0xFFFF;

            break;
    }
}

void RadioSetTxConfig( RadioModems_t modem, int8_t power, uint32_t fdev,
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn, bool freqHopOn,
                        uint8_t hopPeriod, bool iqInverted, uint32_t timeout )
{
    uint8_t syncWordBuf[] = { 0xC1, 0x94, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00 };
		
    switch( modem )
    {
        case MODEM_FSK:
            LLCC68.ModulationParams.PacketType = PACKET_TYPE_GFSK;
            LLCC68.ModulationParams.Params.Gfsk.BitRate = datarate;

            LLCC68.ModulationParams.Params.Gfsk.ModulationShaping = MOD_SHAPING_G_BT_1;
            LLCC68.ModulationParams.Params.Gfsk.Bandwidth = RadioGetFskBandwidthRegValue( bandwidth << 1 ); // LLCC68 badwidth is double sided
            LLCC68.ModulationParams.Params.Gfsk.Fdev = fdev;

            LLCC68.PacketParams.PacketType = PACKET_TYPE_GFSK;
            LLCC68.PacketParams.Params.Gfsk.PreambleLength = ( preambleLen << 3 ); // convert byte into bit
            LLCC68.PacketParams.Params.Gfsk.PreambleMinDetect = RADIO_PREAMBLE_DETECTOR_08_BITS;
            LLCC68.PacketParams.Params.Gfsk.SyncWordLength = 3 << 3 ; // convert byte into bit
            LLCC68.PacketParams.Params.Gfsk.AddrComp = RADIO_ADDRESSCOMP_FILT_OFF;
            LLCC68.PacketParams.Params.Gfsk.HeaderType = ( fixLen == true ) ? RADIO_PACKET_FIXED_LENGTH : RADIO_PACKET_VARIABLE_LENGTH;

            if( crcOn == true )
            {
                LLCC68.PacketParams.Params.Gfsk.CrcLength = RADIO_CRC_2_BYTES_CCIT;
            }
            else
            {
                LLCC68.PacketParams.Params.Gfsk.CrcLength = RADIO_CRC_OFF;
            }
            LLCC68.PacketParams.Params.Gfsk.DcFree = RADIO_DC_FREEWHITENING;

            RadioStandby( );
            RadioSetModem( ( LLCC68.ModulationParams.PacketType == PACKET_TYPE_GFSK ) ? MODEM_FSK : MODEM_LORA );
            LLCC68SetModulationParams( &LLCC68.ModulationParams );
            LLCC68SetPacketParams( &LLCC68.PacketParams );
            LLCC68SetSyncWord( syncWordBuf );
            LLCC68SetWhiteningSeed( 0x01FF );
            break;

        case MODEM_LORA:
            LLCC68.ModulationParams.PacketType = PACKET_TYPE_LORA;
            LLCC68.ModulationParams.Params.LoRa.SpreadingFactor = ( RadioLoRaSpreadingFactors_t ) datarate;
            LLCC68.ModulationParams.Params.LoRa.Bandwidth =  Bandwidths[bandwidth];
            LLCC68.ModulationParams.Params.LoRa.CodingRate= ( RadioLoRaCodingRates_t )coderate;

            if( ( ( bandwidth == 0 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
            ( ( bandwidth == 1 ) && ( datarate == 12 ) ) )
            {
                LLCC68.ModulationParams.Params.LoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                LLCC68.ModulationParams.Params.LoRa.LowDatarateOptimize = 0x00;
            }

            LLCC68.PacketParams.PacketType = PACKET_TYPE_LORA;

            if( ( LLCC68.ModulationParams.Params.LoRa.SpreadingFactor == LORA_SF5 ) ||
                ( LLCC68.ModulationParams.Params.LoRa.SpreadingFactor == LORA_SF6 ) )
            {
                if( preambleLen < 12 )
                {
                    LLCC68.PacketParams.Params.LoRa.PreambleLength = 12;
                }
                else
                {
                    LLCC68.PacketParams.Params.LoRa.PreambleLength = preambleLen;
                }
            }
            else
            {
                LLCC68.PacketParams.Params.LoRa.PreambleLength = preambleLen;
            }

            LLCC68.PacketParams.Params.LoRa.HeaderType = ( RadioLoRaPacketLengthsMode_t )fixLen;
            LLCC68.PacketParams.Params.LoRa.PayloadLength = MaxPayloadLength;
            LLCC68.PacketParams.Params.LoRa.CrcMode = ( RadioLoRaCrcModes_t )crcOn;
            LLCC68.PacketParams.Params.LoRa.InvertIQ = ( RadioLoRaIQModes_t )iqInverted;

            RadioStandby( );
            RadioSetModem( ( LLCC68.ModulationParams.PacketType == PACKET_TYPE_GFSK ) ? MODEM_FSK : MODEM_LORA );
            LLCC68SetModulationParams( &LLCC68.ModulationParams );
            LLCC68SetPacketParams( &LLCC68.PacketParams );
            break;
    }

    // WORKAROUND - Modulation Quality with 500 kHz LoRa Bandwidth, see DS_SX1261-2_V1.2 datasheet chapter 15.1
    if( ( modem == MODEM_LORA ) && ( LLCC68.ModulationParams.Params.LoRa.Bandwidth == LORA_BW_500 ) )
    {
        // RegTxModulation = @address 0x0889
        LLCC68WriteRegister( 0x0889, LLCC68ReadRegister( 0x0889 ) & ~( 1 << 2 ) );
    }
    else
    {
        // RegTxModulation = @address 0x0889
        LLCC68WriteRegister( 0x0889, LLCC68ReadRegister( 0x0889 ) | ( 1 << 2 ) );
    }
    // WORKAROUND END

    LLCC68SetRfTxPower( power );
    TxTimeout = timeout;
}

bool RadioCheckRfFrequency( uint32_t frequency )
{
    return true;
}

static uint32_t RadioGetLoRaBandwidthInHz( RadioLoRaBandwidths_t bw )
{
    uint32_t bandwidthInHz = 0;

    switch( bw )
    {
    case LORA_BW_007:
        bandwidthInHz = 7812UL;
        break;
    case LORA_BW_010:
        bandwidthInHz = 10417UL;
        break;
    case LORA_BW_015:
        bandwidthInHz = 15625UL;
        break;
    case LORA_BW_020:
        bandwidthInHz = 20833UL;
        break;
    case LORA_BW_031:
        bandwidthInHz = 31250UL;
        break;
    case LORA_BW_041:
        bandwidthInHz = 41667UL;
        break;
    case LORA_BW_062:
        bandwidthInHz = 62500UL;
        break;
    case LORA_BW_125:
        bandwidthInHz = 125000UL;
        break;
    case LORA_BW_250:
        bandwidthInHz = 250000UL;
        break;
    case LORA_BW_500:
        bandwidthInHz = 500000UL;
        break;
    }

    return bandwidthInHz;
}

static uint32_t RadioGetGfskTimeOnAirNumerator( uint32_t datarate, uint8_t coderate,
                              uint16_t preambleLen, bool fixLen, uint8_t payloadLen,
                              bool crcOn )
{
    const RadioAddressComp_t addrComp = RADIO_ADDRESSCOMP_FILT_OFF;
    const uint8_t syncWordLength = 3;

    return ( preambleLen << 3 ) +
           ( ( fixLen == false ) ? 8 : 0 ) +
             ( syncWordLength << 3 ) +
             ( ( payloadLen +
               ( addrComp == RADIO_ADDRESSCOMP_FILT_OFF ? 0 : 1 ) +
               ( ( crcOn == true ) ? 2 : 0 ) 
               ) << 3 
             );
}

static uint32_t RadioGetLoRaTimeOnAirNumerator( uint32_t bandwidth,
                              uint32_t datarate, uint8_t coderate,
                              uint16_t preambleLen, bool fixLen, uint8_t payloadLen,
                              bool crcOn )
{
    int32_t crDenom           = coderate + 4;
    bool    lowDatareOptimize = false;
    int32_t ceilDenominator;
    int32_t ceilNumerator = 0;
    int32_t intermediate = 0;

    // Ensure that the preamble length is at least 12 symbols when using SF5 or
    // SF6
    if( ( datarate == 5 ) || ( datarate == 6 ) )
    {
        if( preambleLen < 12 )
        {
            preambleLen = 12;
        }
    }

    if( ( ( bandwidth == 0 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
        ( ( bandwidth == 1 ) && ( datarate == 12 ) ) )
    {
        lowDatareOptimize = true;
    }

    
    ceilNumerator = ( payloadLen << 3 ) +
                            ( crcOn ? 16 : 0 ) -
                            ( 4 * datarate ) +
                            ( fixLen ? 0 : 20 );

    if( datarate <= 6 )
    {
        ceilDenominator = 4 * datarate;
    }
    else
    {
        ceilNumerator += 8;

        if( lowDatareOptimize == true )
        {
            ceilDenominator = 4 * ( datarate - 2 );
        }
        else
        {
            ceilDenominator = 4 * datarate;
        }
    }

    if( ceilNumerator < 0 )
    {
        ceilNumerator = 0;
    }

    // Perform integral ceil()
    intermediate =
        ( ( ceilNumerator + ceilDenominator - 1 ) / ceilDenominator ) * crDenom + preambleLen + 12;

    if( datarate <= 6 )
    {
        intermediate += 2;
    }

    return ( uint32_t )( ( 4 * intermediate + 1 ) * ( 1 << ( datarate - 2 ) ) );
}

uint32_t RadioTimeOnAir( RadioModems_t modem, uint32_t bandwidth,
                              uint32_t datarate, uint8_t coderate,
                              uint16_t preambleLen, bool fixLen, uint8_t payloadLen,
                              bool crcOn )
{
    uint32_t numerator = 0;
    uint32_t denominator = 1;

    switch( modem )
    {
    case MODEM_FSK:
        {
            numerator   = 1000U * RadioGetGfskTimeOnAirNumerator( datarate, coderate,
                                                                  preambleLen, fixLen,
                                                                  payloadLen, crcOn );
            denominator = datarate;
        }
        break;
    case MODEM_LORA:
        {
            numerator   = 1000U * RadioGetLoRaTimeOnAirNumerator( bandwidth, datarate,
                                                                  coderate, preambleLen,
                                                                  fixLen, payloadLen, crcOn );
            denominator = RadioGetLoRaBandwidthInHz( Bandwidths[bandwidth] );
        }
        break;
    }
    // Perform integral ceil()
    return ( numerator + denominator - 1 ) / denominator;
}

void RadioSend( uint8_t *buffer, uint8_t size )
{
    LLCC68SetDioIrqParams( IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                           IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                           IRQ_RADIO_NONE,
                           IRQ_RADIO_NONE );

    if( LLCC68GetPacketType( ) == PACKET_TYPE_LORA )
    {
        LLCC68.PacketParams.Params.LoRa.PayloadLength = size;
    }
    else
    {
        LLCC68.PacketParams.Params.Gfsk.PayloadLength = size;
    }
    LLCC68SetPacketParams( &LLCC68.PacketParams );

    LLCC68SendPayload( buffer, size, 0xFFFFFF );
    LLCC68SetTxTimerValue(TxTimeout);
    LLCC68TxTimerStart();
}

void RadioSleep( void )
{
    SleepParams_t params = { 0 };

    params.Fields.WarmStart = 1;
    LLCC68SetSleep( params );

    LLCC68DelayMs( 2 );
}

void RadioStandby( void )
{
    LLCC68SetStandby( STDBY_RC );
}

void RadioRx( uint32_t timeout )
{
    LLCC68SetDioIrqParams( IRQ_RADIO_ALL, //IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
                           IRQ_RADIO_ALL, //IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
                           IRQ_RADIO_NONE,
                           IRQ_RADIO_NONE );

    if( timeout != 0 )
    {
        LLCC68SetRxTimerValue(timeout);
        LLCC68RxTimerStart();
    }

    if( RxContinuous == true )
    {
        LLCC68SetRx( 0xFFFFFF ); // Rx Continuous
    }
    else
    {
        LLCC68SetRx( RxTimeout << 6 );
    }
}

void RadioRxBoosted( uint32_t timeout )
{
    LLCC68SetDioIrqParams( IRQ_RADIO_ALL, //IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
                           IRQ_RADIO_ALL, //IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
                           IRQ_RADIO_NONE,
                           IRQ_RADIO_NONE );

    if( timeout != 0 )
    {
        LLCC68SetRxTimerValue(timeout);
        LLCC68RxTimerStart();
    }

    if( RxContinuous == true )
    {
        LLCC68SetRxBoosted( 0xFFFFFF ); // Rx Continuous
    }
    else
    {
        LLCC68SetRxBoosted( RxTimeout << 6 );
    }
}

void RadioSetRxDutyCycle( uint32_t rxTime, uint32_t sleepTime )
{
    LLCC68SetRxDutyCycle( rxTime, sleepTime );
}

void RadioStartCad( void )
{
    LLCC68SetDioIrqParams( IRQ_CAD_DONE | IRQ_CAD_ACTIVITY_DETECTED, IRQ_CAD_DONE | IRQ_CAD_ACTIVITY_DETECTED, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
    LLCC68SetCad( );
}

void RadioSetTxContinuousWave( uint32_t freq, int8_t power, uint16_t time )
{
    uint32_t timeout = ( uint32_t )time * 1000;

    LLCC68SetRfFrequency( freq );
    LLCC68SetRfTxPower( power );
    LLCC68SetTxContinuousWave( );

    LLCC68SetTxTimerValue(timeout);
    LLCC68TxTimerStart();
}

int16_t RadioRssi( RadioModems_t modem )
{
    return LLCC68GetRssiInst( );
}

void RadioWrite( uint32_t addr, uint8_t data )
{
    LLCC68WriteRegister( addr, data );
}

uint8_t RadioRead( uint32_t addr )
{
    return LLCC68ReadRegister( addr );
}

void RadioWriteBuffer( uint32_t addr, uint8_t *buffer, uint8_t size )
{
    LLCC68WriteRegisters( addr, buffer, size );
}

void RadioReadBuffer( uint32_t addr, uint8_t *buffer, uint8_t size )
{
    LLCC68ReadRegisters( addr, buffer, size );
}

void RadioSetMaxPayloadLength( RadioModems_t modem, uint8_t max )
{
    if( modem == MODEM_LORA )
    {
        LLCC68.PacketParams.Params.LoRa.PayloadLength = MaxPayloadLength = max;
        LLCC68SetPacketParams( &LLCC68.PacketParams );
    }
    else
    {
        if( LLCC68.PacketParams.Params.Gfsk.HeaderType == RADIO_PACKET_VARIABLE_LENGTH )
        {
            LLCC68.PacketParams.Params.Gfsk.PayloadLength = MaxPayloadLength = max;
            LLCC68SetPacketParams( &LLCC68.PacketParams );
        }
    }
}

void RadioSetPublicNetwork( bool enable )
{
    RadioPublicNetwork.Current = RadioPublicNetwork.Previous = enable;

    RadioSetModem( MODEM_LORA );
    if( enable == true )
    {
        // Change LoRa modem SyncWord
        LLCC68WriteRegister( REG_LR_SYNCWORD, ( LORA_MAC_PUBLIC_SYNCWORD >> 8 ) & 0xFF );
        LLCC68WriteRegister( REG_LR_SYNCWORD + 1, LORA_MAC_PUBLIC_SYNCWORD & 0xFF );
    }
    else
    {
        // Change LoRa modem SyncWord
        LLCC68WriteRegister( REG_LR_SYNCWORD, ( LORA_MAC_PRIVATE_SYNCWORD >> 8 ) & 0xFF );
        LLCC68WriteRegister( REG_LR_SYNCWORD + 1, LORA_MAC_PRIVATE_SYNCWORD & 0xFF );
    }
}

uint32_t RadioGetWakeupTime( void )
{
    return LLCC68GetBoardTcxoWakeupTime( ) + RADIO_WAKEUP_TIME;
}

void RadioOnTxTimeoutIrq( void* context )
{
    if( ( RadioEvents != NULL ) && ( RadioEvents->TxTimeout != NULL ) )
    {
        RadioEvents->TxTimeout( );
    }
}

void RadioOnRxTimeoutIrq( void* context )
{
    if( ( RadioEvents != NULL ) && ( RadioEvents->RxTimeout != NULL ) )
    {
        RadioEvents->RxTimeout( );
    }
}

void RadioOnDioIrq( void* context )
{
    IrqFired = true;
}

void RadioIrqProcess( void )
{
    uint16_t irqRegs = 0;
    if( IrqFired == true )
    {
        CRITICAL_SECTION_BEGIN( );
        // Clear IRQ flag
        IrqFired = false;
        CRITICAL_SECTION_END( );

        irqRegs = LLCC68GetIrqStatus( );
        LLCC68ClearIrqStatus( irqRegs );

        /* ---------------- 【只加计数, 不改行为】#lora 诊断用 ----------------
         * 顺序不敏感、没有副作用; 作用见 g_irqStat 的说明。 */
        g_irqStat.IrqProcessed++;
        if( ( irqRegs & IRQ_RX_DONE ) == IRQ_RX_DONE )
        {
            g_irqStat.RxDone++;
            g_irqStat.EverRxDone = true;
        }
        if( ( irqRegs & IRQ_TX_DONE ) == IRQ_TX_DONE )
        {
            g_irqStat.TxDone++;
        }
        if( ( irqRegs & IRQ_PREAMBLE_DETECTED ) == IRQ_PREAMBLE_DETECTED )
        {
            g_irqStat.PreambleDetected++;
        }
        if( ( irqRegs & IRQ_SYNCWORD_VALID ) == IRQ_SYNCWORD_VALID )
        {
            g_irqStat.SyncWordValid++;
        }
        if( ( irqRegs & IRQ_HEADER_VALID ) == IRQ_HEADER_VALID )
        {
            g_irqStat.HeaderValid++;
        }
        if( ( irqRegs & IRQ_HEADER_ERROR ) == IRQ_HEADER_ERROR )
        {
            g_irqStat.HeaderError++;
        }
        if( ( irqRegs & IRQ_CRC_ERROR ) == IRQ_CRC_ERROR )
        {
            g_irqStat.CrcError++;
        }
        if( ( irqRegs & IRQ_RX_TX_TIMEOUT ) == IRQ_RX_TX_TIMEOUT )
        {
            g_irqStat.RxTxTimeout++;
        }
        if( ( irqRegs & IRQ_CAD_DONE ) == IRQ_CAD_DONE )
        {
            g_irqStat.CadDone++;
        }
        if( ( irqRegs & IRQ_CAD_ACTIVITY_DETECTED ) == IRQ_CAD_ACTIVITY_DETECTED )
        {
            g_irqStat.CadActivityDetected++;
        }
        /* RxDone 与 CRC_ERROR 同时置位 = "收到了包但校验没过" ——
         * 这一种最容易被现场误读成"一个字节都没收到" */
        if( ( ( irqRegs & IRQ_RX_DONE ) == IRQ_RX_DONE ) &&
            ( ( irqRegs & IRQ_CRC_ERROR ) == IRQ_CRC_ERROR ) )
        {
            g_irqStat.RxDoneCrcErr++;
        }
        /* ---------------- 计数结束 ---------------- */

#if MAIN_LORA_RAW_TEST
        /* 【最底层的一条证据】把这个 IRQ 字的【原始值】原样打出来。
         * 位置刻意选在 ClearIrqStatus 与计数【之后】、事件分发【之前】:
         *   · 在 GetIrqStatus/ClearIrqStatus 之间不插任何打印, 那条时间缝与
         *     生产代码一模一样(打印一行约 1ms, 插在中间会改变原来的时序);
         *   · 又比下面的 RadioEvents->RxDone 回调更低 —— "射频有没有收到字节"
         *     与"上层解没解析"在这里就分开了。
         * 详见本文件上部 LoraRawTestDumpIrq() 的说明。 */
        LoraRawTestDumpIrq( irqRegs );
#endif

        if( ( irqRegs & IRQ_TX_DONE ) == IRQ_TX_DONE )
        {
            LLCC68TxTimerStop();
            //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
            LLCC68SetOperatingMode( MODE_STDBY_RC );
            if( ( RadioEvents != NULL ) && ( RadioEvents->TxDone != NULL ) )
            {
                RadioEvents->TxDone( );
            }
        }

        if( ( irqRegs & IRQ_RX_DONE ) == IRQ_RX_DONE )
        {
            if( ( irqRegs & IRQ_CRC_ERROR ) == IRQ_CRC_ERROR )
            {
                if( RxContinuous == false )
                {
                    //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
                    LLCC68SetOperatingMode( MODE_STDBY_RC );
                }
#if MAIN_LORA_RAW_TEST
                {
                    /* 【CRC 错的包也要看见字节】
                     * 这一条路径【不会】走到 RadioEvents->RxDone, 所以 lora_proto.c 里
                     * 那行 "[LoRa RX] size=.. data:.." 对 CRC 错的包是【完全不打印】的。
                     * "空口上收到了字节但校验没过" 和 "一个字节都没有" 是两个完全
                     * 不同的结论, 所以这里单独把长度 + 原始字节读出来打一遍。
                     * 缓冲一律 static(栈只有 4KB); 只打前 64 字节, 不刷屏。 */
                    static uint8_t         rawtest_buf[64];
                    static uint8_t         rawtest_len;
                    static uint8_t         rawtest_off;
                    static PacketStatus_t  rawtest_pkt;

                    LLCC68GetRxBufferStatus( &rawtest_len, &rawtest_off );
                    if( rawtest_len > (uint8_t)sizeof( rawtest_buf ) )
                    {
                        rawtest_len = (uint8_t)sizeof( rawtest_buf );
                    }
                    LLCC68ReadBuffer( rawtest_off, rawtest_buf, rawtest_len );
                    LLCC68GetPacketStatus( &rawtest_pkt );

                    LoraRawTestDumpBytes( "CRC错包(IRQ 层原样, 未经任何解析)",
                                          rawtest_buf, rawtest_len,
                                          rawtest_pkt.Params.LoRa.RssiPkt,
                                          rawtest_pkt.Params.LoRa.SnrPkt );

                    /* 顺带把"收到该包时的信号/频偏"也报一行 —— 与上面的 RSSI/SNR
                     * 互为旁证: 有字节 + 信号明显抬起来 = 对端确实在发, 只是校验没过。 */
                    log_info( "[RAW-RX] CRC错包 详情: RxBufferStatus len=%u offset=%u | "
                              "RSSI=%d SNR=%d 信号RSSI=%d 频偏=%ldHz\r\n",
                              (unsigned)rawtest_len, (unsigned)rawtest_off,
                              (int)rawtest_pkt.Params.LoRa.RssiPkt,
                              (int)rawtest_pkt.Params.LoRa.SnrPkt,
                              (int)rawtest_pkt.Params.LoRa.SignalRssiPkt,
                              (long)rawtest_pkt.Params.LoRa.FreqError );
                }
#endif
                if( ( RadioEvents != NULL ) && ( RadioEvents->RxError ) )
                {
                    RadioEvents->RxError( );
                }
            }
            else
            {
                uint8_t size;

                LLCC68RxTimerStop();
                if( RxContinuous == false )
                {
                    //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
                    LLCC68SetOperatingMode( MODE_STDBY_RC );

                    // WORKAROUND - Implicit Header Mode Timeout Behavior, see DS_SX1261-2_V1.2 datasheet chapter 15.3
                    // RegRtcControl = @address 0x0902
                    LLCC68WriteRegister( 0x0902, 0x00 );
                    // RegEventMask = @address 0x0944
                    LLCC68WriteRegister( 0x0944, LLCC68ReadRegister( 0x0944 ) | ( 1 << 1 ) );
                    // WORKAROUND END
                }
                LLCC68GetPayload( RadioRxPayload, &size , 255 );
                LLCC68GetPacketStatus( &RadioPktStatus );
#if MAIN_LORA_RAW_TEST
                /* 【IRQ 层的原始字节】在事件回调之前先打一遍。
                 * 它与 lora_proto.c 的 "[LoRa RX] ..." 是同一包字节的两次打印:
                 * 这里是【芯片 IRQ 层】, 那里是【驱动事件回调层】——
                 * 两条都出现 = 链路一直通到驱动; 只有这一条 = 回调集成有问题。 */
                LoraRawTestDumpBytes( "解调成功(IRQ 层原样, 早于事件回调/解析)",
                                      RadioRxPayload, size,
                                      RadioPktStatus.Params.LoRa.RssiPkt,
                                      RadioPktStatus.Params.LoRa.SnrPkt );
#endif
                if( ( RadioEvents != NULL ) && ( RadioEvents->RxDone != NULL ) )
                {
                    RadioEvents->RxDone( RadioRxPayload, size, RadioPktStatus.Params.LoRa.RssiPkt, RadioPktStatus.Params.LoRa.SnrPkt );
                }
            }
        }

        if( ( irqRegs & IRQ_CAD_DONE ) == IRQ_CAD_DONE )
        {
            //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
            LLCC68SetOperatingMode( MODE_STDBY_RC );
            if( ( RadioEvents != NULL ) && ( RadioEvents->CadDone != NULL ) )
            {
                RadioEvents->CadDone( ( ( irqRegs & IRQ_CAD_ACTIVITY_DETECTED ) == IRQ_CAD_ACTIVITY_DETECTED ) );
            }
        }

        if( ( irqRegs & IRQ_RX_TX_TIMEOUT ) == IRQ_RX_TX_TIMEOUT )
        {
            if( LLCC68GetOperatingMode( ) == MODE_TX )
            {
                LLCC68TxTimerStop();
                //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
                LLCC68SetOperatingMode( MODE_STDBY_RC );
                if( ( RadioEvents != NULL ) && ( RadioEvents->TxTimeout != NULL ) )
                {
                    RadioEvents->TxTimeout( );
                }
            }
            else if( LLCC68GetOperatingMode( ) == MODE_RX )
            {
                LLCC68RxTimerStop();
                /* ★★2026-10-10 修: 补上 RxContinuous 判断★★
                 * 老代码在这里【无条件】去 STDBY, 而同一文件里另外两条路径都判了
                 * RxContinuous == false 才改模式:
                 *     RxDone        (约 1693 行)  if( RxContinuous == false )
                 *     IRQ_HEADER_ERROR (约 1776 行) if( RxContinuous == false )
                 * ⇒ 只有这一条漏了。连续接收下如此处理 = 芯片被强行踢出接收,
                 *   而驱动侧不一定有人把它带回来(补开机制是"请求驱动", 不是必然动作)
                 *   ⇒ 表现为"网关此后一直收不到"。
                 * 【风险很低】本工程一直是连续接收, 这个 IRQ 本来就极少触发;
                 *   但漏判留着迟早出事, 顺手补上。 */
                if( RxContinuous == false )
                {
                    //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
                    LLCC68SetOperatingMode( MODE_STDBY_RC );
                }
                if( ( RadioEvents != NULL ) && ( RadioEvents->RxTimeout != NULL ) )
                {
                    RadioEvents->RxTimeout( );
                }
            }
        }

        if( ( irqRegs & IRQ_PREAMBLE_DETECTED ) == IRQ_PREAMBLE_DETECTED )
        {
            //__NOP( );
        }

        if( ( irqRegs & IRQ_SYNCWORD_VALID ) == IRQ_SYNCWORD_VALID )
        {
            //__NOP( );
        }

        if( ( irqRegs & IRQ_HEADER_VALID ) == IRQ_HEADER_VALID )
        {
            //__NOP( );
        }

        if( ( irqRegs & IRQ_HEADER_ERROR ) == IRQ_HEADER_ERROR )
        {
            LLCC68RxTimerStop();
            if( RxContinuous == false )
            {
                //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
                LLCC68SetOperatingMode( MODE_STDBY_RC );
            }
            if( ( RadioEvents != NULL ) && ( RadioEvents->RxTimeout != NULL ) )
            {
                RadioEvents->RxTimeout( );
            }
        }
    }
}
