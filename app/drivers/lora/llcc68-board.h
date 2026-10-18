/*!
 * \file      LLCC68-board.h
 *
 * \brief     Target board LLCC68 driver implementation
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
#ifndef __LLCC68_BOARD_H__
#define __LLCC68_BOARD_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include "llcc68.h"

// 临界区宏：有 OS 时用 RTOS 的进入/退出，裸机下为空
#if HAVE_OS
	#define CRITICAL_SECTION_BEGIN( )	vPortEnterCritical()
	#define CRITICAL_SECTION_END( )	vPortExitCritical()
#else
	#define CRITICAL_SECTION_BEGIN( )
	#define CRITICAL_SECTION_END( )
#endif

/*!
 * \brief Initializes the radio I/Os pins interface
 */
void LLCC68IoInit( void );

/*!
 * \brief Initializes DIO IRQ handlers
 *
 * \param [IN] irqHandlers Array containing the IRQ callback functions
 */
void LLCC68IoIrqInit( DioIrqHandler dioIrq );

/*!
 * \brief De-initializes the radio I/Os pins interface.
 *
 * \remark Useful when going in MCU low power modes
 */
void LLCC68IoDeInit( void );

/*!
 * \brief Initializes the radio debug pins.
 */
void LLCC68IoDbgInit( void );

/*!
 * \brief HW Reset of the radio
 */
void LLCC68ResetInit( void ); 
/*!
 * \brief Blocking loop to wait while the Busy pin in high
 *
 * \note 【已加上限】: 单次最多等 LLCC68_BUSY_TIMEOUT_MS; 连续超时会置
 *       g_lora_busy_stuck 并返回(不再死等), 见 LLCC68N32H482-board.c。
 *       原实现是没有退出条件的死循环 —— LoRa 模组一不应答, 整个固件(包括 4G
 *       初始化)都会被卡在这里, 日志里只剩每秒一行的 "wait busy pin timeout"。
 */
void LLCC68WaitOnBusy( void );

/*!
 * \brief 当前 BUSY 引脚电平 (1 = 忙/模组没应答)
 */
uint8_t LLCC68BusyLevel( void );

/*!
 * \brief LoRa 模组是否已被判定为"不应答" (1 = 是)
 */
uint8_t LLCC68IsStuck( void );

/*!
 * \brief 累计"等 BUSY 超时"次数 (#lora 射频诊断用; 只增不减)
 *
 * \note 0 = 所有 SPI 命令都正常等到 BUSY 拉低(配置确实是写进芯片的);
 *       不为 0 = 有命令在 BUSY 还高时就发出去了, 那一次的写入/读取结果不可信,
 *       芯片里的实际配置可能并不是固件以为的那个 —— 这正是 #lora 要回答的问题。
 */
uint32_t LLCC68BusyTimeoutCount( void );

/*!
 * \brief Checks if the given RF frequency is supported by the hardware
 *
 * \param [IN] frequency RF frequency to be checked
 * \retval isSupported [true: supported, false: unsupported]
 */
bool LLCC68CheckRfFrequency( uint32_t frequency );

void LLCC68DelayMs(uint32_t ms);

/*
 * 定时器初始化（TX/RX 超时用的硬件定时器）
 */
void LLCC68TimerInit(void);
void LLCC68SetTxTimerValue(uint32_t nMs);
void LLCC68TxTimerStart(void);
void LLCC68TxTimerStop(void);
void LLCC68SetRxTimerValue(uint32_t nMs);
void LLCC68RxTimerStart(void);
void LLCC68RxTimerStop(void);

void LLCC68SetNss(uint8_t lev );
uint8_t LLCC68SpiInOut(uint8_t data);

/*!
 * Radio hardware and global parameters
 */
extern LLCC68_t LLCC68;

#ifdef __cplusplus
}
#endif

#endif // __LLCC68_BOARD_H__
