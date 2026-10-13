#ifndef __BSP_TIM_H__
#define __BSP_TIM_H__

/**
 * \file  bsp_tim.h
 * \brief 系统时基定时器
 *
 * 时钟链路 (由编译宏 BOARD_USE_HSE 选择, 见 system_n32h47x_48x.c):
 *   BOARD_USE_HSE=0 (默认, 无外部晶振):
 *     内部 HSI 8MHz RC → PLL 倍频 ×30 → SYSCLK = 240MHz
 *   BOARD_USE_HSE=1 (外接晶振 PH0/PH1):
 *     外部 HSE 16MHz   → PLL 倍频 ×30(HSE/2=8MHz 输入) → SYSCLK = 240MHz
 *   → HCLK  = 240MHz (AHB 不分频)
 *   → PCLK2 = 120MHz (APB2 /2)
 *   → PCLK1 = 120MHz (APB1 /2)
 *
 * ATIM1 挂在 AHB 域, 计数时钟 = HCLK = 240MHz
 *   Prescaler = 239  → 240M/(239+1) = 1MHz
 *   ARR    = 999  → 1MHz/1000 = 1ms 溢出中断
 */

#include "n32h47x_48x_tim.h"
#include "n32h47x_48x_rcc.h"

#include <stdint.h>


void bsp_tim1_init_1ms(void);
uint32_t get_tick_ms(void); //获取系统ms时间戳
void bsp_tim_tick_inc(void); //1ms时基累加,仅供ATIM1_UP_IRQHandler调用
/** @brief 基于ATIM1 tick的阻塞ms延时(误差<1ms,不进入低功耗) */
void delay_ms(uint32_t ms);

#endif /* __BSP_TIM_H__ */ 
