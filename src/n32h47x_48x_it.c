/**
*     Copyright (c) 2023, Nations Technologies Inc.
* 
*     All rights reserved.
*
*     This software is the exclusive property of Nations Technologies Inc. (Hereinafter 
* referred to as NATIONS). This software, and the product of NATIONS described herein 
* (Hereinafter referred to as the Product) are owned by NATIONS under the laws and treaties
* of the People's Republic of China and other applicable jurisdictions worldwide.
*
*     NATIONS does not grant any license under its patents, copyrights, trademarks, or other 
* intellectual property rights. Names and brands of third party may be mentioned or referred 
* thereto (if any) for identification purposes only.
*
*     NATIONS reserves the right to make changes, corrections, enhancements, modifications, and 
* improvements to this software at any time without notice. Please contact NATIONS and obtain 
* the latest version of this software before placing orders.

*     Although NATIONS has attempted to provide accurate and reliable information, NATIONS assumes 
* no responsibility for the accuracy and reliability of this software.
* 
*     It is the responsibility of the user of this software to properly design, program, and test 
* the functionality and safety of any application made of this information and any resulting product. 
* In no event shall NATIONS be liable for any direct, indirect, incidental, special,exemplary, or 
* consequential damages arising in any way out of the use of this software or the Product.
*
*     NATIONS Products are neither intended nor warranted for usage in systems or equipment, any
* malfunction or failure of which may cause loss of human life, bodily injury or severe property 
* damage. Such applications are deemed, "Insecure Usage".
*
*     All Insecure Usage shall be made at user's risk. User shall indemnify NATIONS and hold NATIONS 
* harmless from and against all claims, costs, damages, and other liabilities, arising from or related 
* to any customer's Insecure Usage.

*     Any express or implied warranty with regard to this software or the Product, including,but not 
* limited to, the warranties of merchantability, fitness for a particular purpose and non-infringement
* are disclaimed to the fullest extent permitted by law.

*     Unless otherwise explicitly permitted by NATIONS, anyone may not duplicate, modify, transcribe
* or otherwise distribute this software for any purposes, in whole or in part.
*
*     NATIONS products and technologies shall not be used for or incorporated into any products or systems
* whose manufacture, use, or sale is prohibited under any applicable domestic or foreign laws or regulations. 
* User shall comply with any applicable export control laws and regulations promulgated and administered by 
* the governments of any countries asserting jurisdiction over the parties or transactions.
**/

/**
*\*\file n32h47x_48x_it.c
*\*\author Nations
*\*\version v1.0.0
*\*\copyright Copyright (c) 2023, Nations Technologies Inc. All rights reserved.
**/

#include "n32h47x_48x_it.h"
#include "n32h47x_48x.h"
#include "main.h"
#include "n32h47x_48x_exti.h"
#include "n32h47x_48x_gpio.h"
#include "bsp_tim.h"
#include "log.h"




// n32h47x_48x_it.c - 在文件头部 include 区域之后、函数之前添加以下结构体和全局变量

/**
 * Cortex-M4 异常栈帧结构（自动入栈的 8 个寄存器）
 * 入栈顺序：R0, R1, R2, R3, R12, LR, PC, xPSR
 */
typedef struct
{
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t pc;
    uint32_t xpsr;
} fault_stack_frame_t;

/* 全局变量保存异常现场 —— 调试时在 Keil Watch 窗口添加这些变量 */
static volatile uint32_t g_fault_pc   = 0;  /* 异常指令地址（关键！） */
static volatile uint32_t g_fault_lr   = 0;  /* 返回地址 */
static volatile uint32_t g_fault_type = 0;  /* 0=Hard, 1=Mem, 2=Bus, 3=Usage */
static volatile uint32_t g_fault_cfsr = 0;  /* Configurable Fault Status */
static volatile uint32_t g_fault_hfsr = 0;  /* HardFault Status */
static volatile uint32_t g_fault_bfar = 0;  /* BusFault Address */

static void fault_save_context(uint32_t fault_type)
{
    fault_stack_frame_t *frame;
    __asm volatile("MRS %0, MSP" : "=r"(frame));

    g_fault_type = fault_type;
    g_fault_pc   = frame->pc;
    g_fault_lr   = frame->lr;

    /* 读取 Fault 状态寄存器 */
    g_fault_cfsr = (*((volatile uint32_t *)0xE000ED28)); /* CFSR */
    g_fault_hfsr = (*((volatile uint32_t *)0xE000ED2C)); /* HFSR */
    g_fault_bfar = (*((volatile uint32_t *)0xE000ED38)); /* BFAR */

    /* 在此设断点，或在 Watch 窗口查看 g_fault_pc */
    __asm volatile("BKPT #0");
}




/** Cortex-M4 Processor Exceptions Handlers **/

/**
*\*\name    NMI_Handler.
*\*\fun     This function handles NMI exception.
*\*\param   none
*\*\return  none 
**/
void NMI_Handler(void)
{
}

/**
*\*\name    HardFault_Handler.
*\*\fun     This function handles Hard Fault exception.
*\*\param   none
*\*\return  none 
**/
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    fault_save_context(0);
    while (1)
    {
    }
}

/**
*\*\name    MemManage_Handler.
*\*\fun     This function handles Memory Manage exception.
*\*\param   none
*\*\return  none 
**/
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    fault_save_context(1);
    while (1)
    {
    }
}

/**
*\*\name    BusFault_Handler.
*\*\fun     This function handles Bus Fault exception.
*\*\param   none
*\*\return  none 
**/
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    fault_save_context(2);
    while (1)
    {
    }
}

/**
*\*\name    UsageFault_Handler.
*\*\fun     This function handles Usage Fault exception.
*\*\param   none
*\*\return  none 
**/
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    fault_save_context(3);
    while (1)
    {
    }
}

/**
*\*\name    SVC_Handler.
*\*\fun     This function handles SVCall exception.
*\*\param   none
*\*\return  none 
**/
void SVC_Handler(void)
{
}

/**
*\*\name    DebugMon_Handler.
*\*\fun     This function handles Debug Monitor exception.
*\*\param   none
*\*\return  none 
**/
void DebugMon_Handler(void)
{
}

/**
*\*\name    SysTick_Handler.
*\*\fun     This function handles SysTick Handler.
*\*\param   none
*\*\return  none 
**/
void SysTick_Handler(void)
{
    
}

/**
*\*\name    ATIM1_UP_IRQHandler.
*\*\fun     This function handles ATIM1 update interrupt for 1ms system tick.
*\*\param   none
*\*\return  none 
**/
void ATIM1_UP_IRQHandler(void)
{
    if (TIM_GetIntStatus(ATIM1, TIM_INT_UPDATE) != RESET)
    {
        TIM_ClrIntPendingBit(ATIM1, TIM_INT_UPDATE);
        bsp_tim_tick_inc();
    }
}

/**
*\*\name    USB_HS_WKUP_IRQHandler.
*\*\fun     This function handles USB_HS_WKUP_IRQHandler Handler.
*\*\param   none
*\*\return  none
**/
void USB_HS_WKUP_IRQHandler(void)
{
    EXTI_ClrITPendBit(EXTI_LINE23);
}

/**
*\*\name    USB_HS_IRQHandler.
*\*\fun     This function handles USB_HS_IRQHandler Handler.
*\*\param   none
*\*\return  none
**/
void USB_HS_IRQHandler(void)
{
    /* USB Host 已移除, 此中断不应触发 */
}
