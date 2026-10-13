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
*\*\file usbhs_bsp.h
*\*\author Nations
*\*\version v1.0.0
*\*\copyright Copyright (c) 2023, Nations Technologies Inc. All rights reserved.
**/
#ifndef __USBHS_BSP__H__
#define __USBHS_BSP__H__

#include "usbhs_core.h"

#define KEY_BUTTON_PIN                   GPIO_PIN_8
#define KEY_BUTTON_GPIO_PORT             GPIOC
#define KEY_BUTTON_GPIO_CLK              RCC_AHB_PERIPHEN_GPIOC

#define KEY_BUTTON_UP_PIN                GPIO_PIN_13
#define KEY_BUTTON_UP_PORT               GPIOC
#define KEY_BUTTON_UP_CLK                RCC_AHB_PERIPHEN_GPIOC

#define KEY_BUTTON_DOWN_PIN              GPIO_PIN_15
#define KEY_BUTTON_DOWN_PORT             GPIOA
#define KEY_BUTTON_DOWN_CLK              RCC_AHB_PERIPHEN_GPIOA

#define KEY_BUTTON_LEFT_PIN              GPIO_PIN_6
#define KEY_BUTTON_LEFT_PORT             GPIOC
#define KEY_BUTTON_LEFT_CLK              RCC_AHB_PERIPHEN_GPIOC

#define KEY_BUTTON_RIGHT_PIN             GPIO_PIN_7
#define KEY_BUTTON_RIGHT_PORT            GPIOC
#define KEY_BUTTON_RIGHT_CLK             RCC_AHB_PERIPHEN_GPIOC

#define KEY_BUTTON_SEL_PIN               GPIO_PIN_0
#define KEY_BUTTON_SEL_PORT              GPIOA
#define KEY_BUTTON_SEL_CLK               RCC_AHB_PERIPHEN_GPIOA


void USB_BSP_Init(void);
void USB_BSP_uDelay(const uint32_t usec);
void USB_BSP_mDelay(const uint32_t msec);
void USB_BSP_EnableInterrupt(USB_CORE_MODULE *USBx);
FlagStatus KEY_Press_Status_Get(GPIO_Module* GPIOx, uint16_t Pin);

#ifdef USE_HOST_MODE

/* ==================================================================
 * ⚠️【下面这组宏/函数已废弃, 且不要在实板上使用】
 *
 * 旧板: PB13 是 ML307R 的上下电控制脚(低=工作)。
 * 实板(2026-09 现场确认): PB13 = BAT_STDBY(电池充满时拉低, 输入信号),
 *                         PB12 = BAT_CHRG(充电时拉低, 输入信号)。
 * → 驱动 PB13 会干扰电池检测电路!
 *
 * 4G 模组的上下电只有 PA8 一根脚: 4G_RST【低电平上电, 高电平断电】,
 * 由 app/bsp/bsp_gpio.c 的 G4_PowerSwitch_*() 负责。整板引脚表见
 * app/bsp/bsp_gpio.h 顶部。
 *
 * 这三个函数现在在 src/usbhs_bsp.c 里是【空实现】(符号保留, 不动引脚)。
 * ================================================================== */
#define ML307R_RST_PORT             GPIOB      /* = BAT_STDBY, 勿用 */
#define ML307R_RST_PIN              GPIO_PIN_13/* = BAT_STDBY, 勿用 */
#define ML307R_RST_CLK              RCC_AHB_PERIPHEN_GPIOB

void ML307R_RST_Init(void);     /* 已停用(空实现) */
void ML307R_PowerOn(void);      /* 已停用(空实现) */
void ML307R_PowerOff(void);     /* 已停用(空实现) */

void USB_BSP_ConfigVBUS(USB_CORE_MODULE *pdev);
void USB_BSP_DriveVBUS(USB_CORE_MODULE *pdev,uint8_t state);
void USB_BSP_Resume(USB_CORE_MODULE *pdev) ;
void USB_BSP_Suspend(USB_CORE_MODULE *pdev);

#endif /* USE_HOST_MODE */

#endif /* __USBHS_BSP__H__ */


