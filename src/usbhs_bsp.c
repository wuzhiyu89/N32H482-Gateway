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
*\*\file usbhs_bsp.c
*\*\author Nations
*\*\version v1.0.0
*\*\copyright Copyright (c) 2023, Nations Technologies Inc. All rights reserved.
**/

#include "usbhs_bsp.h"
#include "usbhs_core.h"
#include "n32h47x_48x_gpio.h"
#include "n32h47x_48x_rcc.h"
#include "n32h47x_48x_exti.h"
#include "misc.h"
#include "usbh_user.h"   /* USB_CORE_MODULE / USBH_HOST / __ALIGN_BEGIN */

/* ==================== USB 主机遗留全局对象 ====================
 * 背景: 4G 通讯已从 USB CDC 改为 USART2 串口直连模组 AT 口, USB 主机功能不再
 *       启用 —— 不调用 USBH_Init(), n32h47x_48x_it.c 里的 USB_HS_IRQHandler
 *       也已是空实现。
 *       但 usbh_user.c 里仍然写着
 *           extern USB_CORE_MODULE USBHS_Core;
 *           extern USBH_HOST       USB_Host;
 *       并在多处引用, 而工程里没有第二处定义 —— 不提供这两个符号就是:
 *           Error: L6218E: Undefined symbol USBHS_Core (referred from usbh_user.o)
 *
 * 放在 usbhs_bsp.c(USB 板级支持) 里, 是因为它【本来就在 Keil 和 EIDE 两个工程
 * 文件表里】, 不需要为新增文件去改工程配置。
 * 注意: 只提供符号, 不做任何初始化 —— USB 外设时钟没使能, 它俩就是普通 BSS
 *       内存, 无硬件副作用; 链接器最终还会把它们当未引用段删掉(不吃 Flash)。
 * 彻底清理: 把 usbh_*.c / usbhs_*.c 从工程里移出后, 这块也可以一起删。 */
__ALIGN_BEGIN USB_CORE_MODULE USBHS_Core __ALIGN_END;
USBH_HOST USB_Host;

/**
*\*\name   KEY_Init.
*\*\fun    Initialize a GPIO as KEY port.
*\*\param  none
*\*\return none
*/
void KEY_Init(void)
{
    GPIO_InitType GPIO_InitStructure;
    
    RCC_EnableAHB1PeriphClk(KEY_BUTTON_GPIO_CLK | KEY_BUTTON_UP_CLK | KEY_BUTTON_DOWN_CLK | KEY_BUTTON_LEFT_CLK | KEY_BUTTON_RIGHT_CLK| KEY_BUTTON_SEL_CLK, ENABLE);
    
    GPIO_InitStruct(&GPIO_InitStructure);

    GPIO_InitStructure.Pin            = KEY_BUTTON_PIN;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_INPUT;
    GPIO_InitStructure.GPIO_Pull      = GPIO_PULL_UP;
    GPIO_InitPeripheral(KEY_BUTTON_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin            = KEY_BUTTON_UP_PIN ;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_INPUT;
    GPIO_InitPeripheral(KEY_BUTTON_UP_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.Pin            = KEY_BUTTON_DOWN_PIN ;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_INPUT;
    GPIO_InitPeripheral(KEY_BUTTON_DOWN_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.Pin            = KEY_BUTTON_LEFT_PIN ;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_INPUT;
    GPIO_InitPeripheral(KEY_BUTTON_LEFT_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.Pin            = KEY_BUTTON_RIGHT_PIN ;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_INPUT;
    GPIO_InitPeripheral(KEY_BUTTON_RIGHT_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.Pin            = KEY_BUTTON_SEL_PIN ;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_INPUT;
    GPIO_InitStructure.GPIO_Pull      = GPIO_PULL_DOWN;
    GPIO_InitPeripheral(KEY_BUTTON_SEL_PORT, &GPIO_InitStructure);
}

/**
*\*\name   KEY_Press_Status_Get.
*\*\fun    Get key pressed or not status.
*\*\param  GPIOx: GPIO PORT
*\*\param  Pin: GPIO_PIN
*\*\return none
*/
FlagStatus KEY_Press_Status_Get(GPIO_Module* GPIOx, uint16_t Pin)
{
    if((GPIOx == KEY_BUTTON_SEL_PORT) && (Pin == KEY_BUTTON_SEL_PIN))
    {
        if(GPIO_ReadInputDataBit(GPIOx, Pin))
        {
            return SET;
        }
        else
        {
            return RESET;
        }
        
    }
    else
    {
        if(GPIO_ReadInputDataBit(GPIOx, Pin))
        {
            return RESET;
        }
        else
        {
            return SET;
        }
    }
}

/**
*\*\name    USB_BSP_Init.
*\*\fun     Initializes BSP configurations
*\*\param   none.
*\*\return  none.
*\*\
**/
void USB_BSP_Init(void)
{
    GPIO_InitType GPIO_InitStructure;
    GPIO_InitStruct(&GPIO_InitStructure);

    /* USB HS DP: PB15, Alternate Function (参考 usb_test 示例) */
    GPIO_InitStructure.Pin              = GPIO_PIN_15;
    GPIO_InitStructure.GPIO_Mode        = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate   = 14;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);

    /* USB HS DM: PB14, Alternate Function (参考 usb_test 示例) */
    GPIO_InitStructure.Pin              = GPIO_PIN_14;
    GPIO_InitStructure.GPIO_Mode        = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate   = 14;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);

    /* USBHS_ID: PB12, AF14 */
    GPIO_InitStructure.Pin              = GPIO_PIN_12;
    GPIO_InitStructure.GPIO_Mode        = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate   = 14;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);

    /* ★★ 危险: 下面那句 ML307R_RST_Init() 会操作 PB13, 而实板上
     *    【PB13 = BAT_STDBY(充满电拉低)、PB12 = BAT_CHRG(充电时拉低)】,
     *    两个都是电池状态【输入】信号 —— 驱动它们会干扰电池检测电路。
     *    4G 模组的上下电与 USB 无关, 只有 PA8 一根脚(4G_RST, 低电平上电,
     *    见 app/bsp/bsp_gpio.h 的 G4_PWR_SW_*)。
     *    USB_BSP_Init() 目前无人调用(USB 方案已放弃), 所以这句本来也不会执行;
     *    但为了"以后恢复 USB 时不会踩雷", 这里直接【编译期删掉】。
     *    (2026-09 现场确认引脚表后加的保险, 不要轻易放开。) */
    /* ML307R_RST_Init();  —— 已删除: 会驱动 PB13 = BAT_STDBY */
}

/**
*\*\name    USB_BSP_EnableInterrupt.
*\*\fun     Enable USB HS interrupt
*\*\param   none.
*\*\return  none.
*\*\
**/
void USB_BSP_EnableInterrupt(USB_CORE_MODULE *USBx)
{
    NVIC_InitType NVIC_InitStructure;
    EXTI_InitType EXTI_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitStructure.NVIC_IRQChannel = USB_HS_WKUP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);     

    NVIC_InitStructure.NVIC_IRQChannel = USB_HS_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    /* Configure the EXTI line 23 connected internally to the USBHS IP */
    EXTI_ClrITPendBit(EXTI_LINE23);
    EXTI_InitStructure.EXTI_Line    = EXTI_LINE23; 
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitPeripheral(&EXTI_InitStructure);
}

/**
*\*\name    USB_BSP_DriveVBUS.
*\*\fun     Drives the Vbus signal through IO
*\*\param   none.
*\*\return  none.
**/
// USB_BSP_DriveVBUS() 保持不变（它已被调用但 PE1 驱动在我们场景下无意义，可保留空或不变）
// 如果不想改动太多，可以保持不变，因为 PE1 不会影响你的硬件
void USB_BSP_DriveVBUS(USB_CORE_MODULE * pdev, uint8_t state)
{
    /* ML307R 直连，不需要 VBUS 驱动 */
}

/**
*\*\name    USB_BSP_ConfigVBUS.
*\*\fun     Configures the IO for the Vbus
*\*\param   none.
*\*\return  none.
**/
void USB_BSP_ConfigVBUS(USB_CORE_MODULE * pdev)
{
    /* ML307R 直连，不需要 VBUS 驱动，此函数保留为空 */
}




/**
*\*\name    USB_BSP_uDelay.
*\*\fun     This function provides delay time in micro sec
*\*\param   usec : Value of delay required in micro sec.
*\*\return  none.
*\*\
**/
void USB_BSP_uDelay(const uint32_t usec)
{
    volatile uint32_t i;
    uint32_t j;
    j = 30 * usec;
    for (i = 0UL; i < j; i++)
    {
        __NOP();  // 顺便加一个 nop，防止极端优化
    }
}

/**
*\*\name    USB_BSP_mDelay.
*\*\fun     This function provides delay time in milli sec
*\*\param   usec : Value of delay required in milli sec.
*\*\return  none.
*\*\
**/
void USB_BSP_mDelay (const uint32_t msec)
{
    USB_BSP_uDelay(msec * 1000);
}

/* ==================================================================
 * 【已废弃 + 已上锁】ML307R 的"复位/上下电"三件套
 *
 * 它们原来驱动 PB13(推挽输出, 低电平使能)给 ML307R 上下电。
 *
 * 实板引脚表(2026-09 现场确认, 见 app/bsp/bsp_gpio.h 顶部的整板引脚表):
 *     PB13 = BAT_STDBY(电池充满时拉低)   PB12 = BAT_CHRG(充电时拉低)
 *   两个都是【电池状态输入信号】—— 把 PB13 配成推挽输出并拉低, 会直接干扰
 *   电池检测电路。
 *   4G 模组的上下电只有一根脚: PA8 = 4G_RST【低电平上电, 高电平断电】,
 *   由 app/bsp/bsp_gpio.c 的 G4_PowerSwitch_*() 负责。
 *
 * 所以这里把三个函数做成【空实现】: 符号保留(不破坏已有引用), 但绝不会再
 * 碰任何引脚。要恢复旧板/USB 方案时, 先确认 PB13 在那块板上的实际用途。
 * ================================================================== */

/**
*\*\name    ML307R_RST_Init.
*\*\fun    【已停用, 空实现】原: 初始化 ML307R 复位脚(PB13) —— PB13 现为 BAT_STDBY.
*/
void ML307R_RST_Init(void)
{
    /* 故意什么都不做: 见上面的说明 */
}

/**
*\*\name    ML307R_PowerOn.
*\*\fun    【已停用, 空实现】原: 把 PB13 拉低给模组上电 —— PB13 现为 BAT_STDBY.
*/
void ML307R_PowerOn(void)
{
    /* 故意什么都不做: 4G 上电请用 G4_PowerSwitch_Set(1) */
}

/**
*\*\name    ML307R_PowerOff.
*\*\fun    【已停用, 空实现】原: 把 PB13 拉高给模组断电 —— PB13 现为 BAT_STDBY.
*/
void ML307R_PowerOff(void)
{
    /* 故意什么都不做: 4G 断电请用 G4_PowerSwitch_Set(0) */
}