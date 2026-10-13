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
*\*\file usbh_user.h
*\*\author Nations
*\*\version v1.0.0
*\*\copyright Copyright (c) 2023, Nations Technologies Inc. All rights reserved.
**/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USH_USR_H__
#define __USH_USR_H__


/* Includes ------------------------------------------------------------------*/
#include "usbh_core.h"
#include "usbhs_conf.h"
#include <stdio.h>

#define LINE_01                           (LCD_PIXEL_HEIGHT - 36)
#define LINE_02                           (LCD_PIXEL_HEIGHT - 24)
#define LINE_03                           (LCD_PIXEL_HEIGHT - 12) 

#define NB_BAUD_RATES                     10
#define NB_OF_FILES                       3

typedef enum 
{
    CDC_DEMO_IDLE   = 0,
    CDC_DEMO_WAIT,  
    CDC_DEMO_SEND,
    CDC_DEMO_RECEIVE,
    CDC_DEMO_CONFIGURATION,
}CDC_Demo_State;

typedef enum 
{
    CDC_SEND_IDLE   = 0,
    CDC_SEND_WAIT,
    CDC_SEND_FILE1,
    CDC_SEND_FILE2,
    CDC_SEND_FILE3,
}CDC_Send_State;

typedef enum 
{
    CDC_RECEIVE_IDLE   = 0,
    CDC_RECEIVE_WAIT,
    CDC_RECEIVE_RECEIVE,
}CDC_Receive_State;


typedef enum 
{
    CDC_CONFIGURATION_IDLE   = 0,
    CDC_CONFIGURATION_WAIT,
    CDC_CONFIGURATION_SET_BAUD_RATE,
    CDC_CONFIGURATION_SET_DATA_BITS,
    CDC_CONFIGURATION_SET_PARITY,
    CDC_CONFIGURATION_SET_STOP_BIT,
}CDC_Configuration_State;

typedef struct _DemoStateMachine
{
  
  __IO CDC_Demo_State              state;
  __IO CDC_Send_State              Send_state;
  __IO CDC_Receive_State           Receive_state;
  __IO CDC_Configuration_State     Configuration_state;
  __IO uint8_t                     select;
  __IO uint8_t                     lock;
  
}CDC_DEMO_StateMachine;

typedef struct _DemoSettings
{
    __IO uint8_t                     BaudRateIdx;
    __IO uint8_t                     DataBitsIdx;
    __IO uint8_t                     ParityIdx;
    __IO uint8_t                     StopBitsIdx;
}CDC_DEMO_Settings;

typedef struct _DemoSettingStateMachine
{
    CDC_DEMO_Settings                settings;
    __IO uint8_t                     select;
    __IO uint8_t                     lock;
}CDC_DEMO_SETTING_StateMachine;

typedef enum 
{
    CDC_SELECT_MENU   = 0,
    CDC_SELECT_CONFIG,    
}CDC_DEMO_SelectMode;

typedef enum
{
    BUTTON_NULL = 1u,
    BUTTON_RIGHT,
    BUTTON_LEFT,
    BUTTON_UP,
    BUTTON_DOWN,
    BUTTON_SEL,
}Button_TypeDef;

extern  USBH_User_cb_TypeDef USER_cb;


void USBH_USER_ApplicationSelected(void);
void USBH_USER_Init(void);
void USBH_USER_DeInit(void);
void USBH_USER_DeviceAttached(void);
void USBH_USER_ResetDevice(void);
void USBH_USER_DeviceDisconnected (void);
void USBH_USER_OverCurrentDetected (void);
void USBH_USER_DeviceSpeedDetected(uint8_t DeviceSpeed); 
void USBH_USER_Device_DescAvailable(void *);
void USBH_USER_DeviceAddressAssigned(void);
void USBH_USER_Configuration_DescAvailable(USBH_CfgDesc_TypeDef * cfgDesc,
                                          USBH_InterfaceDesc_TypeDef *itfDesc,
                                          USBH_EpDesc_TypeDef *epDesc);
void USBH_USER_Manufacturer_String(void *);
void USBH_USER_Product_String(void *);
void USBH_USER_SerialNum_String(void *);
void USBH_USER_EnumerationDone(void);
USBH_USER_Status USBH_USER_UserInput(void);
int USBH_USER_Application(void);
void USBH_USER_DeInit(void);
void USBH_USER_DeviceNotSupported(void);
void USBH_USER_UnrecoveredError(void);

void CDC_DEMO_ProbeKey (Button_TypeDef state);

#endif /* __USH_USR_H__ */ 
