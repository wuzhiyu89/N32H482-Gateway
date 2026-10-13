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
*\*\file usbh_user.c
*\*\author Nations
*\*\version v1.0.0
*\*\copyright Copyright (c) 2023, Nations Technologies Inc. All rights reserved.
**/

#include "usbh_user.h"
#include "usbh_fs.h"
#include "usbh_data.h"
#include "usbh_cdc_core.h"
#include "n32h47x_48x_gpio.h"
#include "log.h"
#include "g4_io.h"   /* g4_io_receive_cb:把 CDC 下行数据灌入 AT 接收缓冲 */

extern USB_CORE_MODULE USBHS_Core;;
extern USBH_HOST USB_Host;
extern CDC_LineCodingTypeDef CDC_GetLineCode;
extern CDC_LineCodingTypeDef CDC_SetLineCode;
extern CDC_Usercb_TypeDef UserCb;

uint8_t prev_select = 0;
uint8_t enable_display_received_data = 0;
CDC_DEMO_StateMachine cdc_demo;
CDC_DEMO_SETTING_StateMachine cdc_settings_state;
CDC_DEMO_SelectMode cdc_select_mode;

static uint8_t Prev_BaudRateIdx = 0;
static uint8_t Prev_DataBitsIdx = 0;
static uint8_t Prev_ParityIdx = 0;
static uint8_t Prev_StopBitsIdx = 0;
static uint8_t Prev_Select = 0;

uint8_t *CDC_MAIN_MENU[] = 
{
    (uint8_t *) "      1 - Send Data                                                          \r\n",
    (uint8_t *) "      2 - Receive Data                                                       \r\n",
    (uint8_t *) "      3 - Configuration                                                      \r\n",
};

uint8_t *DEMO_SEND_MENU[] = 
{
    (uint8_t *) "      1 - Send File 1                                                        \r\n",
    (uint8_t *) "      2 - Send File 2                                                        \r\n",
    (uint8_t *) "      3 - Send File 3                                                        \r\n",
    (uint8_t *) "      4 - Return                                                             \r\n",
};

uint8_t *DEMO_RECEIVE_MENU[] = 
{
    (uint8_t *) "      1 - Return                                                             \r\n",
    (uint8_t *) "                                                                             \r\n",
    (uint8_t *) "                                                                             \r\n",
};

uint8_t *DEMO_CONFIGURATION_MENU[] = 
{
    (uint8_t *) "      1 - Apply new settings                                                 \r\n",
    (uint8_t *) "      2 - Return                                                             \r\n",
    (uint8_t *) "                                                                             \r\n",
};

const uint8_t MSG_BITS_PER_SECOND[] =   "      : Bit Per Second                          ";
const uint8_t MSG_DATA_BITS[] =         "      : Data width                              ";
const uint8_t MSG_PARITY[] =            "      : Parity                                  ";
const uint8_t MSG_STOP_BITS[] =         "      : Stop Bits                               ";

const uint32_t BaudRateValue[NB_BAUD_RATES] = { 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600 };

const uint8_t DataBitsValue[4] = { 5, 6, 7, 8 };
const uint8_t ParityArray[3][5] = { "NONE", "EVEN", "ODD" };
const uint8_t StopBitsArray[2][2] = { "1", "2" };

USBH_User_cb_TypeDef USER_cb = 
{
    USBH_USER_Init,
    USBH_USER_DeInit,
    USBH_USER_DeviceAttached,
    USBH_USER_ResetDevice,
    USBH_USER_DeviceDisconnected,
    USBH_USER_OverCurrentDetected,
    USBH_USER_DeviceSpeedDetected,
    USBH_USER_Device_DescAvailable,
    USBH_USER_DeviceAddressAssigned,
    USBH_USER_Configuration_DescAvailable,
    USBH_USER_Manufacturer_String,
    USBH_USER_Product_String,
    USBH_USER_SerialNum_String,
    USBH_USER_EnumerationDone,
    USBH_USER_UserInput,
    USBH_USER_Application,
    USBH_USER_DeviceNotSupported,
    USBH_USER_UnrecoveredError
};

/*--------------- Print Messages ---------------*/
uint8_t MSG_HOST_INIT[] = "> Host Library Initialized\n";
uint8_t MSG_DEV_ATTACHED[] = "> Device Attached\n";
uint8_t MSG_DEV_DISCONNECTED[] = "> Device Disconnected\n";
uint8_t MSG_DEV_ENUMERATED[] = "> Enumeration completed\n";
uint8_t MSG_DEV_HIGHSPEED[] = "> High speed device detected\n";
uint8_t MSG_DEV_FULLSPEED[] = "> Full speed device detected\n";
uint8_t MSG_DEV_LOWSPEED[] = "> Low speed device detected\n";
uint8_t MSG_DEV_ERROR[] = "> Device fault\n";
uint8_t MSG_CDC_CLASS[] = "> CDC device connected\n";
uint8_t MSG_UNREC_ERROR[] = "> Unrecovered error\n";

static void CDC_SelectItem(uint8_t ** menu, uint8_t item);
static void CDC_Handle_Menu(void);
static void CDC_Handle_Send_Menu(void);
static void CDC_Handle_Receive_Menu(void);
static void CDC_Handle_Configuration_Menu(void);
static void CDC_ChangeSelectMode(CDC_DEMO_SelectMode select_mode);
static void CDC_AdjustSettingMenu(CDC_DEMO_SETTING_StateMachine *settings_state);
static void CDC_SelectSettingsItem(uint8_t item);
static void CDC_OutputData(uint8_t * ptr);
static void CDC_SetInitialValue(void);


/**
*\*\name    USBH_USER_Init.
*\*\fun     Printf the message for host lib initialization
*\*\param   none.
*\*\return  none.
*\*\
**/
void USBH_USER_Init(void)
{
    static uint8_t startup = 0;

    if (startup == 0)
    {
        startup = 1;
        log_info("> USB Host library started.\r\n");
    }
}

/**
*\*\name    USBH_USER_DeviceAttached.
*\*\fun     Printf the message for host lib initialization
*\*\param   none.
*\*\return  none.
*\*\
**/
void USBH_USER_DeviceAttached(void)
{
    log_info((void *)MSG_DEV_ATTACHED);
}

/**
*\*\name    USBH_USER_UnrecoveredError.
*\*\fun     Printf the message of unrecover error
*\*\param   none.
*\*\return  none.
*\*\
**/
void USBH_USER_UnrecoveredError(void)
{
    log_info((void *)MSG_UNREC_ERROR);
}

/**
*\*\name    USBH_USER_DeviceDisconnected.
*\*\fun     Printf the message of device disconnect event
*\*\param   none.
*\*\return  none.
*\*\
**/
void USBH_USER_DeviceDisconnected(void)
{
    log_info((void *)MSG_DEV_DISCONNECTED);
}

/**
*\*\name    USBH_USER_ResetDevice.
*\*\fun     Reset USB Device
*\*\param   none.
*\*\return  none.
*\*\
**/
void USBH_USER_ResetDevice(void)
{
  /* Users can do their application actions here for the USB-Reset */
}

/**
*\*\name    USBH_USER_DeviceSpeedDetected.
*\*\fun     Print the message for device speed
*\*\param   Devicespeed : Device Speed.
*\*\return  none.
*\*\
**/
void USBH_USER_DeviceSpeedDetected(uint8_t DeviceSpeed)
{
    if (DeviceSpeed == HPCS_PRTSPD_HIGH_SPEED)
    {                                                
        log_info((void *)MSG_DEV_HIGHSPEED);
    }
    else if (DeviceSpeed == HPCS_PRTSPD_FULL_SPEED)
    {
        log_info((void *)MSG_DEV_FULLSPEED);
    }
    else if (DeviceSpeed == HPCS_PRTSPD_LOW_SPEED)
    {
        log_info((void *)MSG_DEV_LOWSPEED);
    }
    else
    {
        log_info((void *)MSG_DEV_ERROR);
    }
}

/**
*\*\name    USBH_USER_Device_DescAvailable.
*\*\fun     Print the message for device speed
*\*\param   Devicespeed : device descriptor.
*\*\return  none.
*\*\
**/
void USBH_USER_Device_DescAvailable(void *DeviceDesc)
{
    uint8_t temp[50];
    USBH_DevDesc_TypeDef *hs;
    hs = DeviceDesc;

    sprintf((char *)temp, "VID : %04lXh\n", (uint32_t) (*hs).idVendor);
    log_info((char *)temp);
    sprintf((char *)temp, "PID : %04lXh\n", (uint32_t) (*hs).idProduct);
    log_info((char *)temp); 
}

/**
*\*\name    USBH_USER_DeviceAddressAssigned.
*\*\fun     USB device is successfully assigned the Address
*\*\param   none.
*\*\return  none.
*\*\
**/
void USBH_USER_DeviceAddressAssigned(void)
{

}

/**
*\*\name    USBH_USER_Configuration_DescAvailable.
*\*\fun     Print the message for configuration descriptor
*\*\param   ConfDesc : Configuration descriptor.
*\*\param   itfDesc : interface descriptor.
*\*\param   epDesc : Endpoint descriptor.
*\*\return  none.
*\*\
**/
void USBH_USER_Configuration_DescAvailable(USBH_CfgDesc_TypeDef * cfgDesc, USBH_InterfaceDesc_TypeDef * itfDesc, USBH_EpDesc_TypeDef * epDesc)
{
    USBH_InterfaceDesc_TypeDef *id;

    id = itfDesc;

    if ((*id).bInterfaceClass == COMMUNICATION_DEVICE_CLASS_CODE)
    {
        /* ASR 模块 Protocol=0x00, 修正为 COMMON_AT_COMMAND(0x01) 以兼容 CDC 驱动 */
        if ((*id).bInterfaceProtocol == 0x00)
        {
            (*id).bInterfaceProtocol = COMMON_AT_COMMAND;
        }
        log_info((void *)MSG_CDC_CLASS);
    }
    else
    {
        log_info("USB Class NOT supported");
    }
}

/**
*\*\name    USBH_USER_Manufacturer_String.
*\*\fun     Print the message for Manufacturer String
*\*\param   ManufacturerString : Manufacturer String of Device
*\*\return  none.
*\*\
**/
void USBH_USER_Manufacturer_String(void *ManufacturerString)
{
    char temp[100];
    sprintf(temp, "Manufacturer : %s\n", (char *)ManufacturerString);
    log_info((char *)temp);

}

/**
*\*\name    USBH_USER_Product_String.
*\*\fun     Print the message for Product String
*\*\param   ProductString : Product String of Device
*\*\return  none.
*\*\
**/
void USBH_USER_Product_String(void *ProductString)
{
    char temp[100];
    sprintf((char *)temp, "Product : %s\n", (char *)ProductString);
    log_info((char *)temp);

}

/**
*\*\name    USBH_USER_SerialNum_String.
*\*\fun     Print the message for SerialNum String
*\*\param   SerialNumString : SerialNum_String of device
*\*\return  none.
*\*\
**/
void USBH_USER_SerialNum_String(void *SerialNumString)
{
    uint8_t temp[100];
    sprintf((char *)temp, "Serial Number : %s\n", (char *)SerialNumString);
    log_info((char *)temp);
}

/**
*\*\name    USBH_USER_EnumerationDone.
*\*\fun     Print the message for Enumeration complete
*\*\param   none.
*\*\return  none.
*\*\
**/
void USBH_USER_EnumerationDone(void)
{
    /* Enumeration complete */
    log_info((void *)MSG_DEV_ENUMERATED);
    /* 把 CDC 下行数据交给 g4_io 接收缓冲,并启动后台接收 */
    UserCb.Receive = g4_io_receive_cb;
    CDC_StartReception(&USBHS_Core);
}

/**
*\*\name    USBH_USER_DeviceNotSupported.
*\*\fun     Print the message for Device is not supported
*\*\param   none.
*\*\return  none.
*\*\
**/
void USBH_USER_DeviceNotSupported(void)
{
    log_info("> Device not supported.\n");

}

/**
*\*\name    USBH_USER_UserInput.
*\*\fun     User Action for application state entry
*\*\param   none.
*\*\return  USBH_USER_Status : User response for key button.
*\*\
**/
USBH_USER_Status USBH_USER_UserInput(void)
{
    CDC_SelectItem(CDC_MAIN_MENU, 0);
    return USBH_USER_RESP_OK;
}

/**
*\*\name    USBH_USER_OverCurrentDetected.
*\*\fun     Print the message for Device Overcurrent detection event
*\*\param   none.
*\*\return  none.
*\*\
**/
void USBH_USER_OverCurrentDetected(void)
{
    log_info("Overcurrent detected.\n");

}

/**
*\*\name    USBH_USER_DeInit.
*\*\fun     Deint User state and associated variables
*\*\param   none.
*\*\return  none.
*\*\
**/
void USBH_USER_DeInit(void)
{
}

/**
*\*\name    Key_ReadIOPin_continuous.
*\*\fun     Key status read function.
*\*\param   none.
*\*\return  button id.
*\*\
**/
Button_TypeDef Key_ReadIOPin_continuous(void)
{
    Button_TypeDef enKey = BUTTON_NULL;
    Button_TypeDef present_key = BUTTON_NULL;
    
    if(KEY_Press_Status_Get(KEY_BUTTON_UP_PORT, KEY_BUTTON_UP_PIN))
    {
        enKey = BUTTON_UP;
    }
    else if(KEY_Press_Status_Get(KEY_BUTTON_DOWN_PORT, KEY_BUTTON_DOWN_PIN))
    {
        enKey = BUTTON_DOWN;
    }
    else if(KEY_Press_Status_Get(KEY_BUTTON_LEFT_PORT, KEY_BUTTON_LEFT_PIN))
    {
        enKey = BUTTON_LEFT;
    }
    else if(KEY_Press_Status_Get(KEY_BUTTON_RIGHT_PORT, KEY_BUTTON_RIGHT_PIN))
    {
        enKey = BUTTON_RIGHT;
    }
    else if(KEY_Press_Status_Get(KEY_BUTTON_SEL_PORT, KEY_BUTTON_SEL_PIN))
    {
        enKey = BUTTON_SEL;
    }
    else
    {
        enKey = BUTTON_NULL;
    }
    
    if(enKey != BUTTON_NULL)
    {
        present_key = enKey;
        
        USB_BSP_mDelay(20);
        
        if(KEY_Press_Status_Get(KEY_BUTTON_UP_PORT, KEY_BUTTON_UP_PIN))
        {
            enKey = BUTTON_UP;
        }
        else if(KEY_Press_Status_Get(KEY_BUTTON_DOWN_PORT, KEY_BUTTON_DOWN_PIN))
        {
            enKey = BUTTON_DOWN;
        }
        else if(KEY_Press_Status_Get(KEY_BUTTON_LEFT_PORT, KEY_BUTTON_LEFT_PIN))
        {
            enKey = BUTTON_LEFT;
        }
        else if(KEY_Press_Status_Get(KEY_BUTTON_RIGHT_PORT, KEY_BUTTON_RIGHT_PIN))
        {
            enKey = BUTTON_RIGHT;
        }
        else if(KEY_Press_Status_Get(KEY_BUTTON_SEL_PORT, KEY_BUTTON_SEL_PIN))
        {
            enKey = BUTTON_SEL;
        }
        else
        {
            enKey = BUTTON_NULL;
        }
        
        if(present_key != enKey)
        {
            enKey = BUTTON_NULL;
        }
    }
    if(enKey != BUTTON_NULL)
    {
        while(1)
        {
            if(KEY_Press_Status_Get(KEY_BUTTON_UP_PORT, KEY_BUTTON_UP_PIN))
            {
                present_key = BUTTON_UP;
            }
            else if(KEY_Press_Status_Get(KEY_BUTTON_DOWN_PORT, KEY_BUTTON_DOWN_PIN))
            {
                present_key = BUTTON_DOWN;
            }
            else if(KEY_Press_Status_Get(KEY_BUTTON_LEFT_PORT, KEY_BUTTON_LEFT_PIN))
            {
                enKey = BUTTON_LEFT;
            }
            else if(KEY_Press_Status_Get(KEY_BUTTON_RIGHT_PORT, KEY_BUTTON_RIGHT_PIN))
            {
                enKey = BUTTON_RIGHT;
            }
            else if(KEY_Press_Status_Get(KEY_BUTTON_SEL_PORT, KEY_BUTTON_SEL_PIN))
            {
                present_key = BUTTON_SEL;
            }
            else
            {
                present_key = BUTTON_NULL;
                break;
            }
        }
    }
    return enKey;
}


/**
*\*\name    USBH_USER_Application.
*\*\fun     CDC Main application
*\*\param   none.
*\*\return  0.
*\*\
**/
int USBH_USER_Application(void)
{
    return 0;
}

/**
*\*\name    CDC_DEMO_ProbeKey.
*\*\fun     Probe the joystick state
*\*\param   state : joystick state.
*\*\return  0.
*\*\
**/
void CDC_DEMO_ProbeKey(Button_TypeDef state)
{ 
    if (cdc_select_mode == CDC_SELECT_MENU)
    {
        /* Handle Menu inputs */
        if ((state == BUTTON_UP) && (cdc_demo.select > 0))
        {
            cdc_demo.select--;
        }
        else if (((cdc_demo.state == CDC_DEMO_WAIT) && (state == BUTTON_DOWN) && (cdc_demo.select < 2)) || 
            ((cdc_demo.state == CDC_DEMO_SEND) && (state == BUTTON_DOWN) && (cdc_demo.select < 3)) ||
            ((cdc_demo.state == CDC_DEMO_CONFIGURATION) && (state == BUTTON_DOWN) && (cdc_demo.select < 1)))
        {
            cdc_demo.select++;
        }
        else if (state == BUTTON_SEL)
        {
            cdc_demo.select |= 0x80;
        }
    }
    else
    {
        /* Handle Configuration inputs */
        /**** Vertical selection ****/
        if ((state == BUTTON_UP) && (cdc_settings_state.select > 0))
        {
            cdc_settings_state.select--;
        }
        else if ((state == BUTTON_DOWN) && (cdc_settings_state.select < 3))
        {
            cdc_settings_state.select++;
        }
        /**** Horizontal selection ****/
        else if (state == BUTTON_RIGHT)
        {
            if ((cdc_settings_state.select == 0) && (cdc_settings_state.settings.BaudRateIdx < 9))
            {
                cdc_settings_state.settings.BaudRateIdx++;
            }
            if ((cdc_settings_state.select == 1) && (cdc_settings_state.settings.DataBitsIdx < 3))
            {
                cdc_settings_state.settings.DataBitsIdx++;
            }
            if ((cdc_settings_state.select == 2) && (cdc_settings_state.settings.ParityIdx < 2))
            {
                cdc_settings_state.settings.ParityIdx++;
            }
            if ((cdc_settings_state.select == 3) && (cdc_settings_state.settings.StopBitsIdx < 1))
            {
                cdc_settings_state.settings.StopBitsIdx++;
            }
        }
        else if (state == BUTTON_LEFT)
        {
            if ((cdc_settings_state.select == 0) && (cdc_settings_state.settings.BaudRateIdx > 0))
            {
                cdc_settings_state.settings.BaudRateIdx--;
            }
            if ((cdc_settings_state.select == 1) && (cdc_settings_state.settings.DataBitsIdx > 0))
            {
                cdc_settings_state.settings.DataBitsIdx--;
            }
            if ((cdc_settings_state.select == 2) && (cdc_settings_state.settings.ParityIdx > 0))
            {
                cdc_settings_state.settings.ParityIdx--;
            }
            if ((cdc_settings_state.select == 3) && (cdc_settings_state.settings.StopBitsIdx > 0))
            {
                cdc_settings_state.settings.StopBitsIdx--;
            }
        }
    }
}

void DisplayStringLine(uint8_t *ptr)
{
    /* Send the string character by character */
    log_info((char *)ptr);
}


/**
*\*\name    CDC_SelectItem.
*\*\fun     manage the menu
*\*\param   menu: menu table
*\*\param   item : selected item to be highlighted
*\*\return  0.
*\*\
**/
static void CDC_SelectItem(uint8_t ** menu, uint8_t item)
{
    switch (item)
    {
        case 0:
            log_info(" -->");
            DisplayStringLine(menu[0]);
            DisplayStringLine(menu[1]);
            DisplayStringLine(menu[2]);
            break;
        
        case 1:
            DisplayStringLine(menu[0]);
            log_info(" -->");
            DisplayStringLine(menu[1]);
            DisplayStringLine(menu[2]);
            break;
        
        case 2:
            DisplayStringLine(menu[0]);
            DisplayStringLine(menu[1]);
            log_info(" -->");
            DisplayStringLine(menu[2]);
            break;
        
        case 3:
            DisplayStringLine(menu[1]);
            DisplayStringLine(menu[2]);
            log_info(" -->");
            DisplayStringLine(menu[3]);
            break;
        
        case 4:
            DisplayStringLine(menu[2]);
            DisplayStringLine(menu[3]);
            log_info(" -->");
            DisplayStringLine(menu[4]);
            break;
        
        default:
            DisplayStringLine(menu[0]);
            DisplayStringLine(menu[1]);
            DisplayStringLine(menu[2]);
            break;
    }
}

/**
*\*\name    CDC_Handle_Menu.
*\*\fun     Demo state machine
*\*\param   none.
*\*\return  0.
*\*\
**/
static void CDC_Handle_Menu(void)
{
    static uint8_t debounce = 0;

    switch (cdc_demo.state)
    {
        case CDC_DEMO_IDLE:
            CDC_SelectItem(CDC_MAIN_MENU, 0);
            cdc_demo.state = CDC_DEMO_WAIT;
            cdc_demo.select = 0;
            log_info("> Select an operation to start.\n");
            CDC_SetInitialValue();
            break;

        case CDC_DEMO_WAIT:
            if (cdc_demo.select != prev_select)
            {
                prev_select = cdc_demo.select;
                CDC_SelectItem(CDC_MAIN_MENU, cdc_demo.select & 0x7F);

                /* Handle select item */
                if (cdc_demo.select & 0x80)
                {
                    cdc_demo.select &= 0x7F;

                    switch (cdc_demo.select)
                    {
                        case 0:
                          cdc_demo.state = CDC_DEMO_SEND;
                          cdc_demo.Send_state = CDC_SEND_IDLE;
                          break;

                        case 1:
                          cdc_demo.state = CDC_DEMO_RECEIVE;
                          cdc_demo.Receive_state = CDC_RECEIVE_IDLE;
                          break;

                        case 2:
                          cdc_demo.state = CDC_DEMO_CONFIGURATION;
                          cdc_demo.Configuration_state = CDC_CONFIGURATION_IDLE;
                          break;
                        
                        default:
                          break;
                    }
                }
            }
            break;

        case CDC_DEMO_SEND:
            CDC_Handle_Send_Menu();
            break;

        case CDC_DEMO_RECEIVE:
            CDC_Handle_Receive_Menu();
            break;
        case CDC_DEMO_CONFIGURATION:
            CDC_Handle_Configuration_Menu();
            break;

        default:
            break;
    }

    if (cdc_demo.state == CDC_DEMO_CONFIGURATION)
    {
        if (GPIO_ReadInputDataBit(KEY_BUTTON_GPIO_PORT, KEY_BUTTON_PIN) == SET)
        {
            if (debounce == 0)
            {
                if (cdc_select_mode == CDC_SELECT_MENU)
                {
                    cdc_select_mode = CDC_SELECT_CONFIG;
                }
                else
                {
                    cdc_select_mode = CDC_SELECT_MENU;
                }

                CDC_ChangeSelectMode(cdc_select_mode);
                debounce = 1;
            }
        }
        else
        {
            debounce = 0;
        }
        if (cdc_select_mode == CDC_SELECT_CONFIG)
        {
            CDC_AdjustSettingMenu(&cdc_settings_state);
        }
    }
}



/**
*\*\name    CDC_Handle_Send_Menu.
*\*\fun     Handle Send Menu
*\*\param   none.
*\*\return  none.
*\*\
**/
static void CDC_Handle_Send_Menu(void)
{
    struct fs_file file = { 0, 0 };

    switch (cdc_demo.Send_state)
    {
        case CDC_SEND_IDLE:
            cdc_demo.Send_state = CDC_SEND_WAIT;
            CDC_SelectItem(DEMO_SEND_MENU, 0);
            cdc_demo.select = 0;
            log_info("> Select a file to send.\n");
            break;

        case CDC_SEND_WAIT:
            if (cdc_demo.select != prev_select)
            {
                prev_select = cdc_demo.select;
                CDC_SelectItem(DEMO_SEND_MENU, cdc_demo.select & 0x7F);

                /* Handle select item */
                if (cdc_demo.select & 0x80)
                {
                    cdc_demo.select &= 0x7F;
                    switch (cdc_demo.select)
                    {
                        case 0:
                            Fs_Open((uint8_t *) "file1.txt", &file);
                            CDC_SendData(file.data, file.len);
                            log_info("> 'file1.txt' sent.\n");
                            break;

                        case 1:
                            Fs_Open((uint8_t *) "file2.txt", &file);
                            CDC_SendData(file.data, file.len);
                            log_info("> 'file2.txt' sent.\n");
                            break;
                        case 2:
                            Fs_Open((uint8_t *) "file3.txt", &file);
                            CDC_SendData(file.data, file.len);
                            log_info("> 'file3.txt' sent.\n");
                            break;
                        case 3:                /* Return */
                            cdc_demo.state = CDC_DEMO_IDLE;
                            cdc_demo.select = 0;
                            break;
                        default:
                            break;
                    }
                }
            }
            break;

        default:
            break;
    }
}


/**
*\*\name    CDC_Handle_Receive_Menu.
*\*\fun     Handle Receive Menu
*\*\param   none.
*\*\return  none.
*\*\
**/
static void CDC_Handle_Receive_Menu(void)
{
    switch (cdc_demo.Receive_state)
    {
        case CDC_SEND_IDLE:
            cdc_demo.Receive_state = CDC_RECEIVE_WAIT;
            CDC_SelectItem(DEMO_RECEIVE_MENU, 0);
            log_info("> Received data (ASCII):\n");
            enable_display_received_data = 1;
            CDC_StartReception(&USBHS_Core);
            cdc_demo.select = 0;
            break;

        case CDC_SEND_WAIT:
            if (cdc_demo.select != prev_select)
            {
                prev_select = cdc_demo.select;
                CDC_SelectItem(DEMO_RECEIVE_MENU, cdc_demo.select & 0x7F);
                /* Handle select item */
                if (cdc_demo.select & 0x80)
                {
                    cdc_demo.select &= 0x7F;
                    switch (cdc_demo.select)
                    {
                        case 0:                /* Return */
                            cdc_demo.state = CDC_DEMO_IDLE;
                            cdc_demo.select = 0;
                            enable_display_received_data = 0;
                            CDC_StopReception(&USBHS_Core);
                            break;
                        default:
                            break;
                    }
                }
            }
            break;
        default:
            break;
    }
}


/**
*\*\name    CDC_Handle_Configuration_Menu.
*\*\fun     Handle Configuration Menu
*\*\param   none.
*\*\return  none.
*\*\
**/
static void CDC_Handle_Configuration_Menu(void)
{
    switch (cdc_demo.Configuration_state)
    {
        case CDC_CONFIGURATION_IDLE:
            log_info("[  Key   ]: Select menu set\n");
            log_info("[Joystick]: Change settings/Parse Menu\n");
            cdc_settings_state.select = 0;
            CDC_SelectSettingsItem(0xFF);
            cdc_demo.Configuration_state = CDC_CONFIGURATION_WAIT;
            CDC_SelectItem(DEMO_CONFIGURATION_MENU, 0);
            cdc_demo.select = 0;
            break;

        case CDC_CONFIGURATION_WAIT:
            if (cdc_demo.select != prev_select)
            {
                prev_select = cdc_demo.select;
                CDC_SelectItem(DEMO_CONFIGURATION_MENU, cdc_demo.select & 0x7F);
                /* Handle select item */
                if (cdc_demo.select & 0x80)
                {
                    cdc_demo.select &= 0x7F;
                    switch (cdc_demo.select)
                    {
                        case 0:
                            CDC_ChangeStateToIssueSetConfig(&USBHS_Core, &USB_Host);

                        case 1:                /* Return */
                            cdc_demo.state = CDC_DEMO_IDLE;
                            cdc_demo.select = 0;
                            break;
                        default:
                            break;
                    }
                }
            }
            break;
        default:
            break;
    }
}

/**
*\*\name    CDC_ChangeSelectMode.
*\*\fun     change the selection mode
*\*\param   mode: selection mode.
*\*\return  none.
*\*\
**/
static void CDC_ChangeSelectMode(CDC_DEMO_SelectMode select_mode)
{
    if (select_mode == CDC_SELECT_CONFIG)
    {
        CDC_SelectItem(DEMO_CONFIGURATION_MENU, 0xFF);
        cdc_settings_state.select = 0;
        CDC_SelectSettingsItem(0);
    }
    else
    {
        CDC_SelectItem(DEMO_CONFIGURATION_MENU, 0);
        CDC_SelectSettingsItem(0xFF);
    }
}

/**
*\*\name    CDC_AdjustSettingMenu.
*\*\fun     manage the setting menu.
*\*\param   settings_state: set state.
*\*\return  none.
*\*\
**/
static void CDC_AdjustSettingMenu(CDC_DEMO_SETTING_StateMachine *settings_state)
{
    uint8_t StrTemp[40];

    if (settings_state->select != Prev_Select)
    {
        Prev_Select = settings_state->select;
        CDC_SelectSettingsItem(settings_state->select);
    }

    if (settings_state->settings.BaudRateIdx != Prev_BaudRateIdx)
    {
        Prev_BaudRateIdx = settings_state->settings.BaudRateIdx;

        if (BaudRateValue[settings_state->settings.BaudRateIdx] <= 9600)
            sprintf((char *)StrTemp, "         %lu\r\n", BaudRateValue[settings_state->settings.BaudRateIdx]);
        else if (BaudRateValue[settings_state->settings.BaudRateIdx] <= 57600) 
            sprintf((char *)StrTemp, "        %lu\r\n", BaudRateValue[settings_state->settings.BaudRateIdx]);
        else
            sprintf((char *)StrTemp, "       %lu\r\n", BaudRateValue[settings_state->settings.BaudRateIdx]);
        log_info((void *) MSG_BITS_PER_SECOND);
        log_info((char *)StrTemp);

        CDC_SetLineCode.b.dwDTERate = BaudRateValue[settings_state->settings.BaudRateIdx];
        CDC_GetLineCode.b.dwDTERate = CDC_SetLineCode.b.dwDTERate;
    }

    if (settings_state->settings.DataBitsIdx != Prev_DataBitsIdx)
    {
        Prev_DataBitsIdx = settings_state->settings.DataBitsIdx;
        sprintf((char *)StrTemp, "            %d\r\n", DataBitsValue[settings_state->settings.DataBitsIdx]);
        log_info((void *) MSG_DATA_BITS);
        log_info((char *)StrTemp);
        CDC_SetLineCode.b.bDataBits = DataBitsValue[settings_state->settings.DataBitsIdx];
        CDC_GetLineCode.b.bDataBits = CDC_SetLineCode.b.bDataBits;
    }


    if (settings_state->settings.ParityIdx != Prev_ParityIdx)
    {
        Prev_ParityIdx = settings_state->settings.ParityIdx;
        if (settings_state->settings.ParityIdx == 2)
            sprintf((char *)StrTemp, "          %s\r\n", (uint8_t *) ParityArray[settings_state->settings.ParityIdx]);
        else
            sprintf((char *)StrTemp, "         %s\r\n", (uint8_t *) ParityArray[settings_state->settings.ParityIdx]);
        log_info((void *) MSG_PARITY);
        log_info((char *)StrTemp);
        CDC_SetLineCode.b.bParityType = settings_state->settings.ParityIdx;
        CDC_GetLineCode.b.bParityType = CDC_SetLineCode.b.bParityType;
    }


    if (settings_state->settings.StopBitsIdx != Prev_StopBitsIdx)
    {
        Prev_StopBitsIdx = settings_state->settings.StopBitsIdx;
        sprintf((char *)StrTemp, "            %s\r\n", StopBitsArray[settings_state->settings.StopBitsIdx]);
        log_info((void *) MSG_STOP_BITS);
        log_info((char *)StrTemp);
        CDC_SetLineCode.b.bCharFormat = settings_state->settings.StopBitsIdx;
        CDC_GetLineCode.b.bCharFormat = CDC_SetLineCode.b.bCharFormat;
    }
}



/**
*\*\name    CDC_SelectSettingsItem.
*\*\fun     manage the setting menu.
*\*\param   item : selected item to be highlighted
*\*\return  none.
*\*\
**/
static void CDC_SelectSettingsItem(uint8_t item)
{
    uint8_t StrTemp[40];

    switch (item)
    {
        case 0:
            log_info(" -->");
            log_info((void *) MSG_BITS_PER_SECOND);
            if (CDC_GetLineCode.b.dwDTERate <= 9600)
                sprintf((char *)StrTemp, "         %lu\r\n", CDC_GetLineCode.b.dwDTERate);
            else if (CDC_GetLineCode.b.dwDTERate <= 57600)
                sprintf((char *)StrTemp, "        %lu\r\n", CDC_GetLineCode.b.dwDTERate);
            else
                sprintf((char *)StrTemp, "       %lu\r\n", CDC_GetLineCode.b.dwDTERate);
            log_info((char *)StrTemp);

            /* Display the data bits */
            log_info((void *) MSG_DATA_BITS);
            sprintf((char *)StrTemp, "            %d\r\n", CDC_GetLineCode.b.bDataBits);
            log_info((char *)StrTemp);

            /* Display the parity bits */
            log_info((void *) MSG_PARITY);
            if (CDC_GetLineCode.b.bParityType == 2)
                sprintf((char *)StrTemp, "          %s\r\n", ParityArray[CDC_GetLineCode.b.bParityType]);
            else
                sprintf((char *)StrTemp, "         %s\r\n", ParityArray[CDC_GetLineCode.b.bParityType]);
            log_info((char *)StrTemp);

            /* Display the Stop bits */
            log_info((void *) MSG_STOP_BITS);
            sprintf((char *)StrTemp, "            %s\r\n", StopBitsArray[CDC_GetLineCode.b.bCharFormat]);
            log_info((char *)StrTemp);
            break;

        case 1:
            log_info((void *) MSG_BITS_PER_SECOND);
            if (CDC_GetLineCode.b.dwDTERate <= 9600)
                sprintf((char *)StrTemp, "         %lu\r\n", CDC_GetLineCode.b.dwDTERate);
            else if (CDC_GetLineCode.b.dwDTERate <= 57600)
                sprintf((char *)StrTemp, "        %lu\r\n", CDC_GetLineCode.b.dwDTERate);
            else
                sprintf((char *)StrTemp, "       %lu\r\n", CDC_GetLineCode.b.dwDTERate);
            log_info((char *)StrTemp);

            /* Display the data bits */
            log_info(" -->");
            log_info((void *) MSG_DATA_BITS);
            sprintf((char *)StrTemp, "            %d\r\n", CDC_GetLineCode.b.bDataBits);
            log_info((char *)StrTemp);

            /* Display the parity bits */
            log_info((void *) MSG_PARITY);
            if (CDC_GetLineCode.b.bParityType == 2)
                sprintf((char *)StrTemp, "          %s\r\n", ParityArray[CDC_GetLineCode.b.bParityType]);
            else
                sprintf((char *)StrTemp, "         %s\r\n", ParityArray[CDC_GetLineCode.b.bParityType]);
            log_info((char *)StrTemp);

            /* Display the Stop bits */
            log_info((void *) MSG_STOP_BITS);
            sprintf((char *)StrTemp, "            %s\r\n", StopBitsArray[CDC_GetLineCode.b.bCharFormat]);
            log_info((char *)StrTemp);
            break;

        case 2:
            log_info((void *) MSG_BITS_PER_SECOND);
            if (CDC_GetLineCode.b.dwDTERate <= 9600)
                sprintf((char *)StrTemp, "         %lu\r\n", CDC_GetLineCode.b.dwDTERate);
            else if (CDC_GetLineCode.b.dwDTERate <= 57600)
                sprintf((char *)StrTemp, "        %lu\r\n", CDC_GetLineCode.b.dwDTERate);
            else
                sprintf((char *)StrTemp, "       %lu\r\n", CDC_GetLineCode.b.dwDTERate);
            log_info((char *)StrTemp);

            /* Display the data bits */
            log_info((void *) MSG_DATA_BITS);
            sprintf((char *)StrTemp, "            %d\r\n", CDC_GetLineCode.b.bDataBits);
            log_info((char *)StrTemp);


            /* Display the parity bits */
            log_info(" -->");
            log_info((void *) MSG_PARITY);
            if (CDC_GetLineCode.b.bParityType == 2)
                sprintf((char *)StrTemp, "          %s\r\n", ParityArray[CDC_GetLineCode.b.bParityType]);
            else
                sprintf((char *)StrTemp, "         %s\r\n", ParityArray[CDC_GetLineCode.b.bParityType]);
            log_info((char *)StrTemp);

            /* Display the Stop bits */
            log_info((void *) MSG_STOP_BITS);
            sprintf((char *)StrTemp, "            %s\r\n", StopBitsArray[CDC_GetLineCode.b.bCharFormat]);
            log_info((char *)StrTemp);
            break;

        case 3:
            log_info((void *) MSG_BITS_PER_SECOND);
            if (CDC_GetLineCode.b.dwDTERate <= 9600)
                sprintf((char *)StrTemp, "         %lu\r\n", CDC_GetLineCode.b.dwDTERate);
            else if (CDC_GetLineCode.b.dwDTERate <= 57600)
                sprintf((char *)StrTemp, "        %lu\r\n", CDC_GetLineCode.b.dwDTERate);
            else
                sprintf((char *)StrTemp, "       %lu\r\n", CDC_GetLineCode.b.dwDTERate);
            log_info((char *)StrTemp);

            /* Display the data bits */
            log_info((void *) MSG_DATA_BITS);
            sprintf((char *)StrTemp, "            %d\r\n", CDC_GetLineCode.b.bDataBits);
            log_info((char *)StrTemp);


            /* Display the parity bits */
            log_info((void *) MSG_PARITY);
            if (CDC_GetLineCode.b.bParityType == 2)
                sprintf((char *)StrTemp, "          %s\r\n", ParityArray[CDC_GetLineCode.b.bParityType]);
            else
                sprintf((char *)StrTemp, "         %s\r\n", ParityArray[CDC_GetLineCode.b.bParityType]);
            log_info((char *)StrTemp);

            /* Display the Stop bits */
            log_info(" -->");
            log_info((void *) MSG_STOP_BITS);
            sprintf((char *)StrTemp, "            %s\r\n", StopBitsArray[CDC_GetLineCode.b.bCharFormat]);
            log_info((char *)StrTemp);
            break;
        default:
            log_info((void *) MSG_BITS_PER_SECOND);
            if (CDC_GetLineCode.b.dwDTERate <= 9600)
                sprintf((char *)StrTemp, "         %lu\r\n", CDC_GetLineCode.b.dwDTERate);
            else if (CDC_GetLineCode.b.dwDTERate <= 57600)
                sprintf((char *)StrTemp, "        %lu\r\n", CDC_GetLineCode.b.dwDTERate);
            else
                sprintf((char *)StrTemp, "       %lu\r\n", CDC_GetLineCode.b.dwDTERate);
            log_info((char *)StrTemp);

            /* Display the data bits */
            log_info((void *) MSG_DATA_BITS);
            sprintf((char *)StrTemp, "            %d\r\n", CDC_GetLineCode.b.bDataBits);
            log_info((char *)StrTemp);


            /* Display the parity bits */
            log_info((void *) MSG_PARITY);
            if (CDC_GetLineCode.b.bParityType == 2)
                sprintf((char *)StrTemp, "          %s\r\n", ParityArray[CDC_GetLineCode.b.bParityType]);
            else
                sprintf((char *)StrTemp, "         %s\r\n", ParityArray[CDC_GetLineCode.b.bParityType]);
            log_info((char *)StrTemp);

            /* Display the Stop bits */
            log_info((void *) MSG_STOP_BITS);
            sprintf((char *)StrTemp, "            %s\r\n", StopBitsArray[CDC_GetLineCode.b.bCharFormat]);
            log_info((char *)StrTemp);
        break;
    }
    log_info("\r\n");
}


/**
*\*\name    CDC_OutputData.
*\*\fun     printf font char.
*\*\param   ptr: pointer to string to printf.
*\*\return  none.
*\*\
**/
static void CDC_OutputData(uint8_t * ptr)
{
    if (enable_display_received_data != 0)
    {
        /* Send the string character by character*/
        log_info((char*)ptr);
    }
}

/**
*\*\name    CDC_SetInitialValue.
*\*\fun     Set the cdc demo intial values.
*\*\param   none.
*\*\return  none.
*\*\
**/
static void CDC_SetInitialValue(void)
{
    /* Set the initial value */
    CDC_SetLineCode.b.dwDTERate = CDC_GetLineCode.b.dwDTERate;
    CDC_SetLineCode.b.bDataBits = CDC_GetLineCode.b.bDataBits;
    CDC_SetLineCode.b.bParityType = CDC_GetLineCode.b.bParityType;
    CDC_SetLineCode.b.bCharFormat = CDC_GetLineCode.b.bCharFormat;
    CDC_ChangeStateToIssueSetConfig(&USBHS_Core, &USB_Host);

    /* Initialize baud rate index accordingtly */
    switch (CDC_SetLineCode.b.dwDTERate)
    {
        case 2400:
            cdc_settings_state.settings.BaudRateIdx = 0;
            break;
        case 4800:
            cdc_settings_state.settings.BaudRateIdx = 1;
            break;
        case 9600:
            cdc_settings_state.settings.BaudRateIdx = 2;
            break;
        case 19200:
            cdc_settings_state.settings.BaudRateIdx = 3;
            break;
        case 38400:
            cdc_settings_state.settings.BaudRateIdx = 4;
            break;
        case 57600:
            cdc_settings_state.settings.BaudRateIdx = 5;
            break;
        case 115200:
            cdc_settings_state.settings.BaudRateIdx = 6;
            break;
        case 230400:
            cdc_settings_state.settings.BaudRateIdx = 7;
            break;
        case 460800:
            cdc_settings_state.settings.BaudRateIdx = 8;
            break;
        case 921600:
            cdc_settings_state.settings.BaudRateIdx = 9;
            break;
        default:
            break;
    }
    /* Initialize data bits index accordingtly */
    switch (CDC_SetLineCode.b.bDataBits)
    {
        case 5:
            cdc_settings_state.settings.DataBitsIdx = 0;
            break;
        case 6:
            cdc_settings_state.settings.DataBitsIdx = 1;
            break;

        case 7:
            cdc_settings_state.settings.DataBitsIdx = 2;
            break;

        case 8:
            cdc_settings_state.settings.DataBitsIdx = 3;
            break;
        default:
            break;
    }
    /* Initialize stop bits index accordingtly */
    switch (CDC_SetLineCode.b.bCharFormat)
    {
        case 1:
            cdc_settings_state.settings.StopBitsIdx = 0;
            break;
        case 2:
            cdc_settings_state.settings.StopBitsIdx = 1;
            break;
        default:
            break;
    }

    /* Initialize parity index accordingtly */
    switch (CDC_SetLineCode.b.bParityType)
    {
        case 0:
            cdc_settings_state.settings.ParityIdx = 0;
            break;
        case 1:
            cdc_settings_state.settings.ParityIdx = 1;
            break;
        case 2:
            cdc_settings_state.settings.ParityIdx = 2;
            break;
        default:
            break;
    }
    Prev_BaudRateIdx = cdc_settings_state.settings.BaudRateIdx;
    Prev_DataBitsIdx = cdc_settings_state.settings.DataBitsIdx;
    Prev_ParityIdx = cdc_settings_state.settings.StopBitsIdx;
    Prev_StopBitsIdx = cdc_settings_state.settings.ParityIdx;
    Prev_Select = 0;
}
