#include "bsp_gpio.h"
#include "bsp_uart.h"   /* g4_uart_tx_float/resume: 模组断电期间放开 PC8, 见 G4_PowerSwitch_Set */
/**
 * @brief  RS232 UART6 GPIO: PA0=TX(AF11) PA1=RX(AF11) PA2=232_EN(普通输出)
 * @note 注释原来写"PA6 EN"是过期的 —— RS232_EN_PIN 宏是 PA2(PIN_2);
 *       PA6 在新板上是 BAT_AD(电池电压 ADC), 代码不碰它。
 */
void RS232_GPIO_Init(void)
{
    GPIO_InitType GPIO_InitStructure;
    GPIO_InitStruct(&GPIO_InitStructure);
    RCC_EnableAHB1PeriphClk(RCC_AHB_PERIPHEN_GPIOA, ENABLE);
    //TX PA0
    GPIO_InitStructure.Pin        = RS232_TX_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Pull  = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Slew_Rate = GPIO_SLEW_RATE_FAST;
    GPIO_InitStructure.GPIO_Current = GPIO_DC_8mA;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF_UART6_TX_PA0;
    GPIO_InitPeripheral(RS232_GPIO_PORT, &GPIO_InitStructure);
    //RX PA1
    GPIO_InitStructure.Pin        = RS232_RX_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Pull  = GPIO_PULL_UP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF_UART6_RX_PA1;
    GPIO_InitPeripheral(RS232_GPIO_PORT, &GPIO_InitStructure);
    //PA6 EN 输出
    GPIO_InitStructure.Pin        = RS232_EN_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.GPIO_Pull  = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Alternate = GPIO_NO_AF;
    GPIO_InitPeripheral(RS232_GPIO_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(RS232_GPIO_PORT, RS232_EN_PIN);
}
/* 485 控制脚: DIR 高=发送; EN = 收发器使能(极性由 RS485_EN_ACTIVE_LOW 决定) */
#if RS485_EN_ACTIVE_LOW
#define RS485_EN_ASSERT(port, pin)   GPIO_ResetBits((port), (pin))   /* 使能 */
#else
#define RS485_EN_ASSERT(port, pin)   GPIO_SetBits((port), (pin))     /* 使能 */
#endif

/**
 * @brief RS485_0 USART4: PB0 TX(AF13) PB1 RX(AF13); DIR=PC5, EN=PC4
 * @note 上电默认接收模式: DIR=LOW(接收), EN=使能(保持总线可用)
 */
void RS485_0_GPIO_Init(void)
{
    GPIO_InitType GPIO_InitStructure;
    GPIO_InitStruct(&GPIO_InitStructure);
    RCC_EnableAHB1PeriphClk(RCC_AHB_PERIPHEN_GPIOB, ENABLE);
    RCC_EnableAHB1PeriphClk(RCC_AHB_PERIPHEN_GPIOC, ENABLE);
    //PB0 TX USART4_TX
    GPIO_InitStructure.Pin = RS485_0_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Pull = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Slew_Rate = GPIO_SLEW_RATE_FAST;
    GPIO_InitStructure.GPIO_Current = GPIO_DC_8mA;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF_USART4_TX_PB0;
    GPIO_InitPeripheral(RS485_0_TX_PORT, &GPIO_InitStructure);
    //PB1 RX USART4_RX
    GPIO_InitStructure.Pin = RS485_0_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Pull = GPIO_PULL_UP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF_USART4_RX_PB1;
    GPIO_InitPeripheral(RS485_0_RX_PORT, &GPIO_InitStructure);
    //DIR PC5 (普通输出)
    GPIO_InitStructure.Pin = RS485_0_DIR_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.GPIO_Pull = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Alternate = GPIO_NO_AF;
    GPIO_InitPeripheral(RS485_0_DIR_PORT, &GPIO_InitStructure);
    //EN PC4 (普通输出)
    GPIO_InitStructure.Pin = RS485_0_EN_PIN;
    GPIO_InitPeripheral(RS485_0_EN_PORT, &GPIO_InitStructure);
    //上电: 接收方向 + 收发器使能
    GPIO_ResetBits(RS485_0_DIR_PORT, RS485_0_DIR_PIN);
    RS485_EN_ASSERT(RS485_0_EN_PORT, RS485_0_EN_PIN);
}
void RS485_0_SetSendMode(void)
{
    GPIO_SetBits(RS485_0_DIR_PORT, RS485_0_DIR_PIN);     /* DIR 高 = 发送 */
    RS485_EN_ASSERT(RS485_0_EN_PORT, RS485_0_EN_PIN);   /* EN 保持使能 */
}
void RS485_0_SetRecvMode(void)
{
    GPIO_ResetBits(RS485_0_DIR_PORT, RS485_0_DIR_PIN);  /* DIR 低 = 接收 */
    RS485_EN_ASSERT(RS485_0_EN_PORT, RS485_0_EN_PIN);   /* 【关键】EN 不能拉低, 否则收发器被关掉 */
}
/**
 * @brief RS485_1 UART7: PB11 TX(AF8) PB10 RX(AF8); DIR=PB2, EN=PH3
 */
void RS485_1_GPIO_Init(void)
{
    GPIO_InitType GPIO_InitStructure;
    GPIO_InitStruct(&GPIO_InitStructure);
    RCC_EnableAHB1PeriphClk(RCC_AHB_PERIPHEN_GPIOB, ENABLE);
    RCC_EnableAHB1PeriphClk(RCC_AHB_PERIPHEN_GPIOH, ENABLE);
    //PB11 TX UART7_TX
    GPIO_InitStructure.Pin = RS485_1_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Pull = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Slew_Rate = GPIO_SLEW_RATE_FAST;
    GPIO_InitStructure.GPIO_Current = GPIO_DC_8mA;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF_UART7_TX_PB11;
    GPIO_InitPeripheral(RS485_1_TX_PORT, &GPIO_InitStructure);
    //PB10 RX UART7_RX
    GPIO_InitStructure.Pin = RS485_1_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Pull = GPIO_PULL_UP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF_UART7_RX_PB10;
    GPIO_InitPeripheral(RS485_1_RX_PORT, &GPIO_InitStructure);
    //DIR PB2
    GPIO_InitStructure.Pin = RS485_1_DIR_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.GPIO_Pull = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Alternate = GPIO_NO_AF;
    GPIO_InitPeripheral(RS485_1_DIR_PORT, &GPIO_InitStructure);
    //EN PH3
    GPIO_InitStructure.Pin = RS485_1_EN_PIN;
    GPIO_InitPeripheral(RS485_1_EN_PORT, &GPIO_InitStructure);
    //上电接收模式
    GPIO_ResetBits(RS485_1_DIR_PORT, RS485_1_DIR_PIN);
    RS485_EN_ASSERT(RS485_1_EN_PORT, RS485_1_EN_PIN);
}
void RS485_1_SetSendMode(void)
{
    GPIO_SetBits(RS485_1_DIR_PORT, RS485_1_DIR_PIN);
    RS485_EN_ASSERT(RS485_1_EN_PORT, RS485_1_EN_PIN);
}
void RS485_1_SetRecvMode(void)
{
    GPIO_ResetBits(RS485_1_DIR_PORT, RS485_1_DIR_PIN);
    RS485_EN_ASSERT(RS485_1_EN_PORT, RS485_1_EN_PIN);
}
/**
 * @brief RS_DEBUG USART2 PC8‑TX PC9‑RX
 */
void RS_DEBUG_GPIO_Init(void)
{
    GPIO_InitType GPIO_InitStructure;
    GPIO_InitStruct(&GPIO_InitStructure);
    RCC_EnableAHB1PeriphClk(RCC_AHB_PERIPHEN_GPIOC, ENABLE);
    GPIO_InitStructure.Pin = RS_DEBUG_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Pull = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Slew_Rate = GPIO_SLEW_RATE_FAST;
    GPIO_InitStructure.GPIO_Current = GPIO_DC_8mA;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF_USART2_TX_PC8;
    GPIO_InitPeripheral(RS_DEBUG_GPIO_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.Pin = RS_DEBUG_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Pull = GPIO_PULL_UP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF_USART2_RX_PC9;
    GPIO_InitPeripheral(RS_DEBUG_GPIO_PORT, &GPIO_InitStructure);
}
/*
 * 注意: G4_USBHS_Host_GPIO_Init() 已删除!
 * USBHS DP/DM (PB14/PB15) 是 N32H48x HS PHY 专用引脚,
 * USBHS 时钟使能后 PHY 自动接管, 不需要通过 GPIO Alternate Function 配置。
 * 任何 GPIO 配置都会切断 PHY 与引脚的连接, 导致 USB 收发物理通道断开。
 */

extern void delay_ms(uint32_t ms);

/**
 * @brief 4G模块电源引脚初始化
 * @note 新板没有这个脚(G4_HAS_PWR_PIN=0, PB13 是 BAT_STDBY), 此处为空实现。
 *       极性见 G4_PWR_ACTIVE_LOW。
 */
#if G4_HAS_PWR_PIN
#if G4_PWR_ACTIVE_LOW
#define G4_PWR_ON_LEVEL()   GPIO_ResetBits(G4_MODULE_PWR_PORT, G4_MODULE_PWR_PIN)
#define G4_PWR_OFF_LEVEL()  GPIO_SetBits(G4_MODULE_PWR_PORT, G4_MODULE_PWR_PIN)
#else
#define G4_PWR_ON_LEVEL()   GPIO_SetBits(G4_MODULE_PWR_PORT, G4_MODULE_PWR_PIN)
#define G4_PWR_OFF_LEVEL()  GPIO_ResetBits(G4_MODULE_PWR_PORT, G4_MODULE_PWR_PIN)
#endif
#endif /* G4_HAS_PWR_PIN */

/* ==================== PA8: 4G 模组电源开关 ====================
 *
 * 【这一段是整个 4G 调试的地基】
 *   板上模组没有引出 RST, 唯一能被 MCU 控制的就是 PA8, 它控的是模组的【供电】:
 *   一个电平 = 通电, 另一个 = 断电。"复位模组" = 断电一下再上电。
 *
 *   两个致命细节(都踩过):
 *   1) 极性没准。G4_PWR_SW_ON_HIGH 是编译期初值, 运行时还能翻, 因为板上没有
 *      任何丝印能说明哪个电平上电。翻的方法在 g4_at.c 的供电自检里。
 *   2) 【断电期间必须放开 PC8(USART2_TX)】。PC8 空闲时输出高电平, 模组一旦没电,
 *      这个高电平就顺着模组 RX 脚的保护二极管往它电源轨灌电 —— 模组被"半供电":
 *         · 万用表量模组电源脚, 读到 2~4V 乱跳(不是 0V 也不是 3.8V);
 *         · "断电复位"根本没断成: 电一断就被 TX 喂回来, 模组状态机半死不活;
 *         · 模组的电源开关管也可能因此半导通。
 *      所以下面断电时会先 g4_uart_tx_float() 把 PC8 变成高阻。
 *
 * 这一根脚以前是 LED 心跳脚, 现在是模组命门 —— main.c 的 MAIN_HEARTBEAT_LED
 * 必须保持 0, 任何"每 1s 翻转 PA8"的代码都等于每秒给模组断一次电。
 */
static uint8_t s_pwr_on_high = ((G4_PWR_SW_ON_HIGH) != 0) ? 1u : 0u;
/* 【我们自己驱动出去的电平】, 不是从引脚读回来的。
 * 为什么用影子变量: 输出模式下 GPIO 的输入寄存器(DIN)到底跟不跟随输出电平,
 * 跟具体芯片/配置有关; 拿它当判据会出现"程序明明拉高了、日志却说低"的假报警,
 * 而这种假报警会把整个排障方向带偏。要确认引脚上的真实电平(怀疑被短路/被别的
 * 电路压住), 用 G4_PowerSwitch_ReadPin() 或者直接上万用表。 */
static uint8_t s_pwr_level = 1u;

void G4_PowerSwitch_DriveRaw(uint8_t high)
{
    s_pwr_level = (high != 0u) ? 1u : 0u;

    if (s_pwr_level != 0u)
    {
        GPIO_SetBits(G4_PWR_SW_PORT, G4_PWR_SW_PIN);
    }
    else
    {
        GPIO_ResetBits(G4_PWR_SW_PORT, G4_PWR_SW_PIN);
    }
}

void G4_PowerSwitch_Set(uint8_t on)
{
    if (on != 0u)
    {
        G4_PowerSwitch_DriveRaw(s_pwr_on_high);
        /* 上电之后再恢复 TX: 顺序反了的话, 模组刚上电那一下会被 TX 灌着电 */
        g4_uart_tx_resume();
    }
    else
    {
        /* 【顺序很重要】先放开 TX(去灌电的通路), 再把电源切断 */
        g4_uart_tx_float();
        G4_PowerSwitch_DriveRaw((uint8_t)(s_pwr_on_high ^ 1u));
    }
}

uint8_t G4_PowerSwitch_GetLevel(void)
{
    return s_pwr_level;     /* 程序驱动的电平(判据用这个, 不受 DIN 行为影响) */
}

uint8_t G4_PowerSwitch_ReadPin(void)
{
    /* 真正读引脚。输出模式下这个值有可能不跟随输出(取决于芯片的输入通路),
     * 只当辅助参考 —— "程序驱动 H 但这里读到 L"值得怀疑脚被拉着, 但不能当结论。 */
    return (GPIO_ReadInputDataBit(G4_PWR_SW_PORT, G4_PWR_SW_PIN) != 0u) ? 1u : 0u;
}

void G4_PowerSwitch_SetOnHigh(uint8_t on_high)
{
    s_pwr_on_high = (on_high != 0u) ? 1u : 0u;
}

uint8_t G4_PowerSwitch_GetOnHigh(void)
{
    return s_pwr_on_high;
}

void G4_PowerSwitch_SwapOnHigh(void)
{
    s_pwr_on_high = (uint8_t)(s_pwr_on_high ^ 1u);
}

/**
 * @brief 把 PA8 配成推挽输出(不动电平) —— 给下面两个 Init 用
 */
static void g4_pwr_sw_config_pin(void)
{
    GPIO_InitType GPIO_InitStructure;
    GPIO_InitStruct(&GPIO_InitStructure);
    RCC_EnableAHB1PeriphClk(G4_PWR_SW_CLK, ENABLE);
    GPIO_InitStructure.Pin             = G4_PWR_SW_PIN;
    GPIO_InitStructure.GPIO_Mode       = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.GPIO_Pull       = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Slew_Rate  = GPIO_SLEW_RATE_SLOW;
    GPIO_InitStructure.GPIO_Current    = GPIO_DC_8mA;
    GPIO_InitStructure.GPIO_Alternate  = GPIO_NO_AF;
    GPIO_InitPeripheral(G4_PWR_SW_PORT, &GPIO_InitStructure);
}

/**
 * @brief 配置 PA8 为推挽输出并立即上电
 * @note 【不能阻塞】: main() 把它放在很靠前的位置。所以这里只配置 + 上电, 需要
 *       "先断电再上电"的干净复位请调 G4_PowerCycle()(在 g4_at_init 里, 时基已起)。
 */
void G4_PowerSwitch_Init(void)
{
    g4_pwr_sw_config_pin();
    G4_PowerSwitch_Set(1u);
}

/**
 * @brief 配置 PA8 为推挽输出并【驱动到断电电平】
 * @note 隔离测试用(MAIN_G4_POWER_AT_BOOT=0): 悬空是不行的 —— 悬空的电源开关会让
 *       模组处于"半通电"状态(电压乱跳、布朗振荡), 那比明确断电更难查。
 */
void G4_PowerSwitch_InitNoPower(void)
{
    g4_pwr_sw_config_pin();
    G4_PowerSwitch_Set(0u);
}

/**
 * @brief 拔电再上电(本板上"复位模组"的唯一手段)
 */
void G4_PowerCycle(uint32_t off_ms)
{
    G4_PowerSwitch_Set(0u);
    delay_ms(off_ms);           /* 让模组输入电容真的放掉 + 模组认为自己断电了 */
    G4_PowerSwitch_Set(1u);
    delay_ms(50u);              /* 等电源轨稳定(模组内部 LDO/晶振还没起, 先别发 AT) */
}

/**
 * @brief 4G模块电源引脚初始化(PB13 已改用途 → 空实现)
 */
void G4_ModulePWR_GPIO_Init(void)
{
    /* 【不要动 PB13!】新板 PB13 = BAT_STDBY(电池待机状态), 是输入信号。
     * 这里保持空实现, 就是不给自己留"顺手驱动一下"的机会。
     * 模组的供电全靠 PA8, 见上面的 G4_PowerSwitch_*。 */
    /* G4_HAS_PWR_PIN=1 时(将来有独立电源使能脚的板子)才需要真正的实现 */
}

