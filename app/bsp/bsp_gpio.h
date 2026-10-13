#ifndef __BSP_GPIO_H__
#define __BSP_GPIO_H__
#include "n32h47x_48x.h"
#include "n32h47x_48x_gpio.h"
#include "n32h47x_48x_rcc.h"

/* ==================================================================
 * 整板引脚表(N32H482REL7 实板, 2026-09 现场确认) —— 改板子先核对这里
 * ------------------------------------------------------------------
 *  MCU      VBAT(1) -> MCU_3V3 ;  NRST(7) -> 复位 ;  SWD: PA13(SWDIO) PA14(SWCLK)
 *  MCU 供电 独立一路
 *
 *  调试口  USART1 : PA9 (TX)          PA10 (RX)            日志打印
 *  4G      USART2 : PC8 (TX)          PC9  (RX)
 *          电源    PA8 : 【4G_RST, 实为电源开关】★低电平=模组上电, 高电平=断电★
 *                       模组【没有引出 RST 脚】, "复位模组"= 断一下电再上电
 *
 *  RS232   UART6  : PA0 (TX)  PA1 (RX)  PA2 (232_EN)   PA3 = DET_IN1(输入检测)
 *  485_0   USART4 : PB0 (TX)  PB1 (RX)  PC5 (DIR)      PC4 (4850_EN)
 *  485_1   UART7  : PB11(TX)  PB10(RX)  PB2 (DIR)      PH3 (4851_EN)
 *
 *  LoRa    SPI3   : PC12(MOSI) PC11(MISO) PC10(SCK)  PA15(NSS,软)
 *                   PD2(RST)  PB3(BUSY)  PB4(DIO1/EXTI4)
 *          电源    PB5 : ★高电平=模组上电, 低电平=断电★(与 4G_RST 相反!)
 *
 *  电池检测        PA7 (BAT_AD, ADC)   PB12(BAT_CHRG, 充电时拉低)
 *                                     PB13(BAT_STDBY, 充满时拉低)
 *                  ⚠️ PB12/PB13 是【电池状态输入】, 任何代码都不要去驱动它们
 *  USB_HS          PB14(DM)  PB15(DP)   —— 本工程 USB 已停用, 不初始化
 * ================================================================== */

/**
 * @brief RS232接口定义
 * @note 232接口使用UART6，引脚定义如下：
 * PA1-RX (AF11)
 * PA0-TX (AF11)
 * PA2‑EN
 */
#define RS232_GPIO_PORT     GPIOA
#define RS232_TX_PIN        GPIO_PIN_0
#define RS232_RX_PIN        GPIO_PIN_1
#define RS232_EN_PIN        GPIO_PIN_2
/**
 * @brief RS485_0 接口定义 USART4 (TX=PB0 RX=PB1, 都是 AF13)
 * @note 板上是【两个控制脚】:
 *         DIR (PC5)  方向:  高 = 发送, 低 = 接收
 *         EN  (PC4)  收发器使能(总线要用的时候必须保持"使能"电平)
 *
 *       ⚠️ 老代码把这两个脚当成 "DE(高有效) + RE#(低有效)" 用: 发送时两个都拉高、
 *          接收时两个都拉低。按板子的命名, 那等于【接收时把 EN 拉低】——
 *          如果 EN 是高有效使能, 485 收发器就被关掉了, 现象是"发得出去、收不进来"
 *          (这种半通的问题最难查)。现在改成: DIR 切方向, EN 始终保持在"使能"电平。
 *
 *       若板上 EN 是低有效(丝印带 # 或 nEN), 把 RS485_EN_ACTIVE_LOW 改成 1。
 */
#ifndef RS485_EN_ACTIVE_LOW
#define RS485_EN_ACTIVE_LOW 0       /* 0 = EN 高有效(常见); 1 = EN 低有效 */
#endif
#define RS485_0_TX_PORT     GPIOB
#define RS485_0_RX_PORT     GPIOB
#define RS485_0_DIR_PORT    GPIOC
#define RS485_0_EN_PORT     GPIOC
#define RS485_0_TX_PIN      GPIO_PIN_0
#define RS485_0_RX_PIN      GPIO_PIN_1
#define RS485_0_DIR_PIN     GPIO_PIN_5
#define RS485_0_EN_PIN      GPIO_PIN_4
/**
 * @brief RS485_1 接口定义 UART7 (TX=PB11 RX=PB10, 都是 AF8)
 * @note DIR (PB2)  方向: 高 = 发送, 低 = 接收
 *       EN  (PH3)  收发器使能(同上, 极性看 RS485_EN_ACTIVE_LOW)
 */
#define RS485_1_TX_PORT     GPIOB
#define RS485_1_RX_PORT     GPIOB
#define RS485_1_DIR_PORT    GPIOB
#define RS485_1_EN_PORT     GPIOH
#define RS485_1_TX_PIN      GPIO_PIN_11
#define RS485_1_RX_PIN      GPIO_PIN_10
#define RS485_1_DIR_PIN     GPIO_PIN_2
#define RS485_1_EN_PIN      GPIO_PIN_3
/**
 * @brief 4G 模组 AT 口引脚定义 USART2
 * @note PC9-RX / PC8-TX
 *
 * ⚠️【宏名 RS_DEBUG_* 是历史遗留, 别被名字带偏】
 *   这两个宏现在指的是【4G 模组的 AT 串口 USART2(PC8/PC9)】, 不是日志口。
 *   日志打印走的是 USART1(PA9/PA10), 定义在 app/log/log.c 里。
 *   宏名没改是为了不动 usbh_user.c / bsp_uart.c 里已有的引用。
 */
#define RS_DEBUG_GPIO_PORT  GPIOC
#define RS_DEBUG_TX_PIN     GPIO_PIN_8
#define RS_DEBUG_RX_PIN     GPIO_PIN_9

/*
 * USBHS Host 引脚说明:
 *   PB14 = USBHS_DM (硬件直连 HS PHY, 不需要 GPIO 初始化)
 *   PB15 = USBHS_DP (硬件直连 HS PHY, 不需要 GPIO 初始化)
 *
 * ⚠️【旧注释里写的 "PB12 = USBHS_ID" / "PB13 = 4G 模块电源控制" 已经不成立了】
 *   实板(2026-09 确认): PB12 = BAT_CHRG(充电时拉低), PB13 = BAT_STDBY(充满时拉低),
 *   两个都是【电池状态输入】。USB 已停用, 这两根脚谁都不该碰 —— 详见下面的
 *   G4_HAS_PWR_PIN 说明和 src/usbhs_bsp.c 里那段警告。
 *   4G 模组的上下电只有 PA8 一根脚(见 G4_PWR_SW_*)。
 */
#define G4_USBHS_DP_PORT    GPIOB
#define G4_USBHS_DM_PORT    GPIOB
#define G4_USBHS_DP_PIN     GPIO_PIN_15    /* PB15 → 4G 模块 DP */
#define G4_USBHS_DM_PIN     GPIO_PIN_14    /* PB14 → 4G 模块 DM */

/**
 * @brief 4G模块电源使能脚 —— ★新板没有这个脚, 默认关掉★
 *
 * 【新板事实】: PB12/PB13 已经改用途了 —— 一个 BAT_STDBY、一个 BAT_CHRG
 * (电池待机 / 充电状态)。所以【绝对不能】再去驱动 PB13, 会干扰电池电路。
 * 新板只有 PA8 一根线, 而它控的是【模组供电】(不是复位) —— 见下面的 G4_PWR_SW_*。
 *
 * 如果哪天板子又有了独立电源使能脚: 把 G4_HAS_PWR_PIN 设成 1, 并把
 * G4_MODULE_PWR_PORT/PIN/CLK 指到那根脚。
 */
#ifndef G4_HAS_PWR_PIN
#define G4_HAS_PWR_PIN            0
#endif
#ifndef G4_MODULE_PWR_PORT
#define G4_MODULE_PWR_PORT        GPIOB
#endif
#ifndef G4_MODULE_PWR_PIN
#define G4_MODULE_PWR_PIN         GPIO_PIN_13
#endif
#ifndef G4_MODULE_PWR_CLK
#define G4_MODULE_PWR_CLK         RCC_AHB_PERIPHEN_GPIOB
#endif
#ifndef G4_PWR_ACTIVE_LOW
#define G4_PWR_ACTIVE_LOW         1     /* 1 = 低电平上电(常见); 0 = 高电平上电 */
#endif

/**
 * @brief 4G 模组【电源开关】= PA8 —— ★换板子改这里(或在工程 Define 里覆盖)★
 *
 * 【板上事实(现场确认)】: 模组【没有把 RST 引出来】。板上唯一能控制模组的就是
 *   PA8, 而且它控的是【模组的供电】: 拉对一个电平 = 给模组通电, 拉另一个 = 把
 *   模组的电断掉。"复位模组"在本板上就是"断一下电再上电"(等于拔插一次电源)。
 *   所以本工程里凡是叫 "RST" 的老名字, 全部按【电源开关】理解。
 *
 * ⚠️ 这一根脚以前是 LED 心跳脚(main() 每 1s 翻转一次), 接到新板上就变成
 *    "每秒给 4G 模组断一次电" —— 模组永远起不来, 现象是 AT_SYNC 一直重试、
 *    模组【一个字节都不回】。所以 main.c 里 MAIN_HEARTBEAT_LED 默认已改成 0。
 *
 * 【极性: 已实测确认】
 *   ★ PA8 低电平 = 模组上电; 高电平 = 断电 ★
 *   对应 G4_PWR_SW_ON_HIGH = 0。以前这里按"高电平上电"的常见习惯猜成 1, 于是开机
 *   就把 PA8 驱到高 —— 等于【一直在给模组断电】, 现象就是"模组没有电、电源脚电压
 *   来回跳、AT 一个字节都不回"。改板子/换模组时优先核对这一条。
 *   现场想试另一个方向: 日志口敲 "#pwrswap"(翻一次) / "#pwr? "(看当前电平)。
 *   不想重新编译就想让它开机自己试: 把 g4_at.c 的 G4_PWR_SEARCH 改成 1。
 *
 * 【为什么必须显式驱动】: MCU 复位后 GPIO 是输入悬空 —— 悬空的 PA8 让模组供电
 *   处于不确定状态(可能半通, 模组在 2~3V 上来回布朗振荡、永远起动不完, 万用表
 *   量到的就是"电压来回跳")。所以 main() 第一件事就是把这个脚配成推挽输出。
 *   旧板更坑: 这个初始化原本写在 usbhs_bsp.c 的 ML307R_RST_Init() 里, 而它只被
 *   USB_BSP_Init() 调用 —— 4G 改走串口后不再调 USB_BSP_Init(), 它静默失效了。
 */
#ifndef G4_PWR_SW_PORT
#define G4_PWR_SW_PORT            GPIOA
#endif
#ifndef G4_PWR_SW_PIN
#define G4_PWR_SW_PIN             GPIO_PIN_8
#endif
#ifndef G4_PWR_SW_CLK
#define G4_PWR_SW_CLK             RCC_AHB_PERIPHEN_GPIOA
#endif
/* ★现场已确认(2026-09): PA8 = 低电平给 4G 模组上电, 高电平断电 ★
 *
 * 之前这里是 1(按"高电平上电"的常见习惯猜的), 结果开机就把 PA8 驱到高 —— 也就是
 * 【一直给模组断电】, 表现正是"模组没有电 / 万用表量到 2~4V 乱跳 / AT 一个字节都不回"。
 * 现在按实测定死为 0。没事不要改; 要现场试另一个方向用日志口的 "#pwrswap"。
 */
#ifndef G4_PWR_SW_ON_HIGH
#define G4_PWR_SW_ON_HIGH         0     /* 1 = 高电平上电; 0 = 低电平上电(本板) */
#endif

void RS232_GPIO_Init(void);
void RS485_0_GPIO_Init(void);
void RS485_1_GPIO_Init(void);
void RS_DEBUG_GPIO_Init(void);

/* 4G模块电源引脚初始化(PB13 已改用途, G4_HAS_PWR_PIN=0 时为空实现) */
void G4_ModulePWR_GPIO_Init(void);

/* ---------------- PA8: 4G 模组电源开关 ----------------
 * 【为什么全用函数而不是宏】: 极性可以在运行时翻转(供电自检), 所以极性存在一个
 * 变量里。要改板子/改极性优先改宏, 运行期开关只给自检和日志口用。 */
/* 配置 PA8 为推挽输出, 并按当前极性给模组上电(尽早调用, 不能阻塞) */
void G4_PowerSwitch_Init(void);
/* 配置 PA8 为推挽输出, 但驱动到【断电】电平(隔离测试 MAIN_G4_POWER_AT_BOOT=0 用) */
void G4_PowerSwitch_InitNoPower(void);
/* 上电(1) / 断电(0)。断电时会顺手把 PC8(TX) 放开, 免得反向灌电, 见 .c */
void G4_PowerSwitch_Set(uint8_t on);
/* 直接驱动 PA8 电平(1=高,0=低), 不管极性 —— 供电自检/手工排障用 */
void G4_PowerSwitch_DriveRaw(uint8_t high);
/* 程序当前驱动出去的电平(1=高,0=低)。这是判据, 也是日志里打印的那个值 */
uint8_t G4_PowerSwitch_GetLevel(void);
/* 读引脚上的实际电平 —— 辅助排障用: "驱动 H 却读到 L"值得怀疑脚被短路/被压住,
 * 但输出模式下该值未必跟随输出, 不能当结论, 最终以万用表为准 */
uint8_t G4_PowerSwitch_ReadPin(void);
/* 设置/读取"哪个电平算上电"(1=高电平上电) */
void    G4_PowerSwitch_SetOnHigh(uint8_t on_high);
uint8_t G4_PowerSwitch_GetOnHigh(void);
/* 把"上电电平"翻到相反的一边(不会立刻改变 PA8 输出, 要配合 Set/PowerCycle) */
void G4_PowerSwitch_SwapOnHigh(void);
/* 拔电再上电: 断电 off_ms → 上电 → 等电源稳。本板的"复位模组"就是这个 */
void G4_PowerCycle(uint32_t off_ms);

//485控制函数
void RS485_0_SetSendMode(void);
void RS485_0_SetRecvMode(void);
void RS485_1_SetSendMode(void);
void RS485_1_SetRecvMode(void);

#endif /* __BSP_GPIO_H__ */
