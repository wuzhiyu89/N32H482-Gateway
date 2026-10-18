/*!
 * \file      LLCC68N32H482-board.c
 *
 * \brief     Target board LLCC68 driver implementation for N32H482 (NSING)
 *
 * 按照《LLCC68驱动demo使用说明.pdf》第2章"驱动移植"要求，参考
 * LLCC68STM32F103-board.c 新建的中间层适配文件，实现GPIO、外部中断、
 * SPI和定时器驱动。引脚定义见 project_config.h。
 *
 * 资源占用:
 *   - SPI3(APB1): PC10 SCK / PC11 MISO / PC12 MOSI (AF1), PA15 NSS(软件控制)
 *   - EXTI4     : PB4 DIO1 上升沿中断
 *   - GTIM8     : 1ms 周期中断, 驱动 LoRa 收发超时软定时器(时钟=SYSCLK)
 *   - SysTick   : LLCC68DelayMs 阻塞式毫秒延时(轮询COUNTFLAG, 不开中断)
 *   -PA15,PB4 要关闭 JTDI,NJTRST 
 *
 * 系统主频最高180MHz: SystemCoreClock<=180MHz, GTIM8预分频=17999<65535(16bit内),
 * SysTick LOAD=179999<2^24, SPI3在PCLK1<=180MHz下16分频<=11.25MHz<16MHz均满足要求
 */
#include <stdio.h>
#include "project_config.h"
#include "radio.h"
#include "LLCC68-board.h"
#include "n32h47x_48x_conf.h"
#include "log.h"

/* ==================== 现场可调项(排 LoRa "wait busy pin timeout") ====================
 *
 * 【为什么要有超时】: 原来 LLCC68WaitOnBusy() 是【无上限】等 BUSY 拉低的 ——
 * 只要 BUSY 一直高(模组没供电/没焊/接线断/被复位), 它就永远等下去, 并且每 1s
 * 打印一行 "wait busy pin timeout"。后果不只是刷屏:
 *     RadioInit → LLCC68Init → 每条 SPI 命令都卡在等 BUSY
 *     → lora_business_init() 永远返回不了
 *     → main() 根本走不到 g4_at_init(), 【4G 那边的代码一行都不会执行】。
 *   也就是说: LoRa 模组不应答会把整个固件(包括 4G 调试)一起拖死。
 * 现在给单次等待加上限: 超时就不再傻等, 打个明确的错误日志然后让流程继续。
 */
#ifndef LLCC68_BUSY_TIMEOUT_MS
#define LLCC68_BUSY_TIMEOUT_MS      100u   /* 单次等 BUSY 的上限(正常只要几 us) */
#endif
#ifndef LLCC68_BUSY_HARD_FAIL_CNT
#define LLCC68_BUSY_HARD_FAIL_CNT   3u     /* 连续超时这么多次 → 判定"模组不在" */
#endif
#ifndef LLCC68_BUSY_STUCK_WAIT_MS
#define LLCC68_BUSY_STUCK_WAIT_MS   1u     /* 判定不在之后, 每次只等这么久 */
#endif
#ifndef LLCC68_BUSY_LOG_INTERVAL_MS
#define LLCC68_BUSY_LOG_INTERVAL_MS 5000u  /* 判定不在之后, 错误日志的打印间隔 */
#endif

/* ---- LoRa 模组的 EN 脚(PB5) ----
 * project_config.h 里写得很清楚: "高电平 lora 上电, 低电平 lora 下电", 但【原来
 * 这份代码从来没配置过这根脚】—— RADIO_EN_PIN 定义了却没人用, PB5 就这样一直是
 * MCU 复位后的"浮空输入"。模组到底有没有电完全看板上有没有上下拉电阻, 这属于
 * "供电状态不确定", 正是 BUSY 一直高、模组一声不吭的一种典型成因。
 * 现在显式配成推挽输出并拉高(上电)。若板上 PB5 不是 EN / 是低有效, 把下面改成 0
 * 或在工程 Define 里覆盖。 */
#ifndef LORA_EN_DRIVE
#define LORA_EN_DRIVE               1      /* 1 = 显式驱动 EN 脚 */
#endif
/* ★现场已确认: PB5 高电平给 LoRa 上电 ★ */
#ifndef LORA_EN_ACTIVE_HIGH
#define LORA_EN_ACTIVE_HIGH         1      /* 1 = 高电平给 LoRa 上电(本板) */
#endif

/* LoRa 模组是否已被判定为"不应答"(BUSY 一直高) —— 给别的模块查状态用 */
volatile uint8_t g_lora_busy_stuck = 0u;

/* 【#lora 诊断用】累计"单次等 BUSY 超过上限"的次数(只增不减)。
 *
 * 【为什么必须单独记一个累计值】原来只有 LLCC68WaitOnBusy() 里的 static 局部变量
 * s_stuck_ms, 而它在 BUSY 每次正常拉低时就被清零 —— 也就是说
 * "开机初始化那几百毫秒里到底卡过几次"事后【一点痕迹都查不到】。
 * 而现场正怀疑"初始化看着成功、配置其实没写进芯片": 只要某条 SPI 命令撞上超时,
 * 它就会带着"没等到 BUSY 就发出去"的数据继续跑, 那条配置(可能是 SetRxConfig /
 * SetRx)就等于没生效。所以这里给它一个只增不减的累计值, 由 #lora 打出来:
 *     为 0  = 所有 SPI 命令都等到 BUSY 拉低了, 配置确实是写进去的;
 *     不为 0 = 有命令超时, 芯片里的配置可能停留在旧值 —— 优先查这一条。 */
volatile uint32_t g_lora_busy_timeout_cnt = 0u;

/*------------------------- 软定时器 -------------------------*/
typedef struct __softTimer_s
{
    uint8_t  isRun;    /* 0停止,1运行              */
    uint32_t delayMs;  /* 定时长度 ms              */
    uint32_t startMs;  /* 定时开始的计数值         */
} SoftTimer_t;

static DioIrqHandler *dio1IrqCallback = NULL; /* 记录DIO1的回调(RadioOnDioIrq) */
static uint32_t timer_count = 0;              /* 毫秒计数器 */
static SoftTimer_t txTimerHandle, rxTimerHandle;

/*------------------------- GPIO 初始化 -------------------------*/
/* 初始化LLCC68需要的GPIO: NSS为推挽输出(软件控制), BUSY为输入,
 * SPI3引脚配置为AF1复用推挽, 其余未用模组引脚配置为浮空输入
 */
void LLCC68IoInit( void )
{
    GPIO_InitType GPIO_InitStructure;

    /* !! 禁用JTAG保留SWD，释放PA15/PB3/PB4为GPIO !! */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);
    GPIO_ConfigPinRemap(0, 0, GPIO_RMP_SWJ_SWD);

    /* GPIO端口在N32H482上挂AHB1总线 */
    RCC_EnableAHB1PeriphClk(RCC_AHB_PERIPHEN_GPIOA | RCC_AHB_PERIPHEN_GPIOB |
                            RCC_AHB_PERIPHEN_GPIOC | RCC_AHB_PERIPHEN_GPIOD, ENABLE);

    /* ---- EN(PB5): 显式给 LoRa 模组上电 ----
     * 原来这根脚【从头到尾没被配置过】: RADIO_EN_PIN 定义了但没人用, PB5 一直是
     * 复位后的浮空输入。模组有没有电全看板上的上下拉, 属于"供电状态不确定"。
     * 这里配成推挽输出并按 LORA_EN_ACTIVE_HIGH 拉高(上电)。
     * ⚠️ 上电后要给它一点时间起振/内部 LDO 稳定, 否则第一条 SPI 命令就会卡在
     *    "wait busy pin timeout"(BUSY 还没准备好)。 */
    GPIO_InitStruct(&GPIO_InitStructure);
#if LORA_EN_DRIVE
    /* 【关键标记】这一行和下面那行之间如果板子重启了, 那就是 LoRa 上电的浪涌
     * 把电源拉塌了(BOR 复位), 而不是 SPI/接线的问题 —— 现场就是靠这两行定位。 */
    log_info("[LORA] 给 LoRa 上电: EN(PB5) -> %s ...\r\n",
             (LORA_EN_ACTIVE_HIGH) ? "高" : "低");
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin            = RADIO_EN_PIN;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.GPIO_Pull      = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Slew_Rate = GPIO_SLEW_RATE_SLOW;
    GPIO_InitStructure.GPIO_Current   = GPIO_DC_4mA;
    GPIO_InitStructure.GPIO_Alternate = GPIO_NO_AF;
    GPIO_InitPeripheral(RADIO_EN_PORT, &GPIO_InitStructure);
#if LORA_EN_ACTIVE_HIGH
    GPIO_SetBits(RADIO_EN_PORT, RADIO_EN_PIN);      /* 高 = 上电 */
#else
    GPIO_ResetBits(RADIO_EN_PORT, RADIO_EN_PIN);    /* 低 = 上电 */
#endif

    /* ⚠️ 以前这里完全没有: 拉高 EN 之后必须等 LoRa 供电/内部 LDO 稳定, 否则第一条
     * SPI 命令就撞上 BUSY 还没准备好的窗口。这是"wait busy pin timeout"的一个成因。 */
    LLCC68DelayMs(20);

    log_info("[LORA] EN(PB5) 已驱动为%s(给模组上电), BUSY(PB3)=%u\r\n",
             (LORA_EN_ACTIVE_HIGH) ? "高" : "低",
             (unsigned)LLCC68BusyLevel());
#else
    log_info("[LORA] EN(PB5) 未驱动(LORA_EN_DRIVE=0), BUSY(PB3)=%u\r\n",
             (unsigned)LLCC68BusyLevel());
#endif

    /* NSS(PA15): 推挽输出, 空闲保持高电平
     * 注: PA15复位后默认为JTAG的JTDI(AF0), 配置为输出模式后自动释放该复用,
     *     使用SWD调试不受影响 */
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin            = RADIO_NSS_PIN;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.GPIO_Pull      = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Slew_Rate = GPIO_SLEW_RATE_FAST;
    GPIO_InitStructure.GPIO_Current   = GPIO_DC_4mA;
    GPIO_InitStructure.GPIO_Alternate = GPIO_NO_AF;
    GPIO_InitPeripheral(RADIO_NSS_PORT, &GPIO_InitStructure);
    GPIO_SetBits(RADIO_NSS_PORT, RADIO_NSS_PIN);

    /* BUSY(PB3): 上拉输入 */
    GPIO_InitStructure.Pin       = RADIO_DIO4_BUSY_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.GPIO_Pull = GPIO_PULL_UP;
    GPIO_InitPeripheral(RADIO_DIO4_BUSY_PORT, &GPIO_InitStructure);

    /* SCK(PC10)和MOSI(PC12): AF1复用推挽输出 */
    GPIO_InitStructure.Pin            = RADIO_SCK_PIN | RADIO_MOSI_PIN;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Pull      = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF_SPI3_SCK_PC10; /* AF1 */
    GPIO_InitPeripheral(RADIO_SCK_PORT, &GPIO_InitStructure);

    /* MISO(PC11): 主模式下必须为输入, 否则与模组输出冲突 */
    GPIO_InitStructure.Pin            = RADIO_MISO_PIN;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_INPUT;
    GPIO_InitStructure.GPIO_Pull      = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF_SPI3_MISO_PC11; /* AF1 */
    GPIO_InitPeripheral(RADIO_MISO_PORT, &GPIO_InitStructure);


    /* 模组未使用的引脚(TXEN=NONE/DIO2=NONE/DIO3=NONE/RXEN=NONE): 浮空输入 */
    GPIO_InitStructure.Pin            = RADIO_DIO0_TXEN_PIN | RADIO_DIO2_PIN |
                                        RADIO_DIO3_PIN | RADIO_DIO5_RXEN_PIN;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_INPUT;
    GPIO_InitStructure.GPIO_Pull      = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Alternate = GPIO_NO_AF;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);

    /* 使能SPI3时钟并初始化: 主模式/8bit/CPOL=0/CPHA=0/软NSS/MSB
     * SPI3挂APB1, 主频180MHz时PCLK1<=180MHz, 16分频<=11.25MHz, 低于LLCC68 SPI上限16MHz */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_SPI3, ENABLE);
    {
        SPI_InitType SPI_InitStructure;
        SPI_InitStructure.DataDirection = SPI_DIR_DOUBLELINE_FULLDUPLEX;
        SPI_InitStructure.SpiMode       = SPI_MODE_MASTER;
        SPI_InitStructure.DataLen       = SPI_DATA_SIZE_8BITS;
        SPI_InitStructure.CLKPOL        = SPI_CLKPOL_LOW;
        SPI_InitStructure.CLKPHA        = SPI_CLKPHA_FIRST_EDGE;
        SPI_InitStructure.NSS           = SPI_NSS_SOFT;
        SPI_InitStructure.BaudRatePres  = SPI_BR_PRESCALER_32;
        SPI_InitStructure.FirstBit      = SPI_FB_MSB;
        SPI_InitStructure.CRCPoly       = 7;
        SPI_Init(SPI3, &SPI_InitStructure);
        SPI_Enable(SPI3, ENABLE);
    }

    log_info("LLCC68IoInit init sucess\r\n");
}

/* 初始化 DIO1
 * DIO1 物理连接在 PB4 (见 project_config.h: RADIO_DIO1_PIN=GPIO_PIN_4),
 * 将 PB4 配置为外部中断 EXTI Line4 (上升沿触发), 中断回调函数为 dioIrq
 * (即 radio.c 中的 RadioOnDioIrq, 仅置位 IrqFired)
 * 注: EXTI Line4 属于 EXTI4_IRQn
 */
void LLCC68IoIrqInit( DioIrqHandler dioIrq )
{

    GPIO_InitType  GPIO_InitStructure;    
    EXTI_InitType  EXTI_InitStructure;
    NVIC_InitType  NVIC_InitStructure;

    dio1IrqCallback = dioIrq;

    /* DIO1(PB4): 下拉输入, 与模组输出电平匹配 */
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin       = RADIO_DIO1_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.GPIO_Pull = GPIO_PULL_DOWN;
    GPIO_InitPeripheral(RADIO_DIO1_PORT, &GPIO_InitStructure);

    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO,ENABLE);

    /* PB4 -> EXTI Line4 */
    GPIO_ConfigEXTILine(4, GPIOB_PORT_SOURCE, GPIO_PIN_SOURCE4);

    EXTI_InitStructure.EXTI_Line    = EXTI_LINE4;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitPeripheral(&EXTI_InitStructure);

    /* PB4 对应 EXTI Line4, 属于 EXTI4_IRQn  */
    NVIC_InitStructure.NVIC_IRQChannel                   = EXTI4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRE_PRIORITY_2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = NVIC_SUB_PRIORITY_3;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    EXTI_ClrITPendBit(EXTI_LINE4);
}

/* DIO1(PB4) 外部中断服务函数 —— EXTI4 共享中断, 需检查 Line4 */
void EXTI4_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_LINE4) != RESET)
    {
        if(NULL != dio1IrqCallback)
        {
            dio1IrqCallback(NULL);
        }
        EXTI_ClrITPendBit(EXTI_LINE4);
    }
}

void LLCC68IoDeInit( void )
{
    //GPIO去初始化(暂不需要)
}

/* 复位引脚初始化: 拉低RST至少20ms再释放为输入上拉 */
void LLCC68ResetInit( void )
{
    GPIO_InitType GPIO_InitStructure;

    LLCC68DelayMs( 10 );

    RCC_EnableAHB1PeriphClk(RCC_AHB_PERIPHEN_GPIOD, ENABLE);

    /* RST(PD2): 推挽输出并拉低复位 */
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin            = RADIO_nRESET_PIN;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.GPIO_Pull      = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Slew_Rate = GPIO_SLEW_RATE_SLOW;
    GPIO_InitStructure.GPIO_Current   = GPIO_DC_4mA;
    GPIO_InitStructure.GPIO_Alternate = GPIO_NO_AF;
    GPIO_InitPeripheral(RADIO_nRESET_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(RADIO_nRESET_PORT, RADIO_nRESET_PIN);

    LLCC68DelayMs( 20 );

    /* 释放RST: 切为输入上拉, 由模组内部上拉为高 */
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.GPIO_Pull = GPIO_PULL_UP;
    GPIO_InitPeripheral(RADIO_nRESET_PORT, &GPIO_InitStructure);

    LLCC68DelayMs( 10 );
}

/**
 * @brief 等 BUSY(PB3) 拉低, 带【上限】
 *
 * 【原来是死等, 这是本工程最贵的一个坑】
 *   原实现: while(BUSY==1){ 每 1000 次打一行 "wait busy pin timeout"; delay 1ms; }
 *   —— 没有退出条件。BUSY 一直高时(模组没供电 / 没焊 / EN 没拉 / SPI 接线断 /
 *   被复位), 它就永远转下去。而 LLCC68WaitOnBusy() 是【每条 SPI 命令开头】都要
 *   调的, 于是:
 *       RadioInit → LLCC68Init → 第一条命令卡住
 *       → lora_business_init() 回不来
 *       → main() 走不到 g4_at_init(), 4G 的代码一行都不执行
 *   现象就是日志里只有一秒钟一行的 "wait busy pin timeout", 而 4G 那边死寂。
 *
 * 【现在的策略】
 *   单次最多等 LLCC68_BUSY_TIMEOUT_MS(100ms); 正常情况 BUSY 几 us 就掉了, 这条
 *   超时永远不会触发。连续超时 LLCC68_BUSY_HARD_FAIL_CNT 次就认定"模组不在":
 *     · 置 g_lora_busy_stuck = 1, 打印【一次】清晰的诊断(不是每秒刷屏);
 *     · 之后每次只等 1ms 就返回, 保证主循环不会被拖死(4G 照常跑);
 *     · 错误日志限流(LLCC68_BUSY_LOG_INTERVAL_MS 打一次), 不再淹没其它日志。
 *   代价是 SPI 读回的数据是垃圾, 射频功能不可用 —— 但这是"坏掉的 LoRa"应有的
 *   表现, 而不是把整台设备一起拖死。
 */
void LLCC68WaitOnBusy( void )
{
    static uint32_t s_stuck_ms = 0u;    /* 累计"没应答"的毫秒数(近似), 兼作日志限流基准 */
    static uint32_t s_log_ms   = 0u;    /* 上次打印错误日志时的 s_stuck_ms */
    uint32_t limit = (g_lora_busy_stuck != 0u) ? LLCC68_BUSY_STUCK_WAIT_MS
                                               : LLCC68_BUSY_TIMEOUT_MS;
    uint32_t waited = 0u;

    while( GPIO_ReadInputDataBit(RADIO_DIO4_BUSY_PORT, RADIO_DIO4_BUSY_PIN) == 1 )
    {
        if( waited >= limit )
        {
            s_stuck_ms += limit;
            /* 【#lora 诊断】累计"这一次没等到 BUSY" —— 只加计数, 不改任何行为 */
            g_lora_busy_timeout_cnt++;

            if( g_lora_busy_stuck == 0u )
            {
                /* 连续攒够 "超时上限 × 次数" 的毫秒数 → 判定模组不在 */
                if( s_stuck_ms >= ((uint32_t)LLCC68_BUSY_TIMEOUT_MS * LLCC68_BUSY_HARD_FAIL_CNT) )
                {
                    g_lora_busy_stuck = 1u;
                    s_log_ms = s_stuck_ms;
                    log_info("\r\n[LORA] !! BUSY(PB3) 连续超时都不拉低 —— LoRa 模组不应答\r\n");
                    log_info("[LORA] !! 按这个顺序查:\r\n");
                    log_info("[LORA] !!  1) 模组有没有电: 量模组 VCC; 对照上面那行 EN(PB5) 电平\r\n");
                    log_info("[LORA] !!  2) EN(PB5) 是否真接到模组使能脚(高有效还是低有效)\r\n");
                    log_info("[LORA] !!  3) 量 PB3(BUSY): 恒定 3.3V = 模组没在驱动它(没电/没焊/线断)\r\n");
                    log_info("[LORA] !!  4) PD2(nRESET) 有没有被拉住; SPI3 PC10/PC11/PC12 通不通\r\n");
                    log_info("[LORA] !!  5) 暂时不想管 LoRa: 把 main.c 的 MAIN_LORA_ENABLE 设为 0\r\n");
                    log_info("[LORA] !! 从现在起不再死等(每次最多 %ums), 主循环继续跑 4G\r\n\r\n",
                             (unsigned)LLCC68_BUSY_STUCK_WAIT_MS);
                }
            }
            else if( (s_stuck_ms - s_log_ms) >= LLCC68_BUSY_LOG_INTERVAL_MS )
            {
                s_log_ms = s_stuck_ms;
                log_info("[LORA] LoRa 模组仍不应答(BUSY 一直高), 已跳过; 4G 不受影响\r\n");
            }

            return;     /* 【关键】不再死等 */
        }

        LLCC68DelayMs(1);
        waited++;
    }

    /* BUSY 正常拉低了: 说明模组是在应答的, 累计清零(不冤枉它) */
    s_stuck_ms = 0u;
    s_log_ms   = 0u;
}

/** @brief 当前 BUSY(PB3) 电平, 1 = 忙 / 模组没应答 */
uint8_t LLCC68BusyLevel( void )
{
    return (GPIO_ReadInputDataBit(RADIO_DIO4_BUSY_PORT, RADIO_DIO4_BUSY_PIN) != 0u) ? 1u : 0u;
}

/** @brief LoRa 模组是否已被判定为不应答(给 main/状态打印用) */
uint8_t LLCC68IsStuck( void )
{
    return g_lora_busy_stuck;
}

/** @brief 累计"等 BUSY 超时"次数(#lora 诊断用; 只增不减; 芯片正常时恒为 0)
 *  @note  不为 0 就意味着: 确实有 SPI 命令是在 BUSY 还高的时候发出去的 ——
 *         那一次写入(或读取)的结果不可信, 芯片里的配置可能没变成固件以为的样子。 */
uint32_t LLCC68BusyTimeoutCount( void )
{
    return g_lora_busy_timeout_cnt;
}


/* 控制SPI的NSS引脚电平: 0低电平选中, 非0高电平释放 */
void LLCC68SetNss(uint8_t lev)
{
    if(lev)
    {
        GPIO_SetBits(RADIO_NSS_PORT, RADIO_NSS_PIN);    /* 输出高电平 */
    }
    else
    {
        GPIO_ResetBits(RADIO_NSS_PORT, RADIO_NSS_PIN);  /* 输出低电平 */
    }
}

/* SPI收发一个字节的数据 */
uint8_t LLCC68SpiInOut(uint8_t data)
{
    /* 等待发送缓冲空 */
    while(SPI_I2S_GetStatus(SPI3, SPI_I2S_TE_FLAG) == RESET);
    SPI_I2S_TransmitData(SPI3, data);
    /* 等待接收缓冲非空(一个字节收发完成) */
    while(SPI_I2S_GetStatus(SPI3, SPI_I2S_RNE_FLAG) == RESET);
    return (uint8_t)SPI_I2S_ReceiveData(SPI3);
}

/* 检查频率是否需要支持, 本模组直接返回true */
bool LLCC68CheckRfFrequency( uint32_t frequency )
{
    return true;
}

/*------------------------- 延时 -------------------------*/
/* SysTick轮询实现阻塞毫秒延时(不使能SysTick中断)
 * 注意: 若工程中将SysTick用于RTOS心跳, 请改用工程自身的delay实现
 */
void LLCC68DelayMs(uint32_t ms)
{
    SysTick->LOAD = SystemCoreClock / 1000U - 1U;
    SysTick->VAL  = 0U;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    while(ms--)
    {
        while((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0U);
    }

    SysTick->CTRL = 0U;
}

/*------------------------- 收发超时定时器 -------------------------*/
/* 定时器中断服务函数
 * tx超时到期需要回调 RadioOnTxTimeoutIrq
 * rx超时到期需要回调 RadioOnRxTimeoutIrq
 */
void GTIM8_IRQHandler(void)
{
    uint32_t diffMs = 0;

    if(TIM_GetIntStatus(GTIM8, TIM_INT_UPDATE) != RESET)
    {
        timer_count++;
        TIM_ClrIntPendingBit(GTIM8, TIM_INT_UPDATE);

        /* 处理tx超时定时器 */
        if(txTimerHandle.isRun)
        {
            if(timer_count >= txTimerHandle.startMs)
            {
                diffMs = timer_count - txTimerHandle.startMs;
            }
            else /* 计数回绕 */
            {
                diffMs = 0xffffffff - txTimerHandle.startMs + timer_count;
            }
            if(diffMs > txTimerHandle.delayMs)
            {
                LLCC68TxTimerStop();
                txTimerHandle.startMs = timer_count;
                RadioOnTxTimeoutIrq(NULL);
            }
        }

        /* 处理rx超时定时器 */
        if(rxTimerHandle.isRun)
        {
            if(timer_count >= rxTimerHandle.startMs)
            {
                diffMs = timer_count - rxTimerHandle.startMs;
            }
            else /* 计数回绕 */
            {
                diffMs = 0xffffffff - rxTimerHandle.startMs + timer_count;
            }
            if(diffMs > rxTimerHandle.delayMs)
            {
                LLCC68RxTimerStop();
                rxTimerHandle.startMs = timer_count;
                RadioOnRxTimeoutIrq(NULL);
            }
        }
    }
}

/* 初始化定时器(GTIM8定时器模块, 产生1ms定时中断)
 * 注: GTIM8时钟源配置为SYSCLK(=SystemCoreClock), 不受APB分频影响;
 *     该配置为RCC_CFG2.GTIMCLKSEL全局位, 若工程中GTIM8/9/10另有用途需统一时钟方案
 */
void LLCC68TimerInit(void)
{
    TIM_TimeBaseInitType TIM_TimeBaseStructure;
    NVIC_InitType        NVIC_InitStructure;

    /* GTIM8时钟源选择SYSCLK, 并使能GTIM8时钟(挂APB2) */
    RCC_ConfigGTimClk(RCC_GTIM_CLKSRC_SYSCLK);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GTIM8, ENABLE);

    /* 定时器初始化: 预分频至10kHz计数, 周期10 -> 1ms更新中断 */
    TIM_InitTimBaseStruct(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.Prescaler   = SystemCoreClock / 10000U - 1U;
    TIM_TimeBaseStructure.CounterMode = TIM_CNT_MODE_UP;
    TIM_TimeBaseStructure.Period      = 10U - 1U;
    TIM_TimeBaseStructure.ClkDiv      = TIM_CLK_DIV1;
    TIM_InitTimeBase(GTIM8, &TIM_TimeBaseStructure);

    /* 中断优先级NVIC配置(优先级分组由主工程统一设置) */
    NVIC_InitStructure.NVIC_IRQChannel                   = GTIM8_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* 关闭软定时器 */
    txTimerHandle.isRun = 0;
    rxTimerHandle.isRun = 0;

    TIM_Enable(GTIM8, ENABLE);
    TIM_ClrIntPendingBit(GTIM8, TIM_INT_UPDATE);
    TIM_ConfigInt(GTIM8, TIM_INT_UPDATE, ENABLE);
}

void LLCC68SetTxTimerValue(uint32_t nMs)
{
    log_info("[%s()-%d]set timer out %d ms\r\n", __func__, __LINE__, nMs);
    txTimerHandle.delayMs = nMs;
}

void LLCC68TxTimerStart(void)
{
    log_info("[%s()-%d]start timer\r\n", __func__, __LINE__);
    txTimerHandle.startMs = timer_count;
    txTimerHandle.isRun   = 1;
}

void LLCC68TxTimerStop(void)
{
    log_info("[%s()-%d]stop timer\r\n", __func__, __LINE__);
    txTimerHandle.isRun = 0;
}

void LLCC68SetRxTimerValue(uint32_t nMs)
{
    log_info("[%s()-%d]set timer out %d ms\r\n", __func__, __LINE__, nMs);
    rxTimerHandle.delayMs = nMs;
}

void LLCC68RxTimerStart(void)
{
    log_info("[%s()-%d]start timer\r\n", __func__, __LINE__);
    rxTimerHandle.startMs = timer_count;
    rxTimerHandle.isRun   = 1;
}

void LLCC68RxTimerStop(void)
{
    log_info("[%s()-%d]stop timer\r\n", __func__, __LINE__);
    rxTimerHandle.isRun = 0;
}
