#include "bsp_tim.h"
#include "n32h47x_48x.h"
#include "misc.h"

static volatile uint32_t sys_tick_ms = 0U;

/**
 * @brief ATIM1初始化，1ms系统时基中断
 *        SYSCLK=240MHz (HSI 8MHz×PLL30 或 HSE 16MHz÷2×PLL30, 由 BOARD_USE_HSE 选择)
 *        HCLK=240MHz, ATIM1 计数时钟=HCLK, Prescaler=239 → 1MHz, ARR=999 → 1ms
 */
void bsp_tim1_init_1ms(void)
{
    TIM_TimeBaseInitType tim_base_struct;

    /* ATIM1 在AHB域，使用 RCC_EnableAHBPeriphClk */
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPHEN_ATIM1, ENABLE);

    /* 结构体填充默认参数 */
    TIM_InitTimBaseStruct(&tim_base_struct);

    /* 240M/(239+1)=1MHz计数时钟，ARR=999 → 1ms溢出 */
    tim_base_struct.Prescaler  = 239U;
    tim_base_struct.CounterMode= TIM_CNT_MODE_UP;
    tim_base_struct.Period     = 999U;
    tim_base_struct.ClkDiv     = TIM_CLK_DIV1;
    tim_base_struct.RepetCnt   = 0U;

    TIM_InitTimeBase(ATIM1, &tim_base_struct);

    /* 使能更新中断 */
    TIM_ConfigInt(ATIM1, TIM_INT_UPDATE, ENABLE);

    /* CMSIS NVIC配置，抢占2，子优先级0 */
    NVIC_SetPriority(ATIM1_UP_IRQn, NVIC_EncodePriority(5, 2, 0));
    NVIC_EnableIRQ(ATIM1_UP_IRQn);

    /* 打开定时器 */
    TIM_Enable(ATIM1, ENABLE);
}

uint32_t get_tick_ms(void)
{
    uint32_t tick;
    __disable_irq();
    tick = sys_tick_ms;
    __enable_irq();
    return tick;
}

/**
 * @brief 1ms时基累加,仅供 ATIM1_UP_IRQHandler 调用
 */
void bsp_tim_tick_inc(void)
{
    sys_tick_ms++;
}

/**
 * @brief 基于 ATIM1 1ms 时基的阻塞延时
 *
 * @note  必须已在 main 中调用 bsp_tim1_init_1ms()。
 *
 *        这里刻意不用 SysTick 的 COUNTFLAG 轮询: LLCC68 板级驱动的
 *        LLCC68DelayMs() 与 firmware 的 systick_delay_ms() 都会在结束时
 *        关闭 SysTick(CTRL=0), 若 delay_ms 依赖 SysTick 就会死等。
 *        改用 ATIM1 时基后, 与 LoRa/USB 驱动的延时实现完全解耦。
 */
void delay_ms(uint32_t ms)
{
    uint32_t start = get_tick_ms();

    while ((uint32_t)(get_tick_ms() - start) < ms)
    {
        /* 等待 ATIM1 时基推进(中断保持开启) */
    }
}

