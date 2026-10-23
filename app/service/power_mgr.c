/**
 * @file  power_mgr.c
 * @brief 供电管理: 电池电压检测(PA7/ADC2) + 4G/LoRa/232/485 供电域开关
 *
 * 本文件只提供接口, 不接 main(见 power_mgr.h 的接入说明)。
 * 所有引脚宏都来自 bsp_gpio.h / project_config.h, 改板子只改那两处。
 */
#include <string.h>

#include "power_mgr.h"
#include "bsp_gpio.h"
#include "bsp_tim.h"
#include "log.h"
#include "project_config.h"     /* RADIO_EN_PORT/PIN(PB5) */
#include "n32h47x_48x_adc.h"
#include "n32h47x_48x_gpio.h"
#include "n32h47x_48x_rcc.h"

/* 232_EN 的使能极性(板上默认高使能; 若实测相反在工程 Define 里覆盖) */
#ifndef PWR_RS232_EN_ACTIVE_HIGH
#define PWR_RS232_EN_ACTIVE_HIGH   1
#endif

/* 采样: 12 位 / 16 次平均 / 最长等 1 ms 左右 */
#define PWR_ADC_SAMPLES        16u
#define PWR_ADC_WAIT_LOOP      100000u

static bool     s_inited;
static bool     s_cut;                       /* 低电锁存 */
static uint32_t s_cut_cnt;
static uint16_t s_cut_mv   = PWR_BAT_CUT_MV;
static uint16_t s_restore_mv = PWR_BAT_RESTORE_MV;
static uint8_t  s_low_cnt;
static uint32_t s_last_ms;
static bool     s_dom_on[PWR_DOM_COUNT];

/* -------------------------------------------------------------------------
 * 通用: 把一个引脚配成推挽输出并驱动到指定电平
 * ------------------------------------------------------------------------- */
static void pwr_pin_output(GPIO_Module *port, uint32_t clk, uint16_t pin, uint8_t high)
{
    GPIO_InitType g;

    GPIO_InitStruct(&g);
    RCC_EnableAHB1PeriphClk(clk, ENABLE);

    g.Pin            = pin;
    g.GPIO_Mode      = GPIO_MODE_OUTPUT_PP;
    g.GPIO_Pull      = GPIO_NO_PULL;
    g.GPIO_Slew_Rate = GPIO_SLEW_RATE_SLOW;
    g.GPIO_Current   = GPIO_DC_4mA;
    g.GPIO_Alternate = GPIO_NO_AF;
    GPIO_InitPeripheral(port, &g);

    if (high != 0u)
    {
        GPIO_SetBits(port, pin);
    }
    else
    {
        GPIO_ResetBits(port, pin);
    }
}

/* 232 EN: 高=使能 */
static uint8_t pwr_232_en_level(bool on)
{
#if PWR_RS232_EN_ACTIVE_HIGH
    return (on) ? 1u : 0u;
#else
    return (on) ? 0u : 1u;
#endif
}

/* RS485 EN: 使能电平由 RS485_EN_ACTIVE_LOW 决定 */
static uint8_t pwr_485_en_level(bool on)
{
#if RS485_EN_ACTIVE_LOW
    return (on) ? 0u : 1u;
#else
    return (on) ? 1u : 0u;
#endif
}

/* LoRa EN: 高=上电(LORA_EN_ACTIVE_HIGH) */
static uint8_t pwr_lora_en_level(bool on)
{
#if LORA_EN_ACTIVE_HIGH
    return (on) ? 1u : 0u;
#else
    return (on) ? 0u : 1u;
#endif
}

/* -------------------------------------------------------------------------
 * PA7 + ADC2 初始化(12 位, 单通道, 软件触发, 239.5 周期采样)
 * ------------------------------------------------------------------------- */
static void pwr_adc_init(void)
{
    GPIO_InitType g;
    ADC_InitType  a;
    uint32_t      i;

    /* PA7 = ADC2_IN2, 模拟输入 */
    GPIO_InitStruct(&g);
    RCC_EnableAHB1PeriphClk(RCC_AHB_PERIPHEN_GPIOA, ENABLE);
    g.Pin            = GPIO_PIN_7;
    g.GPIO_Mode      = GPIO_MODE_ANALOG;
    g.GPIO_Pull      = GPIO_NO_PULL;
    g.GPIO_Alternate = GPIO_NO_AF;
    GPIO_InitPeripheral(GPIOA, &g);

    /* ADC 在 AHB 域; HCLK 较高, 用 4 分频保证 ADC 时钟在规格内 */
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPHEN_ADC2, ENABLE);
    ADC_ConfigClk(ADC_CTRL3_CKMOD_AHB, RCC_ADCHCLK_DIV4);

    ADC_InitStruct(&a);
    a.WorkMode        = ADC_WORKMODE_INDEPENDENT;
    a.MultiChEn       = DISABLE;
    a.ContinueConvEn  = DISABLE;
    a.ExtTrigSelect   = ADC_EXT_TRIG_REG_CONV_SOFTWARE;
    a.DatAlign        = ADC_DAT_ALIGN_R;
    a.ChsNumber       = 1u;
    a.Resolution      = ADC_DATA_RES_12BIT;
    ADC_Init(ADC2, &a);

    ADC_ConfigRegularChannel(ADC2, ADC2_Channel_02_PA7, 1u, ADC_SAMP_TIME_CYCLES_239_5);

    ADC_Enable(ADC2, ENABLE);

    /* 等 ADC 就绪(有上限, 不让初始化卡死) */
    for (i = 0u; i < PWR_ADC_WAIT_LOOP; i++)
    {
        if (ADC_GetFlagStatus(ADC2, (uint16_t)ADC_FLAG_RDY) != RESET)
        {
            break;
        }
    }

    /* 单端模式校准: 寄存器文档约定 "RESET = 校准完成" */
    ADC_CalibrationOperation(ADC2, ADC_CALIBRATION_SINGLE_MODE);
    for (i = 0u; i < PWR_ADC_WAIT_LOOP; i++)
    {
        if (ADC_GetCalibrationStatus(ADC2, ADC_CALIBRATION_SINGLE_MODE) == RESET)
        {
            break;
        }
    }
}

/* -------------------------------------------------------------------------
 * 初始化
 * ------------------------------------------------------------------------- */
void pwr_init(void)
{
    uint8_t i;

    if (s_inited)
    {
        return;
    }

    /* 232 / 485 明确不用 -> 初始化就关掉(EN 拉到"不使能"电平) */
    pwr_pin_output(RS232_GPIO_PORT, RCC_AHB_PERIPHEN_GPIOA,
                   RS232_EN_PIN, pwr_232_en_level(false));
    pwr_pin_output(RS485_0_EN_PORT, RCC_AHB_PERIPHEN_GPIOC,
                   RS485_0_EN_PIN, pwr_485_en_level(false));
    pwr_pin_output(RS485_1_EN_PORT, RCC_AHB_PERIPHEN_GPIOH,
                   RS485_1_EN_PIN, pwr_485_en_level(false));

    pwr_adc_init();

    for (i = 0u; i < (uint8_t)PWR_DOM_COUNT; i++)
    {
        s_dom_on[i] = false;
    }
    /* 4G 由 G4_PowerSwitch 维护: 按它当前驱动的电平反推"是否上电";
     * LoRa 的 EN 在射频初始化里拉高, 这里按"已上电"记录, 不主动改它的电平。 */
    s_dom_on[PWR_DOM_4G]   = (G4_PowerSwitch_GetLevel() == G4_PowerSwitch_GetOnHigh());
    s_dom_on[PWR_DOM_LORA] = true;

    s_cut     = false;
    s_cut_cnt = 0u;
    s_low_cnt = 0u;
    s_last_ms = 0u;
    s_inited  = true;

    log_info("[PWR] 初始化: 232/485 EN 已关, PA7 ADC2 就绪, 阈值 %umV\r\n",
             (unsigned)s_cut_mv);
}

/* -------------------------------------------------------------------------
 * 电池采样
 * ------------------------------------------------------------------------- */
bool pwr_bat_sample(pwr_bat_info_t *out)
{
    uint32_t sum = 0u;
    uint32_t i;
    uint8_t  n;
    uint16_t raw;
    uint16_t adc_mv;
    uint16_t bat_mv;

    if (!s_inited)
    {
        return false;
    }

    for (n = 0u; n < PWR_ADC_SAMPLES; n++)
    {
        ADC_ClearFlag(ADC2, (uint16_t)ADC_FLAG_ENDC);
        ADC_EnableSoftwareStartConv(ADC2, ENABLE);

        for (i = 0u; i < PWR_ADC_WAIT_LOOP; i++)
        {
            if (ADC_GetFlagStatus(ADC2, (uint16_t)ADC_FLAG_ENDC) != RESET)
            {
                break;
            }
        }
        if (i >= PWR_ADC_WAIT_LOOP)
        {
            return false;               /* ADC 没响应, 本次结果作废 */
        }

        sum += (uint32_t)ADC_GetDat(ADC2);
    }

    raw = (uint16_t)(sum / PWR_ADC_SAMPLES);
    /* 引脚 mV: 12 位满量程 4095 对应 VDDA */
    adc_mv = (uint16_t)(((uint32_t)raw * PWR_BAT_ADC_REF_MV) / 4095u);
    /* 还原电池电压: 分压比 x100(200 = 1/2 分压, 即乘 2) */
    bat_mv = (uint16_t)(((uint32_t)adc_mv * PWR_BAT_DIV_X100) / 100u);

    if (out != NULL)
    {
        out->adc_raw = raw;
        out->adc_mv  = adc_mv;
        out->bat_mv  = bat_mv;
        out->low     = (bat_mv < s_cut_mv) ? 1u : 0u;
    }

    return true;
}

uint16_t pwr_bat_mv(void)
{
    pwr_bat_info_t info;

    if (!pwr_bat_sample(&info))
    {
        return 0u;
    }
    return info.bat_mv;
}

/* -------------------------------------------------------------------------
 * 供电域开关
 * ------------------------------------------------------------------------- */
void pwr_set_domain(pwr_domain_t dom, bool on)
{
    if ((uint32_t)dom >= (uint32_t)PWR_DOM_COUNT)
    {
        return;
    }

    switch (dom)
    {
    case PWR_DOM_4G:
        /* 断电时 G4_PowerSwitch_Set 会先放开 USART2_TX, 防止反向灌电 */
        G4_PowerSwitch_Set((on) ? 1u : 0u);
        break;

    case PWR_DOM_LORA:
        /* EN 可能还没被射频初始化配成输出, 这里补一次配置再驱动 */
        pwr_pin_output(RADIO_EN_PORT, RCC_AHB_PERIPHEN_GPIOB,
                       RADIO_EN_PIN, pwr_lora_en_level(on));
        break;

    case PWR_DOM_RS232:
        pwr_pin_output(RS232_GPIO_PORT, RCC_AHB_PERIPHEN_GPIOA,
                       RS232_EN_PIN, pwr_232_en_level(on));
        break;

    case PWR_DOM_RS485_0:
        pwr_pin_output(RS485_0_EN_PORT, RCC_AHB_PERIPHEN_GPIOC,
                       RS485_0_EN_PIN, pwr_485_en_level(on));
        break;

    case PWR_DOM_RS485_1:
        pwr_pin_output(RS485_1_EN_PORT, RCC_AHB_PERIPHEN_GPIOH,
                       RS485_1_EN_PIN, pwr_485_en_level(on));
        break;

    default:
        return;
    }

    s_dom_on[dom] = on;

    log_info("[PWR] %s -> %s\r\n",
             (dom == PWR_DOM_4G) ? "4G" :
             (dom == PWR_DOM_LORA) ? "LoRa" :
             (dom == PWR_DOM_RS232) ? "232" :
             (dom == PWR_DOM_RS485_0) ? "485_0" : "485_1",
             (on) ? "ON" : "OFF");
}

bool pwr_get_domain(pwr_domain_t dom)
{
    if ((uint32_t)dom >= (uint32_t)PWR_DOM_COUNT)
    {
        return false;
    }
    return s_dom_on[dom];
}

void pwr_cut_all(void)
{
    pwr_set_domain(PWR_DOM_4G, false);
    pwr_set_domain(PWR_DOM_LORA, false);
    pwr_set_domain(PWR_DOM_RS232, false);
    pwr_set_domain(PWR_DOM_RS485_0, false);
    pwr_set_domain(PWR_DOM_RS485_1, false);
    s_cut = true;
    s_cut_cnt++;
}

void pwr_restore_all(void)
{
    /* 232/485 已明确"不用", 保持关闭; 只恢复真正要工作的 4G 与 LoRa。
     * ⚠️ LoRa 重新上电后芯片内部配置丢失, 调用方需要重新初始化射频。 */
    pwr_set_domain(PWR_DOM_4G, true);
    pwr_set_domain(PWR_DOM_LORA, true);

    s_cut     = false;
    s_low_cnt = 0u;

    log_info("[PWR] 已恢复供电(4G/LoRa), 232/485 仍保持关闭\r\n");
}

bool pwr_is_cut(void)
{
    return s_cut;
}

uint32_t pwr_cut_count(void)
{
    return s_cut_cnt;
}

void pwr_set_thresholds(uint16_t cut_mv, uint16_t restore_mv)
{
    if ((cut_mv != 0u) && (restore_mv != 0u) && (cut_mv < restore_mv))
    {
        s_cut_mv     = cut_mv;
        s_restore_mv = restore_mv;
    }
}

uint16_t pwr_cut_threshold_mv(void)
{
    return s_cut_mv;
}

uint16_t pwr_restore_threshold_mv(void)
{
    return s_restore_mv;
}

/* -------------------------------------------------------------------------
 * 周期任务
 * ------------------------------------------------------------------------- */
void pwr_poll(uint32_t now_ms)
{
    pwr_bat_info_t info;

    if (!s_inited)
    {
        return;
    }
    if ((uint32_t)(now_ms - s_last_ms) < PWR_SAMPLE_PERIOD_MS)
    {
        return;
    }
    s_last_ms = now_ms;

    if (!pwr_bat_sample(&info))
    {
        return;                         /* 读失败不当成低电, 保持上次状态 */
    }

    if (!s_cut)
    {
        if (info.bat_mv < s_cut_mv)
        {
            s_low_cnt++;
            if (s_low_cnt >= PWR_LOW_CONFIRM_CNT)
            {
                log_info("[PWR] 电池 %umV < %umV, 连续 %u 次, 断电\r\n",
                         (unsigned)info.bat_mv, (unsigned)s_cut_mv,
                         (unsigned)s_low_cnt);
                pwr_cut_all();
            }
        }
        else
        {
            s_low_cnt = 0u;
        }
    }
    /* 已锁存: 不自动恢复。何时恢复由上层决定(策略待定), 调 pwr_restore_all()。 */
}
