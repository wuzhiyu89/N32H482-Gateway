#ifndef __POWER_MGR_H__
#define __POWER_MGR_H__

#include <stdint.h>
#include <stdbool.h>

/* ==========================================================================
 * 供电管理(电池检测 + 外设供电域开关)
 * ==========================================================================
 * 【这一版只提供接口, 不接 main】 —— 何时恢复供电的策略还没定, 所以:
 *   · `pwr_poll()` 只做"采样 -> 去抖 -> 低电时一键断电并锁存";
 *   · 恢复只能显式调 `pwr_restore_all()`(或按域 `pwr_set_domain()`),
 *     不会因为电压回弹就自己上电(避免在 3.5V 附近反复拉电)。
 *
 * 【板上的供电域(见 bsp_gpio.h 引脚表)】
 *   4G    PA8 : 低=上电, 高=断电(极性由 G4_PowerSwitch 维护, 这里不写死)
 *   LoRa  PB5 : 高=上电, 低=断电(RADIO_EN_*)
 *   232   PA2 : 232_EN, 高=使能(默认关)
 *   485_0 PC4 : 4850_EN, 使能极性见 RS485_EN_ACTIVE_LOW(默认关)
 *   485_1 PH3 : 4851_EN, 同上(默认关)
 *
 * 【电池检测】
 *   PA7 (BAT_AD) -> ADC2 通道 2, 12 位, 16 次平均。
 *   电池电压 = 引脚电压 / 分压比, 分压比用 PWR_BAT_DIV_X100 表示(200 = 1/2 分压)。
 *   ⚠️ 分压电阻各版板子可能不同: 第一次上板请用万用表量电池正极与 PA7 的比值,
 *      再改 PWR_BAT_DIV_X100。参考电压按 VDDA=3300mV 计算(PWR_BAT_ADC_REF_MV)。
 *
 * 【接入方式(等你确认后再改 app_boot, 本次不动 main)】
 *   在 RS232_GPIO_Init()/RS485_0_GPIO_Init()/RS485_1_GPIO_Init() 之后调 pwr_init(),
 *   在 app_loop 里按 >=1s 的节奏调 pwr_poll(get_tick_ms())。
 *   调用顺序反了的话, 后面的 UART GPIO 初始化会把 EN 重新拉回"使能"。
 * ========================================================================== */

/* 电池电压阈值(mV), 可用 pwr_set_thresholds() 在运行期覆盖 */
#ifndef PWR_BAT_CUT_MV
#define PWR_BAT_CUT_MV          3500u   /* 低于它 -> 断电 */
#endif
#ifndef PWR_BAT_RESTORE_MV
#define PWR_BAT_RESTORE_MV      3700u   /* 预留的恢复阈值(本版不自动用) */
#endif

/* 分压比 x100: 200 = 2.0 倍(1/2 分压)。★上板必须实测核对★ */
#ifndef PWR_BAT_DIV_X100
#define PWR_BAT_DIV_X100        200u
#endif
/* ADC 参考电压(VDDA), mV */
#ifndef PWR_BAT_ADC_REF_MV
#define PWR_BAT_ADC_REF_MV      3300u
#endif

/* 采样与去抖 */
#ifndef PWR_SAMPLE_PERIOD_MS
#define PWR_SAMPLE_PERIOD_MS    1000u   /* pwr_poll 内部的最小采样间隔 */
#endif
#ifndef PWR_LOW_CONFIRM_CNT
#define PWR_LOW_CONFIRM_CNT     3u      /* 连续 3 次低于阈值才断电(避开 4G 发射压降) */
#endif

/* 供电域 */
typedef enum
{
    PWR_DOM_4G = 0,     /* PA8, 通过 G4_PowerSwitch_* 操作 */
    PWR_DOM_LORA,       /* PB5 */
    PWR_DOM_RS232,      /* PA2 */
    PWR_DOM_RS485_0,    /* PC4 */
    PWR_DOM_RS485_1,    /* PH3 */
    PWR_DOM_COUNT
} pwr_domain_t;

/* 一次电池采样的结果 */
typedef struct
{
    uint16_t adc_raw;   /* 12 位原始均值(0..4095) */
    uint16_t adc_mv;    /* PA7 引脚电压(mV) —— 分压后的值 */
    uint16_t bat_mv;    /* 还原后的电池电压(mV) */
    uint8_t  low;       /* 1 = 低于当前断电阈值 */
} pwr_bat_info_t;

/**
 * @brief 初始化: 关掉 232/485 的 EN, 配置 PA7+ADC2
 * @note  必须在 RS232/RS485 的 GPIO 初始化【之后】调用;
 *        不改变 4G 和 LoRa 当前的供电状态(它们由各自模块初始化负责)。
 */
void pwr_init(void);

/**
 * @brief 采一次电池电压(16 次平均)
 * @param out 结果; 为空则只做一次采样不返回数据
 * @return true 采样成功; false 未初始化 / ADC 超时
 */
bool pwr_bat_sample(pwr_bat_info_t *out);

/** 便捷接口: 返回电池 mV; 0 = 本次采样失败(与"真的 0V"区分不了, 需要区分请用 pwr_bat_sample) */
uint16_t pwr_bat_mv(void);

/** 当前是否处于"已判定低电并断电"的锁存状态 */
bool pwr_is_cut(void);

/** 上电以来触发断电的次数(累计, 只增) */
uint32_t pwr_cut_count(void);

/** 运行期覆盖阈值(cut 必须 < restore; 传 0 表示保持原值) */
void pwr_set_thresholds(uint16_t cut_mv, uint16_t restore_mv);

/** 读取当前生效的断电阈值(mV) */
uint16_t pwr_cut_threshold_mv(void);

/** 读取当前生效的恢复阈值(mV) —— 本版不做自动恢复, 仅供上层判断用 */
uint16_t pwr_restore_threshold_mv(void);

/** 单个供电域开/关(立即生效; 关 4G 会同步放开 USART2_TX 防止反向灌电) */
void pwr_set_domain(pwr_domain_t dom, bool on);

/** 查询某个域当前的软件状态(不等于引脚实测电平) */
bool pwr_get_domain(pwr_domain_t dom);

/**
 * @brief 低电断电: 4G / LoRa / 232 / 485_0 / 485_1 全部关掉, 并锁存
 * @note  可重复调用; 已关的域不会重复动作。
 */
void pwr_cut_all(void);

/**
 * @brief 恢复供电(目前只恢复 4G 与 LoRa)
 * @note  232/485 已明确"不用", 所以保持关闭; 需要时用 pwr_set_domain() 单独打开。
 *        ⚠️ LoRa 重新上电后射频芯片状态丢失, 需要重新走一遍射频初始化才能收发。
 */
void pwr_restore_all(void);

/**
 * @brief 周期任务: 每 PWR_SAMPLE_PERIOD_MS 采一次电池, 连续低电则断电并锁存
 * @param now_ms 调用方的毫秒时基(get_tick_ms())
 * @note  未初始化直接返回; 采样失败保持上次状态(不因一次读失败就断电)。
 *        低电锁存后【不会】自动恢复 —— 恢复策略待定, 由调用方决定何时调
 *        pwr_restore_all()。
 */
void pwr_poll(uint32_t now_ms);

#endif /* __POWER_MGR_H__ */
