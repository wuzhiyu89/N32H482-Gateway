#ifndef __BSP_TIM_H__
#define __BSP_TIM_H__
#include <stdint.h>

/* 主机单元测试用: 虚拟毫秒时基, 由测试用例推进 */
uint32_t get_tick_ms(void);
void     delay_ms(uint32_t ms);

void host_tick_set(uint32_t ms);
void host_tick_advance(uint32_t ms);

/* 测试用: 每次虚拟时间推进都会回调一次(让"自旋等待"期间假模组也能应答) */
extern void (*host_tick_hook)(void);

/** @brief 读当前虚拟时间(不影响自旋计数, 测试里做调度用) */
uint32_t host_tick_get(void);

#endif
