#ifndef __BSP_GPIO_H__
#define __BSP_GPIO_H__
#include <stdint.h>

/* 主机单元测试用: 4G 模组电源开关(PA8)的替身 */
void    G4_ModulePWR_GPIO_Init(void);
void    G4_PowerSwitch_Init(void);
void    G4_PowerSwitch_InitNoPower(void);
void    G4_PowerSwitch_Set(uint8_t on);
uint8_t G4_PowerSwitch_GetLevel(void);
uint8_t G4_PowerSwitch_ReadPin(void);
uint8_t G4_PowerSwitch_GetOnHigh(void);
void    G4_PowerSwitch_SwapOnHigh(void);
void    G4_PowerCycle(uint32_t off_ms);

/* 测试用: 4G_RST(断电重启)被调用了几次 */
uint32_t host_pwr_cycle_count(void);

#endif
