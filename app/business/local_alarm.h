#ifndef __LOCAL_ALARM_H__
#define __LOCAL_ALARM_H__
#include <stdint.h>

typedef enum
{
    ALARM_NONE              = 0,    // 无告警
    ALARM_MODBUS_TIMEOUT    = 1,    // Modbus应答超时(硬件预留，暂不用)
    ALARM_LORA_OFFLINE      = 2,    // LoRa从机离线
    ALARM_4G_OFFLINE        = 3,    // 4G模块离线
    ALARM_POWER_ABNORMAL    = 4,    // 电源异常
    ALARM_SENSOR_FAULT      = 5,    // 传感器故障
    ALARM_MAX_NUM                   // 告警总数，用于数组边界
} local_alarm_code_t;

typedef struct
{
    local_alarm_code_t code;    // 告警码
    uint8_t is_active;          // 1:告警激活；0:告警已清除
    uint32_t trigger_tick;      // 告警发生tick(ms)，来自get_tick_ms()
    uint32_t clear_tick;        // 告警清除tick(ms)，未清除则为0
} local_alarm_item_t;

void local_alarm_init(void);
void local_alarm_trigger(local_alarm_code_t code);
void local_alarm_clear(local_alarm_code_t code);
uint8_t local_alarm_is_active(local_alarm_code_t code);
uint8_t local_alarm_get_active_count(void);
const local_alarm_item_t* local_alarm_get_item(local_alarm_code_t code);

#endif