#include "local_alarm.h"
#include "bsp_tim.h"
#include <string.h>

/* 内部告警状态数组，RAM全局，断电丢失，属于运行时状态 */
static local_alarm_item_t s_alarm_list[ALARM_MAX_NUM];

/**
 * @brief 初始化告警模块，全部告警置为未激活
 */
void local_alarm_init(void)
{
    memset(s_alarm_list, 0, sizeof(s_alarm_list));
}

/**
 * @brief 触发一条告警
 * @param code 告警码
 */
void local_alarm_trigger(local_alarm_code_t code)
{
    if(code >= ALARM_MAX_NUM)
    {
        return;
    }

    local_alarm_item_t *p_item = &s_alarm_list[code];

    /* 如果已经激活，不再重复更新触发时间 */
    if(p_item->is_active != 1U)
    {
        p_item->code = code;
        p_item->is_active = 1U;
        p_item->trigger_tick = get_tick_ms();
        p_item->clear_tick = 0U;
    }
}

/**
 * @brief 清除一条告警
 * @param code 告警码
 */
void local_alarm_clear(local_alarm_code_t code)
{
    if(code >= ALARM_MAX_NUM)
    {
        return;
    }

    local_alarm_item_t *p_item = &s_alarm_list[code];
    if(p_item->is_active == 1U)
    {
        p_item->is_active = 0U;
        p_item->clear_tick = get_tick_ms();
    }
}

/**
 * @brief 查询该告警是否处于激活状态
 * @param code 告警码
 * @return 1激活，0未激活
 */
uint8_t local_alarm_is_active(local_alarm_code_t code)
{
    if(code >= ALARM_MAX_NUM)
    {
        return 0U;
    }
    return s_alarm_list[code].is_active;
}

/**
 * @brief 获取当前激活告警总数量
 * @return 激活告警数目
 */
uint8_t local_alarm_get_active_count(void)
{
    uint8_t cnt = 0U;
    for(uint8_t i = 0U; i < ALARM_MAX_NUM; i++)
    {
        if(s_alarm_list[i].is_active == 1U)
        {
            cnt++;
        }
    }
    return cnt;
}

/**
 * @brief 获取告警项指针，供业务读取告警信息
 * @param code 告警码
 * @return 返回告警结构体指针，NULL代表无效
 */
const local_alarm_item_t* local_alarm_get_item(local_alarm_code_t code)
{
    if(code >= ALARM_MAX_NUM)
    {
        return NULL;
    }
    return &s_alarm_list[code];
}