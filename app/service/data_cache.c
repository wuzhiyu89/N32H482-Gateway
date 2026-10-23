/**
 * @file  data_cache.c
 * @brief 设备数据缓存 —— ★2026-10-08 起只剩【下发缓存】★
 *
 * 【本次删掉了什么, 为什么】
 *   原来这里还有一套"上行传感器上报环形缓存"(`g_cache_up_buf` + peek/pop/drop/
 *   get_count)和一个 `data_cache_poll()` —— 后者的唯一作用是把队列里最旧的一条交给
 *   `lora_business_publish_report()` 组帧成 hex, 发到那条"默认 MQTT 主题"
 *   `mod/mod_upload`。
 *
 *   甲方明确要求不再使用 `mod/mod_upload`, 历史残留一并清除; 而那条通道本来就和
 *   `rt/gw/{gwId}/telemetry` 是【同一份数据的第二条路】—— 留着只会让平台侧
 *   面对"两个主题各来一遍、可能还不一致"。所以整条 hex 上行通道删掉了。
 *
 * 【上行数据没有丢, 一点都没少】
 *   `data_cache_up_push()` 保留着 —— 因为它【同时】是 `last_value` 快照表的唯一漏斗
 *   (`last_value_update_ex`)。`mod/telemetry` 的 JSON 正是从那张快照表取值的,
 *   所以上游 0x02 解析、快照更新、心跳/周期上报这条主路径【完全不受影响】。
 *   变的是: 不再往一个 FIFO 里塞、也没有人再把它发成 hex。
 */
#include "data_cache.h"
#include "last_value.h"     /* 按 uid 的"最后值"快照表(纯逻辑模块, 主机可测) */
#include "bsp_tim.h"        /* get_tick_ms() */
#include <string.h>

/* 下发：网关待下发指令缓存（普通数组，非环形；同一个uid只允许存在一条待下发） */
static cache_down_item_t g_cache_down_buf[DATA_CACHE_DOWN_MAX_ITEM];

void data_cache_init(void)
{
    memset(g_cache_down_buf, 0, sizeof(g_cache_down_buf));

    last_value_reset();     /* 最后值快照表与缓存同生命周期 */
}

/********************* 上行: 只更新"最后值"快照 *********************/
void data_cache_up_push(const uint8_t *uid, uint16_t level_mm, uint16_t bat_mv,
                        uint32_t time_mdhhm, int8_t rssi_dbm)
{
    /* 【记录"最后值"快照】供 sen.query / dev.query 按 uid 查最近一次上报,
     * 也供 main.c 组装 rt/gw/{gwId}/telemetry 的 JSON。
     *
     * 本函数是【所有传感器上报的唯一漏斗】, 所以放在这里 —— 调用方一行都不用改。
     * 注: 形参名 bat_mv 是历史遗留, 实参传的是电量百分比(见调用点 bat_pct);
     *     uid 的 const 与 rssi_dbm 一路是 2026-09-30 第 10 条加的, 供遥测的
     *     sensors[].rssi 用。 */
    (void)last_value_update_ex(uid, level_mm, bat_mv, time_mdhhm, rssi_dbm, get_tick_ms());
}

/********************* 下发缓存实现 *********************/
int data_cache_down_push(uint8_t *uid, slave_type_t dev_type, uint8_t *buf, uint8_t len)
{
    if (len > sizeof(g_cache_down_buf[0].payload)) return -1;
    /* 先判断该uid是否已经存在待下发条目，存在则覆盖旧的 */
    for(uint8_t i = 0; i < DATA_CACHE_DOWN_MAX_ITEM; i++)
    {
        if(g_cache_down_buf[i].valid == true && (memcmp(g_cache_down_buf[i].dev_uid, uid, sizeof(((last_value_t*)0)->uid)) == 0))
        {
            memcpy(g_cache_down_buf[i].dev_uid, uid, sizeof(((last_value_t*)0)->uid));
            g_cache_down_buf[i].dev_type = dev_type;
            memcpy(g_cache_down_buf[i].payload, buf, len);
            g_cache_down_buf[i].pay_len = len;
            g_cache_down_buf[i].valid = true;
            return 0;
        }
    }
    /* 查找空闲槽 */
    for(uint8_t i = 0; i < DATA_CACHE_DOWN_MAX_ITEM; i++)
    {
        if(g_cache_down_buf[i].valid == false)
        {
            memcpy(g_cache_down_buf[i].dev_uid, uid, sizeof(((last_value_t*)0)->uid));
            g_cache_down_buf[i].dev_type = dev_type;
            memcpy(g_cache_down_buf[i].payload, buf, len);
            g_cache_down_buf[i].pay_len = len;
            g_cache_down_buf[i].valid = true;
            return 0;
        }
    }
    return -1; /* 缓存满 */
}

bool data_cache_down_fetch_by_uid(uint8_t *uid, cache_down_item_t *out_item)
{
    for(uint8_t i = 0; i < DATA_CACHE_DOWN_MAX_ITEM; i++)
    {
        if(g_cache_down_buf[i].valid == true && (memcmp(g_cache_down_buf[i].dev_uid, uid, sizeof(((last_value_t*)0)->uid))==0))
        {
            memcpy(out_item, &g_cache_down_buf[i], sizeof(cache_down_item_t));
            return true;
        }
    }
    return false;
}

void data_cache_down_del_by_uid(uint8_t *uid)
{
    for(uint8_t i = 0; i < DATA_CACHE_DOWN_MAX_ITEM; i++)
    {
        if(g_cache_down_buf[i].valid == true && (memcmp(g_cache_down_buf[i].dev_uid, uid, sizeof(((last_value_t*)0)->uid))==0))
        {
            memset(&g_cache_down_buf[i],0,sizeof(cache_down_item_t));
            g_cache_down_buf[i].valid = false;
            return;
        }
    }
}

/* ★2026-10-23 P1: "删之前先确认缓存里还是刚发出去的那一条"★
 *
 * 【为什么需要】声光(0x03)是三段式: 0x80 收到 -> 等约 10 秒 -> 0x82 执行结果。
 * 这 10 秒里云端完全可能又下发一条新档位, 覆盖掉同一个 uid 的缓存条目
 * (缓存是"每台一条", 后到的直接顶掉前一条)。
 * 老代码在收到 0x82 时【无条件】按 uid 删缓存 —— 于是那条【新指令】被连带删掉:
 * 平台以为下发成功, 设备停在旧档位, 而且再也没有重试机会。
 *
 * 【现在的语义】只有"缓存的 func 和载荷与刚发出去的那条逐字节相同"才删。
 * 不同 = 已经有更新的指令在排队, 保留它交给下一轮下发。 */
bool data_cache_down_del_if_match(uint8_t *uid, uint8_t func, const uint8_t *data, uint8_t len)
{
    for(uint8_t i = 0; i < DATA_CACHE_DOWN_MAX_ITEM; i++)
    {
        cache_down_item_t *it = &g_cache_down_buf[i];

        if(it->valid != true || (memcmp(it->dev_uid, uid, sizeof(((last_value_t*)0)->uid)) != 0))
        {
            continue;
        }

        /* payload[0] = func, payload[1..] = data */
        if((it->pay_len != (uint8_t)(len + 1U)) || (it->payload[0] != func))
        {
            return false;           /* 已经是另一条指令了 -> 不删 */
        }
        if(len > 0U)
        {
            /* data == NULL 时无法确认"是不是同一条" -> 不能删 */
            if((data == NULL) || (memcmp(&it->payload[1], data, len) != 0))
            {
                return false;
            }
        }

        memset(it, 0, sizeof(cache_down_item_t));
        it->valid = false;
        return true;
    }
    return false;                   /* 没这条(可能已被清掉) */
}

uint8_t data_cache_down_get_count(void)
{
    uint8_t cnt = 0;

    for(uint8_t i = 0; i < DATA_CACHE_DOWN_MAX_ITEM; i++)
    {
        if(g_cache_down_buf[i].valid) cnt++;
    }
    return cnt;
}
