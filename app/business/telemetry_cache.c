/**
 * @file  telemetry_cache.c
 * @brief 离线期间的逐条液位上报缓存 —— 环形, 满了覆盖最旧
 *
 * 语义见 telemetry_cache.h 的完整说明。这里只放"环形队列"本身,
 * 不碰网络、不碰 JSON、不碰配置 —— 这样主机上能直接测边界。
 */
#include "telemetry_cache.h"
#include <stddef.h>      /* NULL —— <stdint.h>/<stdbool.h> 不带它 */

/* 2026-10-18: UID 尺寸静态断言 —— 防"改长度忘改缓冲"
 * (现场踩过: g_reply_dev 声明 3 字节却 memcpy 13 字节, 见 lora_business.c 的说明) */
_Static_assert(sizeof(((tc_record_t *)0)->uid) == TELEMETRY_UID_BYTES,
               "tc_record_t.uid 必须等于 TELEMETRY_UID_BYTES");

/* 静态环形区: 不占栈(栈只有 4 KB), 也不在补报时反复 malloc */
static tc_record_t g_tc[TELEMETRY_CACHE_MAX];

static uint16_t g_rd;        /* 读索引(最旧一条) */
static uint16_t g_wr;        /* 写索引(下一条写这里) */
static uint16_t g_cnt;       /* 当前条数 */
static uint32_t g_dropped;   /* 因满了被覆盖掉的累计条数 */

void telemetry_cache_reset(void)
{
    g_rd      = 0u;
    g_wr      = 0u;
    g_cnt     = 0u;
    g_dropped = 0u;
}

uint16_t telemetry_cache_capacity(void)
{
    return (uint16_t)TELEMETRY_CACHE_MAX;
}

uint16_t telemetry_cache_count(void)
{
    return g_cnt;
}

uint32_t telemetry_cache_dropped(void)
{
    return g_dropped;
}

bool telemetry_cache_push(const tc_record_t *rec)
{
    if (rec == NULL)
    {
        return false;
    }

    /* 满: 覆盖最旧(读指针前移), 并如实计数 —— 与 event_cache 同一条纪律 */
    if (g_cnt == TELEMETRY_CACHE_MAX)
    {
        g_rd = (uint16_t)((g_rd + 1u) % TELEMETRY_CACHE_MAX);
        g_cnt--;
        g_dropped++;
    }

    g_tc[g_wr] = *rec;
    g_wr = (uint16_t)((g_wr + 1u) % TELEMETRY_CACHE_MAX);
    g_cnt++;

    return true;
}

const tc_record_t *telemetry_cache_peek(void)
{
    if (g_cnt == 0u)
    {
        return NULL;
    }
    return &g_tc[g_rd];
}

void telemetry_cache_drop(void)
{
    if (g_cnt == 0u)
    {
        return;
    }
    g_rd = (uint16_t)((g_rd + 1u) % TELEMETRY_CACHE_MAX);
    g_cnt--;
}
