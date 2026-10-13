#include "last_value.h"
#include <string.h>

/* 纯内存快照表; 断电即失(见 last_value.h 顶部说明) */
static last_value_t g_last[LAST_VALUE_MAX];
static uint32_t     g_evict;        /* 槽满淘汰累计次数 */
static uint32_t     g_upd_seq;      /* 成功 update 的累计次数(不被 reset 清零) */
static uint32_t   (*s_now_unix)(void);  /* 网关当前 Unix 秒(未注册 = NULL) */

void last_value_set_now_hook(uint32_t (*fn)(void))
{
    s_now_unix = fn;
}

void last_value_reset(void)
{
    memset(g_last, 0, sizeof(g_last));
    g_evict = 0U;
}

/* 按 uid 找槽位; 返回裸槽下标, 未找到 -1 */
static int lv_find(const uint8_t *uid)
{
    uint8_t i;
    for (i = 0U; i < LAST_VALUE_MAX; i++)
    {
        if ((g_last[i].used == true) && (memcmp(g_last[i].uid, uid, 3U) == 0))
        {
            return (int)i;
        }
    }
    return -1;
}

/* 找一个空闲槽位; 无空闲返回 -1 */
static int lv_free_slot(void)
{
    uint8_t i;
    for (i = 0U; i < LAST_VALUE_MAX; i++)
    {
        if (g_last[i].used == false)
        {
            return (int)i;
        }
    }
    return -1;
}

/* 找 rx_tick_ms 最旧的一槽; 表空返回 -1
 * 注: 直接比大小, 未做 tick 回绕处理 —— 本表只用于"淘汰谁"的启发式判断,
 *     且回绕周期(约 49 天)远长于表内条目共存时间, 不构成实际风险。 */
static int lv_oldest_slot(void)
{
    int     idx = -1;
    uint8_t i;
    for (i = 0U; i < LAST_VALUE_MAX; i++)
    {
        if (g_last[i].used == false)
        {
            continue;
        }
        if ((idx < 0) || (g_last[i].rx_tick_ms < g_last[idx].rx_tick_ms))
        {
            idx = (int)i;
        }
    }
    return idx;
}

bool last_value_update(const uint8_t *uid, uint16_t level_mm, uint16_t bat_pct,
                       uint32_t time_mdhhm, uint32_t now_ms)
{
    /* 旧入口: 不带 rssi(等价于"未知")。见 last_value.h 里为什么保留它。 */
    return last_value_update_ex(uid, level_mm, bat_pct, time_mdhhm, 0, now_ms);
}

bool last_value_update_ex(const uint8_t *uid, uint16_t level_mm, uint16_t bat_pct,
                          uint32_t time_mdhhm, int8_t rssi_dbm, uint32_t now_ms)
{
    int idx;

    if (uid == NULL)
    {
        return false;
    }

    idx = lv_find(uid);
    if (idx < 0)
    {
        /* 新设备: 先用空槽, 没空槽就淘汰最旧的(不丢"最新发生的事"是最重要的) */
        idx = lv_free_slot();
        if (idx < 0)
        {
            idx = lv_oldest_slot();
            if (idx < 0)
            {
                return false;       /* 理论不可达: 无空槽则表必然非空 */
            }
            g_evict++;
        }
        memset(&g_last[idx], 0, sizeof(last_value_t));
        memcpy(g_last[idx].uid, uid, 3U);
        g_last[idx].used = true;
    }

    g_last[idx].level_mm   = level_mm;
    g_last[idx].bat_pct    = bat_pct;
    g_last[idx].time_mdhhm = time_mdhhm;
    g_last[idx].rssi_dbm   = rssi_dbm;      /* 第 10 条: 遥测 data.sensors[].rssi */
    g_last[idx].rx_tick_ms = now_ms;
    /* 收到时刻的 Unix 秒: 有钩子才记, 没有就记 0(应答据此【省略】该字段) */
    g_last[idx].rx_unix    = (s_now_unix != NULL) ? s_now_unix() : 0u;
    if (g_last[idx].rx_count < 0xFFFFU)     /* 饱和, 不回绕成"没上报过" */
    {
        g_last[idx].rx_count++;
    }

    g_upd_seq++;            /* 新上报计数(给 level_supervisor 判"有没有新样本") */

    return true;
}

uint32_t last_value_update_seq(void)
{
    return g_upd_seq;
}

bool last_value_get(const uint8_t *uid, last_value_t *out)
{
    int idx;

    if ((uid == NULL) || (out == NULL))
    {
        return false;
    }

    idx = lv_find(uid);
    if (idx < 0)
    {
        return false;               /* out 不动: 调用方不能拿未初始化数据当结论 */
    }

    memcpy(out, &g_last[idx], sizeof(last_value_t));
    return true;
}

bool last_value_at(uint8_t idx, last_value_t *out)
{
    uint8_t i;
    uint8_t seen = 0U;

    if (out == NULL)
    {
        return false;
    }

    for (i = 0U; i < LAST_VALUE_MAX; i++)
    {
        if (g_last[i].used == false)
        {
            continue;
        }
        if (seen == idx)
        {
            memcpy(out, &g_last[i], sizeof(last_value_t));
            return true;
        }
        seen++;
    }

    return false;                   /* 越界 */
}

uint8_t last_value_count(void)
{
    uint8_t i;
    uint8_t cnt = 0U;

    for (i = 0U; i < LAST_VALUE_MAX; i++)
    {
        if (g_last[i].used == true)
        {
            cnt++;
        }
    }
    return cnt;
}

uint32_t last_value_evict_count(void)
{
    return g_evict;
}
