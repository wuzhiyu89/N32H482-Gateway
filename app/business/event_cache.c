#include "event_cache.h"
#include <string.h>

static ec_record_t g_ev[EVENT_CACHE_MAX];
static uint16_t    g_wr;        /* 下一个写入位置 */
static uint16_t    g_rd;        /* 最旧一条的位置 */
static uint16_t    g_cnt;       /* 当前条数(用计数而不是"wr==rd"判空, 满时两指针相等) */
static uint32_t    g_dropped;   /* 因槽满淘汰的累计条数 */

void event_cache_reset(void)
{
    (void)memset(g_ev, 0, sizeof(g_ev));
    g_wr = 0u;
    g_rd = 0u;
    g_cnt = 0u;
    g_dropped = 0u;
}

bool event_cache_push(const ec_record_t *rec)
{
    if (rec == NULL)
    {
        return false;
    }
    /* 只认这三种, 免得存进去一堆渲染层没法处理的记录 */
    if ((rec->kind != (uint8_t)EC_KIND_STATUS_CHANGE) &&
        (rec->kind != (uint8_t)EC_KIND_ACTION) &&
        (rec->kind != (uint8_t)EC_KIND_DEVICE))
    {
        return false;
    }

    if (g_cnt == EVENT_CACHE_MAX)
    {
        /* 满: 淘汰最旧(读指针前进), 让它被覆盖 —— 并如实计数 */
        g_rd = (uint16_t)((g_rd + 1u) % EVENT_CACHE_MAX);
        g_cnt--;
        g_dropped++;
    }

    /* 整条覆盖写: 结构体是定长的, 直接赋值即可(不留上一份的残字节) */
    g_ev[g_wr] = *rec;
    g_wr = (uint16_t)((g_wr + 1u) % EVENT_CACHE_MAX);
    g_cnt++;
    return true;
}

bool event_cache_peek(ec_record_t *out)
{
    if ((out == NULL) || (g_cnt == 0u))
    {
        return false;
    }
    *out = g_ev[g_rd];      /* 未找到/为空时不动 out */
    return true;
}

void event_cache_drop(void)
{
    if (g_cnt > 0u)
    {
        g_rd = (uint16_t)((g_rd + 1u) % EVENT_CACHE_MAX);
        g_cnt--;
    }
}

bool event_cache_pop(ec_record_t *out)
{
    if (!event_cache_peek(out))
    {
        return false;
    }
    event_cache_drop();
    return true;
}

uint16_t event_cache_count(void)
{
    return g_cnt;
}

uint32_t event_cache_dropped(void)
{
    return g_dropped;
}

uint16_t event_cache_capacity(void)
{
    return (uint16_t)EVENT_CACHE_MAX;
}

/* 取环形里第 i 条(0 = 最旧)的只读指针; 调用方保证 i < g_cnt */
static const ec_record_t *ev_at(uint16_t i)
{
    return &g_ev[(uint16_t)((g_rd + i) % EVENT_CACHE_MAX)];
}

int32_t ec_action_code_of(const ec_record_t *rec)
{
    /* 只有动作记录才有"执行码"语义; 其它 kind 调用它是逻辑错误, 返回 0 并让
     * 上层的 validate() 去拦(这里不 assert, 本工程不带 assert)。 */
    return (rec != NULL) ? rec->aux : 0;
}

void ec_action_set_code(ec_record_t *rec, int32_t code)
{
    if (rec != NULL)
    {
        rec->aux = code;
    }
}

uint8_t event_cache_group_at(uint16_t idx, ec_action_pair_t *out, uint8_t max_pairs)
{
    const ec_record_t *head;
    uint8_t            seq;
    uint8_t            n = 0u;

    if ((out == NULL) || (max_pairs == 0u) || (idx >= g_cnt))
    {
        return 0u;
    }

    head = ev_at(idx);
    if ((head->kind != (uint8_t)EC_KIND_ACTION) || (head->seq == 0u))
    {
        return 0u;              /* 这一条不是"成组"的动作记录 */
    }
    seq = head->seq;

    while (((uint16_t)(idx + n) < g_cnt) && (n < max_pairs) &&
           (ev_at((uint16_t)(idx + n))->kind == (uint8_t)EC_KIND_ACTION) &&
           (ev_at((uint16_t)(idx + n))->seq == seq))
    {
        (void)memcpy(out[n].uid, ev_at((uint16_t)(idx + n))->uid, 3u);
        out[n].code = ec_action_code_of(ev_at((uint16_t)(idx + n)));
        n++;
    }

    /* ★ 本组比 out 能装的大: 返回 0, 而不是只报前几台 ★
     * 只报前几台会渲染出一条"少报了几台设备"的动作事件 —— 看起来完全正常。 */
    if ((n == max_pairs) && ((uint16_t)(idx + n) < g_cnt) &&
        (ev_at((uint16_t)(idx + n))->kind == (uint8_t)EC_KIND_ACTION) &&
        (ev_at((uint16_t)(idx + n))->seq == seq))
    {
        return 0u;
    }

    return n;
}

uint8_t event_cache_peek_action_group(ec_action_pair_t *out, uint8_t max_pairs)
{
    return event_cache_group_at(0u, out, max_pairs);        /* 只是"从队头找"的特例 */
}

bool event_cache_peek_at(uint16_t idx, ec_record_t *out)
{
    if ((out == NULL) || (idx >= g_cnt))
    {
        return false;
    }
    *out = *ev_at(idx);
    return true;
}

bool event_cache_validate(const ec_record_t *rec)
{
    if (rec == NULL)
    {
        return false;
    }

    switch (rec->kind)
    {
        case (uint8_t)EC_KIND_STATUS_CHANGE:
            /* from/to 都必须是文档 3.4 的四个状态之一 */
            return (((rec->a >= (uint8_t)EC_ST_RESOLVED) && (rec->a <= (uint8_t)EC_ST_ALARM) &&
                     (rec->b >= (uint8_t)EC_ST_RESOLVED) && (rec->b <= (uint8_t)EC_ST_ALARM)))
                   ? true : false;

        case (uint8_t)EC_KIND_ACTION:
            /* a = 档位 0~3(0 合法); b = 触发来源 */
            return (((rec->a <= 3u) &&
                     (rec->b >= (uint8_t)EC_TRIG_AUTO) && (rec->b <= (uint8_t)EC_TRIG_FAILSAFE)))
                   ? true : false;

        case (uint8_t)EC_KIND_DEVICE:
            /* a = 设备事件码; b = 设备类型 */
            return (((rec->a >= (uint8_t)EC_DEV_EV_BOUND) &&
                     (rec->a <= (uint8_t)EC_DEV_EV_LOW_VOLTAGE) &&
                     ((rec->b == (uint8_t)EC_DEV_SENSOR) || (rec->b == (uint8_t)EC_DEV_DEVICE))))
                   ? true : false;

        default:
            return false;               /* kind 不认识 */
    }
}

uint16_t event_cache_page_count(uint16_t max_events)
{
    uint16_t limit;
    uint16_t n;
    const ec_record_t *tail;

    if ((g_cnt == 0u) || (max_events == 0u))
    {
        return 0u;
    }
    if (max_events >= g_cnt)
    {
        return g_cnt;       /* 一次就能全取走: 不存在"组被切断"的问题 */
    }

    limit = max_events;
    tail  = ev_at((uint16_t)(limit - 1u));

    /* 只有当"页尾那条"属于某个动作组时, 才需要把这一组补完 */
    if ((tail->kind == (uint8_t)EC_KIND_ACTION) && (tail->seq != 0u))
    {
        n = limit;
        while ((n < g_cnt) &&
               (ev_at(n)->kind == (uint8_t)EC_KIND_ACTION) &&
               (ev_at(n)->seq == tail->seq))
        {
            n++;
        }
        /* 若整个组本身就比 max_events 大(从队头就是这个 seq), 上面会一路加到组尾,
         * 返回值自然 > max_events —— 这是刻意的(见头文件说明: 宁可超限也不能卡住)。 */
        return n;
    }

    return limit;
}
