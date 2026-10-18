/**
 * \file  test_telemetry_cache.c
 * \brief 离线期间的"逐条液位上报"环形缓存的主机单测
 *        (见 app/business/telemetry_cache.h)
 *
 * 为什么值得单独测:
 *   这是**现场最难复现**的一块 —— 要断 4G、等液位报几十条、再重连，
 *   才能看出"顺序对不对/满了丢哪条/计数准不准"。而这几件事全是纯逻辑,
 *   主机上一次跑完。
 *
 * 重点验四件:
 *   1) 顺序: peek 永远给【最旧】那条, drop 之后下一条接上(FIFO, 不是 LIFO);
 *   2) 满了: 覆盖【最旧】而不是丢新的 —— 保留最近水位, 早期历史丢弃;
 *   3) 计数: dropped 必须如实累加(丢数据可以, 但必须可观察);
 *   4) 边界: 空表 peek == NULL / drop 安全 / push(NULL) 拒绝且不改状态。
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "telemetry_cache.h"

static int g_fail;
static int g_pass;

#define CHECK(cond, ...)                                        \
    do {                                                        \
        if (cond) { g_pass++; printf("  ok   : "); }             \
        else      { g_fail++; printf("  FAIL : "); }             \
        printf(__VA_ARGS__);                                    \
        printf("\n");                                           \
    } while (0)

/** 造一条记录: 用 level 当作"第几条"的标记, 便于验顺序 */
static void mk_rec(tc_record_t *r, uint16_t level)
{
    (void)memset(r, 0, sizeof(*r));
    r->ts        = 1700000000u + (uint32_t)level;   /* 每条 ts 不同, 便于验"原时间戳" */
    r->level_mm  = level;
    r->msg_seq   = (uint16_t)(level % 1000u);
    r->uid[0]    = 0x21u;
    r->uid[12]   = (uint8_t)(level & 0xFFu);
    r->bat_pct   = 88u;
    r->rssi_dbm  = -70;
    r->state     = 2u;
    r->gear      = 1u;
}

/* ---------------- 1) 记录大小 ---------------- */
static void test_size(void)
{
    printf("\n--- 记录大小 ---\n");
    /* 25 B 有效 -> 对齐到 28 B。256 条 = 7 KB。
     * 写死这个上限是为了"以后谁给结构体加字段把 RAM 撑爆"能被测试挡一下。 */
    CHECK(sizeof(tc_record_t) <= 28u,
          "★ sizeof(tc_record_t) = %u <= 28 (256 条 = %u B)",
          (unsigned)sizeof(tc_record_t),
          (unsigned)(sizeof(tc_record_t) * TELEMETRY_CACHE_MAX));
    CHECK(TELEMETRY_CACHE_MAX == 256u, "容量 = 256 条 (实际 %u)", (unsigned)TELEMETRY_CACHE_MAX);
}

/* ---------------- 2) 基本顺序(最旧先出) ---------------- */
static void test_fifo_order(void)
{
    tc_record_t r;
    const tc_record_t *p;

    printf("\n--- 顺序: 最旧先出 ---\n");
    telemetry_cache_reset();
    CHECK(telemetry_cache_count() == 0u, "reset 后 count == 0");

    mk_rec(&r, 111u); (void)telemetry_cache_push(&r);
    mk_rec(&r, 222u); (void)telemetry_cache_push(&r);
    mk_rec(&r, 333u); (void)telemetry_cache_push(&r);
    CHECK(telemetry_cache_count() == 3u, "push 3 条 -> count 3");

    p = telemetry_cache_peek();
    CHECK((p != NULL) && (p->level_mm == 111u), "★ peek 给【最旧】(111) 而不是最新");
    CHECK((p != NULL) && (p->ts == 1700000111u), "★ 原时间戳原样保留(补报要用的就是它)");
    CHECK((p != NULL) && (p->uid[12] == (uint8_t)(111u & 0xFFu)), "uid 字节原样保留");
    CHECK((p != NULL) && (p->bat_pct == 88u) && (p->rssi_dbm == -70),
          "电量/RSSI 原样保留");

    telemetry_cache_drop();
    CHECK(telemetry_cache_count() == 2u, "drop 后 count 2");
    p = telemetry_cache_peek();
    CHECK((p != NULL) && (p->level_mm == 222u), "★ drop 之后队头变成 222");

    telemetry_cache_drop();
    telemetry_cache_drop();
    CHECK(telemetry_cache_count() == 0u, "全部取空");
    CHECK(telemetry_cache_peek() == NULL, "空表 peek == NULL");
    telemetry_cache_drop();                       /* 空表 drop 不能崩 */
    CHECK(telemetry_cache_count() == 0u, "空表 drop 安全(仍 0)");
}

/* ---------------- 3) 满了覆盖最旧 ---------------- */
static void test_overflow(void)
{
    tc_record_t r;
    const tc_record_t *p;
    uint32_t i;
    uint32_t cap = (uint32_t)telemetry_cache_capacity();

    printf("\n--- 满了覆盖最旧 ---\n");
    telemetry_cache_reset();
    CHECK(cap == 256u, "capacity == 256 (实际 %u)", (unsigned)cap);

    /* 正好填满: 一条都不能丢 */
    for (i = 1u; i <= cap; i++)
    {
        mk_rec(&r, (uint16_t)i);
        (void)telemetry_cache_push(&r);
    }
    CHECK(telemetry_cache_count() == cap, "★ 恰好填满 -> count == 容量");
    CHECK(telemetry_cache_dropped() == 0u, "★ 恰好填满 -> 还没淘汰(不提前丢)");
    p = telemetry_cache_peek();
    CHECK((p != NULL) && (p->level_mm == 1u), "队头是最早那条(level=1)");

    /* 再进 1 条: 必须踢掉 level=1 */
    mk_rec(&r, (uint16_t)(cap + 1u));
    (void)telemetry_cache_push(&r);
    CHECK(telemetry_cache_count() == cap, "★ 第 N+1 条 -> 条数仍是容量");
    CHECK(telemetry_cache_dropped() == 1u, "★ dropped 累加到 1");
    p = telemetry_cache_peek();
    CHECK((p != NULL) && (p->level_mm == 2u),
          "★ 最旧的 level=1 被覆盖, 队头变成 2(保留最近的)");

    /* 再进 10 条: dropped 继续累加, 队头跟着前移 */
    for (i = 1u; i <= 10u; i++)
    {
        mk_rec(&r, (uint16_t)(cap + 1u + i));
        (void)telemetry_cache_push(&r);
    }
    CHECK(telemetry_cache_dropped() == 11u, "★ dropped 累加到 11");
    p = telemetry_cache_peek();
    CHECK((p != NULL) && (p->level_mm == 12u), "★ 队头前移到 12");
    CHECK(telemetry_cache_count() == cap, "条数仍等于容量");
}

/* ---------------- 4) 边界 ---------------- */
static void test_edges(void)
{
    tc_record_t r;

    printf("\n--- 边界 ---\n");
    telemetry_cache_reset();
    CHECK(telemetry_cache_push(NULL) == false, "push(NULL) -> false");
    CHECK(telemetry_cache_count() == 0u, "push(NULL) 不改状态");

    mk_rec(&r, 7u);
    CHECK(telemetry_cache_push(&r) == true, "正常 push -> true");

    telemetry_cache_reset();
    CHECK((telemetry_cache_count() == 0u) && (telemetry_cache_dropped() == 0u),
          "reset 清空 count 与 dropped");
    CHECK(telemetry_cache_capacity() > 0u, "capacity 恒 > 0");
}

int main(void)
{
    printf("==== telemetry_cache: 离线逐条上报缓存 ====\n");
    test_size();
    test_fifo_order();
    test_overflow();
    test_edges();
    printf("\n==== %d passed, %d FAILED ====\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
