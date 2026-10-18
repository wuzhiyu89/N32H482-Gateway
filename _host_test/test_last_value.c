/**
 * \file  test_last_value.c
 * \brief 设备"最后值"快照表的主机单测(见 app/business/last_value.h)
 *
 * 价值在于两件现场很难复现的事:
 *   1) 【按 uid 覆盖, 不是入队】同一设备反复上报应保持 1 条并刷新字段+计数,
 *      而不是像 FIFO 那样堆积 —— 这是它和 data_cache 上行队列的根本区别;
 *   2) 【槽满淘汰最旧, 且不静默】第 9 台设备进来时必须踢掉 rx_tick_ms 最小的那台,
 *      并让 evict_count 如实 +1。丢数据可以, 但必须可观察。
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "last_value.h"

static int g_fail;
static int g_pass;

#define CHECK(cond, ...)                                        \
    do {                                                        \
        if (cond) { g_pass++; printf("  ok   : "); }             \
        else      { g_fail++; printf("  FAIL : "); }             \
        printf(__VA_ARGS__);                                    \
        printf("\n");                                           \
    } while (0)

static void mk_uid(uint8_t *uid, uint8_t a, uint8_t b, uint8_t c)
{
    memset(uid, 0, 13u);   /* 2026-10-18: uid 扩到 13 字节, 先清零免得 [3..12] 是栈垃圾 */
    uid[0] = a; uid[1] = b; uid[2] = c;
}

/* 空表 + 未知 uid 的边界 */
static void test_empty_and_unknown(void)
{
    last_value_t v;
    uint8_t      uid[13];

    printf("\n--- 空表/未知 uid ---\n");
    last_value_reset();

    CHECK(last_value_count() == 0U, "reset 后 count==0");
    CHECK(last_value_evict_count() == 0U, "reset 后 evict==0");
    CHECK(last_value_at(0U, &v) == false, "空表 at(0) -> false");

    mk_uid(uid, 0x11U, 0x22U, 0x33U);
    /* 显式赋值后再查: 未找到时 get() 必须【完全不改】out,
     * 否则调用方会拿上一次的残值当成"这台设备的当前值"报上去。 */
    (void)memset(&v, 0, sizeof(v));
    v.level_mm = 0xBEEFU;
    CHECK(last_value_get(uid, &v) == false, "未知 uid get -> false");
    CHECK(v.level_mm == 0xBEEFU, "★未找到时不改 out(level 保持 0xBEEF)");
    CHECK(v.used == false, "★未找到时不改 out(used 保持 false)");
}

/* 同一 uid 覆盖 + 计数 */
static void test_overwrite_by_uid(void)
{
    last_value_t v;
    uint8_t      uid[13];

    printf("\n--- 同 uid 覆盖(不是入队) ---\n");
    last_value_reset();
    mk_uid(uid, 0x01U, 0x02U, 0x03U);

    CHECK(last_value_update(uid, 1234U, 88U, 0x09121130U, 1000U) == true, "首次 update -> true");
    CHECK(last_value_count() == 1U, "count==1");
    CHECK(last_value_get(uid, &v) == true, "get -> true");
    CHECK(v.level_mm == 1234U, "level_mm 正确");
    CHECK(v.bat_pct == 88U, "bat_pct 正确");
    CHECK(v.time_mdhhm == 0x09121130U, "time_mdhhm 原样保存");
    CHECK(v.rx_tick_ms == 1000U, "rx_tick_ms 正确");
    CHECK(v.rx_count == 1U, "rx_count==1");

    CHECK(last_value_update(uid, 999U, 70U, 0x09121159U, 2000U) == true, "再次 update -> true");
    CHECK(last_value_count() == 1U, "★同 uid 仍是 1 条(覆盖, 不堆积)");
    CHECK(last_value_get(uid, &v) == true, "get 仍 true");
    CHECK(v.level_mm == 999U, "level_mm 已刷新");
    CHECK(v.bat_pct == 70U, "bat_pct 已刷新");
    CHECK(v.rx_tick_ms == 2000U, "rx_tick_ms 已刷新");
    CHECK(v.rx_count == 2U, "★rx_count 累加到 2");
    CHECK(last_value_evict_count() == 0U, "★覆盖不产生淘汰计数");
}

/* 多设备互不干扰 */
static void test_multi_device(void)
{
    last_value_t v;
    uint8_t      a[13], b[13];

    printf("\n--- 多设备独立 ---\n");
    last_value_reset();
    mk_uid(a, 0xA0U, 0xA1U, 0xA2U);
    mk_uid(b, 0xB0U, 0xB1U, 0xB2U);

    (void)last_value_update(a, 100U, 10U, 1U, 500U);
    (void)last_value_update(b, 200U, 20U, 2U, 600U);

    CHECK(last_value_count() == 2U, "count==2");
    CHECK(last_value_get(a, &v) == true && v.level_mm == 100U, "A 的值独立");
    CHECK(last_value_get(b, &v) == true && v.level_mm == 200U, "B 的值独立");

    /* uid 只有前 3 字节参与比较: 第 4 字节不同必须视为【不同】设备(不能越界读) */
    {
        uint8_t c[13];
        memcpy(c, a, 13u);
        (void)last_value_update(c, 300U, 30U, 3U, 700U);
        CHECK(last_value_count() == 2U, "前 3 字节相同 -> 视为同一设备, 不新增");
    }
}

/* 遍历接口: at() 是"有效槽的稠密序号", 越界返回 false */
static void test_iterate(void)
{
    last_value_t v;
    uint8_t      uid[13];
    uint8_t      i;

    printf("\n--- at() 遍历 ---\n");
    last_value_reset();

    for (i = 0U; i < 3U; i++)
    {
        mk_uid(uid, 0x10U, 0x00U, i);
        (void)last_value_update(uid, (uint16_t)(100U * i), i, i, 100U * i);
    }

    CHECK(last_value_count() == 3U, "count==3");
    for (i = 0U; i < 3U; i++)
    {
        CHECK(last_value_at(i, &v) == true, "at(%u) 取到", (unsigned)i);
        CHECK(v.used == true, "  used==true");
    }
    CHECK(last_value_at(3U, &v) == false, "at(count) -> false(越界)");
    CHECK(last_value_at(255U, &v) == false, "at(255) -> false");
    CHECK(last_value_at(0U, NULL) == false, "at(0,NULL) -> false");
}

/* 槽满淘汰最旧 + 计数可观察 */
static void test_evict_oldest(void)
{
    last_value_t v;
    uint8_t      uid[13];
    uint8_t      i;

    printf("\n--- 槽满淘汰最旧 ---\n");
    last_value_reset();

    /* 填满 LAST_VALUE_MAX 槽, tick 递增: 第 0 槽最旧 */
    for (i = 0U; i < LAST_VALUE_MAX; i++)
    {
        mk_uid(uid, 0x20U, 0x00U, i);
        (void)last_value_update(uid, (uint16_t)(1000U + i), 50U, i, 1000U + i);
    }
    CHECK(last_value_count() == LAST_VALUE_MAX, "填满 count==%u", (unsigned)LAST_VALUE_MAX);
    CHECK(last_value_evict_count() == 0U, "★填满但不淘汰 -> evict 仍 0");

    /* 第 LAST_VALUE_MAX+1 台: 应踢掉 tick 最小的 0x200000 */
    mk_uid(uid, 0x20U, 0x00U, 0xEEU);
    (void)last_value_update(uid, 4321U, 99U, 0x01020304U, 99999U);

    CHECK(last_value_count() == LAST_VALUE_MAX, "淘汰后 count 不超上限");
    CHECK(last_value_evict_count() == 1U, "★evict==1");

    mk_uid(uid, 0x20U, 0x00U, 0x00U);
    CHECK(last_value_get(uid, &v) == false, "★最旧的那台已被踢掉");
    mk_uid(uid, 0x20U, 0x00U, 0xEEU);
    CHECK(last_value_get(uid, &v) == true, "新设备在表里");
    CHECK(v.level_mm == 4321U, "新设备字段正确");
    CHECK(v.rx_count == 1U, "新设备 rx_count==1(不是复用旧计数)");

    /* 第 1 槽(tick=1001)现在最旧, 应第二个被踢 */
    mk_uid(uid, 0x20U, 0x00U, 0xFFU);
    (void)last_value_update(uid, 1U, 1U, 1U, 100000U);
    CHECK(last_value_evict_count() == 2U, "★继续淘汰 evict==2");
    mk_uid(uid, 0x20U, 0x00U, 0x01U);
    CHECK(last_value_get(uid, &v) == false, "★第二旧的也被踢(淘汰顺序=最旧优先)");
}

/* 参数非法 / NULL */
static void test_bad_args(void)
{
    last_value_t v;
    uint8_t      uid[13];

    printf("\n--- NULL 与非法参数 ---\n");
    last_value_reset();
    mk_uid(uid, 0x30U, 0x30U, 0x30U);

    CHECK(last_value_update(NULL, 1U, 1U, 1U, 1U) == false, "update(uid=NULL) -> false");
    CHECK(last_value_count() == 0U, "非法 update 不建条目");
    CHECK(last_value_get(NULL, &v) == false, "get(uid=NULL) -> false");
    CHECK(last_value_get(uid, NULL) == false, "get(out=NULL) -> false");

    (void)last_value_update(uid, 1U, 1U, 1U, 1U);
    CHECK(last_value_count() == 1U, "正常 update 仍可用");
}

/* rx_count 饱和: 65535 次之后不回绕成 0(否则"从没上报过"会误判) */
static void test_rx_count_saturate(void)
{
    last_value_t v;
    uint8_t      uid[13];
    uint32_t     i;

    printf("\n--- rx_count 饱和 ---\n");
    last_value_reset();
    mk_uid(uid, 0x40U, 0x00U, 0x01U);

    for (i = 0U; i < 70000U; i++)       /* > 0xFFFF, 必须停在 0xFFFF 而不是回 0 */
    {
        (void)last_value_update(uid, 5U, 5U, 5U, i);
    }

    CHECK(last_value_get(uid, &v) == true, "get -> true");
    CHECK(v.rx_count == 0xFFFFU, "★rx_count 饱和在 0xFFFF(未回绕)");
    CHECK(last_value_count() == 1U, "7 万次上报仍只占 1 槽");
}

/* 网关时钟钩子: 注册了才记 rx_unix, 没注册就记 0(应答据此【省略】lastTs) */
static uint32_t s_test_unix;

static uint32_t test_now_unix(void)
{
    return s_test_unix;
}

static void test_now_hook(void)
{
    last_value_t v;
    uint8_t      uid[13];

    printf("\n--- 时钟钩子 -> rx_unix ---\n");
    last_value_reset();
    mk_uid(uid, 0x50U, 0x00U, 0x01U);

    last_value_set_now_hook(NULL);
    (void)last_value_update(uid, 100U, 10U, 1U, 100U);
    CHECK(last_value_get(uid, &v) == true, "get -> true");
    CHECK(v.rx_unix == 0u, "★ 没注册钩子 -> rx_unix = 0");

    s_test_unix = 1700000123u;
    last_value_set_now_hook(test_now_unix);
    (void)last_value_update(uid, 200U, 20U, 2U, 200U);
    CHECK(last_value_get(uid, &v) == true, "get -> true");
    CHECK(v.rx_unix == 1700000123u, "★ 注册后 rx_unix 取到时钟值");

    last_value_set_now_hook(NULL);
    (void)last_value_update(uid, 300U, 30U, 3U, 300U);
    CHECK(last_value_get(uid, &v) == true, "get -> true");
    CHECK(v.rx_unix == 0u, "★ 撤销注册后又记 0(不沿用上次的时间)");

    last_value_reset();
}

/* =====================================================================
 * 第 10 条(2026-09-30): 该帧的 LoRa RSSI 要一路存下来
 * ---------------------------------------------------------------------
 * 遥测 telemetry.report 的 data.sensors[].rssi(文档 7.1)以前恒为 0:
 * 射频层拿到了 rssi 却只打了日志。现在把它存进最后值快照。
 * ===================================================================== */
static void test_rssi(void)
{
    last_value_t v;
    uint8_t      uid[13];

    printf("\n--- RSSI 落库(第 10 条) ---\n");
    last_value_reset();
    mk_uid(uid, 0x0AU, 0x0BU, 0x0CU);

    (void)last_value_update_ex(uid, 1234U, 88U, 0x09121130U, -72, 5000U);
    CHECK(last_value_get(uid, &v) == true, "带 rssi 的 update -> 能查到");
    CHECK(v.rssi_dbm == -72, "★ RSSI 存下来了 (实际 %d)", (int)v.rssi_dbm);
    CHECK(v.level_mm == 1234U, "同一帧的水位照常存");

    /* 旧入口(不关心 rssi 的老调用点) -> 记 0 = 未知, 且不沿用上一帧的旧值 */
    (void)last_value_update(uid, 1200U, 87U, 0x09121131U, 6000U);
    CHECK(last_value_get(uid, &v) == true, "旧入口 -> 能查到");
    CHECK(v.rssi_dbm == 0, "★ 旧入口 rssi 记 0(未知), 不沿用旧值 (实际 %d)",
          (int)v.rssi_dbm);

    /* 正值也不该被截断(有些口径 rssi 给的是信号质量正数) */
    (void)last_value_update_ex(uid, 1200U, 87U, 0x09121131U, 100, 7000U);
    CHECK(last_value_get(uid, &v) && (v.rssi_dbm == 100), "正 rssi 也能存");
}

/* ==================== [10] clear_uid: 重绑设备时只清【一台】 ====================
 * ★2026-10-14 新增。对应用户场景: 设备坏了换新, 重新烧录后回到同一默认地址
 *   ⇒ 允许同 UID 再绑一次, 但【新设备不该继承旧设备的历史值】。
 *
 * ★★ 本用例的【最重要】两条断言是"其它 UID 不受影响" ★★
 *   它们专门钉住一个陷阱: 实现者图省事改用 last_value_reset()(全清)。
 *   那会造成"重绑一台, 其它台全掉线" —— 现场极难查(单台重绑却多台消失)。
 *   (同样性质的两个陷阱在 config_store_rebind_device 那边: 用 bind 会静默不更新;
 *    用 unbind+bind 会清掉 gears[].target_uid ⇒ 那台设备收不到联动指令。) */
static void test_clear_uid(void)
{
    const uint8_t a[13] = { 0x21, 0,0,0,0,0,0,0,0,0,0,0, 0x01 };
    const uint8_t b[13] = { 0x21, 0,0,0,0,0,0,0,0,0,0,0, 0x02 };
    const uint8_t c[13] = { 0x31, 0,0,0,0,0,0,0,0,0,0,0, 0x01 };
    last_value_t  v;

    printf("\n[10] clear_uid: 只清目标 UID, 其它设备不受影响\n");

    last_value_reset();
    (void)last_value_update(a, 111U, 11U, 0U, 1000U);
    (void)last_value_update(b, 222U, 22U, 0U, 2000U);
    (void)last_value_update(c, 333U, 33U, 0U, 3000U);
    CHECK(last_value_count() == 3U, "准备: 3 台在表 (实际 %u)", (unsigned)last_value_count());

    /* --- 清 A --- */
    CHECK(last_value_clear_uid(a) == 1U, "★ 清 A -> 返回 1(清了一条)");
    CHECK(last_value_get(a, &v) == false, "★ A 的快照没了");
    CHECK(last_value_count() == 2U, "★ count 3 -> 2");

    /* --- ★★ 关键: 其它两个【必须】完好无损 ★★ --- */
    CHECK(last_value_get(b, &v) == true, "★★ B 仍能查到(没被误清)");
    CHECK((last_value_get(b, &v) == true) && (v.level_mm == 222U), "★★ B 的值没变(222)");
    CHECK(last_value_get(c, &v) == true, "★★ C 仍能查到(没被误清)");
    CHECK((last_value_get(c, &v) == true) && (v.level_mm == 333U), "★★ C 的值没变(333)");

    /* --- 重复清同一个: 幂等, 返回 0 而不是报错 --- */
    CHECK(last_value_clear_uid(a) == 0U, "★ 再清一次 -> 0(本来就没有, 不算错)");
    CHECK(last_value_count() == 2U, "count 仍是 2");

    /* --- 清一个从没上报过的: 也是 0 --- */
    {
        const uint8_t never[13] = { 0x21, 0,0,0,0,0,0,0,0,0,0,0, 0x09 };
        CHECK(last_value_clear_uid(never) == 0U, "清从没上报过的 UID -> 0");
        CHECK(last_value_count() == 2U, "count 不受影响");
    }

    /* --- 参数防护 --- */
    CHECK(last_value_clear_uid(NULL) == 0U, "clear_uid(NULL) -> 0(不崩)");
    CHECK(last_value_count() == 2U, "count 仍不受影响");

    /* --- 清掉之后还能重新写入(模拟新设备第一次上报) --- */
    CHECK(last_value_update(a, 444U, 44U, 0U, 4000U) == true, "清掉后能重新写入(新设备首报)");
    CHECK((last_value_get(a, &v) == true) && (v.level_mm == 444U), "★ 新设备的值是 444, 不是旧的 111");
    CHECK(last_value_count() == 3U, "count 回到 3");
}

int main(void)
{
    printf("==== test_last_value: 设备最后值快照表 ====\n");
    test_empty_and_unknown();
    test_overwrite_by_uid();
    test_multi_device();
    test_iterate();
    test_evict_oldest();
    test_bad_args();
    test_rx_count_saturate();
    test_now_hook();
    test_rssi();
    test_clear_uid();

    printf("\n==== 通过 %d, 失败 %d ====\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
