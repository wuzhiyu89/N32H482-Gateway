/**
 * \file ota_health.c
 * \brief A/B 双槽的【健康判据 + 自动回滚】状态机实现（2026-10-11）
 *
 * ============================================================================
 * ⚠️⚠️ 改本文件前必读：Flash 只能写 1->0 ⚠️⚠️
 *
 *   本模块的状态存在标记页里，而 ota_port_rollback_save() 【不擦页】
 *   （同页还有 OTA 固件标记和 HTTP 续传记录，擦页会一起毁掉）。
 *   ⇒ 因此本模块的每一次写，都必须满足"相对当前内容只把 1 变成 0"：
 *        · tries_left  递减（清最低的 1 位）      -> 1->0  ✓
 *        · unconfirmed 清 0（配合"反转语义"）      -> 1->0  ✓
 *        · magic / boot_slot 是常量，首次写入后不再动 ✓
 *   ⇒ 【不要】引入任何"会被改大"的字段，也不要加校验和
 *     （校验和是整条内容的函数，内容一变它必变，而新值必含 0->1 ⇒ 写不进去）。
 *
 *   ⚠️ 这三个坑的共同特征是【主机单测全绿、上板全废】——
 *     因为 _host_test 的 RAM 假实现没有 1->0 约束。
 *     ⇒ 所以 test_ota_health 的假实现【必须自己模拟这个约束】
 *       （见 _host_test/fake_ota_port.c 的 ota_port_rollback_save）。
 * ============================================================================
 */
#include "ota_health.h"
#include "ota_port.h"
#include "log.h"

#include <string.h>

/* 本次运行缓存的记录。用静态的理由：本工程单线程、函数不重入；
 * 而且 ota_rollback_t 只有 32 字节，放静态比反复读 Flash 更省。 */
static ota_rollback_t s_rec;
static bool           s_valid;      /* 本次运行读到了 / 建了有效记录 */
static bool           s_done;       /* 本次运行已定案，不再写 Flash */

/** 数位图里有几个 1（= 还剩几次机会）。只用于日志，不参与判定。 */
static uint8_t bits_left(uint8_t m)
{
    uint8_t n = 0u;

    while (m != 0u)
    {
        n = (uint8_t)(n + (m & 1u));
        m = (uint8_t)(m >> 1);
    }
    return n;
}

ota_health_boot_t ota_health_boot_step(void)
{
    ota_rollback_t r;

    s_done = false;

    if (!ota_port_rollback_load(&r))
    {
        /* 【无记录 / 记录损坏】-> 新建一份，给满次数。
         *
         * ⚠️ 这里【绝对不能】返回 ROLLBACK。理由：
         *   读不到记录的常见原因有三个 —— (a) 全新设备从没写过;
         *   (b) 上一轮 OTA 下载擦过标记页; (c) 记录被写坏。
         *   这三种情况下"本槽是不是有问题"是【未知的】，而不是"已知有问题"。
         *   把未知当已知去回滚，就是"把好固件回滚掉"——比不回滚严重得多。
         *   ⇒ 一律给满次数重来。真有问题的话，接下来几次启动自然会扣完。 */
        (void)memset(&r, 0, sizeof(r));
        r.boot_slot   = 0u;                        /* 默认按 Slot A 记 */
        r.tries_left  = OTA_ROLLBACK_TRIES_INIT;   /* 0xFF = 8 次 */
        r.unconfirmed = 1u;                        /* 1 = 还没确认 */

        s_valid = ota_port_rollback_save(&r);
        s_rec   = r;

        log_info("[OTAHL] 无有效回滚记录 -> 新建(槽=%u, 机会=%u)\r\n",
                 0u, (unsigned)OTA_ROLLBACK_TRIES_MAX);
        return OTA_HEALTH_BOOT_OK;
    }

    s_rec   = r;
    s_valid = true;

    /* 已定案：本槽确认过健康，从此不再计数、不再写 Flash。 */
    if (r.unconfirmed == 0u)
    {
        log_info("[OTAHL] 本槽已定案(健康), 不再计数\r\n");
        return OTA_HEALTH_BOOT_OK;
    }

    /* 机会用完 -> 请求回滚。调用方负责复位，由 BL 把另一槽搬回来。 */
    if (r.tries_left == 0u)
    {
        log_info("[OTAHL] !! 机会已用完(槽=%u) -> 请求回滚\r\n",
                 (unsigned)r.boot_slot);
        return OTA_HEALTH_BOOT_ROLLBACK;
    }

    /* 扣一次机会：清掉最低的那个 1 位。
     * ★ 为什么是 `m & (m-1)` 而不是 `m - 1`：
     *   减法会产生进位，可能把某些位从 0 变回 1（例如 0x01-1 = 0x00 没问题，
     *   但 0x10-1 = 0x0F 就把低 4 位从 0 全变成 1）——那是 0->1，Flash 写不进去。
     *   `m & (m-1)` 只清一个 1 位，其余位不变，严格满足 1->0。 */
    r.tries_left = (uint8_t)(r.tries_left & (uint8_t)(r.tries_left - 1u));

    if (ota_port_rollback_save(&r))
    {
        s_rec = r;
        log_info("[OTAHL] 本次启动扣一次机会: 剩 %u/%u 次\r\n",
                 (unsigned)bits_left(r.tries_left),
                 (unsigned)OTA_ROLLBACK_TRIES_MAX);
    }
    else
    {
        /* 写失败不改变判定 —— 只是这次机会没扣掉，下次启动会再扣一次。
         * ⚠️ 绝不能因为"写失败"就回滚（那是把存储故障当成固件故障）。 */
        s_rec = r;
        log_info("[OTAHL] !! 扣机会写 Flash 失败, 本次不生效(下次启动会重试)\r\n");
    }

    return OTA_HEALTH_BOOT_OK;
}

void ota_health_poll(bool g4_online, bool up_acked, uint32_t uptime_s)
{
    if (!s_valid)                { return; }   /* 没有有效记录: 不碰 Flash */
    if (s_done)                  { return; }   /* 本次已定案: 不重复写 */
    if (s_rec.unconfirmed == 0u) { s_done = true; return; }

    /* 三条判据缺一不可 —— 理由见 ota_health.h 的头注释。 */
    if ((!g4_online) || (!up_acked) || (uptime_s <= (uint32_t)OTA_HEALTH_MIN_UPTIME_S))
    {
        return;
    }

    /* 定案：unconfirmed 1 -> 0，合法（见文件头的 1->0 约束）。 */
    s_rec.unconfirmed = 0u;

    if (ota_port_rollback_save(&s_rec))
    {
        s_done = true;      /* 成功才置 done；失败下次 poll 还会再试 */
        log_info("[OTAHL] ★本槽定案(健康): 4G在线 + 上行已确认 + uptime %us > %us★\r\n",
                 (unsigned)uptime_s, (unsigned)OTA_HEALTH_MIN_UPTIME_S);
    }
    else
    {
        s_rec.unconfirmed = 1u;     /* 写失败就还原内存状态，下次再试 */
        log_info("[OTAHL] !! 定案写 Flash 失败, 下次再试\r\n");
    }
}

void ota_health_reset_for_new_fw(uint8_t boot_slot)
{
    ota_rollback_t r;

    (void)memset(&r, 0, sizeof(r));
    r.boot_slot   = (boot_slot <= 1u) ? boot_slot : 1u;
    r.tries_left  = OTA_ROLLBACK_TRIES_INIT;
    r.unconfirmed = 1u;

    /* ⚠️ 本次运行的缓存也要跟着更新 —— 否则 main.c 里接着调的 poll()
     *    还在拿旧记录（可能 unconfirmed==0 = 已定案）⇒ 新固件永远不会被计数。 */
    s_rec   = r;
    s_valid = true;
    s_done  = false;

    if (!ota_port_rollback_save(&r))
    {
        /* 见 ota_health.h 的说明：只有在"标记页刚被擦过"时写才合法
         * （tries_left 0x00 -> 0xFF 是 0->1）。写失败通常是调用时机不对。 */
        log_info("[OTAHL] !! 新固件重置回滚记录失败(标记页是否未擦?)\r\n");
        return;
    }

    log_info("[OTAHL] 新固件已装: 槽=%u, 机会重置为 %u 次\r\n",
             (unsigned)r.boot_slot, (unsigned)OTA_ROLLBACK_TRIES_MAX);
}

/* ---------------- 诊断 ---------------- */

uint8_t ota_health_tries_left(void)
{
    return s_valid ? bits_left(s_rec.tries_left) : 0u;
}

uint8_t ota_health_boot_slot(void)
{
    return s_valid ? s_rec.boot_slot : 0u;
}

bool ota_health_confirmed(void)
{
    return s_valid && (s_rec.unconfirmed == 0u);
}

bool ota_health_valid(void)
{
    return s_valid;
}
