/**
 * \file  test_ota_health.c
 * \brief A/B 回滚状态机的主机单测（2026-10-11）
 *
 * ============================================================================
 * 【为什么用真的 ota_port.c 而不是 fake_ota_port.c】
 *   本套件链接的是 app/ota/ota_port.c + _host_test/fake_storage.c。
 *   那个 fake_storage 把 storage_flash_write() 实现成 `new = old & data`
 *   （见它的文件头），也就是【真的只能 1->0】——
 *   ⇒ 我们测的是【真实实现 + 真实 Flash 语义】，而不是一个"想当然的假实现"。
 *   这一点很关键：本模块的三个设计决定（tries_left 用递减位图、unconfirmed
 *   反转语义、不放校验和）全都是为了满足 1->0；如果测试环境不强制这条，
 *   那些决定就无从验证，"单测全绿"会是假绿。
 *
 * ⚠️ fake_storage 的 storage_flash_write() 【写不进去也返回 true】(它只是 &=)，
 *    ⇒ 违规时表现为【数据被静默改掉】，而不是返回失败。
 *    所以用例 5 断言的是"记录内容没变成期望值"，不是"save 返回 false"。
 * ============================================================================
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "flash_map.h"
#include "ota_port.h"
#include "ota_health.h"
#include "storage.h"        /* storage_flash_erase_page —— 用例 6 要模拟"擦过页" */

/* fake_storage.c 提供的(它没有头文件, 按本工程惯例内联声明 —— 见 test_ota_http.c) */
extern void           fake_storage_reset(void);
extern const uint8_t *fake_storage_ptr(void);

#define FS_BASE      0x08000000u
#define RB_ADDR      (FM_META_ADDR + OTA_ROLLBACK_OFFSET)

static int s_pass, s_fail;

#define CHECK(cond, ...)                                            \
    do {                                                            \
        if (cond) { s_pass++; }                                     \
        else {                                                      \
            s_fail++;                                               \
            printf("  FAIL : ");                                    \
            printf(__VA_ARGS__);                                    \
            printf("\n");                                           \
        }                                                           \
    } while (0)

/** 读【当前有效】的那条记录。
 *  ★2026-10-14: 追加式改造后不能死读槽 0 —— save() 会写到"第一个空槽",
 *  所以记录可能在槽 1/2/3...。这里直接用被测代码自己的 load()(它扫全部槽、
 *  取最后一个有效的), 这才是"当前状态"的正确定义。
 *  load() 失败(无记录)时回落到读槽 0 的原始字节, 好让"坏记录"类用例
 *  仍能看到实际写进去的字节。 */
static void rb_peek(ota_rollback_t *out)
{
    if (!ota_port_rollback_load(out))
    {
        (void)memcpy(out, fake_storage_ptr() + (RB_ADDR - FS_BASE), sizeof(*out));
    }
}

/** 直接往 Flash 里塞原始字节(绕过 storage_flash_write 的 &=) —— 用来造坏记录 */
static void rb_poke(const ota_rollback_t *in)
{
    (void)memcpy((void *)(fake_storage_ptr() + (RB_ADDR - FS_BASE)), in, sizeof(*in));
}

/** 直接写 Flash 里的字节(测边界用) */
static void rb_poke8(uint32_t off, uint8_t v)
{
    ((uint8_t *)fake_storage_ptr())[RB_ADDR - FS_BASE + off] = v;
}

static uint8_t popcount8(uint8_t m)
{
    uint8_t n = 0u;
    while (m != 0u) { n = (uint8_t)(n + (m & 1u)); m = (uint8_t)(m >> 1); }
    return n;
}

/* ========================================================================== */
int main(void)
{
    ota_rollback_t r;
    int            i;

    printf("==== test_ota_health: A/B 回滚状态机 ====\n");

    /* ==================================================================== */
    printf("\n[1] 无记录 -> 建一份满次数的, 且【绝不回滚】\n");
    {
        ota_health_boot_t v;

        fake_storage_reset();                 /* 整片 Flash = 0xFF */
        v = ota_health_boot_step();

        /* ★ 核心断言: 读不到记录时返回 OK, 不是 ROLLBACK。
         *   读不到的常见原因有"全新设备/下载擦过页/记录写坏"三种, 那时
         *   "本槽有没有问题"是未知 —— 把未知当已知去回滚 = 把好固件回滚掉。 */
        CHECK(v == OTA_HEALTH_BOOT_OK, "读不到记录时必须返回 OK(不能回滚), 实际 %d", (int)v);
        CHECK(ota_health_valid(), "应已建立有效记录");
        CHECK(ota_health_tries_left() == (uint8_t)OTA_ROLLBACK_TRIES_MAX,
              "新建时应给满次数 %u, 实际 %u",
              (unsigned)OTA_ROLLBACK_TRIES_MAX, (unsigned)ota_health_tries_left());
        CHECK(!ota_health_confirmed(), "新建时不该是已定案");

        rb_peek(&r);
        CHECK(r.magic == OTA_ROLLBACK_MAGIC, "Flash 里 magic 应写成了 ABRT");
        CHECK(r.unconfirmed == 1u, "新建时 unconfirmed 应为 1(还没确认)");
    }

    /* ==================================================================== */
    printf("\n[2] 逐次启动扣机会: 第 %u 次仍 OK, 第 %u 次请求回滚\n",
           (unsigned)OTA_ROLLBACK_TRIES_MAX, (unsigned)(OTA_ROLLBACK_TRIES_MAX + 1u));
    {
        ota_health_boot_t v = OTA_HEALTH_BOOT_OK;
        int               n;

        fake_storage_reset();
        (void)ota_health_boot_step();         /* 第 1 次: 建记录, 不扣 */

        /* 之后每次启动扣一次。注意 boot_step 会重新 load, 所以可以直接连调 ——
         * 但内存缓存 s_rec 会被覆盖成 Flash 里的值, 这正是我们要的"跨启动"语义。 */
        for (n = 1; n <= (int)OTA_ROLLBACK_TRIES_MAX; n++)
        {
            v = ota_health_boot_step();
            CHECK(v == OTA_HEALTH_BOOT_OK, "第 %d 次启动仍应 OK(还剩机会)", n);
        }

        /* 此时 tries_left 应该已经清空了 */
        CHECK(ota_health_tries_left() == 0u,
              "扣了 %u 次后机会应为 0, 实际 %u",
              (unsigned)OTA_ROLLBACK_TRIES_MAX, (unsigned)ota_health_tries_left());

        v = ota_health_boot_step();
        CHECK(v == OTA_HEALTH_BOOT_ROLLBACK, "机会用完后应请求回滚, 实际 %d", (int)v);
    }

    /* ==================================================================== */
    printf("\n[3] 健康确认 -> 定案; 之后不再写 Flash\n");
    {
        uint8_t before[sizeof(ota_rollback_t)];
        uint8_t after[sizeof(ota_rollback_t)];

        fake_storage_reset();
        (void)ota_health_boot_step();

        /* 三条件缺一都不算健康 —— 逐个否掉 */
        ota_health_poll(false, true,  100u);   CHECK(!ota_health_confirmed(), "4G 离线不该定案");
        ota_health_poll(true,  false, 100u);   CHECK(!ota_health_confirmed(), "无上行确认不该定案");
        ota_health_poll(true,  true,  60u);    CHECK(!ota_health_confirmed(), "uptime 刚好 60s 不该定案(判据是 >)");

        /* 三条都满足 -> 定案 */
        ota_health_poll(true, true, (uint32_t)OTA_HEALTH_MIN_UPTIME_S + 1u);
        CHECK(ota_health_confirmed(), "三条都满足应定案");

        /* ★ 定案之后再调多少次 poll, Flash 都不该再被写 ★ */
        (void)memcpy(before, fake_storage_ptr() + (RB_ADDR - FS_BASE), sizeof(before));
        for (i = 0; i < 50; i++)
        {
            ota_health_poll(true, true, 100000u);
        }
        (void)memcpy(after, fake_storage_ptr() + (RB_ADDR - FS_BASE), sizeof(after));
        CHECK(memcmp(before, after, sizeof(before)) == 0,
              "★ 定案后 Flash 一个字节都不该再变(避免反复写坏)");

        rb_peek(&r);
        CHECK(r.unconfirmed == 0u, "定案后 unconfirmed 应为 0");
    }

    /* ==================================================================== */
    printf("\n[4] 坏记录/半截记录 -> 当'从未记录', 【不回滚】\n");
    {
        ota_health_boot_t v;

        /* (a) magic 对, 但 boot_slot = 0xFF(典型半截记录: 后面还没写完) */
        fake_storage_reset();
        (void)memset(&r, 0xFF, sizeof(r));
        r.magic = OTA_ROLLBACK_MAGIC;
        rb_poke(&r);
        v = ota_health_boot_step();
        CHECK(v == OTA_HEALTH_BOOT_OK, "★ 半截记录必须当'无记录'(不能回滚), 实际 %d", (int)v);

        /* (b) magic 全 0xFF(从没写过) */
        fake_storage_reset();
        v = ota_health_boot_step();
        CHECK(v == OTA_HEALTH_BOOT_OK, "全 0xFF 应返回 OK");

        /* (c) magic 被 clear 清成 0(模拟 OTA 下载擦过页之后) */
        fake_storage_reset();
        ota_port_rollback_clear();
        v = ota_health_boot_step();
        CHECK(v == OTA_HEALTH_BOOT_OK, "被 clear 之后应返回 OK, 实际 %d", (int)v);

        /* (d) 完全垃圾 */
        fake_storage_reset();
        for (i = 0; i < (int)sizeof(r); i++) { rb_poke8((uint32_t)i, (uint8_t)(0x30 + i)); }
        v = ota_health_boot_step();
        CHECK(v == OTA_HEALTH_BOOT_OK, "垃圾记录应返回 OK, 实际 %d", (int)v);
    }

    /* ==================================================================== */
    printf("\n[5] ★ 1->0 约束: 记录【只清一位】, 不是减法 ★\n");
    {
        ota_rollback_t a, b;
        uint8_t        t0, t1;

        /* tries_left 初值 0xFF, 第一次扣完应是 0x7F(只清 bit7)。
         * 若实现写成 `m - 1`, 那 0xFF-1 = 0xFE —— 也是清一位, 这个用例抓不到;
         * 但换成 0x10 就不同: `&(m-1)` -> 0x00(合法), `m-1` -> 0x0F(bit0..3 从 0 变 1, 非法)。
         * 所以这里【直接构造 tries_left = 0x10】来验。 */
        fake_storage_reset();
        (void)memset(&r, 0, sizeof(r));
        r.magic       = OTA_ROLLBACK_MAGIC;
        r.boot_slot   = 0u;
        r.tries_left  = 0x10u;
        r.unconfirmed = 1u;
        rb_poke(&r);

        rb_peek(&a);
        t0 = a.tries_left;
        (void)ota_health_boot_step();
        rb_peek(&b);
        t1 = b.tries_left;

        CHECK(t0 == 0x10u, "前置: tries_left 应为 0x10, 实际 0x%02X", (unsigned)t0);
        CHECK(t1 == 0x00u, "★ 0x10 扣一次后应是 0x00(&(m-1)), 实际 0x%02X", (unsigned)t1);
        CHECK(popcount8(t1) == (popcount8(t0) - 1u),
              "★ 扣一次机会应【只少一个 1 位】(1->0), 位 0x%02X -> 0x%02X",
              (unsigned)t0, (unsigned)t1);

        /* 再验一次: 0xFF 扣一次变 0xFE —— ⚠️ 是清【最低】的 1 位(bit0),
         * 不是清最高位变 0x7F。只要"每次只少一个 1 位"就满足 1->0 约束,
         * 具体是哪一位无所谓。这里写死数值是为了抓"实现改用减法"的回归:
         * 若写成 `m - 1`, 0xFF-1 也是 0xFE —— 这个用例抓不到它, 所以
         * 上面 [5] 开头那个 0x10 的用例才是关键(减法会得到 0x0F = 0->1 非法)。 */
        fake_storage_reset();
        (void)memset(&r, 0, sizeof(r));
        r.magic = OTA_ROLLBACK_MAGIC; r.boot_slot = 0u;
        r.tries_left = 0xFFu; r.unconfirmed = 1u;
        rb_poke(&r);
        (void)ota_health_boot_step();
        rb_peek(&b);
        CHECK(b.tries_left == 0xFEu, "0xFF 扣一次应是 0xFE(清最低位), 实际 0x%02X", (unsigned)b.tries_left);
    }

    /* ==================================================================== */
    printf("\n[6] reset_for_new_fw: 只在【擦过页】之后才有效\n");
    {
        ota_rollback_t a, b;

        /* (a) 正常路径: 先擦页(模拟 OTA 下载擦过标记页), 再 reset ⇒ 应生效 */
        fake_storage_reset();
        (void)ota_health_boot_step();                       /* 建立记录 */
        ota_health_poll(true, true, 100u);                  /* 定案 */
        CHECK(ota_health_confirmed(), "前置: 应已定案");

        (void)storage_flash_erase_page(FM_META_ADDR);       /* ★ 擦页(OTA 下载会这么做) */
        ota_health_reset_for_new_fw(1u);                    /* 新固件装在槽 B */

        CHECK(!ota_health_confirmed(), "★ reset 后必须变回'未定案'(内存缓存也要同步)");
        CHECK(ota_health_boot_slot() == 1u, "reset 后槽号应为 1, 实际 %u", (unsigned)ota_health_boot_slot());
        CHECK(ota_health_tries_left() == (uint8_t)OTA_ROLLBACK_TRIES_MAX,
              "reset 后机会应满, 实际 %u", (unsigned)ota_health_tries_left());

        /* (b) 追加式下的 reset: 【不擦页也能成功】。
         * ⚠️ 这条断言在 2026-10-14 改过:
         *   旧设计(原地改写同一位置)下 reset 要把 tries_left 从 0x00 写回 0xFF,
         *   那是 0->1, Flash 写不进去 ⇒ 当时断言的是"不生效"。
         *   改成追加式之后, save() 写的是【新空槽】(全 0xFF, 必然可编程)
         *   ⇒ 不擦页也能补满。这不是放宽, 是语义确实变了。 */
        fake_storage_reset();
        (void)memset(&r, 0, sizeof(r));
        r.magic = OTA_ROLLBACK_MAGIC; r.boot_slot = 0u;
        r.tries_left = 0x00u; r.unconfirmed = 1u;           /* 机会已用完 */
        rb_poke(&r);

        rb_peek(&a);
        ota_health_reset_for_new_fw(1u);                    /* 不擦页直接 reset */
        rb_peek(&b);

        CHECK(a.tries_left == 0x00u, "前置: 机会应为 0");
        /* ⚠️ 这里要比的是【位图初值】0xFF, 不是次数 8。
         *   tries_left 是位图(0xFF = 8 个 1), OTA_ROLLBACK_TRIES_MAX 是"几次"。
         *   两者差一个 popcount —— 我第一版就写错了, 被这条断言抓出来。 */
        CHECK(b.tries_left == (uint8_t)OTA_ROLLBACK_TRIES_INIT,
              "★ 追加式: reset 写新空槽 ⇒ 不擦页也能把机会补满, 实际 0x%02X",
              (unsigned)b.tries_left);
        CHECK(b.boot_slot == 1u, "reset 后槽号应为 1, 实际 %u", (unsigned)b.boot_slot);
    }

    /* ==================================================================== */
    printf("\n==== test_ota_health: 通过 %d, 失败 %d ====\n", s_pass, s_fail);
    return (s_fail == 0) ? 0 : 1;
}
