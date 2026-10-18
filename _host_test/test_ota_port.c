/* ============================================================================
 * test_ota_port.c —— app/ota/ota_port.c 的主机测试
 * ----------------------------------------------------------------------------
 * 【为什么现在才补】: ota_port.c 一直编不进主机测试(它要 N32 固件库的
 * Flash 原语), 所以 2026-10-02 之前的改动全靠"上板看现象"。
 * 而这次为了做掉电续传, 我动了两处【最危险】的东西:
 *   · ota_port_flash_erase_begin_at()  —— 改擦除范围, 错一点就擦掉不该擦的
 *   · ota_port_http_resume_*()          —— 往标记页里写记录
 * 标记页里还住着 bootloader 的 "OTAV" 标记, 擦错了会把待搬运的升级取消掉。
 * 所以这里用手工替身把它拉到 PC 上测。
 *
 * 替身的关键: fake_storage.c 把 Flash 写成"只能 1->0"(&= 而不是 memcpy)。
 * 于是"没擦就写"这种真板上最难查的错, 在这里直接变成断言失败。
 *
 * 覆盖:
 *   [1] erase_begin_at: 只擦指定页, 前面的页一个字节都不许动
 *   [2] 续传记录 save -> load 往返 + written 页对齐
 *   [3] ★ mark_invalid() 会整页擦除 -> 续传记录必须跟着失效
 *   [4] ★ mark_valid()   同上
 *   [5] ★ clear() 只清 magic, 【不许】碰 bootloader 的 "OTAV" 标记
 *   [6] 空记录 / 畸形记录 必须判为"无记录"
 *   [7] 参数防护
 * ========================================================================== */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "ota_port.h"
#include "ota.h"
#include "flash_map.h"
#include "storage.h"

/* fake_storage.c */
extern void           fake_storage_reset(void);
extern const uint8_t *fake_storage_ptr(void);
/* 2026-10-14: 专用直写接口(绕过编程约束) —— 见 fake_storage.c 的说明 */
extern void           fake_storage_poke8(uint32_t flash_addr, const uint8_t *data, uint16_t len);
/* host_stub.c */
extern uint32_t       get_tick_ms(void);

static int g_pass;
static int g_fail;

#define CHECK(cond, ...)                                        \
    do {                                                        \
        if (cond) { g_pass++; printf("  ok   : "); }             \
        else      { g_fail++; printf("  FAIL : "); }             \
        printf(__VA_ARGS__);                                    \
        printf("\n");                                           \
    } while (0)

#define MD5A "9e96d33cf71a79caabd00b319e115af5"

/** @brief 读一个字节(越界/失败时返回 0x00, 便于断言里一眼看出不对) */
static uint8_t rd8(uint32_t addr)
{
    uint8_t b = 0u;

    if (!storage_flash_read(addr, &b, 1u))
    {
        return 0u;
    }
    return b;
}

/** @brief 直接写一个 double word(绕过 ota_port, 用来【铺测试数据 / 造坏数据】)
 *
 *  ★2026-10-14 改: 不再走 storage_flash_write()。
 *    因为假实现已经收紧成"只能写擦除态的字"(与真机一致), 而本函数要做的
 *    恰恰是真机做不到的事 —— 往已编程过的位置塞任意值(模拟掉电写坏/破坏 magic)。
 *    ⇒ 改用 fake_storage_poke8()(专用直写接口, 明确绕过编程约束)。
 *    以前它用 storage_flash_write() 能"成功", 只是因为当时假实现是
 *    `&=`, 比真机宽松 —— 那正是"单测全绿、上板全废"的来源。 */
static void poke8(uint32_t addr, uint64_t val)
{
    uint8_t b[8];
    int i;

    for (i = 0; i < 8; i++)
    {
        b[i] = (uint8_t)((val >> (i * 8)) & 0xFFu);
    }
    fake_storage_poke8(addr, b, 8u);
}

/** @brief 把擦除跑到完成; 返回 erase_ok() */
static bool drain_erase(void)
{
    int guard = 0;

    while (!ota_port_flash_erase_step() && (guard++ < 1000))
    {
        /* 一直推到完成 */
    }
    return ota_port_flash_erase_ok();
}

/* ==================== [1] 从指定页开始擦 ==================== */
static void test_erase_begin_at(void)
{
    uint32_t max_pages = FM_STAGE_SIZE / FM_PAGE_SIZE;

    printf("\n[1] ota_port_flash_erase_begin_at: 只擦指定页\n");

    fake_storage_reset();

    /* 给前 4 页各铺一个"指纹"(每页首 8 字节写 0x1122334455667788 的变体) */
    poke8(FM_STAGE_ADDR + 0u * FM_PAGE_SIZE, 0x1111111111111111ull);
    poke8(FM_STAGE_ADDR + 1u * FM_PAGE_SIZE, 0x2222222222222222ull);
    poke8(FM_STAGE_ADDR + 2u * FM_PAGE_SIZE, 0x3333333333333333ull);
    poke8(FM_STAGE_ADDR + 3u * FM_PAGE_SIZE, 0x4444444444444444ull);

    /* 从第 2 页起擦 2 页 (即擦掉页 2、3) */
    CHECK(ota_port_flash_erase_begin_at(2u, 2u), "begin_at(2,2) 可以启动");
    CHECK(drain_erase(), "擦除全部成功");

    CHECK(rd8(FM_STAGE_ADDR + 0u * FM_PAGE_SIZE) == 0x11u,
          "★ 页 0 【没被擦】(0x%02X)", rd8(FM_STAGE_ADDR + 0u * FM_PAGE_SIZE));
    CHECK(rd8(FM_STAGE_ADDR + 1u * FM_PAGE_SIZE) == 0x22u,
          "★ 页 1 【没被擦】(0x%02X)", rd8(FM_STAGE_ADDR + 1u * FM_PAGE_SIZE));
    CHECK(rd8(FM_STAGE_ADDR + 2u * FM_PAGE_SIZE) == 0xFFu,
          "页 2 已被擦 (0x%02X)", rd8(FM_STAGE_ADDR + 2u * FM_PAGE_SIZE));
    CHECK(rd8(FM_STAGE_ADDR + 3u * FM_PAGE_SIZE) == 0xFFu,
          "页 3 已被擦 (0x%02X)", rd8(FM_STAGE_ADDR + 3u * FM_PAGE_SIZE));

    /* 老 API 必须还是"从第 0 页开始"(ota.c 的 MQTT 分包版还在用) */
    fake_storage_reset();
    poke8(FM_STAGE_ADDR + 0u * FM_PAGE_SIZE, 0xAAAAAAAAAAAAAAAAull);
    CHECK(ota_port_flash_erase_begin(1u), "老 API erase_begin(1) 仍可启动");
    CHECK(drain_erase(), "老 API 擦除成功");
    CHECK(rd8(FM_STAGE_ADDR) == 0xFFu, "★ 老 API 行为不变: 从第 0 页擦起");

    /* 越界必须拒绝 */
    fake_storage_reset();
    CHECK(!ota_port_flash_erase_begin_at(max_pages, 1u),
          "起始页越界 -> 拒绝");
    CHECK(!ota_port_flash_erase_begin_at(0u, max_pages + 1u),
          "页数越界 -> 拒绝");
    CHECK(!ota_port_flash_erase_begin_at(0u, 0u), "页数 0 -> 拒绝");
    CHECK(ota_port_flash_erase_begin_at(max_pages - 1u, 1u),
          "最后一页 -> 允许");
}

/* ==================== [2] 续传记录往返 ==================== */
static void test_resume_roundtrip(void)
{
    uint32_t w = 0u;
    uint32_t s = 0u;
    char     md5[33];
    char     ver[24];

    printf("\n[2] 续传记录 save -> load 往返\n");

    fake_storage_reset();
    memset(md5, 0, sizeof(md5));
    memset(ver, 0, sizeof(ver));

    CHECK(ota_port_http_resume_load(&w, &s, md5, (uint16_t)sizeof(md5),
                                    ver, (uint16_t)sizeof(ver)) == false,
          "★ 空 Flash -> 没有记录");

    CHECK(ota_port_http_resume_save(12345u, 65536u, MD5A, "1.2.1"),
          "save 成功");

    CHECK(ota_port_http_resume_load(&w, &s, md5, (uint16_t)sizeof(md5),
                                    ver, (uint16_t)sizeof(ver)) == true,
          "load 成功");
    CHECK(s == 65536u, "size = %lu", (unsigned long)s);
    CHECK(w == (12345u & ~(FM_PAGE_SIZE - 1u)),
          "★ written 被页对齐: 12345 -> %lu", (unsigned long)w);
    CHECK(strcmp(md5, MD5A) == 0, "md5 = %s", md5);
    CHECK(strcmp(ver, "1.2.1") == 0, "version = %s", ver);

    /* 页对齐边界: 8192 正好不动; 16383 向下落到 8192 */
    CHECK(ota_port_http_resume_save(FM_PAGE_SIZE, 65536u, MD5A, NULL), "save(整页)");
    CHECK(ota_port_http_resume_load(&w, &s, NULL, 0u, NULL, 0u), "load(只要数字)");
    CHECK(w == FM_PAGE_SIZE, "★ 正好页边界不动: %lu", (unsigned long)w);

    CHECK(ota_port_http_resume_save((2u * FM_PAGE_SIZE) - 1u, 65536u, MD5A, NULL),
          "save(2 页 - 1 字节)");
    CHECK(ota_port_http_resume_load(&w, &s, NULL, 0u, NULL, 0u), "load");
    CHECK(w == FM_PAGE_SIZE,
          "★ 16383 向下页对齐到 8192 (得到 %lu)", (unsigned long)w);

    /* 覆盖写: 第二次 save 必须能改掉第一次的值(内部要擦页) */
    CHECK(ota_port_http_resume_save(32768u, 65536u, MD5A, "1.2.2"), "再 save 一次");
    CHECK(ota_port_http_resume_load(&w, &s, md5, (uint16_t)sizeof(md5),
                                    ver, (uint16_t)sizeof(ver)), "load");
    CHECK(w == 32768u, "★ 覆盖写生效: written = %lu", (unsigned long)w);
    CHECK(strcmp(ver, "1.2.2") == 0, "★ version 也更新到 %s", ver);
}

/* ==================== [3][4] 整页擦除会带走续传记录 ==================== */
static void test_mark_erase_kills_resume(void)
{
    ota_meta_t meta;

    printf("\n[3][4] ★标记页是整页擦: mark_* 必须让续传记录一起失效\n");

    memset(&meta, 0, sizeof(meta));
    meta.total_size = 65536u;
    meta.crc32      = 0x12345678u;
    (void)strncpy(meta.version, "1.2.1", sizeof(meta.version) - 1u);

    /* --- mark_invalid --- */
    fake_storage_reset();
    CHECK(ota_port_http_resume_save(16384u, 65536u, MD5A, "1.2.1"), "先写一条记录");
    CHECK(ota_port_http_resume_load(NULL, NULL, NULL, 0u, NULL, 0u), "记录在");
    ota_port_mark_invalid();
    CHECK(!ota_port_http_resume_load(NULL, NULL, NULL, 0u, NULL, 0u),
          "★★ mark_invalid() 之后记录失效(整页擦除语义)");

    /* --- mark_valid --- */
    fake_storage_reset();
    CHECK(ota_port_http_resume_save(16384u, 65536u, MD5A, "1.2.1"), "先写一条记录");
    CHECK(ota_port_mark_valid(&meta), "写有效标记");
    CHECK(!ota_port_http_resume_load(NULL, NULL, NULL, 0u, NULL, 0u),
          "★★ mark_valid() 之后记录失效");

    /* 标记本身要写对 */
    CHECK(rd8(FM_META_ADDR + 0u) == (uint8_t)'O', "标记 magic[0] = 'O'");
    CHECK(rd8(FM_META_ADDR + 3u) == (uint8_t)'V', "标记 magic[3] = 'V'");
}

/* ==================== [5] clear() 不许碰标记 ==================== */
static void test_clear_keeps_mark(void)
{
    ota_meta_t meta;

    printf("\n[5] ★clear() 只清 magic, 不许动 bootloader 的标记\n");

    memset(&meta, 0, sizeof(meta));
    meta.total_size = 65536u;
    meta.crc32      = 0xABCDEF01u;

    fake_storage_reset();

    /* 先写标记(这一步会擦页, 所以它必须【先】做) */
    CHECK(ota_port_mark_valid(&meta), "写有效标记");
    CHECK(rd8(FM_META_ADDR) == (uint8_t)'O', "标记已就位");

    /* 再 clear 续传记录(此时记录本来是空的, clear 应该幂等且无害) */
    ota_port_http_resume_clear();

    CHECK(rd8(FM_META_ADDR + 0u) == (uint8_t)'O', "★ clear 之后 magic[0] 仍是 'O'");
    CHECK(rd8(FM_META_ADDR + 1u) == (uint8_t)'T', "★ magic[1] 仍是 'T'");
    CHECK(rd8(FM_META_ADDR + 2u) == (uint8_t)'A', "★ magic[2] 仍是 'A'");
    CHECK(rd8(FM_META_ADDR + 3u) == (uint8_t)'V', "★ magic[3] 仍是 'V'");

    /* 记录被清掉了吗? —— 本来就没有, 但 clear 之后必须仍然"没有" */
    CHECK(!ota_port_http_resume_load(NULL, NULL, NULL, 0u, NULL, 0u),
          "clear 之后没有记录");
}

/* ==================== [6] 畸形记录 ==================== */
static void test_bad_record(void)
{
    uint32_t off = FM_META_ADDR + OTA_HTTP_RESUME_OFFSET;

    printf("\n[6] 畸形 / 半截记录必须判为'无记录'\n");

    /* magic 对但 md5 不完整(长度 != 32) */
    fake_storage_reset();
    poke8(off, (uint64_t)OTA_HTTP_RESUME_MAGIC);          /* magic 对, 后面全 0xFF */
    CHECK(!ota_port_http_resume_load(NULL, NULL, NULL, 0u, NULL, 0u),
          "★ 半截记录(magic 写了, 其余还是 0xFF) -> 拒绝");

    /* 同理: magic + 合理的 size, 但 written 超过 size */
    fake_storage_reset();
    {
        uint8_t rec[80];
        memset(rec, 0xFF, sizeof(rec));
        rec[0] = 0x48; rec[1] = 0x54; rec[2] = 0x54; rec[3] = 0x50;  /* "HTTP" 小端 */
        rec[4] = 0x00; rec[5] = 0x00; rec[6] = 0x01; rec[7] = 0x00;  /* written = 65536 */
        rec[8] = 0x00; rec[9] = 0x00; rec[10] = 0x00; rec[11] = 0x01;/* size    = 0x01000000 (过大) */
        (void)storage_flash_write(off, rec, (uint16_t)sizeof(rec));
    }
    CHECK(!ota_port_http_resume_load(NULL, NULL, NULL, 0u, NULL, 0u),
          "★ size 超出暂存区容量 -> 拒绝");

    /* magic 写成别的值 */
    fake_storage_reset();
    poke8(off, 0xDEADBEEFDEADBEEFull);
    CHECK(!ota_port_http_resume_load(NULL, NULL, NULL, 0u, NULL, 0u),
          "★ magic 不对 -> 拒绝");

    /* ==========================================================================
     * ★★2026-10-14 已知缺陷(如实记录, 不修)★★
     * --------------------------------------------------------------------------
     * 原断言是:
     *     ota_port_http_resume_clear(); 之后必须读不到记录(且可重复调)。
     * 收紧 fake_storage()(改成"只能编程处于擦除态的字", 与真机一致)之后,
     * 这两条【必然失败】—— 因为 resume_clear() 的实现是:
     *     storage_flash_write(OTA_META_ADDR + OTA_HTTP_RESUME_OFFSET, zero8, 8)
     * 而那个位置【已经被记录写过了】, 不是擦除态
     * ⇒ N32H48x 的编程只对擦除态的字成功 ⇒ 这个写**在真机上同样会失败**。
     *
     * ⇒ 结论: ota_port_http_resume_clear() 在本芯片上是【无效操作】。
     *   (不是测试环境的问题, 是真机行为 —— 以前假实现用 &= 才让它"看起来能工作"。)
     *
     * 【为什么不修】真正让续传记录失效的是 mark_valid()/mark_invalid() ——
     *   它们【整页擦除】(见 ota_port.h 的说明), 记录随之消失。
     *   改 resume_clear() 会动到已验证过的 OTA 下载路径, 回归风险 > 收益。
     *   ⇒ 按用户 2026-10-14 决定: 如实标注, 不改实现。
     *
     * 【所以这里验的是"事实", 不是"期望"】: clear 之后记录【仍在】。
     * ========================================================================== */
    fake_storage_reset();
    CHECK(ota_port_http_resume_save(8192u, 65536u, MD5A, "1.0"), "写一条好记录");
    CHECK(ota_port_http_resume_load(NULL, NULL, NULL, 0u, NULL, 0u), "好记录可读");
    ota_port_http_resume_clear();
    CHECK(ota_port_http_resume_load(NULL, NULL, NULL, 0u, NULL, 0u),
          "★ 【已知缺陷】clear 在本芯片上无效(改写已编程的字必然失败) -> 记录仍在");

    /* 重复调也一样无效(不是"幂等", 是"从来没生效过") —— 同样只断言事实 */
    ota_port_http_resume_clear();
    ota_port_http_resume_clear();
    CHECK(ota_port_http_resume_load(NULL, NULL, NULL, 0u, NULL, 0u),
          "★ 【已知缺陷】clear 重复调同样无效 -> 记录仍在");
}

/* ==================== [7] 参数防护 ==================== */
static void test_guards(void)
{
    printf("\n[7] 参数防护\n");

    fake_storage_reset();

    CHECK(!ota_port_http_resume_save(0u, 65536u, NULL, NULL), "md5=NULL -> 拒绝");
    CHECK(!ota_port_http_resume_save(0u, 0u, MD5A, NULL), "size=0 -> 拒绝");
    CHECK(ota_port_http_resume_save(0u, 65536u, MD5A, NULL), "version=NULL 可以");

    /* load 的输出参数允许为 NULL(只要数字 / 只要字符串 都行) */
    CHECK(ota_port_http_resume_load(NULL, NULL, NULL, 0u, NULL, 0u),
          "load 全部输出参数传 NULL 也不崩");

    /* 缓冲很小: 必须截断并封口, 不越界 */
    {
        char tiny[4];
        char md[6];
        uint32_t w = 0u;
        uint32_t s = 0u;

        memset(tiny, 'X', sizeof(tiny));
        memset(md, 'Y', sizeof(md));
        CHECK(ota_port_http_resume_load(&w, &s, md, (uint16_t)sizeof(md),
                                        tiny, (uint16_t)sizeof(tiny)),
              "小缓冲 load 也成功");
        CHECK(md[sizeof(md) - 1u] == '\0', "★ md5 小缓冲被封口");
        CHECK(tiny[sizeof(tiny) - 1u] == '\0', "★ version 小缓冲被封口");
    }

    /* 端到端留个证据: 记录区确实落在标记页里, 且不与 mark/ctx 重叠 */
    printf("       记录偏移 = %u, 结构 %u 字节, 标记页 %lu 字节\n",
           (unsigned)OTA_HTTP_RESUME_OFFSET, (unsigned)sizeof(ota_http_resume_t),
           (unsigned long)FM_META_SIZE);
    CHECK((OTA_HTTP_RESUME_OFFSET + sizeof(ota_http_resume_t)) <= FM_META_SIZE,
          "★ 记录完全落在标记页内");
    CHECK(OTA_HTTP_RESUME_OFFSET >= 144u,
          "★ 不与 mark([0..31]) 和 MQTT ctx([40..~143]) 重叠");
    CHECK((sizeof(ota_http_resume_t) % 8u) == 0u, "★ 结构 8 字节对齐");
    CHECK((OTA_HTTP_RESUME_OFFSET % 8u) == 0u, "★ 偏移 8 字节对齐");
}

int main(void)
{
    printf("==== app/ota/ota_port.c 主机测试(手工 Flash 替身) ====\n");
    printf("Flash 替身: 只能 1->0, 8 字节对齐强校验 —— 比真实现更严\n");

    fake_storage_reset();

    test_erase_begin_at();
    test_resume_roundtrip();
    test_mark_erase_kills_resume();
    test_clear_keeps_mark();
    test_bad_record();
    test_guards();

    printf("\n==== 通过 %d, 失败 %d ====\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
