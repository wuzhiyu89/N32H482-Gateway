/**
 * \file  fake_ota_port.c
 * \brief 主机单测用的 ota_port 替身(RAM 后端)
 *
 * 只实现 ota_http.c 会调到的那些: Flash 擦/写、CRC32、时基、标记页。
 * 真实实现见 app/ota/ota_port.c(N32H482 固件库 + storage.c), 那份不在主机上编。
 *
 * 【为什么值得这么做】: ota_http.c 最容易错的恰恰是与 Flash 打交道的两个点 ——
 *   1) 8 字节对齐(ota_port_flash_write 会自动补 0xFF, 补过的 double word
 *      不能再写第二次) —— 这里有 align_programmed[] 记账, 谁重复写就报出来;
 *   2) 片边界是否真的对齐 —— 每片收完暂存区必须正好空, 否则 s_written 会错位。
 * 这两点在真板上只会表现为"烧进去的固件校验不过", 很难定位。
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "ota_port.h"
#include "ota_mark.h"   /* ota_mark_t 的定义在 bootloader.h */
#include "flash_map.h"

/* ==================== 假 Flash ==================== */
/* FM_STAGE_SIZE = 240 KB —— 用 static 而不是 malloc, 出错时更好看 */
static uint8_t  g_stage[FM_STAGE_SIZE];
static uint8_t  g_written[FM_STAGE_SIZE / 8u];   /* 每个 double word 是否已写 */
static uint32_t g_erase_first_page;
static uint32_t g_erase_end_page;
static uint32_t g_erase_step_done;
static bool     g_erase_all_ok;
static int      g_double_write_err;              /* 重复写 double word 的次数 */
static int      g_fail_next_write;               /* 测试注入: 让下一次写失败 */

/* 标记页 */
static uint8_t  g_mark[FM_META_SIZE];
static bool     g_mark_valid;
static uint32_t g_mark_erase_cnt;

/* OTA over HTTP 的续传记录(和标记同在一个 8KB 页里)。
 * ★ 这里必须【忠实模拟硬件的整页擦除语义】: mark_valid()/mark_invalid() 一擦页,
 *    续传记录也必须跟着失效。否则测试会在假实现上通过、真板上失败。 */
static ota_http_resume_t g_resume;
#define FAKE_NO_RECORD  0xFFFFFFFFu

/* 时基: 委托给 host_stub.c */
extern uint32_t get_tick_ms(void);

/* ==================== 诊断(测试用) ==================== */

const uint8_t *fake_stage_ptr(void)   { return g_stage; }
int   fake_double_write_err(void)     { return g_double_write_err; }
bool  fake_mark_valid(void)           { return g_mark_valid; }
uint32_t fake_mark_erase_count(void)  { return g_mark_erase_cnt; }

const ota_mark_t *fake_mark_ptr(void) { return (const ota_mark_t *)g_mark; }

void fake_reset(void)
{
    memset(g_stage, 0xFF, sizeof(g_stage));
    memset(g_written, 0, sizeof(g_written));
    memset(g_mark, 0xFF, sizeof(g_mark));
    g_resume.magic = FAKE_NO_RECORD;          /* 整页 0xFF = 无记录 */
    g_erase_first_page = 0u;
    g_erase_end_page = 0u;
    g_erase_step_done = 0u;
    g_erase_all_ok = true;
    g_double_write_err = 0;
    g_fail_next_write = 0;
    g_mark_valid = false;
    g_mark_erase_cnt = 0u;
}

/** @brief 只读地取续传记录(测试断言用) */
const ota_http_resume_t *fake_resume_ptr(void) { return &g_resume; }

void fake_fail_next_write(void) { g_fail_next_write = 1; }

/* ==================== Flash 原语 ==================== */

bool ota_port_flash_erase_begin_at(uint32_t first_page, uint32_t pages)
{
    uint32_t max_pages = FM_STAGE_SIZE / FM_PAGE_SIZE;

    if ((pages == 0u) || (first_page >= max_pages) ||
        (pages > (max_pages - first_page)))
    {
        return false;
    }
    g_erase_first_page = first_page;
    g_erase_end_page   = first_page + pages;
    g_erase_step_done  = first_page;
    g_erase_all_ok     = true;
    return true;
}

bool ota_port_flash_erase_begin(uint32_t pages)
{
    return ota_port_flash_erase_begin_at(0u, pages);
}

bool ota_port_flash_erase_step(void)
{
    if (g_erase_step_done >= g_erase_end_page)
    {
        return true;
    }
    /* 真板是按页擦; 这里整片预置 0xFF 即可 —— 但要"擦"掉已写标记 */
    memset(&g_written[g_erase_step_done * (FM_PAGE_SIZE / 8u)], 0,
           FM_PAGE_SIZE / 8u);
    memset(&g_stage[g_erase_step_done * FM_PAGE_SIZE], 0xFF, FM_PAGE_SIZE);

    g_erase_step_done++;
    return (g_erase_step_done >= g_erase_end_page);
}

bool ota_port_flash_erase_ok(void) { return g_erase_all_ok; }

bool ota_port_flash_write(uint32_t offset, const uint8_t *data, uint16_t len)
{
    uint16_t i;
    uint32_t base;
    uint32_t tail;

    if ((data == NULL) || (len == 0u))
    {
        return false;
    }
    if (g_fail_next_write)
    {
        g_fail_next_write = 0;
        return false;
    }
    if ((offset + len) > FM_STAGE_SIZE)
    {
        return false;
    }

    (void)memcpy(&g_stage[offset], data, len);

    /* 记账 double word: 已写过的再写一次 = 真板上必然数据错乱 */
    base = offset / 8u;
    tail = (offset + len + 7u) / 8u;
    for (i = 0; (uint32_t)i < (tail - base); i++)
    {
        uint32_t dw = base + (uint32_t)i;
        if (g_written[dw] != 0u)
        {
            g_double_write_err++;
            printf("      !! double word %lu 被写了第二次(offset=%lu len=%u)\n",
                   (unsigned long)dw, (unsigned long)offset, (unsigned)len);
        }
        g_written[dw] = 1u;
    }
    return true;
}

bool ota_port_flash_read(uint32_t offset, uint8_t *buf, uint16_t len)
{
    if ((buf == NULL) || ((offset + len) > FM_STAGE_SIZE))
    {
        return false;
    }
    (void)memcpy(buf, &g_stage[offset], len);
    return true;
}

/* ==================== CRC32(与 ota_port.c 同参数) ==================== */

uint32_t ota_port_crc32_update(uint32_t init, const uint8_t *data, uint16_t len)
{
    uint32_t crc = init;
    uint16_t i;
    uint8_t  b;

    for (i = 0u; i < len; i++)
    {
        crc ^= data[i];
        for (b = 0u; b < 8u; b++)
        {
            crc = ((crc & 1u) != 0u) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
        }
    }
    return crc;
}

/* ==================== 标记页 ==================== */

bool ota_port_mark_valid(const ota_meta_t *meta)
{
    ota_mark_t mk;

    if (meta == NULL)
    {
        return false;
    }
    memset(&mk, 0, sizeof(mk));
    mk.magic[0] = 'O'; mk.magic[1] = 'T'; mk.magic[2] = 'A'; mk.magic[3] = 'V';
    mk.crc32      = meta->crc32;
    mk.total_size = meta->total_size;

    (void)memcpy(g_mark, &mk, sizeof(mk));
    g_mark_valid = true;

    /* ★ 整页擦除语义: mark_valid 内部是先擦页再写标记, 续传记录必须跟着消失 */
    g_resume.magic = FAKE_NO_RECORD;
    return true;
}

void ota_port_mark_invalid(void)
{
    memset(g_mark, 0xFF, sizeof(g_mark));
    g_mark_valid = false;
    g_mark_erase_cnt++;

    /* ★ 整页擦除语义: 见上面 mark_valid 的说明 */
    g_resume.magic = FAKE_NO_RECORD;
}

/* ==================== OTA over HTTP 续传记录(RAM 模拟) ==================== */

bool ota_port_http_resume_save(uint32_t written, uint32_t size,
                               const char *md5hex, const char *version)
{
    if ((md5hex == NULL) || (size == 0u))
    {
        return false;
    }
    memset(&g_resume, 0, sizeof(g_resume));
    g_resume.magic   = OTA_HTTP_RESUME_MAGIC;
    g_resume.written = written & ~(FM_PAGE_SIZE - 1u);   /* 页对齐, 与真实现一致 */
    g_resume.size    = size;
    (void)snprintf(g_resume.md5hex, sizeof(g_resume.md5hex), "%s", md5hex);
    if (version != NULL)
    {
        (void)snprintf(g_resume.version, sizeof(g_resume.version), "%s", version);
    }
    return true;
}

bool ota_port_http_resume_load(uint32_t *written, uint32_t *size,
                               char *md5hex, uint16_t md5cap,
                               char *version, uint16_t vercap)
{
    if (g_resume.magic != OTA_HTTP_RESUME_MAGIC)
    {
        return false;
    }
    if (strlen(g_resume.md5hex) != 32u)
    {
        return false;
    }
    /* 与真实现同样的记录自校验(见 ota_port.c 里的说明) */
    if ((g_resume.size == 0u) || (g_resume.size > FM_STAGE_SIZE) ||
        (g_resume.written > g_resume.size))
    {
        return false;
    }
    if (written != NULL) { *written = g_resume.written; }
    if (size    != NULL) { *size    = g_resume.size;    }
    if ((md5hex != NULL) && (md5cap > 0u))
    {
        (void)snprintf(md5hex, md5cap, "%s", g_resume.md5hex);
    }
    if ((version != NULL) && (vercap > 0u))
    {
        (void)snprintf(version, vercap, "%s", g_resume.version);
    }
    return true;
}

void ota_port_http_resume_clear(void)
{
    /* 真实现是把 magic 写 0(不擦页), 这里等价 */
    g_resume.magic = 0u;
}

/* ==================== A/B 回滚记录(2026-10-11 新增) ====================
 *
 * ⚠️⚠️ 这个假实现的重点【不是】"能存能取", 而是【必须模拟 Flash 的 1->0 约束】⚠️⚠️
 *
 *   真机上 ota_port_rollback_save() 【不擦页】(同页还有 OTA 标记和续传记录),
 *   所以它只能写"相对当前内容只把 1 变成 0"的值 —— 违反了就写不进去。
 *
 *   如果这里用一个普通结构体直接赋值(最自然的写法), 那么三个【上板才炸】的坑
 *   在主机单测里根本不会出现:
 *     · 用普通计数器递增(tries_left 1->2 需要 0->1)
 *     · 重算校验和(内容一变校验和就变, 新值必含 0->1)
 *     · 用 confirmed 0->1 置位
 *   ⇒ "单测全绿、上板全废" 就是这么来的。
 *
 *   ⇒ 所以下面拿【字节数组当 Flash】, save() 时逐位检查 (old & new) == new,
 *     不满足就返回 false —— 和真实现的行为一致。
 * ==================================================================== */

static uint8_t g_rb[sizeof(ota_rollback_t)];   /* 模拟这块 Flash 的内容 */
static bool    g_rb_inited;

static void rb_ensure_init(void)
{
    if (!g_rb_inited)
    {
        (void)memset(g_rb, 0xFF, sizeof(g_rb));   /* 擦除态 */
        g_rb_inited = true;
    }
}

/* ---- 给测试用的辅助(见 fake_ota_port.h 的声明) ---- */

/** 把这块区域弄回擦除态 —— 模拟"OTA 下载擦过标记页" */
void fake_ota_rb_erase(void)
{
    (void)memset(g_rb, 0xFF, sizeof(g_rb));
    g_rb_inited = true;
}

/** 直接把原始字节塞进去(绕过 1->0 检查) —— 用来造"半截记录/坏记录" */
void fake_ota_rb_poke(const void *raw)
{
    rb_ensure_init();
    (void)memcpy(g_rb, raw, sizeof(g_rb));
}

/** 读原始字节 —— 用来断言"记录里到底写了什么" */
const uint8_t *fake_ota_rb_bytes(void)
{
    rb_ensure_init();
    return g_rb;
}

bool ota_port_rollback_load(ota_rollback_t *out)
{
    ota_rollback_t rec;

    if (out == NULL) { return false; }
    rb_ensure_init();
    (void)memcpy(&rec, g_rb, sizeof(rec));

    /* 与真实现同一套判据(值域校验取代校验和, 见 ota_port.c) */
    if (rec.magic != OTA_ROLLBACK_MAGIC) { return false; }
    if ((rec.boot_slot > 1u) || (rec.unconfirmed > 1u)) { return false; }

    *out = rec;
    return true;
}

bool ota_port_rollback_save(const ota_rollback_t *rec)
{
    ota_rollback_t w;
    const uint8_t *neu = (const uint8_t *)&w;
    size_t         i;

    if (rec == NULL) { return false; }
    rb_ensure_init();

    w = *rec;
    w.magic = OTA_ROLLBACK_MAGIC;
    w.rsv0  = 0u;
    (void)memset(w.rsv, 0, sizeof(w.rsv));

    /* ★★ 核心: 逐位检查"新值有没有把任何 0 变回 1" ★★
     * (old & new) == new  <=>  new 的每一个 1 位在 old 里也是 1
     *                      <=>  只做了 1->0, 没有 0->1 */
    for (i = 0u; i < sizeof(w); i++)
    {
        if ((uint8_t)(g_rb[i] & neu[i]) != neu[i])
        {
            return false;       /* 需要 0->1 —— 真机上同样写不进去 */
        }
    }

    (void)memcpy(g_rb, neu, sizeof(w));
    return true;
}

void ota_port_rollback_clear(void)
{
    /* 与真实现一致: 只把 magic 那 8 字节写 0, 【不擦页】。
     * 写 0 永远是合法的 1->0, 所以这里不做失败检查(和真实现一样忽略返回值)。 */
    uint8_t zero8[8] = { 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u };

    rb_ensure_init();
    (void)memcpy(g_rb, zero8, sizeof(zero8));
}

/* ==================== 其它 ==================== */

uint32_t ota_port_get_tick_ms(void) { return get_tick_ms(); }

const char *ota_port_get_version(void) { return "0.0.0-test"; }

void ota_port_reboot(void) { /* 主机上什么也不做 */ }

uint16_t ota_port_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    uint16_t i;
    uint8_t  b;
    for (i = 0u; i < len; i++)
    {
        crc ^= data[i];
        for (b = 0u; b < 8u; b++)
        {
            crc = ((crc & 1u) != 0u) ? ((crc >> 1) ^ 0xA001u) : (crc >> 1);
        }
    }
    return crc;
}

/* ==================== A/B 槽（主机替身） ==================== */
uint8_t  ota_port_current_slot(void) { return 0u; }
uint8_t  ota_port_target_slot(void)  { return 1u; }
void     ota_port_set_stage(uint32_t addr, uint32_t size) { (void)addr; (void)size; }
uint32_t ota_port_stage_addr(void) { return (uint32_t)FM_STAGE_ADDR; }
uint32_t ota_port_stage_size(void) { return (uint32_t)FM_STAGE_SIZE; }
void     ota_port_boot_poll(bool online, bool uplink_ok, uint32_t uptime_s)
{ (void)online; (void)uplink_ok; (void)uptime_s; }
bool ota_port_boot_load(ota_boot_rec_t *out) { if (out) { memset(out, 0, sizeof(*out)); } return false; }
bool ota_port_boot_save(const ota_boot_rec_t *rec) { (void)rec; return true; }

