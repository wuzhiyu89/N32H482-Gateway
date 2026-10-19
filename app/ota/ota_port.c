/**
 * \file  ota_port.c
 * \brief OTA 移植层 N32H482 实现
 *
 * Flash: 复用 storage.c 的读写接口(带中断保护 + Unlock/Lock),
 *        上层只需处理"尾部补 0xFF 凑 8 字节对齐"这个 ota 协议特有需求
 * CRC32: 纯软件逐位计算(IEEE 802.3, 多项式 0xEDB88320, 与 zip/zlib 一致)
 * CRC16: 纯软件逐位计算(Modbus, 多项式 0xA001, 初始值 0xFFFF)
 *        选择软件而非硬件的原因: 硬件 CRC32 只接受 32-bit word 输入,
 *        固件长度非 4 字节对齐时尾部补 0 会改变 CRC 语义, 与上位机
 *        (Python hashlib / zip) 按字节流计算的 CRC 无法对齐。
 * Reboot: NVIC_SystemReset
 *
 * 与 param_save.c / storage.c 的隔离:
 *   - 地址完全不重叠: OTA 用 0x08040000~0x0807BFFF(固件缓存) + 0x0807C000(标记页),
 *     param_save 用 0x0807E000 起
 *   - Flash 操作统一走 storage.c, 共享同一套 Unlock/Lock + 中断保护,
 *     不会与 param_save 并发时互相踩寄存器
 */
#include "ota_port.h"
#include "storage.h"           /* storage_flash_read / erase_page / write */
#include "utils.h"              /* crc16_modbus — 与 lora_proto 共用, 不重复实现 */
#include "n32h47x_48x.h"       /* NVIC_SystemReset */
#include "bsp_tim.h"           /* get_tick_ms */
#include <string.h>
#include <stdio.h>              /* snprintf: 版本字符串注入 */

/* 续传记录的合法性上界: 每个分包至少 512 字节, 所以 packet_count 不可能超过
 * OTA_FLASH_SIZE / 512。这个 512 来自旧的 MQTT 分包版(已归档), 这里只是拿它
 * 当"最小分包"下界用, 与当前 HTTP 通道的分块大小无关。 */
#define OTA_PACKET_PAYLOAD_MAX   512u

/* ==================== Flash 擦除(分步异步,每次擦一页) ====================
 * 复用 storage_flash_erase_page: 内部已做 0x2000 对齐检查 + 中断保护 +
 * Unlock/Lock 配对。ota_port 只需维护"下一页地址"和"已擦页数"两个计数器。
 *
 * 失败跟踪: g_erase_failed 记录擦除过程中是否有任何一页失败。
 * erase_step 即使某页失败也会继续推进(快速跑完), 最终返回 true(完成)后,
 * 调用方必须用 erase_ok() 检查。
 */
static uint32_t g_erase_end_page;    /* 开区间上界: 擦 [start, end) */
static uint32_t g_erase_done_pages;  /* 已擦到哪一页(同时也是下一页的页号) */
static bool     g_erase_failed;

bool ota_port_flash_erase_begin_at(uint32_t first_page, uint32_t pages)
{
    uint32_t max_pages = OTA_FLASH_SIZE / OTA_FLASH_PAGE_SIZE;

    if (pages == 0u)
    {
        return false;
    }
    if (first_page >= max_pages)
    {
        return false;
    }
    /* 上限检查: 不能超出 OTA_FLASH_SIZE 范围 */
    if (pages > (max_pages - first_page))
    {
        return false;
    }
    g_erase_end_page   = first_page + pages;
    g_erase_done_pages = first_page;
    g_erase_failed     = false;
    return true;
}

bool ota_port_flash_erase_begin(uint32_t pages)
{
    /* 老行为: 从第 0 页开始(ota.c 的 MQTT 分包版还在用) */
    return ota_port_flash_erase_begin_at(0u, pages);
}

bool ota_port_flash_erase_step(void)
{
    if (g_erase_done_pages >= g_erase_end_page)
    {
        return true;    /* 全部完成 */
    }

    uint32_t page_addr = OTA_FLASH_ADDR + g_erase_done_pages * OTA_FLASH_PAGE_SIZE;
    if (!storage_flash_erase_page(page_addr))
    {
        g_erase_failed = true;
    }
    g_erase_done_pages++;   /* 无论成功失败都推进, 让状态机尽快读到结果 */

    if (g_erase_failed)
    {
        return true;        /* 有失败, 快速跑完, 让 ota.c 检查后立即 goto_error */
    }
    return false;           /* 仍在进行中 */
}

bool ota_port_flash_erase_ok(void)
{
    return !g_erase_failed;
}

/* ==================== Flash 写 ====================
 *
 * ota 协议保证: offset = seq * 512, 512 = 8*64, 所以 offset 天然 8B 对齐;
 * OTA_FLASH_ADDR 也是 8B 对齐(0x08040000), 所以 addr = OTA_FLASH_ADDR + offset 天然 8B 对齐。
 * 唯一的问题是 len: 中间包 len=512(8 的倍数), 但最后一个包 len 可能任意值。
 *
 * 策略: 尾部补 0xFF 向上凑到 8B 对齐, 然后委托 storage_flash_write(它要求
 * addr 和 len 都是 8B 对齐, 内部已做中断保护 + Unlock/Lock)。
 */
bool ota_port_flash_write(uint32_t offset, const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0u))
    {
        return true;
    }

    uint32_t addr = OTA_FLASH_ADDR + offset;

    /* 安全检查: 写入范围不能超出 OTA 缓存区 */
    if ((offset + len) > OTA_FLASH_SIZE)
    {
        return false;
    }
    /* 理论上 offset 应该 8B 对齐(ota.c 保证), 但这里仍做防御性检查 */
    if ((addr & 0x7U) != 0U)
    {
        return false;
    }

    /* 主体: 直接把 8B 对齐的整段交给 storage_flash_write */
    uint16_t align_len = (len & ~0x7U);     /* 向下取整到 8B 边界 */
    if (align_len >= 8u)
    {
        if (!storage_flash_write(addr, data, align_len))
        {
            return false;
        }
    }

    /* 尾部不足 8 字节: 补 0xFF 凑齐一个 double word 再写 */
    uint16_t tail = len & 0x7U;
    if (tail != 0u)
    {
        uint8_t tail_buf[8];
        memcpy(tail_buf, data + align_len, tail);
        for (uint16_t i = tail; i < 8u; i++)
        {
            tail_buf[i] = 0xFFU;
        }
        if (!storage_flash_write(addr + align_len, tail_buf, 8u))
        {
            return false;
        }
    }

    return true;
}

/* ==================== Flash 读 ====================
 * 委托 storage_flash_read(简单内存映射读取, 无需锁 Flash)
 */
bool ota_port_flash_read(uint32_t offset, uint8_t *buf, uint16_t len)
{
    if ((buf == NULL) || (len == 0u))
    {
        return true;
    }
    /* 安全检查: 读范围不能超出 OTA 缓存区 */
    if ((offset + len) > OTA_FLASH_SIZE)
    {
        return false;
    }
    return storage_flash_read(OTA_FLASH_ADDR + offset, buf, len);
}

/* ==================== CRC32 (纯软件, IEEE 802.3, 多项式 0xEDB88320) ====================
 * 逐位计算, 天然支持任意字节长度, 与 Python hashlib / zip / linux cksum 结果一致。
 * 流式接口: init == 0xFFFFFFFF 表示新会话, 返回值可作为下一次的 init。
 * 最终结果 = ota_port_crc32_update(0xFFFFFFFF, data, len) ^ 0xFFFFFFFF
 * 选择软件而非硬件的原因: N32H482 CRC32DAT 只接受 32-bit word, 固件长度
 * 非 4 字节对齐时尾部补 0 会改变 CRC 语义。
 */
uint32_t ota_port_crc32_update(uint32_t init, const uint8_t *data, uint16_t len)
{
    uint32_t crc = init;
    for (uint16_t i = 0u; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t b = 0u; b < 8u; b++)
        {
            if ((crc & 1u) != 0u)
            {
                crc = (crc >> 1) ^ 0xEDB88320u;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/* ==================== CRC16 (Modbus, 复用 utils.c) ====================
 * 统一走 crc16_modbus (初始 0xFFFF, poly 0xA001, 结果低字节在前)。
 * 用于 OTA 包头校验(协议中 crc16 字段填 0 表示跳过)。
 * ota_port.h 保留 ota_port_crc16 包装以兼容旧调用方。
 */
uint16_t ota_port_crc16(const uint8_t *data, uint16_t len)
{
    return crc16_modbus(data, len);
}

/* ==================== 时间 ==================== */
uint32_t ota_port_get_tick_ms(void)
{
    return get_tick_ms();
}

/* ==================== 升级上下文持久化(断点续传) ====================
 * 持久化位置: OTA_META_ADDR + 40 (标记页的前 32B 留给 mark_valid,
 *              偏移 40 起留给 ctx, 互不冲突)
 *
 * 标记页布局(0x0807C000, 8KB, 全页 0xFF 表示"无升级状态"):
 *   [0..31]   mark_valid   "OTAV"+crc32+size — bootloader 读, 升级完成时写
 *   [32..39]  (gap, 0)
 *   [40..79]  ota_ctx_storage_t — 升级中断后重进时恢复下载会话
 *   [80..]    空闲, 预留给未来扩展
 *
 * 写/读约束: storage_flash_write 要求 addr 和 len 都 8 字节对齐,
 *            _Static_assert 已校验 ota_ctx_storage_t sizeof % 8 == 0,
 *            OTA_CTX_WRITE_LEN 宏再做一层 round-up 保险。
 *
 * 并发安全:
 *   - 同一升级会话中, save_ctx 在 DOWNLOADING 阶段周期性调用;
 *     写之前需确保该页已被 mark_invalid() 擦除(全 0xFF), 否则 Flash 编程失败。
 *   - 新升级开始时, ota.c 会先调 mark_invalid() 擦除整页, 然后 erase_begin()
 *     擦固件缓存区(不碰标记页), 之后 save_ctx 就可以安全写了。
 *   - mark_valid() 在 VERIFYING 成功时调用, 会擦除标记页并写 32B 标记,
 *     顺便清除了 ctx 区 — 正确, 因为升级完成后 ctx 不该再保留。
 */
typedef struct
{
    uint32_t   magic;
    ota_meta_t meta;
    uint16_t   recv_packets;
    uint16_t   bitmap_words;      /* 位图有效字数 */
    uint32_t   pkt_bitmap[15];    /* 位图, 支持最多 480 包, 15*32=480 bit */
} ota_ctx_storage_t;

/* 15 = ceil(240KB / 512B / 32), 必须与 ota.c OTA_BITMAP_WORDS 一致 */
#define OTA_CTX_BITMAP_WORDS  15u

/* ctx 结构体大小必须 8 字节对齐(ARM 编译器会给 ota_meta_t 末尾 padding) */
_Static_assert((sizeof(ota_ctx_storage_t) % 8u) == 0u,
               "ota_ctx_storage_t must be 8B-aligned for Flash programming");

#define OTA_CTX_OFFSET    40u   /* ctx 在标记页内的偏移(8B 对齐) */
#define OTA_CTX_WRITE_LEN ((sizeof(ota_ctx_storage_t) + 7u) & ~7u)  /* 向上凑 8B */

bool ota_port_save_ctx(const ota_meta_t *meta, uint16_t recv_packets,
                       const uint32_t *bitmap, uint16_t bitmap_words)
{
    if (meta == NULL)
    {
        return false;
    }

    /* Flash 只能 1→0, 写前检查 ctx 区是否已写过, 是则先擦除 */
    ota_ctx_storage_t probe;
    bool need_erase = false;
    if (storage_flash_read(OTA_META_ADDR + OTA_CTX_OFFSET,
                           (uint8_t *)&probe, sizeof(probe)))
    {
        if (probe.magic != 0xFFFFFFFFu)
        {
            need_erase = true;
        }
    }
    else
    {
        need_erase = true;
    }

    if (need_erase)
    {
        if (!storage_flash_erase_page(OTA_META_ADDR))
        {
            return false;
        }
    }

    ota_ctx_storage_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.magic        = OTA_CTX_MAGIC;
    ctx.meta         = *meta;
    ctx.recv_packets = recv_packets;
    ctx.bitmap_words = bitmap_words;
    if ((bitmap != NULL) && (bitmap_words > 0u))
    {
        for (uint16_t i = 0u; (i < bitmap_words) && (i < OTA_CTX_BITMAP_WORDS); i++)
        {
            ctx.pkt_bitmap[i] = bitmap[i];
        }
    }

    return storage_flash_write(OTA_META_ADDR + OTA_CTX_OFFSET,
                               (const uint8_t *)&ctx, OTA_CTX_WRITE_LEN);
}   

bool ota_port_load_ctx(ota_meta_t *meta, uint16_t *recv_packets,
                       uint32_t *bitmap, uint16_t *bitmap_words)
{
    if ((meta == NULL) || (recv_packets == NULL))
    {
        return false;
    }
    ota_ctx_storage_t ctx;
    if (!storage_flash_read(OTA_META_ADDR + OTA_CTX_OFFSET,
                            (uint8_t *)&ctx, sizeof(ctx)))
    {
        return false;
    }
    if (ctx.magic != OTA_CTX_MAGIC)
    {
        return false;
    }

    /* 基本字段合法性校验 */
    if ((ctx.meta.total_size == 0u) || (ctx.meta.total_size > OTA_FLASH_SIZE))
    {
        return false;
    }
    if ((ctx.meta.packet_count == 0u) ||
        (ctx.meta.packet_count > (OTA_FLASH_SIZE / OTA_PACKET_PAYLOAD_MAX)))
    {
        return false;
    }
    if ((ctx.recv_packets == 0u) || (ctx.recv_packets > ctx.meta.packet_count))
    {
        return false;
    }

    *meta         = ctx.meta;
    *recv_packets = ctx.recv_packets;

    /* 输出位图(调用方可选传入, NULL 则不写) */
    if ((bitmap != NULL) && (bitmap_words != NULL))
    {
        uint16_t copy_words = (ctx.bitmap_words < OTA_CTX_BITMAP_WORDS)
                              ? ctx.bitmap_words : OTA_CTX_BITMAP_WORDS;
        for (uint16_t i = 0u; i < copy_words; i++)
        {
            bitmap[i] = ctx.pkt_bitmap[i];
        }
        *bitmap_words = copy_words;
    }

    return true;
}

bool ota_port_clear_ctx(void)
{
    uint8_t zeros[8] = {0};
    return storage_flash_write(OTA_META_ADDR + OTA_CTX_OFFSET, zeros, sizeof(zeros));
}

/* ==================== OTA 缓存区有效标记 ====================
 * 位于 OTA_META_ADDR(固件缓存区的下一页, 独立 8KB 页)。
 * 标记区格式(前 32 字节):
 *   [0..3]   magic "OTAV"
 *   [4..7]   crc32 (固件 CRC32, 小端)
 *   [8..11]  size  (固件总字节数, 小端)
 *   [12..31] 0
 *
 * mark_valid: 擦除标记页 -> 写 32 字节标记
 * mark_invalid: 擦除标记页(0xFF 即无效, 不需要先擦再写)
 */
bool ota_port_mark_valid(const ota_meta_t *meta)
{
    if (meta == NULL)
    {
        return false;
    }

    /* 先擦除标记页 */
    if (!storage_flash_erase_page(OTA_META_ADDR))
    {
        return false;
    }

    /* 构造 32 字节标记区 */
    uint8_t buf[32];
    memset(buf, 0, sizeof(buf));
    buf[0] = 'O'; buf[1] = 'T'; buf[2] = 'A'; buf[3] = 'V';

    /* crc32 小端 */
    buf[4]  = (uint8_t)(meta->crc32 & 0xFFu);
    buf[5]  = (uint8_t)((meta->crc32 >> 8) & 0xFFu);
    buf[6]  = (uint8_t)((meta->crc32 >> 16) & 0xFFu);
    buf[7]  = (uint8_t)((meta->crc32 >> 24) & 0xFFu);

    /* size 小端 */
    buf[8]  = (uint8_t)(meta->total_size & 0xFFu);
    buf[9]  = (uint8_t)((meta->total_size >> 8) & 0xFFu);
    buf[10] = (uint8_t)((meta->total_size >> 16) & 0xFFu);
    buf[11] = (uint8_t)((meta->total_size >> 24) & 0xFFu);

    /* 32 字节已对齐, 直接写 */
    return storage_flash_write(OTA_META_ADDR, buf, sizeof(buf));
}

void ota_port_mark_invalid(void)
{
    /* 擦除标记页 = 整页 0xFF, bootloader 读不到 magic 就认为无效。
     * ⚠️ 注意它会连【续传记录】一起擦掉(同一页) —— 这是刻意的语义,
     *    见 ota_port.h 的说明; 调用方要读记录必须先读再调本函数。 */
    (void)storage_flash_erase_page(OTA_META_ADDR);
}

/* ==================== OTA over HTTP 的掉电续传记录 ====================
 * 布局与语义见 ota_port.h。这里只强调三个实现要点:
 *   1) save 必须【先擦页】: Flash 只能 1→0, 改一个已经从 1 变成 0 的位是不可能的。
 *      下载阶段没有有效标记, 所以擦整页是安全的(这一点由 ota_http 的调用时机保证)。
 *   2) clear 只用"把 magic 写 0"实现, 【不擦页】—— 这样它永远不会误伤
 *      bootloader 的 "OTAV" 标记。写 0 是幂等的(0 AND 0 = 0)。
 *   3) written 向下【页对齐】: 擦除粒度是 8 KB, 断点落在页中间时那一页必须重下,
 *      所以干脆把断点对齐到页边界, 免得"续传起点"和"能保留的数据"对不上。
 */
_Static_assert((sizeof(ota_http_resume_t) % 8u) == 0u,
               "ota_http_resume_t must be 8B-aligned for Flash programming");

/* ★ 标记页里三个住户不能重叠: mark[0..31] / MQTT ctx[40..] / HTTP 续传[256..] ★
 * OTA_HTTP_RESUME_OFFSET 当初是按"MQTT ctx 大约 104 字节"估出来的;
 * 与其靠估算, 不如让编译器盯着 —— 将来 ota_ctx_storage_t 长大(比如位图加宽、
 * ota_meta_t 加字段), 这条断言会【编译失败】, 而不是让升级悄悄写坏续传记录。
 * (记录被写坏的表现是"续传永远不生效", 现场几乎查不出来。) */
_Static_assert((OTA_CTX_OFFSET + OTA_CTX_WRITE_LEN) <= OTA_HTTP_RESUME_OFFSET,
               "MQTT ctx and HTTP resume record overlap in the meta page");
_Static_assert((OTA_HTTP_RESUME_OFFSET + sizeof(ota_http_resume_t)) <= FM_META_SIZE,
               "HTTP resume record does not fit in the meta page");
_Static_assert(OTA_HTTP_RESUME_OFFSET >= 32u + 8u,
               "HTTP resume record must not overlap the OTAV mark");

bool ota_port_http_resume_save(uint32_t written, uint32_t size,
                               const char *md5hex, const char *version)
{
    ota_http_resume_t rec;

    if ((md5hex == NULL) || (size == 0u))
    {
        return false;
    }

    memset(&rec, 0, sizeof(rec));
    rec.magic   = OTA_HTTP_RESUME_MAGIC;
    /* ★ 页对齐: 见上面要点 3 */
    rec.written = written & ~(OTA_FLASH_PAGE_SIZE - 1u);
    rec.size    = size;
    (void)strncpy(rec.md5hex, md5hex, sizeof(rec.md5hex) - 1u);
    if (version != NULL)
    {
        (void)strncpy(rec.version, version, sizeof(rec.version) - 1u);
    }

    /* 先擦整页(下载阶段没有有效标记, 安全) */
    if (!storage_flash_erase_page(OTA_META_ADDR))
    {
        return false;
    }
    return storage_flash_write(OTA_META_ADDR + OTA_HTTP_RESUME_OFFSET,
                               (const uint8_t *)&rec, (uint16_t)sizeof(rec));
}

bool ota_port_http_resume_load(uint32_t *written, uint32_t *size,
                               char *md5hex, uint16_t md5cap,
                               char *version, uint16_t vercap)
{
    ota_http_resume_t rec;

    if (!storage_flash_read(OTA_META_ADDR + OTA_HTTP_RESUME_OFFSET,
                            (uint8_t *)&rec, (uint16_t)sizeof(rec)))
    {
        return false;
    }
    if (rec.magic != OTA_HTTP_RESUME_MAGIC)
    {
        return false;       /* 无记录(全 0xFF)或已被 clear 清掉 */
    }
    rec.md5hex[sizeof(rec.md5hex) - 1u] = '\0';     /* 防被写坏后不封口 */
    rec.version[sizeof(rec.version) - 1u] = '\0';
    if (strlen(rec.md5hex) != 32u)
    {
        return false;       /* 记录本身不完整, 当作没有 */
    }

    /* 【记录自校验】只看 magic 是不够的: 如果只写进去前 8 个字节(magic),
     * 其余还是 0xFF, 那 33 个 0xFF 里有 32 个非零字节 -> strlen 恰好也是 32,
     * 光靠长度判不出来。这里再用 size/written 的量纲兜一道:
     *   · size 必须是 (0, 暂存区容量] 内的合理值(0xFFFFFFFF 直接被挡)
     *   · written 不能超过 size
     * (2026-10-02 主机测试 test_ota_port [6] 抓到的。) */
    if ((rec.size == 0u) || (rec.size > OTA_FLASH_SIZE) || (rec.written > rec.size))
    {
        return false;
    }

    if (written != NULL) { *written = rec.written; }
    if (size    != NULL) { *size    = rec.size;    }
    if ((md5hex != NULL) && (md5cap > 0u))
    {
        (void)strncpy(md5hex, rec.md5hex, (size_t)(md5cap - 1u));
        md5hex[md5cap - 1u] = '\0';
    }
    if ((version != NULL) && (vercap > 0u))
    {
        (void)strncpy(version, rec.version, (size_t)(vercap - 1u));
        version[vercap - 1u] = '\0';
    }
    return true;
}

void ota_port_http_resume_clear(void)
{
    /* 只把 magic 写 0 —— 不擦页, 所以不会动 bootloader 的标记。
     * 8 字节对齐写: storage_flash_write 的要求。
     *
     * ★★2026-10-14 已知缺陷: 本函数在 N32H48x 上【无效】★★
     *   它改写的位置 OTA_META_ADDR + OTA_HTTP_RESUME_OFFSET 通常【已经被
     *   续传记录写过了】, 不是擦除态; 而 N32 的 Flash 编程只对【处于擦除态的
     *   double word】成功(见 storage.c 的 FLASH_ProgramdoubleWord)。
     *   ⇒ 这个写会失败, magic 清不掉, 记录依旧可被 load() 读到。
     *   (以前主机单测能过, 只因为假实现是 `&=`, 比真机宽松 ——
     *    2026-10-14 收紧 fake_storage.c 后这个缺陷立刻暴露。)
     *
     * 【为什么不修】真正让续传记录失效的是 mark_valid()/mark_invalid(),
     *   它们【整页擦除】, 记录随之消失(见 ota_port.h 里那段说明)。
     *   改本函数会动到已验证过的 OTA 下载路径, 回归风险 > 收益。
     *   ⇒ 用户 2026-10-14 决定: 如实标注, 不改实现。
     *   ⇒ 也就是说: 别指望这个函数能"清掉"记录; 需要失效请走 mark_*()。 */
    uint8_t zero8[8] = { 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u };

    (void)storage_flash_write(OTA_META_ADDR + OTA_HTTP_RESUME_OFFSET,
                              zero8, (uint16_t)sizeof(zero8));
}

/* ==================== A/B 回滚状态(2026-10-11 新增) ====================
 * 布局与语义见 ota_port.h。三个实现要点:
 *   1) save 【只写、不擦页】—— 同页还有 OTA 固件标记和续传记录, 擦页会一起毁掉。
 *      ⚠️ 正因为如此, tries_left 必须是【倒数的位图】(每启动只多清一个 bit),
 *      不能用普通计数器 —— 递增需要 0->1, 而 Flash 只支持 1->0。详见头文件。
 *   2) load 必须【连 chk 一起校验】—— 写到一半掉电会留下"magic 对、后面是垃圾"
 *      的记录。只判 magic 的话, 那种半截记录会被当成有效状态。
 *      校验不过一律返回 false, 调用方按"从未记录"处理(重新计数, 【不回滚》)——
 *      宁可多给几次机会, 也不能把好固件回滚掉。
 *   3) clear 只清 magic —— 与 resume_clear 同模式, 幂等且不伤别人。
 *
 * 【为什么校验用自写的 8 位累加而不是 CRC32】
 *   只需要抓"写到一半掉电"这种粗粒度损坏, 不需要强度; 而且本文件里没有任何
 *   CRC 实现(CRC 在 ota.c 那边), 自己写一段能避免引入跨模块依赖。
 *   真正的强度来自紧随其后的【值域校验】(boot_slot 只能 0/1、confirmed 只能 0/1)。
 */
_Static_assert((sizeof(ota_rollback_t) % 8u) == 0u,
               "ota_rollback_t must be 8B-aligned for Flash programming");
_Static_assert((OTA_ROLLBACK_OFFSET % 8u) == 0u,
               "rollback offset must be 8B-aligned");
/* ★ 标记页里【四个】住户不能重叠: OTAV 标记 / MQTT ctx / HTTP 续传 / 本记录 ★ */
_Static_assert((OTA_HTTP_RESUME_OFFSET + sizeof(ota_http_resume_t)) <= OTA_ROLLBACK_OFFSET,
               "rollback record overlaps the HTTP resume record in the meta page");
_Static_assert((OTA_ROLLBACK_OFFSET + sizeof(ota_rollback_t)) <= FM_META_SIZE,
               "rollback record does not fit in the meta page");

/* ★★2026-10-14 重新设计: 由"原地改写"改成【追加式(append-only)】★★
 *
 * 【为什么必须改 —— 上板实测打脸了】
 *   2026-10-11 第一版按"只写不擦页 + 每次只把 1 变成 0"设计, 主机单测 36/36 全绿。
 *   一上板就刷了几千行:
 *       [OTAHL] !! 定案写 Flash 失败, 下次再试
 *   根因在 storage.c 的 storage_flash_write():
 *       sts = FLASH_ProgramdoubleWord(addr, w0, w1);
 *       if (sts != FLASH_EOP) break;      -> 返回 false
 *   ⇒ N32H48x 的 Flash 编程【只能写处于擦除态(全 0xFF)的 double word】。
 *     向一个【已经编程过】的位置再写一次 —— 哪怕内容完全相同、哪怕只是 1->0 ——
 *     硬件直接返回错误。
 *   ⇒ "1->0 是必要条件"这个分析方向对, 但【不充分】:
 *     真正的条件是"整个 double word 必须是擦除态"。
 *     结论: "同一位置重复写"在本芯片上物理上不可能。
 *
 * 【代价(诚实记录)】主机单测为什么没抓到:
 *   _host_test/fake_storage.c 当时实现成 `new = old & data` —— 允许重复写,
 *   比真实现【宽松】。⇒ 假实现比真实现松, 就会"单测全绿、上板全废"。
 *   ★ 已同步收紧 fake_storage.c: 被编程过的 double word 不允许再写。
 *     这条教训比这次改动本身更值钱。
 *
 * 【现在的做法: 追加式】
 *   把 512 之后的空闲区切成 240 个 32 字节槽:
 *       save()  写【第一个整槽全 0xFF 的空槽】(每槽只写一次 -> 永远满足"擦除态")
 *       load()  扫全部槽, 取【最后一个有效】的(= 最新状态)
 *   ⇒ 完全绕开"重复写"问题, 不需要在页内做任何擦除。
 *
 * 【槽满怎么办 —— 用户 2026-10-14 确认: 停止写入, 不回滚】
 *   240 槽 / (每轮固件约 9 条: 8 次启动 + 1 次定案) ≈ 20+ 轮 OTA 才用完。
 *   用完时【停止写入】而不是"擦页重置": 擦页会毁掉同页的 OTA 固件标记和
 *   HTTP 续传记录。退化成"从不回滚"是 fail-safe 方向 ——
 *   "好固件不会被误回滚"比"回滚一定能触发"重要。
 *
 * ⚠️ 上面那段"1->0 约束"的旧注释仍然有效(它解释了为什么字段要那么设计),
 *    但【不再是充分条件】—— 现在真正的门槛由"只写空槽"来保证。 */
#define OTA_ROLLBACK_SLOT_SZ   32u
#define OTA_ROLLBACK_SLOTS     (((uint32_t)FM_META_SIZE - OTA_ROLLBACK_OFFSET) / OTA_ROLLBACK_SLOT_SZ)

/** 该槽是否处于擦除态(整槽全 0xFF) —— 只有这种槽才能编程 */
static bool ota_rb_slot_erased(const ota_rollback_t *r)
{
    const uint8_t *p = (const uint8_t *)r;
    uint16_t       i;

    for (i = 0u; i < sizeof(*r); i++)
    {
        if (p[i] != 0xFFu) { return false; }
    }
    return true;
}

bool ota_port_rollback_load(ota_rollback_t *out)
{
    ota_rollback_t rec;
    ota_rollback_t best;
    bool           found = false;
    uint32_t       i;

    if (out == NULL) { return false; }

    for (i = 0u; i < OTA_ROLLBACK_SLOTS; i++)
    {
        if (!storage_flash_read(OTA_META_ADDR + OTA_ROLLBACK_OFFSET + i * OTA_ROLLBACK_SLOT_SZ,
                                (uint8_t *)&rec, (uint16_t)sizeof(rec)))
        {
            break;
        }
        if (rec.magic != OTA_ROLLBACK_MAGIC) { continue; }   /* 空槽/别的数据 -> 跳过后面的 */
        /* 半截记录 -> 跳过。值域校验代替校验和(见 ota_port.h 的说明):
         * "写到一半掉电"留下的典型特征就是那些字节还是 0xFF,
         * 而 boot_slot/unconfirmed 会是 0xFF ⇒ 这两条直接挡掉。
         * ⇒ 宁可当"从未记录", 也绝不把半截记录当有效状态去回滚。 */
        if ((rec.boot_slot > 1u) || (rec.unconfirmed > 1u)) { continue; }

        best  = rec;                /* 取最后一个有效的 = 最新状态 */
        found = true;
    }

    if (found) { *out = best; }
    return found;
}

bool ota_port_rollback_save(const ota_rollback_t *rec)
{
    ota_rollback_t cur;
    ota_rollback_t w;
    uint32_t       i;

    if (rec == NULL) { return false; }

    w = *rec;
    w.magic = OTA_ROLLBACK_MAGIC;
    w.rsv0  = 0u;
    (void)memset(w.rsv, 0, sizeof(w.rsv));

    for (i = 0u; i < OTA_ROLLBACK_SLOTS; i++)
    {
        uint32_t addr = OTA_META_ADDR + OTA_ROLLBACK_OFFSET + i * OTA_ROLLBACK_SLOT_SZ;

        if (!storage_flash_read(addr, (uint8_t *)&cur, (uint16_t)sizeof(cur)))
        {
            return false;
        }
        /* ★ 只往【整槽全 0xFF】的空槽写 —— 这是本芯片能编程的唯一条件。
         *   不能只判 magic: 半截写入的槽 magic 可能不是 ABRT,
         *   但里面已经有非 0xFF 字节, 往那里写照样会失败。 */
        if (!ota_rb_slot_erased(&cur)) { continue; }

        return storage_flash_write(addr, (const uint8_t *)&w, (uint16_t)sizeof(w));
    }

    /* 槽满: 停止写入, 返回 false(调用方 ota_health 会打日志)。
     * 【为什么不擦页重置】见本段开头: 擦页会毁掉同页的 OTA 标记和续传记录。
     * 退化成"从不回滚"是 fail-safe —— 好固件不会被误回滚。 */
    return false;
}

void ota_port_rollback_clear(void)
{
    /* 追加式下"清除"没有干净的做法: 前面写过的槽都留着, load 取的是最新那条。
     * ⇒ 这里【什么也不做】。保留函数只为接口兼容。
     * ⚠️ 旧实现写 8 个 0 把 magic 清掉 —— 那是"原地改写"时代的做法:
     *   在追加式语义下它只作废【第一个槽】(而 load 取最后一条, 毫无作用),
     *   更糟的是它会去写一个已经编程过的槽 ⇒ 必然失败(白走一次 Flash)。
     * ⇒ 干脆不写。要真正"忘记"记录, 唯一干净的办法是擦整页(会毁 OTA 标记), 不做。 */
}


/* ==================== 版本 / 重boot ==================== */
/* 当前固件版本: 由 app_boot 在上电时注入 MAIN_FW_VERSION。
 * ★以前这里返回写死的 OTA_APP_VERSION "1.0.0"★ —— 于是 ota.start 的
 *   "与当前版本相同 -> 1006" 永远不成立: 升到 1.0.3 之后再发 1.0.3 照样受理,
 *   平台也永远看不到真实版本。默认值只作兜底(未注入时)。 */
static char s_app_version[24] = "0.0.0";

void ota_port_set_version(const char *ver)
{
    if ((ver == NULL) || (ver[0] == '\0')) { return; }
    (void)snprintf(s_app_version, sizeof(s_app_version), "%s", ver);
}

const char *ota_port_get_version(void)
{
    return s_app_version;
}

void ota_port_reboot(void)
{
    NVIC_SystemReset();
}
