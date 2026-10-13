/**
 * @file  cjson_port.c
 * @brief cJSON 落地层实现：架在静态数组上的池式分配器 + hooks 安装(见 cjson_port.h)
 *
 * 分配器形态：地址连续的块链 + 块头 {size, used}
 *   · 块头固定 8 字节，池基址 8 字节对齐 -> 数据区天然 8 字节对齐(cJSON 的
 *     double valuedouble 需要)；
 *   · 下一个块的位置 = 本块地址 + 8 + size，所以【不需要额外的链表指针】；
 *   · 释放时做一次全池相邻空闲块合并(pool_coalesce)，把碎片压掉。
 *
 * 复杂度：分配是 O(块数) 首次适配；释放后合并也是 O(块数)。
 *   池只有几 KB、块数是几十~几百量级，240MHz 下一次几百微秒，够用；
 *   刻意不引入更复杂的数据结构(本工程栈只有 4KB，简单可靠优先)。
 */
#include "cjson_port.h"
#include "log.h"

#define POOL_ALIGN      8u      /* 对齐粒度 = 块头大小，保证数据区 8 字节对齐 */
#define HDR_SIZE        8u      /* 块头字节数 {uint32_t size; uint32_t used;} */

typedef struct
{
    uint32_t size;              /* 本块可用数据字节数(不含块头) */
    uint32_t used;              /* 1 = 已分配, 0 = 空闲 */
} pool_hdr_t;

static uint8_t  s_pool[CJSON_PORT_POOL_SIZE] __attribute__((aligned(8)));

static uint32_t s_used;         /* 当前已分配字节数 */
static uint32_t s_peak;         /* 峰值(只增不减) */
static uint32_t s_fail;         /* 分配失败累计(只增不减) */
static uint32_t s_ok;           /* 成功分配累计 */
static bool     s_ready;        /* hooks 是否已装(见 cjson_port_is_ready 的说明) */

static uint32_t pool_align_up(uint32_t n)
{
    return (n + (POOL_ALIGN - 1u)) & ~(POOL_ALIGN - 1u);
}

/* ------------------------------------------------------------------ */
/* 池操作                                                              */
/* ------------------------------------------------------------------ */

void cjson_port_reset(void)
{
    pool_hdr_t *h = (pool_hdr_t *)(void *)s_pool;

    h->size = ((uint32_t)CJSON_PORT_POOL_SIZE) - HDR_SIZE;
    h->used = 0u;

    s_used = 0u;
    s_peak = 0u;
    s_fail = 0u;
    s_ok   = 0u;
}

/* 全池扫一遍，把相邻的两个空闲块合并成一个。(free 之后调用) */
static void pool_coalesce(void)
{
    uint32_t off = 0u;

    for (;;)
    {
        pool_hdr_t *h    = (pool_hdr_t *)(void *)(s_pool + off);
        uint32_t    next = off + HDR_SIZE + h->size;

        if ((next + HDR_SIZE) > ((uint32_t)CJSON_PORT_POOL_SIZE))
        {
            break;                          /* h 是最后一个块 */
        }

        {
            pool_hdr_t *n = (pool_hdr_t *)(void *)(s_pool + next);

            if ((h->used == 0u) && (n->used == 0u))
            {
                h->size += (HDR_SIZE + n->size);
                continue;                   /* 吃掉 n；off 不动，继续跟后面的比 */
            }
        }
        off = next;
    }
}

static void *pool_alloc(size_t want)
{
    uint32_t need;
    uint32_t off;

    if (want == 0u)
    {
        want = 1u;                          /* 0 字节也要返回一个唯一指针 */
    }
    if (want > (size_t)((uint32_t)CJSON_PORT_POOL_SIZE))
    {
        s_fail++;
        return NULL;                        /* 溢出保护：不让 align_up 回绕 */
    }
    need = pool_align_up((uint32_t)want);

    for (off = 0u; (off + HDR_SIZE) <= ((uint32_t)CJSON_PORT_POOL_SIZE); )
    {
        pool_hdr_t *h    = (pool_hdr_t *)(void *)(s_pool + off);
        uint32_t    next = off + HDR_SIZE + h->size;

        if ((h->used == 0u) && (h->size >= need))
        {
            /* 够大就切一块；剩下的还装得下一个最小块才分裂 */
            if (h->size >= (need + HDR_SIZE + POOL_ALIGN))
            {
                pool_hdr_t *rest = (pool_hdr_t *)(void *)(s_pool + off + HDR_SIZE + need);

                rest->size = h->size - need - HDR_SIZE;
                rest->used = 0u;
                h->size    = need;
            }
            h->used = 1u;
            s_used += h->size;
            if (s_used > s_peak)
            {
                s_peak = s_used;
            }
            s_ok++;
            return (void *)(s_pool + off + HDR_SIZE);
        }

        if ((next + HDR_SIZE) > ((uint32_t)CJSON_PORT_POOL_SIZE))
        {
            break;                          /* 已经是最后一个块 */
        }
        off = next;
    }

    s_fail++;
    return NULL;
}

static void pool_free(void *ptr)
{
    uint32_t    off;
    pool_hdr_t *h;

    if (ptr == NULL)
    {
        return;
    }

    off = (uint32_t)((const uint8_t *)ptr - s_pool);
    if ((off < HDR_SIZE) || (off >= ((uint32_t)CJSON_PORT_POOL_SIZE)))
    {
        return;                             /* 不是本池的指针：忽略，不崩 */
    }

    h = (pool_hdr_t *)(void *)(s_pool + off - HDR_SIZE);
    if (h->used == 0u)
    {
        return;                             /* 重复释放：忽略 */
    }

    h->used = 0u;
    s_used -= h->size;

    pool_coalesce();
}

/* 【这里为什么没有 realloc】这版 cJSON 的公开 cJSON_Hooks 只有 malloc_fn / free_fn
 * 两个成员，而 cJSON_InitHooks() 的行为是：一旦发现我们给了自定义 malloc/free，就把
 * 内部的 global_hooks.reallocate 置成 NULL（见 cJSON.c 的 cJSON_InitHooks 末尾）。
 * 之后 cJSON_Print 的缓冲扩容会走"手动扩容"= 新分配 + memcpy + 释放旧块
 * （见 cJSON.c 的 printbuffer ensure / cJSON_PrintBuffered），
 * 【绝不会让 libc 的 realloc 去动我们池里的指针】—— 这正是我们要的：本工程不许有
 * 任何堆分配。所以这里刻意不提供 realloc；也不要往 hooks 里塞第三个成员，
 * 这个版本的 cJSON_Hooks 结构体里没有它（塞了编译不过，已实际验证）。 */

/* ------------------------------------------------------------------ */
/* 对外                                                                */
/* ------------------------------------------------------------------ */

void cjson_port_init(void)
{
    cJSON_Hooks hooks;

    cjson_port_reset();

    hooks.malloc_fn = pool_alloc;
    hooks.free_fn   = pool_free;
    /* 只装这两个成员：本版 cJSON_Hooks 结构体里没有 realloc 成员，而且 cJSON 在我们
     * 给了自定义 malloc/free 之后会把 reallocate 置 NULL、改用手动扩容
     * （见本文件"这里为什么没有 realloc"那一节）。 */
    cJSON_InitHooks(&hooks);
    s_ready = true;                 /* 见 cjson_port_is_ready(): 这是给"忘了 init"兜底的护栏 */

    log_info("[CJSON] 池 %u B, 分配器 hooks 已装(全程无堆分配)\r\n",
             (unsigned)CJSON_PORT_POOL_SIZE);
}

bool cjson_port_is_ready(void)        { return s_ready; }
uint32_t cjson_port_pool_size(void)   { return (uint32_t)CJSON_PORT_POOL_SIZE; }
uint32_t cjson_port_used(void)        { return s_used; }
uint32_t cjson_port_peak(void)        { return s_peak; }
uint32_t cjson_port_fail_count(void)  { return s_fail; }
uint32_t cjson_port_alloc_count(void) { return s_ok; }

bool cjson_port_check(void)
{
    uint32_t off      = 0u;
    uint32_t used_sum = 0u;

    for (;;)
    {
        pool_hdr_t *h;
        uint32_t    next;

        if ((off + HDR_SIZE) > ((uint32_t)CJSON_PORT_POOL_SIZE))
        {
            return false;                   /* 越过了池尾 */
        }
        if (((off + HDR_SIZE) % POOL_ALIGN) != 0u)
        {
            return false;                   /* 数据区没按 8 字节对齐 */
        }

        h    = (pool_hdr_t *)(void *)(s_pool + off);
        next = off + HDR_SIZE + h->size;

        if (h->size == 0u)
        {
            return false;                   /* 空块无意义 */
        }
        if (next > ((uint32_t)CJSON_PORT_POOL_SIZE))
        {
            return false;                   /* 块尾超出池 */
        }

        if (h->used != 0u)
        {
            used_sum += h->size;
        }
        else if ((next + HDR_SIZE) <= ((uint32_t)CJSON_PORT_POOL_SIZE))
        {
            pool_hdr_t *n = (pool_hdr_t *)(void *)(s_pool + next);

            if (n->used == 0u)
            {
                return false;               /* 相邻两个空闲块：合并没做干净 */
            }
        }

        if (next == ((uint32_t)CJSON_PORT_POOL_SIZE))
        {
            break;                          /* 块链正好铺满整个池 */
        }
        off = next;
    }

    return (used_sum == s_used);            /* 记账与实际占用必须一致 */
}

void cjson_port_log_stats(const char *tag)
{
    log_info("[CJSON] %s 池=%u 当前占用=%u 峰值=%u 分配=%lu 失败=%lu\r\n",
             (tag != NULL) ? tag : "-",
             (unsigned)CJSON_PORT_POOL_SIZE,
             (unsigned)s_used,
             (unsigned)s_peak,
             (unsigned long)s_ok,
             (unsigned long)s_fail);
}
