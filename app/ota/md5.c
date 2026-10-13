/**
 * \file  md5.c
 * \brief 增量 MD5 (RFC 1321) —— 见 md5.h 说明为什么是流式
 *
 * 实现路线: RFC 1321 参考实现的【表驱动】版(不是展开 64 轮)。
 *   为什么选表驱动: OTA 一共就 128 KB / 64 B = 2048 块, 展开版省下的那点
 *   周期在这里毫无意义, 而展开版要写 64 行宏, ROM 也更大。表驱动 4 行循环
 *   搞定, 代价是两张 64 项的常量表(256 + 64 字节)。
 *
 * 已知测试向量(主机侧单测用的就是这些):
 *   ""                    -> d41d8cd98f00b204e9800998ecf8427e
 *   "a"                   -> 0cc175b9c0f1b6a831c399e269772661
 *   "abc"                 -> 900150983cd24fb0d6963f7d28e17f72
 *   "message digest"      -> f96b697d7cb7938d525a2f31aaf161d0
 *   "a"*1000000           -> 7707d6ae4e027c70eea2a935c2296f21
 *   "12345678901234567890123456789012345678901234567890123456789012345678901234567890"
 *                         -> 57edf4a22be3c955ac49da2e2107b67a
 */

#include "md5.h"
#include <string.h>

/* ==================== 常量表 ==================== */

/* K[i] = floor(abs(sin(i + 1)) * 2^32), 就是 RFC 1321 里那 64 个魔术数 */
static const uint32_t MD5_K[64] = {
    0xd76aa478u, 0xe8c7b756u, 0x242070dbu, 0xc1bdceeeu,
    0xf57c0fafu, 0x4787c62au, 0xa8304613u, 0xfd469501u,
    0x698098d8u, 0x8b44f7afu, 0xffff5bb1u, 0x895cd7beu,
    0x6b901122u, 0xfd987193u, 0xa679438eu, 0x49b40821u,
    0xf61e2562u, 0xc040b340u, 0x265e5a51u, 0xe9b6c7aau,
    0xd62f105du, 0x02441453u, 0xd8a1e681u, 0xe7d3fbc8u,
    0x21e1cde6u, 0xc33707d6u, 0xf4d50d87u, 0x455a14edu,
    0xa9e3e905u, 0xfcefa3f8u, 0x676f02d9u, 0x8d2a4c8au,
    0xfffa3942u, 0x8771f681u, 0x6d9d6122u, 0xfde5380cu,
    0xa4beea44u, 0x4bdecfa9u, 0xf6bb4b60u, 0xbebfbc70u,
    0x289b7ec6u, 0xeaa127fau, 0xd4ef3085u, 0x04881d05u,
    0xd9d4d039u, 0xe6db99e5u, 0x1fa27cf8u, 0xc4ac5665u,
    0xf4292244u, 0x432aff97u, 0xab9423a7u, 0xfc93a039u,
    0x655b59c3u, 0x8f0ccc92u, 0xffeff47du, 0x85845dd1u,
    0x6fa87e4fu, 0xfe2ce6e0u, 0xa3014314u, 0x4e0811a1u,
    0xf7537e82u, 0xbd3af235u, 0x2ad7d2bbu, 0xeb86d391u
};

/* 每轮的循环左移位数 */
static const uint8_t MD5_S[64] = {
    7u, 12u, 17u, 22u, 7u, 12u, 17u, 22u, 7u, 12u, 17u, 22u, 7u, 12u, 17u, 22u,
    5u,  9u, 14u, 20u, 5u,  9u, 14u, 20u, 5u,  9u, 14u, 20u, 5u,  9u, 14u, 20u,
    4u, 11u, 16u, 23u, 4u, 11u, 16u, 23u, 4u, 11u, 16u, 23u, 4u, 11u, 16u, 23u,
    6u, 10u, 15u, 21u, 6u, 10u, 15u, 21u, 6u, 10u, 15u, 21u, 6u, 10u, 15u, 21u
};

/* ==================== 基础运算 ==================== */

#define MD5_ROTL(x, n)  (((x) << (n)) | ((x) >> (32u - (n))))

/** @brief 处理一个 64 字节块(就地更新 ctx->state) */
static void md5_process_block(md5_ctx_t *ctx, const uint8_t block[64])
{
    uint32_t m[16];
    uint32_t a = ctx->state[0];
    uint32_t b = ctx->state[1];
    uint32_t c = ctx->state[2];
    uint32_t d = ctx->state[3];
    uint32_t i;
    uint32_t f;
    uint32_t g;
    uint32_t tmp;

    /* 小端读入 16 个 32 位字 —— 用手工拼字节, 不假设主机字节序/对齐 */
    for (i = 0u; i < 16u; i++)
    {
        m[i] = ((uint32_t)block[i * 4u]) |
               ((uint32_t)block[i * 4u + 1u] << 8) |
               ((uint32_t)block[i * 4u + 2u] << 16) |
               ((uint32_t)block[i * 4u + 3u] << 24);
    }

    for (i = 0u; i < 64u; i++)
    {
        if (i < 16u)
        {
            f = (b & c) | (~b & d);
            g = i;
        }
        else if (i < 32u)
        {
            f = (d & b) | (~d & c);
            g = (5u * i + 1u) & 15u;
        }
        else if (i < 48u)
        {
            f = b ^ c ^ d;
            g = (3u * i + 5u) & 15u;
        }
        else
        {
            f = c ^ (b | ~d);
            g = (7u * i) & 15u;
        }

        tmp = d;
        d   = c;
        c   = b;
        b   = b + MD5_ROTL(a + f + MD5_K[i] + m[g], MD5_S[i]);
        a   = tmp;
    }

    ctx->state[0] += a;
    ctx->state[1] += b;
    ctx->state[2] += c;
    ctx->state[3] += d;
}

/* ==================== 对外接口 ==================== */

void md5_init(md5_ctx_t *ctx)
{
    if (ctx == NULL)
    {
        return;
    }
    ctx->state[0]    = 0x67452301u;
    ctx->state[1]    = 0xefcdab89u;
    ctx->state[2]    = 0x98badcfeu;
    ctx->state[3]    = 0x10325476u;
    ctx->byte_count  = 0u;
    ctx->used        = 0u;
}

void md5_update(md5_ctx_t *ctx, const uint8_t *data, uint32_t len)
{
    uint32_t take;

    if ((ctx == NULL) || (data == NULL) || (len == 0u))
    {
        return;
    }

    ctx->byte_count += len;

    /* 先把上次剩下的半块补齐 */
    if (ctx->used > 0u)
    {
        take = 64u - ctx->used;
        if (take > len)
        {
            take = len;
        }
        memcpy(&ctx->buffer[ctx->used], data, take);
        ctx->used += take;
        data      += take;
        len       -= take;

        if (ctx->used == 64u)
        {
            md5_process_block(ctx, ctx->buffer);
            ctx->used = 0u;
        }
    }

    /* 整块直通, 不过 buffer */
    while (len >= 64u)
    {
        md5_process_block(ctx, data);
        data += 64u;
        len  -= 64u;
    }

    /* 尾巴留到下次 */
    if (len > 0u)
    {
        memcpy(ctx->buffer, data, len);
        ctx->used = len;
    }
}

void md5_final(md5_ctx_t *ctx, uint8_t out[16])
{
    uint8_t  pad[72];
    uint32_t pad_len;
    uint32_t bit_lo;
    uint32_t bit_hi;
    uint32_t i;

    if ((ctx == NULL) || (out == NULL))
    {
        return;
    }

    /* 补齐: 先补 0x80, 再补 0x00 到 "56 mod 64", 最后 8 字节小端位长度。
     * 最多需要 1 + 63 + 8 = 72 字节, 所以 pad 开 72。 */
    pad[0]  = 0x80u;
    pad_len = ((ctx->used < 56u) ? (56u - ctx->used) : (120u - ctx->used));

    memset(&pad[1], 0, 71u);

    /* 位数 = 字节数 << 3, 手工拆高低位, 不用 64 位整型 */
    bit_lo = (uint32_t)(ctx->byte_count << 3);
    bit_hi = (uint32_t)(ctx->byte_count >> 29);

    /* 长度字段接在 0 填充之后: 先喂 pad, 再把 8 字节长度当成一段数据喂进去。
     * 用 md5_update 而不是直接改状态, 是为了让 "0x80 + 0x00.. + 长度" 恰好
     * 组成完整块 —— 这也保证 byte_count 的累加不影响 bit_lo/bit_hi(已算完)。 */
    md5_update(ctx, pad, pad_len);

    {
        uint8_t lenbuf[8];
        lenbuf[0] = (uint8_t)(bit_lo & 0xFFu);
        lenbuf[1] = (uint8_t)((bit_lo >> 8) & 0xFFu);
        lenbuf[2] = (uint8_t)((bit_lo >> 16) & 0xFFu);
        lenbuf[3] = (uint8_t)((bit_lo >> 24) & 0xFFu);
        lenbuf[4] = (uint8_t)(bit_hi & 0xFFu);
        lenbuf[5] = (uint8_t)((bit_hi >> 8) & 0xFFu);
        lenbuf[6] = (uint8_t)((bit_hi >> 16) & 0xFFu);
        lenbuf[7] = (uint8_t)((bit_hi >> 24) & 0xFFu);
        md5_update(ctx, lenbuf, 8u);
    }

    /* 输出: 小端, A/B/C/D 依次 */
    for (i = 0u; i < 4u; i++)
    {
        out[i * 4u]      = (uint8_t)(ctx->state[i] & 0xFFu);
        out[i * 4u + 1u] = (uint8_t)((ctx->state[i] >> 8) & 0xFFu);
        out[i * 4u + 2u] = (uint8_t)((ctx->state[i] >> 16) & 0xFFu);
        out[i * 4u + 3u] = (uint8_t)((ctx->state[i] >> 24) & 0xFFu);
    }

    /* 用完擦掉中间状态(不是安全要求, 只是避免"看起来还能继续用") */
    memset(ctx, 0, sizeof(*ctx));
}

void md5_to_hex(const uint8_t digest[16], char out[33])
{
    static const char HEXDIGIT[] = "0123456789abcdef";
    uint32_t i;

    if ((digest == NULL) || (out == NULL))
    {
        return;
    }
    for (i = 0u; i < 16u; i++)
    {
        out[i * 2u]      = HEXDIGIT[(digest[i] >> 4) & 0x0Fu];
        out[i * 2u + 1u] = HEXDIGIT[digest[i] & 0x0Fu];
    }
    out[32] = '\0';
}

void md5_calc(const uint8_t *data, uint32_t len, uint8_t out[16])
{
    md5_ctx_t ctx;

    md5_init(&ctx);
    md5_update(&ctx, data, len);
    md5_final(&ctx, out);
}

/** @brief 单个 hex 字符 -> 数值; 非法返回 -1 */
static int md5_hex_val(char c)
{
    if ((c >= '0') && (c <= '9')) { return (int)(c - '0'); }
    if ((c >= 'a') && (c <= 'f')) { return (int)(c - 'a') + 10; }
    if ((c >= 'A') && (c <= 'F')) { return (int)(c - 'A') + 10; }
    return -1;
}

bool md5_equal_hex(const uint8_t digest[16], const char *hex)
{
    uint32_t i;

    if ((digest == NULL) || (hex == NULL))
    {
        return false;
    }

    for (i = 0u; i < 16u; i++)
    {
        int hi = md5_hex_val(hex[i * 2u]);
        int lo = md5_hex_val(hex[i * 2u + 1u]);

        if ((hi < 0) || (lo < 0))
        {
            return false;
        }
        if ((uint8_t)((hi << 4) | lo) != digest[i])
        {
            return false;
        }
    }
    return true;
}
