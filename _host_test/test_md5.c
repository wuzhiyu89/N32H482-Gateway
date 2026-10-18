/* ============================================================================
 * test_md5.c —— app/ota/md5.c 的主机测试
 * ----------------------------------------------------------------------------
 * 为什么必须测死: MD5 是 OTA 的【唯一完整性防线】(协议 5.6 ota.start 的
 * <md5> 字段)。它算错的表现是"固件完整却判 2003 校验失败", 或者更糟 ——
 * "固件损坏却放行烧进运行区"。而且它跑在设备上, 现场没法单步。
 *
 * 覆盖:
 *   [1] RFC 1321 全部官方测试向量(含 100 万个 'a')
 *   [2] ★流式 == 一次性★: 同一段数据按【每一种切法】喂进去, 结果必须一致
 *       —— 这是 OTA 的生命线: 分片边界由网络决定, 不可能是整齐的 64 字节。
 *       特别是 55/56/57/63/64/65 这几个点(正好卡在 MD5 补位边界上)
 *   [3] md5_to_hex 小写 + 长度
 *   [4] md5_equal_hex 大小写容忍 + 各种坏输入
 *   [5] ★真实靶子★: _ota_platform_test/fw_test.bin (65536 B)
 *       期望 9e96d33cf71a79caabd00b319e115af5 —— 与 kappa.lol 上传响应里
 *       平台自己算的 checksum 一致, 也和 OTA 台架要下的是同一个文件。
 * ========================================================================== */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "md5.h"

static int g_pass;
static int g_fail;

#define CHECK(cond, ...)                                        \
    do {                                                        \
        if (cond) { g_pass++; printf("  ok   : "); }             \
        else      { g_fail++; printf("  FAIL : "); }             \
        printf(__VA_ARGS__);                                    \
        printf("\n");                                           \
    } while (0)

/* 算一段数据的 hex 摘要 */
static void hex_of(const uint8_t *data, uint32_t len, char out[33])
{
    uint8_t d[16];
    md5_calc(data, len, d);
    md5_to_hex(d, out);
}

/* ==================== [1] RFC 1321 官方向量 ==================== */
static void test_vectors(void)
{
    static const struct
    {
        const char *msg;
        const char *want;
    } V[] = {
        { "",                                                                   "d41d8cd98f00b204e9800998ecf8427e" },
        { "a",                                                                  "0cc175b9c0f1b6a831c399e269772661" },
        { "abc",                                                                "900150983cd24fb0d6963f7d28e17f72" },
        { "message digest",                                                     "f96b697d7cb7938d525a2f31aaf161d0" },
        { "abcdefghijklmnopqrstuvwxyz",                                         "c3fcd3d76192e4007dfb496cca67e13b" },
        { "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789",     "d174ab98d277d9f5a5611c2c9f419d9f" },
        { "12345678901234567890123456789012345678901234567890123456789012345678901234567890",
                                                                                "57edf4a22be3c955ac49da2e2107b67a" }
    };
    unsigned i;

    printf("\n[1] RFC 1321 官方测试向量\n");
    for (i = 0U; i < (unsigned)(sizeof(V) / sizeof(V[0])); i++)
    {
        char got[33];
        hex_of((const uint8_t *)V[i].msg, (uint32_t)strlen(V[i].msg), got);
        CHECK(strcmp(got, V[i].want) == 0,
              "\"%s\"(%u B) -> %s", V[i].msg, (unsigned)strlen(V[i].msg), got);
    }

    /* 100 万个 'a' —— 顺带验"跨很多块"的累加不出错 */
    {
        const uint32_t N = 1000000U;
        uint8_t *buf = (uint8_t *)malloc(N);
        char got[33];
        if (buf != NULL)
        {
            memset(buf, 'a', N);
            hex_of(buf, N, got);
            CHECK(strcmp(got, "7707d6ae4e027c70eea2a935c2296f21") == 0,
                  "'a' x 1000000 -> %s", got);
            free(buf);
        }
    }
}

/* ==================== [2] 流式 == 一次性 ====================
 * 这是 OTA 最要紧的性质: 下载分片边界由网络决定, 可能在任意字节处断开。 */
static void test_streaming(void)
{
    uint8_t  data[300];
    uint8_t  one[16];
    uint32_t total = 0U;
    uint32_t i;
    uint32_t chunk;
    int      bad = 0;
    int      cases = 0;

    /* 造一段可辨认的数据 */
    for (i = 0U; i < sizeof(data); i++)
    {
        data[i] = (uint8_t)((i * 7U + 3U) & 0xFFU);
    }

    printf("\n[2] 流式 == 一次性(每一种切法)\n");

    /* 对 0..300 每一个总长度, 用若干种 chunk 切 */
    for (total = 0U; total <= sizeof(data); total++)
    {
        md5_calc(data, total, one);

        for (chunk = 1U; chunk <= 70U; chunk++)
        {
            md5_ctx_t ctx;
            uint8_t   got[16];
            uint32_t  off = 0U;

            md5_init(&ctx);
            while (off < total)
            {
                uint32_t n = total - off;
                if (n > chunk) { n = chunk; }
                md5_update(&ctx, &data[off], n);
                off += n;
            }
            md5_final(&ctx, got);
            cases++;

            if (memcmp(one, got, 16U) != 0)
            {
                bad++;
                if (bad <= 3)
                {
                    printf("       len=%u chunk=%u 不一致\n", (unsigned)total, (unsigned)chunk);
                }
            }
        }
    }
    CHECK(bad == 0, "★ %d 种(长度 x 切法)组合全部一致 (不一致 %d)", cases, bad);

    /* 补位边界专项: 55/56/57/63/64/65 是 MD5 最容易写错的地方 */
    {
        static const uint32_t EDGE[] = { 55U, 56U, 57U, 63U, 64U, 65U, 119U, 120U, 127U, 128U, 129U };
        unsigned k;
        int      edge_bad = 0;
        for (k = 0U; k < (unsigned)(sizeof(EDGE) / sizeof(EDGE[0])); k++)
        {
            uint8_t one2[16];
            uint8_t got2[16];
            md5_ctx_t c2;
            uint32_t len = EDGE[k];

            md5_calc(data, len, one2);
            /* 每字节喂一次 —— 最碎的切法 */
            md5_init(&c2);
            for (i = 0U; i < len; i++) { md5_update(&c2, &data[i], 1U); }
            md5_final(&c2, got2);
            if (memcmp(one2, got2, 16U) != 0) { edge_bad++; }
        }
        CHECK(edge_bad == 0, "★ 补位边界 55/56/57/63/64/65/119/120/127/128/129 逐字节喂入一致");
    }

    /* 空 update 不能改变结果 */
    {
        md5_ctx_t c;
        uint8_t   a[16];
        uint8_t   b[16];
        md5_init(&c);
        md5_update(&c, data, 100U);
        md5_update(&c, NULL, 0U);          /* 必须被忽略 */
        md5_update(&c, data, 0U);          /* len=0 也必须被忽略 */
        md5_final(&c, a);
        md5_calc(data, 100U, b);
        CHECK(memcmp(a, b, 16U) == 0, "空 update(NULL/len=0) 不影响结果");
    }
}

/* ==================== [3] hex 输出 ==================== */
static void test_hex(void)
{
    uint8_t d[16];
    char    hex[33];
    char    big[64];

    printf("\n[3] md5_to_hex\n");
    md5_calc((const uint8_t *)"abc", 3U, d);
    md5_to_hex(d, hex);
    CHECK(strlen(hex) == 32U, "长度 = 32 (得到 %u)", (unsigned)strlen(hex));
    CHECK(strcmp(hex, "900150983cd24fb0d6963f7d28e17f72") == 0, "abc -> %s", hex);
    CHECK(strchr(hex, 'A') == NULL, "★ 全小写(协议要求 HEX 小写)");

    /* 调用方给个更大的缓冲, 不能越界写 */
    memset(big, 'X', sizeof(big));
    md5_to_hex(d, big);
    CHECK((strlen(big) == 32U) && (big[32] == '\0'), "33 字节足够, 第 33 字节是 '\\0'");
    CHECK(big[33] == 'X', "★ 不越界写(第 34 字节未被动过)");
}

/* ==================== [4] hex 比较 ==================== */
static void test_equal(void)
{
    uint8_t d[16];

    printf("\n[4] md5_equal_hex\n");
    md5_calc((const uint8_t *)"abc", 3U, d);

    CHECK(md5_equal_hex(d, "900150983cd24fb0d6963f7d28e17f72"), "小写 32 字符 -> 一致");
    CHECK(md5_equal_hex(d, "900150983CD24FB0D6963F7D28E17F72"), "★ 大写也接受");
    CHECK(md5_equal_hex(d, "900150983cD24fB0d6963f7d28e17f72"), "混合大小写也接受");
    CHECK(!md5_equal_hex(d, "900150983cd24fb0d6963f7d28e17f73"), "★ 末位差 1 -> 拒绝");
    CHECK(!md5_equal_hex(d, "000150983cd24fb0d6963f7d28e17f72"), "★ 首位差 1 -> 拒绝");
    CHECK(!md5_equal_hex(d, "900150983cd24fb0d6963f7d28e17f7"),  "★ 只有 31 字符 -> 拒绝");
    CHECK(!md5_equal_hex(d, ""),                                "空串 -> 拒绝");
    CHECK(!md5_equal_hex(d, "zz0150983cd24fb0d6963f7d28e17f72"), "★ 非 hex 字符 -> 拒绝");
    CHECK(!md5_equal_hex(d, NULL),                               "hex=NULL -> 拒绝");
    CHECK(!md5_equal_hex(NULL, "900150983cd24fb0d6963f7d28e17f72"), "digest=NULL -> 拒绝");
}

/* ==================== [5] 真实靶子 ==================== */
static void test_real_target(void)
{
    const char *path = "_ota_platform_test/fw_test.bin";
    FILE       *fp;
    uint8_t     buf[4096];
    size_t      n;
    uint32_t    total = 0U;
    md5_ctx_t   ctx;
    uint8_t     d[16];
    char        hex[33];

    printf("\n[5] ★真实 OTA 靶子: %s\n", path);

    fp = fopen(path, "rb");
    if (fp == NULL)
    {
        printf("  skip : 文件不在(被清理过?) —— 不是失败, 但 OTA 台架锚点没验到\n");
        return;
    }

    md5_init(&ctx);
    while ((n = fread(buf, 1U, sizeof(buf), fp)) > 0U)
    {
        md5_update(&ctx, buf, (uint32_t)n);
        total += (uint32_t)n;
    }
    fclose(fp);
    md5_final(&ctx, d);
    md5_to_hex(d, hex);

    CHECK(total == 65536U, "大小 = 65536 B (得到 %u)", (unsigned)total);
    CHECK(strcmp(hex, "9e96d33cf71a79caabd00b319e115af5") == 0,
          "★ MD5 = %s", hex);
    CHECK(md5_equal_hex(d, "9E96D33CF71A79CAABD00B319E115AF5"),
          "★ 与 kappa.lol 上传响应里的 checksum 一致(大写形式)");
}

int main(void)
{
    printf("==== app/ota/md5.c 增量 MD5 ====\n");
    test_vectors();
    test_streaming();
    test_hex();
    test_equal();
    test_real_target();
    printf("\n==== 通过 %d, 失败 %d ====\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
