/* ============================================================================
 * test_utf8.c —— app/common/utils.c 的 UTF-8 文本工具主机测试
 * ----------------------------------------------------------------------------
 * 为什么要单独测: "云端 JSON 里的 UTF-8 文案 -> LoRa 帧的三字节组帧" 这条链
 * (用户 2026-09-29 第 6 条) 全靠 utf8_pack_3byte(), 而它一旦出错的表现是
 * 【设备侧显示乱码】—— 现场最难查的一类问题。所以把边界都钉死:
 *   · 汉字恒 3 字节、ASCII 恒 1 字节(总字节数 = 3*汉字 + 1*ASCII)
 *   · 非法 UTF-8 一律拒绝: 被截断、续字节错位、过长编码、0xF8+ 开头
 *   · 缓冲不足必须【拒绝而不是截断】(半截汉字 = 乱码, 比不发更难查)
 *   · utf8_char_len() 逐字符长度判定
 *   · utf8_to_hexstr(): out_len 是 uint16(256 不能再被截成 0)
 * ========================================================================== */
#include <stdio.h>
#include <string.h>

#include "utils.h"

static int g_pass;
static int g_fail;

#define CHECK(cond, ...)                                        \
    do {                                                        \
        if (cond) { g_pass++; printf("  ok   : "); }             \
        else      { g_fail++; printf("  FAIL : "); }             \
        printf(__VA_ARGS__);                                    \
        printf("\n");                                           \
    } while (0)

/* 参数自洽: 返回字节数 */
static int pack(const char *s, uint8_t *out, uint16_t cap, uint16_t *n)
{
    return utf8_pack_3byte(s, out, cap, n);
}

static void test_char_len(void)
{
    uint8_t n = 0U;

    printf("\n[1] utf8_char_len: 逐字符长度\n");
    CHECK(utf8_char_len("A", &n) && (n == 1U), "'A' -> 1 字节 (n=%u)", (unsigned)n);
    CHECK(utf8_char_len("路", &n) && (n == 3U), "'路'(汉字) -> 3 字节 (n=%u)", (unsigned)n);
    CHECK(utf8_char_len("，", &n) && (n == 3U), "'，'(全角标点) -> 3 字节 (n=%u)", (unsigned)n);
    CHECK(utf8_char_len("é", &n) && (n == 2U), "'é'(2 字节字符) -> 2 (n=%u)", (unsigned)n);
    CHECK(utf8_char_len("\xF0\x9F\x98\x80", &n) && (n == 4U), "emoji(4 字节) -> 4 (n=%u)", (unsigned)n);

    CHECK(!utf8_char_len("\xE6\xA8", &n), "★被截断的 3 字节序列 -> 拒绝");
    CHECK(!utf8_char_len("\xE6\x41\x41", &n), "★续字节错位(0x41) -> 拒绝");
    CHECK(!utf8_char_len("\xC0\xAF", &n), "★过长编码 C0 AF -> 拒绝");
    CHECK(!utf8_char_len("\xF8\x80\x80\x80", &n), "★0xF8 开头(非法) -> 拒绝");
    CHECK(!utf8_char_len("\x80", &n), "★孤立续字节 0x80 -> 拒绝");
    CHECK(!utf8_char_len("", &n), "空串 -> 拒绝(NULL 字符不算一个字符)");
    CHECK(!utf8_char_len(NULL, &n), "s=NULL -> 拒绝");
    CHECK(!utf8_char_len("A", NULL), "out_len=NULL -> 拒绝");
}

static void test_pack(void)
{
    uint8_t  out[64];
    uint16_t n = 0U;
    int      c;

    printf("\n[2] utf8_pack_3byte: 三字节组帧\n");

    /* 字符串实际是 4 汉字(12) + "12CM "(5) + 4 汉字(12) = 29 字节, 13 个字符
     * (先算清楚再断言 —— 之前这里手算成 15/35, 测试自己错了。) */
    c = pack("路面积水12CM 禁止通行", out, sizeof(out), &n);
    CHECK((c == 13) && (n == 29U),
          "★ 8汉字+5ASCII -> 13 字符 / 29 字节 (c=%d n=%u)", c, (unsigned)n);
    CHECK(memcmp(out, "路面积水12CM 禁止通行", 29U) == 0,
          "★ 字节与输入逐字节一致(原样搬运)");
    CHECK((out[0] == 0xE8U) && (out[1] == 0xB7U) && (out[2] == 0xAFU),
          "★ 首汉字 '路' = E8 B7 AF(UTF-8 3 字节)");

    c = pack("", out, sizeof(out), &n);
    CHECK((c == 0) && (n == 0U), "空串 -> 0 字符 0 字节");

    c = pack("ABC", out, sizeof(out), &n);
    CHECK((c == 3) && (n == 3U), "纯 ASCII -> 3 字符 3 字节 (c=%d n=%u)", c, (unsigned)n);

    c = pack("水深", out, sizeof(out), NULL);
    CHECK(c == 2, "out_len 传 NULL 也能用(只取字符数) (c=%d)", c);

    printf("\n[3] 非法输入必须拒绝, 且【不截断】\n");
    CHECK(pack("\xE6\xA8", out, sizeof(out), &n) == -1, "★被截断的汉字 -> -1");
    CHECK(pack("\xE6\x41\x41", out, sizeof(out), &n) == -1, "★续字节错位 -> -1");
    CHECK(pack("\xC0\xAF", out, sizeof(out), &n) == -1, "★过长编码 -> -1");
    CHECK(pack("水深危险", out, 8U, &n) == -1, "★缓冲不足(8<12) -> -1 而不是截断");
    CHECK(pack("水深危险", out, 12U, &n) == 4, "正好放得下(12=4*3) -> 4 字符");
    CHECK(pack(NULL, out, sizeof(out), &n) == -1, "utf8=NULL -> -1");
    CHECK(pack("A", NULL, sizeof(out), &n) == -1, "out=NULL -> -1");
    CHECK(pack("A", out, 0U, &n) == -1, "out_cap=0 -> -1");

    printf("\n[4] 帧头/长度域的实用口径\n");
    /* LoRa 文本域常见上限 64 字节: 21 个汉字正好 63 字节, 22 个就放不下。
     * ⚠️ 断言与取值必须分成两句: CHECK(pack(...,&n), "...%u", n) 里 n 的求值
     *    顺序在 C 里是未指定的, 会读到旧值。 */
    {
        const char *s21 = "一二三四五六七八九十一二三四五六七八九十一";
        const char *s22 = "一二三四五六七八九十一二三四五六七八九十一二";
        int r21;
        int r22;
        uint16_t n21 = 0U;

        r21 = pack(s21, out, 64U, &n21);
        CHECK(r21 == 21, "21 个汉字 -> 21 字符 (r=%d)", r21);
        CHECK(n21 == 63U, "21 个汉字 = %u 字节(<=64)", (unsigned)n21);
        r22 = pack(s22, out, 64U, &n21);
        CHECK(r22 == -1, "22 个汉字(66 字节) > 64 -> 拒绝(不切半个字)");
    }
}

static void test_hexstr(void)
{
    char buf[64];

    printf("\n[5] utf8_to_hexstr: out_len 已是 uint16\n");
    CHECK(utf8_to_hexstr("AB", buf, (uint16_t)sizeof(buf)) == 0, "正常转换");
    CHECK(strcmp(buf, "4142") == 0, "AB -> 4142 (得到 %s)", buf);
    CHECK(utf8_to_hexstr("水", buf, (uint16_t)sizeof(buf)) == 0, "汉字也能转");
    CHECK(strcmp(buf, "E6B0B4") == 0, "水(E6 B0 B4) -> E6B0B4 (得到 %s)", buf);
    CHECK(utf8_to_hexstr("A", buf, 2U) == -1, "缓冲不足 -> -1");
    CHECK(utf8_to_hexstr("A", buf, 3U) == 0, "恰好 3 字节(A->41 + '\\0') -> 0");
}

int main(void)
{
    printf("==== utils UTF-8 文本工具 ====\n");
    test_char_len();
    test_pack();
    test_hexstr();
    printf("\n==== 通过 %d, 失败 %d ====\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
