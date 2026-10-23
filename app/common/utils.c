#include "utils.h"
#include <stdbool.h>
#include <stdio.h>

/* ---- msgId 生成 ----
 * 输出固定 8 位小写十六进制。算法是"上电种子 + 类型 + 序号"的无状态混洗:
 *   1) 每次调用序号不同 -> ID 不同;
 *   2) 同一 (salt, seq) 在本次上电内结果恒定 -> 断线补报可原样重建, 平台幂等不误判;
 *   3) 上电时由 app_boot 用"网关 UID + 时基"播种 -> 不同设备、不同轮次不撞号。
 * 不调用 utils_msg_id_seed() 时退化为固定种子(主机测试可复现)。
 */
static uint32_t s_msg_id_seed = 0x9E3779B9u;
static uint16_t s_msg_id_seq  = 0u;

static uint32_t msg_id_mix(uint32_t x)
{
    x ^= x >> 16;
    x *= 0x7FEB352Du;
    x ^= x >> 15;
    x *= 0x846CA68Bu;
    x ^= x >> 16;
    return x;
}

void utils_msg_id_seed(uint32_t seed)
{
    s_msg_id_seed = (seed != 0u) ? seed : 0x9E3779B9u;
}

bool utils_gen_msg_id_seq(char *out, uint16_t cap, uint8_t salt, uint16_t seq)
{
    uint32_t v;
    int      n;

    if (out == NULL)
    {
        return false;
    }
    if (cap == 0u)
    {
        return false;
    }
    if (cap < 9u)
    {
        out[0] = '\0';
        return false;
    }

    v = msg_id_mix(s_msg_id_seed ^ ((uint32_t)salt << 24) ^ (uint32_t)seq);
    n = snprintf(out, (size_t)cap, "%08lx", (unsigned long)v);
    return (n == 8);
}

bool utils_gen_msg_id(char *out, uint16_t cap)
{
    s_msg_id_seq++;
    if (s_msg_id_seq == 0u)
    {
        s_msg_id_seq = 1u;
    }
    return utils_gen_msg_id_seq(out, cap, 0u, s_msg_id_seq);
}

uint16_t crc16_modbus(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i, j;

    for (i = 0U; i < len; i++)
    {
        crc ^= data[i];
        for (j = 0U; j < 8U; j++)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc >>= 1U;
                crc ^= 0xA001U;
            }
            else
            {
                crc >>= 1U;
            }
        }
    }
    return crc;
}

bool crc16_modbus_check(const uint8_t *frame, uint16_t frame_len)
{
    /* 帧长度至少要大于2字节，否则直接失败 */
    if (frame_len < 2U)
    {
        return false;
    }
    uint16_t calc_crc = crc16_modbus(frame, frame_len - 2U);
    /* 帧末尾低字节在前 */
    uint8_t crc_low  = frame[frame_len - 2U];
    uint8_t crc_high = frame[frame_len - 1U];
    uint16_t recv_crc = ((uint16_t)crc_high << 8U) | crc_low;

    return (calc_crc == recv_crc);
}

void crc16_modbus_append(uint8_t *buf, uint16_t data_len)
{
    uint16_t crc = crc16_modbus(buf, data_len);
    /* modbus 低字节在前 */
    buf[data_len]     = (uint8_t)(crc & 0x00FFU);
    buf[data_len + 1U] = (uint8_t)((crc >> 8U) & 0x00FFU);
}

uint32_t crc32_ieee(const uint8_t *buf, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    uint32_t i;
    uint8_t  bit;

    for (i = 0u; i < len; i++)
    {
        crc ^= buf[i];
        for (bit = 0u; bit < 8u; bit++)
        {
            crc = (crc & 1u) ? ((crc >> 1u) ^ 0xEDB88320u) : (crc >> 1u);
        }
    }
    return ~crc;
}

#include <stdint.h>
#include <string.h>

/**
 * @brief utf8字符串转十六进制文本字符串
 * @param utf8 输入UTF‑8字符串，以'\0'结尾
 * @param out_hex 输出缓冲区，存放大写16进制文本，不带分隔符
 * @param out_len out_hex缓冲区总字节大小
 * @return 成功返回0；缓冲区不足返回‑1
 * 示例：输入"水深"，输出 "E7B0B4E6B18F"
 * 注意：输出缓冲区大小至少 = strlen(utf8)*2 + 1；每个字节占2个hex字符 + 结束符
 */
int utf8_to_hexstr(const char *utf8, char *out_hex, uint16_t out_len)
{
    static const char hex_table[] = "0123456789ABCDEF";
    size_t in_idx = 0;
    size_t out_idx = 0;
    if(utf8 == NULL || out_hex == NULL || out_len == 0)
        return -1;

    while(utf8[in_idx] != '\0')
    {
        uint8_t byte = (uint8_t)utf8[in_idx];
        // 需要2字节输出 + 预留末尾'\0'
        if ((out_idx + 2) >= out_len)
            return -1;

        out_hex[out_idx++] = hex_table[(byte >> 4) & 0x0F];
        out_hex[out_idx++] = hex_table[byte & 0x0F];
        in_idx++;
    }
    // 字符串结束符
    if(out_idx >= out_len)
        return -1;
    out_hex[out_idx] = '\0';
    return 0;
}

/* ============================================================================
 * UTF-8 文本 -> "每个汉字固定 3 字节" 的定长单元(供 LoRa 组帧)
 * ----------------------------------------------------------------------------
 * 【为什么需要它】(2026-09-29 用户第 6 条)
 *   云端下发的 LED 文本/语音文本是 JSON 里的 UTF-8 字符串; 网关要把它塞进 LoRa
 *   帧的文本域。本项目对文本域的口径是【一个汉字 = 3 字节】—— UTF-8 编码下汉字
 *   本来就正好 3 字节, 所以这里【原样搬运 UTF-8 字节】, 不做 GB2312 转码:
 *       · 汉字(含全角标点): 3 字节
 *       · ASCII(数字/字母/空格/半角标点): 1 字节, 原样
 *   这样设备侧只要按 UTF-8 解码即可; 长度域直接用返回的字节数。
 *   ⚠️ 注意: 既有的 0x03 声光帧注释里写的是 GB2312(2 字节/汉字), 那是另一套口径。
 *      声光设备最终用哪种编码要等设备侧固件定了再统一 —— 用户已明确"先不写与
 *      声光报警器的通讯协议"。届时若确认用 GB2312, 本函数旁边再加一个转码版即可。
 *
 * 【严格校验】非法 UTF-8 一律拒绝(返回 -1), 不做"猜"或静默截断:
 *   · 续字节必须落在 0x80~0xBF; 长度必须与首字节自洽
 *   · 拒绝过长编码(0xC0/0xC1 开头的 2 字节)、拒绝 0xF5~0xFF 开头
 *   · 拒绝被截断的多字节序列(字符串中途结束)
 * ========================================================================== */
bool utf8_char_len(const char *s, uint8_t *out_len)
{
    const uint8_t *p = (const uint8_t *)s;
    uint8_t b0;
    uint8_t need;
    uint8_t i;

    if (s == NULL || out_len == NULL || p[0] == '\0') { return false; }

    b0 = p[0];
    if      ((b0 & 0x80U) == 0x00U) { need = 1U; }   /* ASCII */
    else if ((b0 & 0xE0U) == 0xC0U) { need = 2U; }
    else if ((b0 & 0xF0U) == 0xE0U) { need = 3U; }   /* 汉字在这里 */
    else if ((b0 & 0xF8U) == 0xF0U) { need = 4U; }
    else                            { return false; } /* 0x80~0xBF / 0xF8~0xFF */

    /* 拒绝过长编码(overlong): C0/C1 开头、E0 后跟 80~9F、F0 后跟 80~8F */
    if ((need == 2U) && (b0 == 0xC0U || b0 == 0xC1U)) { return false; }

    for (i = 1U; i < need; i++)
    {
        uint8_t c = p[i];
        if (c == '\0')                          { return false; }  /* 被截断 */
        if ((c & 0xC0U) != 0x80U)               { return false; }  /* 不是续字节 */
    }
    if ((need == 3U) && (b0 == 0xE0U) && ((uint8_t)p[1] < 0xA0U)) { return false; }
    if ((need == 4U) && (b0 == 0xF0U) && ((uint8_t)p[1] < 0x90U)) { return false; }

    *out_len = need;
    return true;
}

int utf8_pack_3byte(const char *utf8, uint8_t *out, uint16_t out_cap, uint16_t *out_len)
{
    uint16_t w    = 0U;      /* 已写字节数 */
    int      n    = 0;       /* 已打包字符数 */
    const char *s = utf8;

    if (utf8 == NULL || out == NULL || out_cap == 0U) { return -1; }

    while (*s != '\0')
    {
        uint8_t clen = 0U;
        uint8_t i;

        if (!utf8_char_len(s, &clen)) { return -1; }         /* 非法 UTF-8 */
        if ((uint16_t)(w + clen) > out_cap) { return -1; }   /* 放不下: 不截断 */

        for (i = 0U; i < clen; i++) { out[w++] = (uint8_t)s[i]; }
        s += clen;
        n++;
    }

    if (out_len != NULL) { *out_len = w; }
    return n;                                                /* 字符个数 */
}

