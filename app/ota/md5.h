#ifndef __MD5_H__
#define __MD5_H__

#include <stdint.h>
#include <stdbool.h>

/**
 * \file  md5.h
 * \brief 增量 MD5 (RFC 1321) —— OTA 固件完整性校验用
 *
 * ============================ 为什么必须"增量" ============================
 *
 * 固件 128 KB, 而 OTA 是【边下载边写 Flash】的: 每收到一片就写进暂存区、
 * 顺手喂给 MD5。等全下完再回头读 128 KB 算 MD5 的话, 要么得再占 128 KB RAM
 * (没有), 要么得从 Flash 再读一遍(慢, 且掉电后无法续算)。
 * 所以必须是流式接口: md5_init() -> 反复 md5_update() -> md5_final()。
 *
 * ============================ 为什么不用 64 位计数 ============================
 *
 * 标准实现用 uint64 记总位数。这里改用 uint32 记【总字节数】, 补齐时手工
 * 拆成低/高 32 位写进 8 字节长度字段 —— 省掉 ARMCC 的 64 位运算辅助函数,
 * 也省 4 字节 RAM。上限 4 GB, 对 240 KB 的暂存区来说绰绰有余。
 *
 * ============================ 与工具链对齐 ============================
 *
 * 校验值以【小写 hex 字符串】和云端交互(协议 5.6 ota.start 的 <md5> 字段)。
 * md5_equal_hex() 同时接受大小写, 免得云端哪天给大写就判失败。
 */

/* ==================== 上下文 ==================== */
typedef struct
{
    uint32_t state[4];     /* A/B/C/D 四个 32 位累加器 */
    uint32_t byte_count;   /* 已喂入的总字节数(不是位数) */
    uint8_t  buffer[64];   /* 攒满 64 字节才处理一块 */
    uint32_t used;         /* buffer 中已用字节数, 0..63 */
} md5_ctx_t;

/* ==================== 流式接口 ==================== */

/** @brief 开始一次新的计算 */
void md5_init(md5_ctx_t *ctx);

/**
 * @brief 喂入一段数据(可反复调用, 长度任意)
 * @param data 数据指针(可为 NULL, 此时 len 必须为 0)
 * @param len  字节数
 */
void md5_update(md5_ctx_t *ctx, const uint8_t *data, uint32_t len);

/**
 * @brief 收尾并输出 16 字节摘要
 * @param out 16 字节输出缓冲
 * @note  调用后 ctx 不应再被使用(除非重新 md5_init)
 */
void md5_final(md5_ctx_t *ctx, uint8_t out[16]);

/* ==================== 便捷接口 ==================== */

/** @brief 16 字节摘要 -> 32 字符小写 hex + '\0'(out 至少 33 字节) */
void md5_to_hex(const uint8_t digest[16], char out[33]);

/** @brief 一次性算完(小数据用; OTA 请用流式接口) */
void md5_calc(const uint8_t *data, uint32_t len, uint8_t out[16]);

/**
 * @brief 摘要与 hex 串比较(大小写均可)
 * @param hex 32 字符 hex(允许尾部有空白); 长度不足 32 直接判不等
 * @return true = 一致
 */
bool md5_equal_hex(const uint8_t digest[16], const char *hex);

#endif /* __MD5_H__ */
