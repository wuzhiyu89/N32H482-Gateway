#ifndef __UTILS_H__
#define __UTILS_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Modbus CRC16计算
 * @param data 待计算数据缓冲区
 * @param len 数据长度，**不包含末尾2字节CRC本身**
 * @return 原始crc16值；发送帧时：低字节在前，高字节在后
 */
uint16_t crc16_modbus(const uint8_t *data, uint16_t len);

/**
 * @brief 校验一帧完整数据的CRC
 * @param frame 完整帧，末尾2字节为CRC
 * @param frame_len 完整帧总长度
 * @return true:crc校验通过 false:校验失败
 */
bool crc16_modbus_check(const uint8_t *frame, uint16_t frame_len);

/**
 * @brief 把crc16值填入缓冲区末尾
 * @param buf 数据缓冲区，前面是有效数据，末尾预留2字节放crc
 * @param data_len 有效数据长度（不含CRC）
 */
void crc16_modbus_append(uint8_t *buf, uint16_t data_len);

uint32_t crc32_ieee(const uint8_t *buf, uint32_t len);

/**
 * @brief 生成 8 位小写十六进制 msgId（形如 "f1ed8d11"，每次调用都不同）
 * @param out 输出缓冲区，至少 9 字节（8 个字符 + '\0'）
 * @param cap out 的总容量
 * @return true 成功; false 参数非法/缓冲不足
 * @note  内部用"上电种子 + 自增序号"混洗，不依赖 rand()，不占堆。
 */
bool utils_gen_msg_id(char *out, uint16_t cap);

/**
 * @brief 由事件类型与序号重建 msgId（断线补报用，同一 seq 必须得到同一个 ID）
 * @param out  输出缓冲区，至少 9 字节
 * @param cap  out 的总容量
 * @param salt 事件类型字符（'s' 状态迁移 / 'v' 设备上下线 / 'a' 联动结果）
 * @param seq  事件序号（event_cache 记录里的 msg_seq）
 * @return true 成功; false 参数非法/缓冲不足
 */
bool utils_gen_msg_id_seq(char *out, uint16_t cap, uint8_t salt, uint16_t seq);

/**
 * @brief 给 msgId 生成器播种（上电调用一次；不调用则用固定种子）
 * @param seed 建议混入 SysTick->VAL 等上电抖动
 */
void utils_msg_id_seed(uint32_t seed);

/**
 * @brief utf8字符串转十六进制文本字符串
 * @param utf8    输入UTF-8字符串(以'\0'结尾)
 * @param out_hex 输出缓冲区(大写HEX, 无分隔符)
 * @param out_len out_hex 缓冲区【总字节数】(2026-09-29 从 uint8_t 改为 uint16_t:
 *                256 字节的缓冲传进 uint8_t 会被截成 0, 函数直接返回 -1)
 * @return 0 成功; -1 参数非法或缓冲不足
 * @note  输出缓冲区至少 strlen(utf8)*2 + 1
 */
int utf8_to_hexstr(const char *utf8, char *out_hex, uint16_t out_len);

/**
 * @brief 取 UTF-8 字符串里【一个字符】占几个字节(并做合法性校验)
 * @param s       指向某个字符的首字节
 * @param out_len 输出: 该字符字节数(1~4)
 * @return true 合法; false 非法(续字节错位/过长编码/被截断/参数 NULL)
 * @note  汉字在 UTF-8 下恒为 3 字节 —— 这是本项目"三字节组帧"的依据。
 */
bool utf8_char_len(const char *s, uint8_t *out_len);

/**
 * @brief 把 UTF-8 文本按"一个汉字 3 字节"打包成定长单元(供 LoRa 组帧)
 *
 * 【口径】汉字(含全角标点) = 3 字节, ASCII = 1 字节, **原样搬运 UTF-8 字节**,
 *         不做 GB2312 转码。理由与注意点见 utils.c 里本函数的注释块。
 * 【严格】非法 UTF-8 或放不下就返回 -1, **绝不截断** —— 截断出来的半截汉字会让
 *         设备侧显示乱码, 比不发更难查。
 *
 * @param utf8    输入(以'\0'结尾)
 * @param out     输出缓冲(二进制, 不写结尾'\0')
 * @param out_cap out 的容量(字节)
 * @param out_len 输出: 实际写入字节数(可为 NULL)
 * @return >=0 打包的字符个数; -1 参数非法 / 含非法 UTF-8 / 缓冲不足
 */
int utf8_pack_3byte(const char *utf8, uint8_t *out, uint16_t out_cap, uint16_t *out_len);

#ifdef __cplusplus
}
#endif

#endif
