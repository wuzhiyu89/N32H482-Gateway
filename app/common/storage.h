/**
 * @file storage.h
 * @brief common 片内Flash读写通用工具，N32H482
 */
#ifndef __STORAGE_H__
#define __STORAGE_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 读取flash地址到buf
 * @param flash_addr flash源地址
 * @param buf 输出缓存
 * @param len 字节数
 * @return true成功
 */
bool storage_flash_read(uint32_t flash_addr, uint8_t *buf, uint16_t len);

/**
 * @brief 擦除一页flash(N32H482页大小8KB，地址必须0x2000对齐)
 * @param page_addr 页起始地址
 * @return true成功
 */
bool storage_flash_erase_page(uint32_t page_addr);

/**
 * @brief 往flash写数据，需要预先擦除该页
 * @param flash_addr 写入起始地址，必须8字节对齐(N32H48x按double word编程)
 * @param data 数据源
 * @param len 字节长度，8字节对齐
 * @return true成功
 */
bool storage_flash_write(uint32_t flash_addr, const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif
