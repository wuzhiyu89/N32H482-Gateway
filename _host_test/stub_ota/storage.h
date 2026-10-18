#ifndef __STORAGE_H__
#define __STORAGE_H__
/* 主机单测替身: 只声明 ota_port.c 用到的那三个。
 * 真实现见 app/common/storage.h + storage.c(N32 固件库)。
 * 本目录只给 test_ota_port 用, 靠 -I 顺序盖住 app/common 里的真头文件。 */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

bool storage_flash_read(uint32_t flash_addr, uint8_t *buf, uint16_t len);
bool storage_flash_erase_page(uint32_t page_addr);
bool storage_flash_write(uint32_t flash_addr, const uint8_t *data, uint16_t len);

#endif
