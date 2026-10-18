/**
 * @file  ota_mark.h
 * @brief OTA 标记页里的"固件有效"记录 —— App 与 bootloader 之间的契约
 *
 * 位置: OTA_META_ADDR + 0 (见 flash_map.h / ota_port.c)
 *       前 32 字节: "OTAV" + crc32 + total_size + 20 字节 0
 *
 * 谁用它:
 *   · ota_port.c       校验通过后按这个布局写标记页
 *   · bootloader(模式 2) 开机读它, 决定要不要把暂存区搬到运行区
 *   · 主机测试         用它检查标记页是否写对
 *
 * 历史: 原来定义在 app/bootloader/bootloader.h。bootloader 在模式 1 下不参与
 * 构建(已归档到 archive/legacy/bootloader/), 但"标记页长什么样"是当前 OTA
 * 仍然依赖的接口, 所以把结构单独抽出来, 不跟着 BL 一起进档案。
 *
 * ⚠ 改这个结构 = 改 Flash 上的持久化格式。改字段必须同时改 bootloader 和
 *   ota_port.c 的写入/校验, 并升版本或擦掉旧标记页。
 */
#ifndef __OTA_MARK_H__
#define __OTA_MARK_H__

#include <stdint.h>

#pragma pack(push, 1)
typedef struct
{
    uint8_t  magic[4];      /* "OTAV" = OTA valid */
    uint32_t crc32;         /* 整个固件 CRC32 (小端) */
    uint32_t total_size;    /* 固件总字节数 (小端) */
    uint8_t  reserved[20];  /* 0 填充 */
} ota_mark_t;
#pragma pack(pop)

#endif /* __OTA_MARK_H__ */
