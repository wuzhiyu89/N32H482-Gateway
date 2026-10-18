/**
 * @file  ota_meta.h
 * @brief OTA 升级元数据 —— ota_port / ota_http 共用的数据类型
 *
 * 历史: 这个结构原来定义在 ota.h(旧的 MQTT 分包版 OTA)里。
 * 2026-10-18 旧版整条链路移入 archive/legacy/ota-mqtt/, 但升级元数据仍然被
 * ota_port.c(断点续传记录)和 ota_http.c(校验)使用, 因此单独抽到这里。
 *
 * ⚠ ota_port.c 里有 _Static_assert 依赖本结构的 sizeof 为 8 的倍数:
 *    加字段前先看那里的断言, 别让 Flash 里的续传记录悄悄变形。
 */
#ifndef __OTA_META_H__
#define __OTA_META_H__

#include <stdint.h>

typedef struct
{
    char     version[24];   /* 新版本号字符串 */
    uint32_t total_size;    /* 固件总字节数 */
    uint32_t crc32;         /* 整个固件的 CRC32 */
    uint16_t packet_count;  /* 总包数 */
} ota_meta_t;

#endif /* __OTA_META_H__ */
