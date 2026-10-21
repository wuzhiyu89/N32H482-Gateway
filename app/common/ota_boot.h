/**
 * @file  ota_boot.h
 * @brief A/B 双槽的“启动记录”布局（BootLoader 与 App 共用，必须逐字节一致）
 *
 * 放在标记页（FM_META_ADDR）偏移 512 起，按 32 字节一条【追加】写入：
 *   · 每条 = magic + seq + 状态 + CRC32；
 *   · 写入前要求目标槽全 0xFF（本芯片只能对擦除态编程）；
 *   · 读的时候取"最后一条 magic/CRC 都合法"的记录；
 *   · 写到一半掉电 → 那条 CRC 不过 → 仍用上一条，天然掉电安全。
 *
 * 语义：
 *   active    = 当前“转正”的槽（0=A, 1=B）
 *   pending   = 正在试跑的新槽（0/1，0xFF=没有）
 *   confirmed = active 是否已经确认健康
 *   tries     = pending 还允许启动几次（每次 pending 启动扣 1；0=该回退了）
 *
 * 与旧的 ota_health 回滚记录共用一个区域：magic 不同，互不干扰；
 * 新方案不再使用 ota_health 的 ABRT 记录。
 */
#ifndef OTA_BOOT_H
#define OTA_BOOT_H

#include <stdint.h>
#include "flash_map.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OTA_BOOT_MAGIC        0x544F4F42u   /* 'B','O','O','T' (小端) */
#define OTA_BOOT_REGION_OFFSET 512u         /* FM_META_ADDR + 512 */
#define OTA_BOOT_REC_SZ        32u
#define OTA_BOOT_SLOTS         (((uint32_t)FM_META_SIZE - OTA_BOOT_REGION_OFFSET) / OTA_BOOT_REC_SZ)

#define OTA_BOOT_SLOT_A        0u
#define OTA_BOOT_SLOT_B        1u
#define OTA_BOOT_SLOT_NONE     0xFFu
#define OTA_BOOT_TRIES_INIT    8u

typedef struct
{
    uint32_t magic;       /* OTA_BOOT_MAGIC */
    uint32_t seq;         /* 单调递增，选最大者 */
    uint8_t  active;      /* 0=A / 1=B */
    uint8_t  pending;     /* 0=A / 1=B / 0xFF=无 */
    uint8_t  confirmed;   /* 1=active 已确认 */
    uint8_t  tries;       /* pending 剩余启动次数 */
    uint32_t crc;         /* 覆盖前 12 字节(magic..tries) */
    uint32_t rsv[4];      /* 补齐 32 字节 */
} ota_boot_rec_t;

/** 槽编号 -> 运行基址 */
static inline uint32_t ota_boot_slot_base(uint8_t slot)
{
    return (slot == OTA_BOOT_SLOT_B) ? (uint32_t)FM_SLOT_B_ADDR
                                     : (uint32_t)FM_SLOT_A_ADDR;
}

/** 槽编号 -> 槽容量 */
#define OTA_BOOT_SLOT_SIZE  ((uint32_t)FM_SLOT_SIZE)

#ifdef __cplusplus
}
#endif

#endif /* OTA_BOOT_H */
