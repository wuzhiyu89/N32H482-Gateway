/**
 * \file  bootloader.h
 * \brief N32H482 Bootloader — OTA 固件搬运 + App 跳转
 *
 * Flash 分区定义已全部收拢到 app/common/flash_map.h (唯一真相源)。
 * 本文件只保留 BL_* 别名, 便于既有代码平滑过渡, 不再自己定义地址。
 *
 * 搬运安全链:
 *   1. bootloader 读标记页前 32B, magic != "OTAV" → 直接跳 app
 *   2. magic 有效 → CRC32 校验暂存区(FM_STAGE_ADDR 起)与标记一致?
 *      - 不一致 → 擦标记页 → 跳 app
 *      - 一致 → 擦 app 区 → 逐页复制暂存区 → 擦标记页 → 跳 app
 *   3. 跳 app: 设 VTOR=FM_APP_ADDR, MSP=*(FM_APP_ADDR), PC=*(FM_APP_ADDR+4)
 *
 * 掉电安全性(原设计已具备, 无需额外进度标记):
 *   CRC 校验在擦 app 区【之前】完成; 搬运的源(暂存区)全程不被破坏;
 *   标记页只在复制成功后才清除。故搬运中掉电 → 下次重启重新走
 *   "校验 → 擦 → 复制" 全流程, 可自愈, 不会变砖。
 *   真正缺的是"新固件本身有 bug 时回退旧固件", 那需要双 Slot + 确认启动。
 */
#ifndef __BOOTLOADER_H__
#define __BOOTLOADER_H__

#include <stdint.h>
#include <stdbool.h>
#include "flash_map.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== Flash 分区常量(别名, 指向 flash_map.h) ==================== */
#define BL_BOOT_ADDR        FM_BOOT_ADDR          /* Bootloader 起始, 本模块不碰自己 */
#define BL_APP_ADDR         FM_APP_ADDR           /* App 运行区起始 */
#define BL_APP_MAX_SIZE     FM_APP_SIZE           /* App 区上限 */
#define BL_OTA_BUFFER_ADDR  FM_STAGE_ADDR         /* OTA 固件暂存区起始 */
#define BL_OTA_BUFFER_SIZE  FM_STAGE_SIZE         /* OTA 暂存区大小 */
#define BL_META_ADDR        FM_META_ADDR          /* 标记页起始 */
#define BL_PAGE_SIZE        FM_PAGE_SIZE          /* N32H482 页大小 8KB */

/* ==================== 标记区格式 (前 32 字节) ==================== */
#define BL_META_MAGIC0      'O'
#define BL_META_MAGIC1      'T'
#define BL_META_MAGIC2      'A'
#define BL_META_MAGIC3      'V'

#pragma pack(push, 1)
typedef struct
{
    uint8_t  magic[4];      /* "OTAV" = OTA valid */
    uint32_t crc32;         /* 整个固件 CRC32 (小端) */
    uint32_t total_size;    /* 固件总字节数 (小端) */
    uint8_t  reserved[20];  /* 0 填充 */
} ota_mark_t;
#pragma pack(pop)

/* ==================== 对外 API ==================== */

/**
 * @brief bootloader 主入口: 检测 OTA 标记, 按需搬运, 跳 app
 *        (调用后不再返回, 除非搬运前标记无效直接返回)
 *
 * 典型用法:
 *   int main(void) { bl_run(); for(;;); }
 *
 * @note 必须在 bootloader 独立工程中使用; 若放在 app 工程内调用,
 *       跳转会把 VTOR 切到 FM_APP_ADDR, 而当前 app 实际运行在 0x08000000
 *       (未启用双 Slot 时), 会 HardFault。
 */
void bl_run(void);

/**
 * @brief 仅检测 OTA 标记是否有效(不执行搬运)
 * @param out_mark 输出标记内容(可 NULL)
 * @return true 标记有效(magic="OTAV" + size 合法); false 无效/不存在
 */
bool bl_check_mark(ota_mark_t *out_mark);

/**
 * @brief 执行搬运流程(读标记 → CRC 校验 → 擦 app → 复制 → 擦标记)
 * @return true 搬运成功(已调 bl_jump_app 不再返回); false 标记无效/CRC 错/Flash 错
 */
bool bl_do_transfer(void);

/**
 * @brief 跳转到 app(设 VTOR, 关中断, 设 MSP, 跳转 Reset_Handler)
 *        执行后不再返回
 */
void bl_jump_app(void);

/**
 * @brief 擦除 OTA 标记页(清除 "OTAV" 标记), 防止重复搬运
 */
void bl_clear_mark(void);

#ifdef __cplusplus
}
#endif

#endif /* __BOOTLOADER_H__ */
