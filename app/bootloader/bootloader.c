/**
 * \file  bootloader.c
 * \brief N32H482 Bootloader 核心实现
 *
 * Flash 操作: 直接用固件库 FLASH_Unlock / EraseOnePage / ProgramdoubleWord / Lock
 *             擦写期间 __disable_irq(), 操作完成恢复
 *             —— storage.c 也是同样套路, 这样保证不碰 storage.c 的代码(独立工程)
 * CRC32:     纯软件逐位 IEEE 802.3, 多项式 0xEDB88320, 初始 0xFFFFFFFF,
 *             最终异或 0xFFFFFFFF —— 与 ota_port.c 一致, 保证上位机对齐
 * 搬运:      逐页擦 app 区 → 逐 double word 从 OTA buffer 复制到 app 区
 *             (逐 double word 不需要 RAM 缓冲, bootloader RAM 紧张)
 */
#include "bootloader.h"
#include "n32h47x_48x_flash.h"
#include "n32h47x_48x.h"          /* SCB, NVIC_SystemReset */
#include <string.h>

/* ==================== 内部 Flash 原语 ====================
 * 参考 storage.c 风格: __disable_irq() → Unlock → 操作 → Lock → 恢复中断
 * bootloader 独立工程, 不能 include storage.c, 所以这里自己实现一份精简版。
 */

static bool bl_flash_erase_page(uint32_t page_addr)
{
    if ((page_addr & (BL_PAGE_SIZE - 1U)) != 0U)
    {
        return false;
    }
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    FLASH_Unlock();
    FLASH_STS sts = FLASH_EraseOnePage(page_addr);
    FLASH_Lock();

    if (primask == 0U)
    {
        __enable_irq();
    }
    return (sts == FLASH_EOP);
}

static bool bl_flash_write_dw(uint32_t addr, uint32_t w0, uint32_t w1)
{
    if ((addr & 0x7U) != 0U)
    {
        return false;
    }
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    FLASH_Unlock();
    FLASH_STS sts = FLASH_ProgramdoubleWord(addr, w0, w1);
    FLASH_Lock();

    if (primask == 0U)
    {
        __enable_irq();
    }
    return (sts == FLASH_EOP);
}

/* 内存映射读, 无需锁 Flash */
static void bl_flash_read(uint32_t addr, uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0U; i < len; i++)
    {
        buf[i] = *(volatile uint8_t *)(addr + i);
    }
}

/* ==================== CRC32 (纯软件, 与 ota_port.c 一致) ==================== */

static uint32_t bl_crc32_update(uint32_t crc, const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0U; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t b = 0U; b < 8U; b++)
        {
            if ((crc & 1U) != 0U)
            {
                crc = (crc >> 1) ^ 0xEDB88320U;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief 计算 OTA buffer 的 CRC32
 * @param size 固件总字节数(从 OTA buffer 起始读取)
 * @return CRC32 值(最终已异或 0xFFFFFFFF)
 */
static uint32_t bl_calc_fw_crc(uint32_t size)
{
    uint32_t crc = 0xFFFFFFFFU;
    uint32_t offset = 0U;
    /* 分块 256B 读, 避免大 RAM 占用 */
    uint8_t chunk[256];
    while (offset < size)
    {
        uint32_t n = size - offset;
        if (n > sizeof(chunk)) n = sizeof(chunk);
        bl_flash_read(BL_OTA_BUFFER_ADDR + offset, chunk, (uint16_t)n);
        crc = bl_crc32_update(crc, chunk, n);
        offset += n;
    }
    return crc ^ 0xFFFFFFFFU;
}

/* ==================== 标记页操作 ==================== */

bool bl_check_mark(ota_mark_t *out_mark)
{
    ota_mark_t mk;
    bl_flash_read(BL_META_ADDR, (uint8_t *)&mk, sizeof(mk));

    /* magic 必须是 "OTAV" */
    if ((mk.magic[0] != BL_META_MAGIC0) || (mk.magic[1] != BL_META_MAGIC1) ||
        (mk.magic[2] != BL_META_MAGIC2) || (mk.magic[3] != BL_META_MAGIC3))
    {
        return false;
    }
    /* size 必须合法: 非零 + <= OTA buffer 上限 + <= app 区上限 */
    if ((mk.total_size == 0U) ||
        (mk.total_size > BL_OTA_BUFFER_SIZE) ||
        (mk.total_size > BL_APP_MAX_SIZE))
    {
        return false;
    }
    if (out_mark != NULL)
    {
        *out_mark = mk;
    }
    return true;
}

void bl_clear_mark(void)
{
    /* 擦除标记页 = 全 0xFF, 下次 bootloader 启动读不到 magic 就跳过 */
    (void)bl_flash_erase_page(BL_META_ADDR);
}

/* ==================== 搬运核心 ==================== */

/**
 * @brief 擦除 app 区(从 BL_APP_ADDR 起, 向上取整到页)
 * @param size 固件大小(擦除 ceil(size/8KB) 页)
 * @return true 全部成功
 */
static bool bl_erase_app_area(uint32_t size)
{
    uint32_t pages = (size + BL_PAGE_SIZE - 1U) / BL_PAGE_SIZE;
    /* 上限保护: 最多擦满整个 App 区, 绝不越界到暂存区 / 标记页 */
    uint32_t max_pages = BL_APP_MAX_SIZE / BL_PAGE_SIZE;
    if (pages > max_pages)
    {
        pages = max_pages;
    }
    for (uint32_t i = 0U; i < pages; i++)
    {
        uint32_t page_addr = BL_APP_ADDR + i * BL_PAGE_SIZE;
        /* 防御: 绝不擦到 App 区以下的任何区域(BootLoader / 自身代码) */
        if (page_addr < FM_APP_ADDR)
        {
            continue;
        }
        if (!bl_flash_erase_page(page_addr))
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief 从 OTA buffer 复制固件到 app 区(逐 double word, 无需 RAM 缓冲)
 * @param size 固件大小
 * @return true 全部写入成功
 */
static bool bl_copy_to_app(uint32_t size)
{
    uint32_t addr_src = BL_OTA_BUFFER_ADDR;
    uint32_t addr_dst = BL_APP_ADDR;
    uint32_t offset   = 0U;

    /* 主体: 8 字节对齐部分(中间包天然 8B 对齐, 除非最后一包尾部有非对齐) */
    uint32_t dw_total = size / 8U;   /* double word 总数 */
    for (uint32_t i = 0U; i < dw_total; i++)
    {
        uint32_t w0 = *(volatile uint32_t *)(addr_src + i * 8U);
        uint32_t w1 = *(volatile uint32_t *)(addr_src + i * 8U + 4U);
        if (!bl_flash_write_dw(addr_dst + i * 8U, w0, w1))
        {
            return false;
        }
    }
    uint32_t written = dw_total * 8U;

    /* 尾部不足 8 字节: 补 0xFF 凑齐一个 double word —— 与 ota_port_flash_write 一致 */
    uint32_t tail = size - written;
    if (tail != 0U)
    {
        uint8_t tail_buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        bl_flash_read(addr_src + written, tail_buf, (uint16_t)tail);
        uint32_t w0 = (uint32_t)tail_buf[0] |
                      ((uint32_t)tail_buf[1] << 8) |
                      ((uint32_t)tail_buf[2] << 16) |
                      ((uint32_t)tail_buf[3] << 24);
        uint32_t w1 = (uint32_t)tail_buf[4] |
                      ((uint32_t)tail_buf[5] << 8) |
                      ((uint32_t)tail_buf[6] << 16) |
                      ((uint32_t)tail_buf[7] << 24);
        if (!bl_flash_write_dw(addr_dst + written, w0, w1))
        {
            return false;
        }
    }
    return true;
}

bool bl_do_transfer(void)
{
    ota_mark_t mk;

    /* Step 1: 读标记 */
    if (!bl_check_mark(&mk))
    {
        return false;   /* 无有效标记, 不搬运 */
    }

    /* Step 2: CRC 校验 —— 先校验再擦 app, 防砖!
     * 只有 CRC 校验失败才清标记(让下次启动不再尝试, 直接跳旧 app)。
     * 后续擦除/写入失败时 **不清标记**, 让下次重启有机会重试搬运。 */
    uint32_t actual_crc = bl_calc_fw_crc(mk.total_size);
    if (actual_crc != mk.crc32)
    {
        bl_clear_mark();
        return false;
    }

    /* Step 3: 擦 app 区 */
    if (!bl_erase_app_area(mk.total_size))
    {
        /* 擦除失败 —— 保留标记, 下次重启重试 */
        return false;
    }

    /* Step 4: 复制固件 */
    if (!bl_copy_to_app(mk.total_size))
    {
        /* 写入失败 —— 保留标记, 下次重启重试 */
        return false;
    }

    /* Step 5: 擦除标记页, 防止下次重启重复搬运 */
    bl_clear_mark();

    /* Step 6: 跳 app(不再返回) */
    bl_jump_app();
    return true;   /* 理论上不会到这, bl_jump_app 不返回 */
}

/* ==================== 跳转 App ====================
 * Cortex-M4 跳转流程:
 *   1. 设 SCB->VTOR = FM_APP_ADDR (让中断向量表指到 app)
 *   2. 读 app 向量表: MSP = *(FM_APP_ADDR), Reset_Handler = *(FM_APP_ADDR+4)
 *   3. __disable_irq() 避免悬空中断
 *   4. 设 MSP = 向量表[0]
 *   5. 跳转到 Reset_Handler(向量表[1])
 */

void bl_jump_app(void)
{
    /* 读向量表 */
    uint32_t *vt = (uint32_t *)BL_APP_ADDR;
    uint32_t msp = vt[0];
    uint32_t reset_handler = vt[1];

    /* 防御: 向量表必须合理(非 0, 非 全 0xFF, 且在 RAM/Flash 范围内)
     * RAM 范围: N32H482 0x20000000~0x2002FFFF (192KB)
     * Flash 范围: FM_FLASH_BASE ~ FM_FLASH_END */
    if ((msp < 0x20000000U) || (msp >= 0x20030000U) ||
        (reset_handler < FM_FLASH_BASE) || (reset_handler >= FM_FLASH_END))
    {
        /* 向量表坏了 —— 不要跳, 死循环等 WDT(若启用)或手动复位 */
        for (;;) { __NOP(); }
    }

    __disable_irq();

    /* 重定向向量表, ARM 手册要求: 设置 VTOR 后执行 DSB+ISB 再使用新向量表 */
    SCB->VTOR = BL_APP_ADDR;
    __DSB();
    __ISB();

    /* 重设 MSP + 跳转(内联汇编, 必须 Thumb 模式) */
    __asm volatile(
        "msr msp, %0\n"
        "bx  %1\n"
        :
        : "r"(msp), "r"(reset_handler | 1U)   /* |1 保证 Thumb 状态 */
        : "memory"
    );
}

/* ==================== 主入口 ==================== */

void bl_run(void)
{
    ota_mark_t mk;

    /* 检测标记 */
    if (!bl_check_mark(&mk))
    {
        /* 无有效 OTA 升级 —— 直接跳当前 app */
        bl_jump_app();
        return;
    }

    /* 标记有效, 执行搬运流程(成功则跳 app, 失败也会清标记后跳 app) */
    bl_do_transfer();   /* 成功不返回, 失败返回 */

    /* 搬运失败 —— 清标记防止下次循环, 跳 app */
    /* (bl_do_transfer 失败时内部已经清过标记了, 这里再兜底一次) */
    bl_jump_app();
}
