/**
 * @file storage.c
 * @brief common 片内Flash读写工具 N32H482
 */
#include "storage.h"
#include "n32h47x_48x_flash.h"

/* N32H47x/48x 主闪存页大小 8KB,擦除地址必须 0x2000 对齐 */
#define STORAGE_PAGE_SIZE   0x2000U

bool storage_flash_read(uint32_t flash_addr, uint8_t *buf, uint16_t len)
{
    if(buf == NULL)
    {
        return false;
    }
    for(uint16_t i = 0U; i < len; i++)
    {
        buf[i] = *(volatile uint8_t*)(flash_addr + i);
    }
    return true;
}

bool storage_flash_erase_page(uint32_t page_addr)
{
    if((page_addr & (STORAGE_PAGE_SIZE - 1U)) != 0U)
    {
        return false;
    }

    uint32_t primask = __get_PRIMASK();   /* 保存原来中断状态 */
    __disable_irq();                      /* Flash操作期间关闭全局中断 */

    FLASH_Unlock();
    /* N32H48x接口：FLASH_EraseOnePage(内部已清标志+等待完成)，页大小8KB */
    FLASH_STS sts = FLASH_EraseOnePage(page_addr);
    FLASH_Lock();

    if(primask == 0U)                     /* 如果进来时中断是打开的，才恢复打开 */
    {
        __enable_irq();
    }
    /* N32H48x操作成功返回FLASH_EOP */
    return (sts == FLASH_EOP);
}

bool storage_flash_write(uint32_t flash_addr, const uint8_t *data, uint16_t len)
{
    if((data == NULL) || ((flash_addr & 0x7U) != 0U) || ((len & 0x7U) != 0U))
    {
        return false;   /* N32H48x只支持double word编程，必须8字节对齐且长度为8的倍数 */
    }

    uint32_t primask = __get_PRIMASK();

    /* ★★2026-10-07: 临界区【按 double word 分段】, 不再整段关中断★★
     *
     * 【原来的写法】__disable_irq() 之后一口气把 len/8 个 double word 全编完才开中断。
     * 平时 len 小没暴露, 但 OTA 下载时 ota_port_flash_write() 一次写 512 字节
     * = 64 个 double word, 每个编程约 60us -> 【全程关中断约 3.8ms】。
     * 而 4G 口是 115200(一个字符 87us), 3.8ms 相当于 43 个字符时间没人取数据,
     * UART 硬件必然溢出(OREF), 硬件层直接丢字节 —— 现象就是:
     *   [STAT2] 丢字节 1 -> 2 -> 3 -> 5
     *   [OTA_HTTP] !! content 本块长度不符: 模组声称 200, 实际解出 180 字节
     *              (hex 段 360 字符) —— 行中间被啃掉一段
     * 行被啃断之后, content URC 的数据就错位了; 字节总数还可能凑得上, 只有最后的
     * MD5 才暴露(19:10 那次: 头对尾对总长对, 只有 MD5 不对)。
     *
     * 【现在】每个 double word 单独进出临界区: 单次编程 ~60us < 87us(一个字符时间),
     * 中断来得及在两次编程之间把 UART 数据取走, 不会再溢出。
     * Flash 控制器要求"编程 + 等完成"这一整段不可被打断, 所以临界区【只包
     * FLASH_ProgramdoubleWord 这一个调用】—— 既满足 Flash 时序, 又给中断留出缝隙。
     * 代价是写入略慢(多几次 primask 存取), 与"下载不再丢字节"相比完全值得。 */
    FLASH_Unlock();
    uint16_t dw_cnt = len / 8U;
    FLASH_STS sts = FLASH_EOP;
    for(uint16_t i = 0U; i < dw_cnt; i++)
    {
        /* 按小端拼出两个word，避免data指针非对齐访问 */
        uint32_t w0 = (uint32_t)data[i*8U + 0U] |
                      ((uint32_t)data[i*8U + 1U] << 8U) |
                      ((uint32_t)data[i*8U + 2U] << 16U) |
                      ((uint32_t)data[i*8U + 3U] << 24U);
        uint32_t w1 = (uint32_t)data[i*8U + 4U] |
                      ((uint32_t)data[i*8U + 5U] << 8U) |
                      ((uint32_t)data[i*8U + 6U] << 16U) |
                      ((uint32_t)data[i*8U + 7U] << 24U);

        __disable_irq();
        sts = FLASH_ProgramdoubleWord(flash_addr + i*8U, w0, w1);
        if(primask == 0U)
        {
            __enable_irq();     /* 每次编程完立刻放开, 让 UART/定时器中断插进来 */
        }

        if(sts != FLASH_EOP)
        {
            break;
        }
    }
    FLASH_Lock();

    if(primask == 0U)
    {
        __enable_irq();
    }
    return (sts == FLASH_EOP);
}
