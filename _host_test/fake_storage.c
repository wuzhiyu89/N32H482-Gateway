/**
 * \file  fake_storage.c
 * \brief 主机单测用的 storage 替身(512 KB RAM 后端)
 *
 * ★★★ 关键设计: storage_flash_write() 实现成 "new = old & data" ★★★
 *
 * 真实 NOR Flash 只能把 bit 从 1 写成 0; 想把 0 变回 1 必须【先擦整页】。
 * 而"写之前忘了擦"正是最容易出的 Flash bug —— 如果这里图省事写成 memcpy,
 * 那类 bug 在主机上【永远测不出来】, 只能等上板时表现为"数据莫名其妙不对"。
 * 用 & 之后, 同一类 bug 在这里就是一次可复现的断言失败。
 *
 * 同理, 这里比真实现【更严】地要求 8 字节对齐: 能过这里就一定过真实现。
 */
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "storage.h"

#define FS_BASE   0x08000000u
#define FS_SIZE   0x00080000u      /* 512 KB, 与 N32H482 一致 */
#define FS_PAGE   0x00002000u      /* 8 KB */

static uint8_t g_flash[FS_SIZE];

static bool fs_ok(uint32_t addr, uint16_t len)
{
    if (addr < FS_BASE)
    {
        return false;
    }
    if (((addr - FS_BASE) + (uint32_t)len) > FS_SIZE)
    {
        return false;
    }
    return true;
}

void fake_storage_reset(void)
{
    memset(g_flash, 0xFF, sizeof(g_flash));
}

const uint8_t *fake_storage_ptr(void)
{
    return g_flash;
}

/* ★★2026-10-14 新增: 测试专用的"直写字节"接口 —— 【绕过编程约束】★★
 *
 * 【用途】造"硬件层面被破坏的数据"(掉电写坏 / 老化 / 位翻转)。
 *   这类破坏【本来就不是"正常编程"能做到的】—— 真机上你没法用
 *   FLASH_ProgramdoubleWord 往一个已编程的字里塞任意值。
 *   ⇒ 所以模拟它必须绕过编程语义, 直接改 RAM 映像。
 *
 * 【为什么必须有它】2026-10-14 收紧 storage_flash_write()(改成"只能写擦除态的字")
 *   之后, test_ota_port 里原来的 poke8() 立刻开始失败 ——
 *   因为它一直用 storage_flash_write() 来"铺测试数据/破坏 magic",
 *   而那在真机上根本做不到。⇒ 那个 helper 一直在偷懒, 只是以前假实现太宽松。
 *
 * ⚠️⚠️ 绝对不要用它来验证"能不能写入" —— 那必须走 storage_flash_write(),
 *   否则又变成"假实现比真实现宽松", 单测就失去意义了。
 *   本函数的命名带 poke, 就是为了让调用处一眼看出"这是在造坏数据"。 */
void fake_storage_poke8(uint32_t flash_addr, const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0u) || !fs_ok(flash_addr, len))
    {
        return;
    }
    (void)memcpy(&g_flash[flash_addr - FS_BASE], data, len);
}

bool storage_flash_read(uint32_t flash_addr, uint8_t *buf, uint16_t len)
{
    if ((buf == NULL) || (len == 0u) || !fs_ok(flash_addr, len))
    {
        return false;
    }
    (void)memcpy(buf, &g_flash[flash_addr - FS_BASE], len);
    return true;
}

bool storage_flash_erase_page(uint32_t page_addr)
{
    if ((page_addr & (FS_PAGE - 1u)) != 0u)
    {
        return false;               /* 真实现要求页对齐 */
    }
    if (!fs_ok(page_addr, (uint16_t)FS_PAGE))
    {
        return false;
    }
    memset(&g_flash[page_addr - FS_BASE], 0xFF, FS_PAGE);
    return true;
}

bool storage_flash_write(uint32_t flash_addr, const uint8_t *data, uint16_t len)
{
    uint16_t i;

    if ((data == NULL) || (len == 0u) || !fs_ok(flash_addr, len))
    {
        return false;
    }
    if (((flash_addr | (uint32_t)len) & 0x7u) != 0u)
    {
        return false;               /* 真实现要求 addr 与 len 都 8 字节对齐 */
    }

    /* ★★2026-10-14 收紧: 只能编程【处于擦除态(全 0xFF)的 double word】★★
     *
     * 【为什么必须收紧】原来这里是 `g_flash[..] &= data[i]` —— 它允许【重复写】
     *   同一个位置(只要新值是旧的子集)。而真机不是这样:
     *   app/common/storage.c 的 storage_flash_write() 里
     *       sts = FLASH_ProgramdoubleWord(addr, w0, w1);
     *       if (sts != FLASH_EOP) break;   -> 返回 false
     *   ⇒ N32H48x 的编程【只对擦除态的字成功】; 向已编程过的位置再写,
     *     哪怕内容完全相同、哪怕只是 1->0, 硬件也直接报错。
     *
     * 【这个漏洞的代价(真实发生过)】
     *   ota_port 的 A/B 回滚记录第一版按"只写不擦页 + 每次只把 1 变 0"设计,
     *   这里用 &= 模拟, 主机单测 36/36 全绿 —— 一上板刷了几千行
     *       [OTAHL] !! 定案写 Flash 失败, 下次再试
     *   ⇒ 假实现比真实现【宽松】, 就会"单测全绿、上板全废"。
     *
     * ★ 教训: 假实现要【比真实现更严】, 不能更松。★
     *   (文件头原本就写着"用 & 之后, 同一类 bug 在这里就是一次可复现的断言失败" ——
     *    方向是对的, 但 & 还不够严: 它漏掉了"已编程过的字不能再写"这一条。) */
    for (i = 0u; i < len; i = (uint16_t)(i + 8u))
    {
        uint16_t j;

        for (j = 0u; j < 8u; j++)
        {
            if (g_flash[flash_addr - FS_BASE + i + j] != 0xFFu)
            {
                return false;       /* 该 double word 已被编程过 -> 真机同样失败 */
            }
        }
    }

    (void)memcpy(&g_flash[flash_addr - FS_BASE], data, len);
    return true;
}

/* ota_port.c 的 ota_port_reboot() 会调它; 主机上什么也不做(不返回是不行的) */
void NVIC_SystemReset(void)
{
}
