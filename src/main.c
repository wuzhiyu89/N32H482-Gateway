/**
 * @file  main.c
 * @brief 固件入口：芯片 UID 钩子 + 三种编译期工程模式的唯一分发点
 *
 * main() 只做三件事：
 *   1. 注册芯片 UID 来源（param_save / config_store 的出厂地址都靠它）；
 *   2. 配对 / LoRa 测试模式下先把 4G 断电，再进各自循环（不返回）；
 *   3. 云端业务：app_boot_run() 初始化，app_loop_run() 进主循环。
 *
 * 各功能开关的唯一真相源是 app/core/app_config.h；
 * 整机数据流、任务节拍与硬件资源见 docs/ARCHITECTURE.md。
 */
#include <stdint.h>

#include "app_core.h"
#include "param_save.h"
#include "config_store.h"
#include "bsp_gpio.h"
#include "n32h47x_48x_dbg.h"   /* GetUID(): 芯片 96 位唯一 ID */
#include "project_config.h"    /* MAIN_MODE_*: 工程模式三选一 */

/* 编译期自检：三种模式必须恰好选一个，多选/少选直接编不过，
 * 避免"以为在跑 A、实际编的是 B"。真相源在 project_config.h。 */
#if ((MAIN_MODE_CLOUD + MAIN_MODE_PAIRING + MAIN_MODE_LORA_TEST) != 1)
#error "主功能模式必须恰好选一个: 见 app/drivers/lora/project_config.h 的『主功能模式(三选一)』"
#endif

/* 把厂商 GetUID() 填的 12 字节(96 位)芯片 UID 交给 param_save/config_store */
static void main_read_chip_uid(uint8_t out[12])
{
    GetUID(out);
}

int main(void)
{
    /* 必须在任何 param_save 使用之前注册：出厂 gw_uid / gwId 都靠它 */
    param_save_set_uid_hook(main_read_chip_uid);
    config_store_set_uid_hook(main_read_chip_uid);

#if (MAIN_MODE_LORA_TEST != 0) || (MAIN_MODE_PAIRING != 0)
    /* MCU 复位后 PA8 悬空 = 给 4G 上电；测试模式要尽量早断电，
     * 避免日志口还没初始化完模组就已经启动。 */
    G4_PowerSwitch_InitNoPower();
#endif

#if MAIN_MODE_PAIRING
    app_mode_pairing_run();
#elif MAIN_MODE_LORA_TEST
    app_mode_rawtest_run();
#else
    app_boot_run();
    app_loop_run();
#endif
}
