/**
 * \file ota_health.h
 * \brief A/B 双槽的【健康判据 + 自动回滚】状态机（2026-10-11 新增）
 *
 * ============================================================================
 * 【它解决什么问题】
 *   新固件装上另一槽后，怎么判断"它到底能不能用"？
 *   做法是经典的"三次自证"：
 *     · 每次启动给本槽扣一次机会（计数器存在 Flash 的标记页里）
 *     · 运行期一旦确认健康（见下面的三条判据），就把本槽【定案】
 *     · 机会扣完还没定案 ⇒ 说明新固件起不来/起不稳 ⇒ 请求回滚到另一槽
 *
 * 【健康判据（三条全满足才算健康）】
 *   1. 4G 在线          —— 说明通信链路起来了
 *   2. 至少一条上行被 puback 确认 —— 说明 broker 真的收到了（"模组回 OK"不算）
 *   3. uptime > OTA_HEALTH_MIN_UPTIME_S（默认 60 秒）—— 说明不是"起来就崩"
 *   三条缺一不可：只看 1 会把"连上了但发不出去"当成健康；
 *   只看 3 会把"根本没联网"当成健康。
 *
 * 【设计成"纯逻辑"是刻意的】
 *   三个判据都是入参（bool + 秒数），状态转换完全确定
 *   ⇒ 主机单测可以完整驱动（不依赖射频/网络/Flash 时序）。
 *   boot_step() 【不在内部复位】—— 只返回"该回滚"，由调用方决定怎么执行
 *   （这样单测能验"何时该回滚"，而不用假装复位）。
 *
 * 【与存储层的关系】
 *   状态存在 ota_port 的 ota_rollback_t 里（标记页偏移 512）。
 *   ⚠️ 那个记录有一条硬约束：【所有写操作只能把 1 变成 0】——
 *     见 ota_port.h / ota_port.c 里的长注释。
 *     本模块的所有写操作都遵守它（tries_left 递减、unconfirmed 清 0）。
 *
 * 【回滚由谁执行】
 *   本模块只【判定】+ 写记录。真正的搬运（把另一槽拷回运行区）是 bootloader
 *   的职责（模式 2 双槽 + BL）。⇒ 在模式 1（单镜像）下本模块也能编译运行，
 *   只是没人消费"回滚请求"—— 那时调用方记一条日志即可。
 * ============================================================================
 */
#ifndef OTA_HEALTH_H
#define OTA_HEALTH_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 健康判据里的最小运行时长（秒）。比它短就不算健康 —— 挡"起来就崩"那种。 */
#ifndef OTA_HEALTH_MIN_UPTIME_S
#define OTA_HEALTH_MIN_UPTIME_S   60u
#endif

/** boot_step() 的判定结果 */
typedef enum
{
    OTA_HEALTH_BOOT_OK       = 0,   /* 正常（已扣一次机会），继续跑 */
    OTA_HEALTH_BOOT_ROLLBACK = 1    /* 机会已用完 -> 调用方应复位，让 BL 回滚 */
} ota_health_boot_t;

/**
 * \brief 开机时调一次：读记录 / 扣机会 / 判定是否该回滚
 * \return OTA_HEALTH_BOOT_ROLLBACK = 应该回滚（调用方负责复位）
 * \note   本函数内部【不】复位 —— 便于主机单测。
 *         ⚠️ 必须【早于】任何"可能失败"的初始化之后？不必 —— 它只碰标记页，
 *         和射频/4G 无关，放开机早期最合适（越早扣机会越准）。
 *         ⚠️ 读不到有效记录时【新建一份并给满次数】，
 *         绝不"因为读不到就回滚"（那会把好固件回滚掉）。
 */
ota_health_boot_t ota_health_boot_step(void);

/**
 * \brief 主循环周期调用：判健康，满足就"定案"（只写一次 Flash）
 * \param g4_online 4G 是否在线
 * \param up_acked  是否至少有一条上行被 broker puback 确认过
 * \param uptime_s  本次运行的秒数
 * \note  定案之后本函数立即变成空操作（不会再碰 Flash）。
 */
void ota_health_poll(bool g4_online, bool up_acked, uint32_t uptime_s);

/**
 * \brief 装完新固件后重置（给新槽满次数）
 * \param boot_slot 新固件要跑的槽（0=A / 1=B；越界按 1 处理）
 * \note  ⚠️ 必须在【擦过标记页之后】调用（例如紧跟 ota_port_http_resume_save()）。
 *        因为"重置"要把 tries_left 从 0x00 写回 0xFF，那是 0->1 ——
 *        只有该区域已是擦除态时写才合法（见 ota_port.h 的 1->0 约束）。
 */
void ota_health_reset_for_new_fw(uint8_t boot_slot);

/* ---------------- 诊断（给日志 / #status 用） ---------------- */

uint8_t ota_health_tries_left(void);   /* 还剩几次机会（0 = 该回滚） */
uint8_t ota_health_boot_slot(void);    /* 当前记录里的槽号 */
bool    ota_health_confirmed(void);    /* 本槽是否已定案 */
bool    ota_health_valid(void);        /* 本次运行是否读到/建了有效记录 */

#ifdef __cplusplus
}
#endif

#endif /* OTA_HEALTH_H */
