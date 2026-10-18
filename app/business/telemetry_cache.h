/**
 * @file  telemetry_cache.h
 * @brief 离线期间的【逐条液位上报】缓存 —— 重连后按"当时的时间戳"补报
 *
 * ============================================================================
 * 【它存什么】一条 = 某台液位传感器【一次 0x02 上报】，不是 30 秒聚合遥测。
 *   用户 2026-10-18 明确: "离线后补报的应该是每次传感器上报的水位,
 *   而不是我们 30s 的遥测"。
 *   ⇒ 所以每条记录是"一台、一次、一个水位值", 补报时一条记录发一条
 *     telemetry.report(sensors[] 里就那一个元素)。
 *
 * 【为什么需要它】4G 掉线时遥测发不出去, 而水位还在照常上报(LoRa 与 4G 独立)。
 *   以前这些数据【直接丢掉了】: main_telemetry_step() 一开头就
 *   `if (!g4_at_is_online()) return;` —— 断线期间根本不生成遥测。
 *   现在: 断线期间每次 0x02 都记一条, 重连后逐条补发。
 *
 * 【为什么单独一个模块, 不复用 event_cache】
 *   · ec_record_t 是 28 B 的【事件】记录(kind/a/b/seq 那套语义), 塞液位数据要
 *     另开 kind 和字段, 会把 512 条的环形撑大且语义混乱;
 *   · 这里记录的是【数值型历史】(水位/电量/rssi/时间戳), 消费者是遥测渲染器,
 *     与事件的消费者(事件渲染器)完全不同。
 *
 * 【时间戳口径】(用户 2026-10-18 要求)
 *   rec.ts = 网关【收到】这一帧上报时的 Unix 秒 ⇒ 补报时原样使用,
 *   **不是**补报那一刻的时间。信封里的 ts 与 sensors[].ts 都用它。
 *   (前提: 绑定设备必须云端下发命令 ⇒ 4G 至少通过一次 ⇒ epoch 有效 —— 用户确认。)
 *
 * 【容量与淘汰】(用户 2026-10-18 选"覆盖最旧")
 *   TELEMETRY_CACHE_MAX = 256 条, 每条 28 B ⇒ 静态 RAM 7 KB。
 *   满了就覆盖最旧的一条并累加 dropped 计数 —— 保留最近的, 早期历史丢弃。
 *   4 台液位 × 60s/次 ⇒ 约 4 条/分 ⇒ 256 条覆盖约 64 分钟。
 * ============================================================================
 */
#ifndef __TELEMETRY_CACHE_H__
#define __TELEMETRY_CACHE_H__

#include <stdint.h>
#include <stdbool.h>

/** 最多缓存的【条数】(一条 = 一次传感器上报)。256 × 28 B = 7 KB。 */
#define TELEMETRY_CACHE_MAX   256U

/** UID 字节数(与 LORA_UID_BYTES / CFG_UID_BYTES 一致, 由 lora_business.c 的静态断言盯着) */
#define TELEMETRY_UID_BYTES   13U

/**
 * 一条"液位上报"记录。
 * 布局刻意把 4 字节字段放前面, 保持 4 字节对齐; 合计 25 B -> 对齐到 28 B。
 */
typedef struct
{
    uint32_t ts;                              /* 收到该上报时的 Unix 秒(补报不改) */
    uint16_t level_mm;                        /* 该次上报的水位(mm) */
    uint16_t msg_seq;                         /* 补报时还原 msgId 的 3 位序号 */
    uint8_t  uid[TELEMETRY_UID_BYTES];        /* 哪台设备 */
    uint8_t  bat_pct;                         /* 电量百分比 */
    int8_t   rssi_dbm;                        /* 该帧的 LoRa RSSI */
    uint8_t  state;                           /* 快照时刻的 level_state_t 码值 */
    uint8_t  gear;                            /* 快照时刻的档位 0~3 */
} tc_record_t;

/** 清空(只清索引; 数据区不清, 没必要) */
void     telemetry_cache_reset(void);

/**
 * @brief 追加一条记录
 * @param rec 不可为 NULL
 * @return 恒为 true(满了会覆盖最旧并计数, 不是失败)
 * @note  覆盖最旧是【刻意的】: 水位看近期趋势更有意义, 而且不能让"缓存满"
 *        变成"新数据全丢"。
 */
bool     telemetry_cache_push(const tc_record_t *rec);

/** 当前条数(0..TELEMETRY_CACHE_MAX) */
uint16_t telemetry_cache_count(void);

/** 容量(恒为 TELEMETRY_CACHE_MAX) */
uint16_t telemetry_cache_capacity(void);

/** 因"满了覆盖"而丢掉的累计条数(现场判断离线多久会丢历史) */
uint32_t telemetry_cache_dropped(void);

/** 看一眼最旧的一条(不出队); 空 -> NULL */
const tc_record_t *telemetry_cache_peek(void);

/** 丢掉最旧的一条(补报成功或确认渲染不出来时调); 空时安全无操作 */
void     telemetry_cache_drop(void);

#endif /* __TELEMETRY_CACHE_H__ */
