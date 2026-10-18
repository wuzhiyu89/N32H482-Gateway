/**
 * @file  lora_internal.h
 * @brief links/lora 内部接口（lora_business.c 与拆出来的状态机文件之间）
 *
 * 只允许 app/links/lora/*.c include；对外 API 一律在 lora_business.h。
 */
#ifndef LORA_INTERNAL_H
#define LORA_INTERNAL_H

#include <stdint.h>
#include "lora_business.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 本网关的 13 字节地址（lora_business_init 时从 param_save 读出） */
extern uint8_t g_gw_uid[LORA_UID_BYTES];

/* ==================== 从机表（lora_slave.c） ====================
 * 表是动态的：init 时从 config_store 装载，运行期由绑定/解绑增删。
 * 除只读遍历（get_slave_list）外，请不要直接改 g_slave_list / g_slave_count。 */
extern lora_slave_t g_slave_list[];
extern uint8_t      g_slave_count;

/** 按 13 字节地址查索引；0xFF = 不在表里 */
uint8_t lora_slave_find(const uint8_t uid[LORA_UID_BYTES]);

/** 收到帧时"摸一下"：标记在线 + 记 last_recv_tick；0xFF = 不在表里 */
uint8_t lora_slave_touch(const uint8_t uid[LORA_UID_BYTES]);

/** 往表尾追加一台；返回索引，-1 = 表满/已在表里 */
int lora_slave_add(const uint8_t uid[LORA_UID_BYTES], slave_type_t type);

/** 删除指定地址（末尾元素前移补洞）；0 = 成功，-1 = 不在表里 */
int lora_slave_del(const uint8_t uid[LORA_UID_BYTES]);

/** 从 config_store 重新装载整张表（init 时调一次） */
void lora_slave_load(void);

/** 设备地址首位 -> 类型（1 网关 / 2 液位 / 3 声光），现场规则 */
uint8_t lora_dev_type_of_uid(const uint8_t uid[LORA_UID_BYTES]);

/** ASCII hex 文本 -> 二进制（旧格式下行的兼容分支用）；-1 = 非法 */
int lora_hex_str2bin(const char *hex, int hex_len, uint8_t *out, int out_max);

/* ==================== 声光 0x03 执行结果状态机（lora_alarm.c） ====================
 * 0x03 是"三段式应答"：0x80 = 收到了，0x82 = 执行成功，0x83 = 执行失败。
 * 15 秒窗口、最多 3 条，3 条都没成功 = 声光报警器离线。 */
#define LORA_ALARM_TIMEOUT_MS   15000u   /* 单条 0x03 等 0x82/0x83 的窗口(用户指定 15s) */
#define LORA_ALARM_MAX_ATTEMPTS 3u       /* 单条动作最多发几条(用户指定 3 条) */
#define LORA_ALARM_RETRY_GAP_MS 500u     /* 重发前的最小间隔(给设备射频恢复时间) */

/** 正在下发的这条动作是哪个档位（档位不在空口载荷里，上报 event.action 要用） */
extern uint8_t g_alarm_gear_inflight;

/** 上报一次 0x03 执行结果（成功/失败/放弃/恢复） */
void lora_alarm_fire_result(const lora_slave_t *s, bool ok, bool gave_up, bool recovered);

/** 本条 0x03 失败：还有配额就回到 IDLE 等重发，用完就收尾并判离线 */
void lora_alarm_on_fail(lora_slave_t *s);

/* ==================== 下发队列（lora_downlink.c） ====================
 * 投递在 data_cache，真正的空口发送 / ACK 重传 / 应答时序都在这个文件里。 */
/** 把某台从机的待发指令发出去（组帧 + 发送）；true = 已发出 */
bool lora_tx_downlink(uint8_t idx);

/** 每秒一次：ACK 超时重传 + 声光主动下发 */
void lora_downlink_logic(void);
void lora_try_send_alarm_idle(void);

/** 收到请求帧后"先回 0x80、再等 3s 下发"的两步时序 */
void lora_reply_schedule(const uint8_t dev_uid[LORA_UID_BYTES], uint8_t func, uint8_t idx);
void lora_reply_logic(void);

/** 4G/MQTT 下行回调（注册给 g4_urc；唯一的下行入口） */
void lora_mqtt_recv_cb(const char *topic, const uint8_t *data, uint16_t len);

/* ==================== 绑定 0x04（lora_bind.c） ====================
 * 窗口只在云端 dev.bind/sen.bind 时打开（平时收到 0x04 一律丢弃）。 */
/** 窗口是否开着（任意外部状态），on_frame 据此决定要不要把 0x04 交给握手 */
bool lora_bind_accepts_0x04(void);
/** 是否正在等这台设备回 0x80 确认 */
bool lora_bind_waiting_dev_ack(const uint8_t uid[LORA_UID_BYTES]);

void lora_bind_handle_0x04(const lora_frame_t *frame);
void lora_bind_handle_dev_ack(const lora_frame_t *frame);
void lora_bind_logic(void);

/** 上报一次绑定/解绑事件（绑定与解绑共用这个出口） */
void lora_bind_fire_event(const char *uid_hex, bool is_sensor, lora_bind_event_t event);

/* ==================== 解绑 0x05（lora_unbind.c） ====================
 * 两阶段状态机；0x05 不进 data_cache，解绑期间目标从机被当作"锁"。 */
/** 这条 ack 是否属于正在进行的解绑（是则交给 lora_unbind_on_ack） */
bool lora_unbind_owns_ack(uint8_t slave_idx);
void lora_unbind_on_ack(void);
void lora_unbind_logic(void);

/** 0x08 出厂字段应答（on_frame 里 0x08 分支调用） */
void lora_field_handle_resp(const lora_frame_t *frame, uint8_t idx);

/** 写字段期间设备回的 0x80/0x81；true = 已被字段状态机消费 */
bool lora_field_on_ack(const lora_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* LORA_INTERNAL_H */
