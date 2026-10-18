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

/** 0x08 出厂字段应答（on_frame 里 0x08 分支调用） */
void lora_field_handle_resp(const lora_frame_t *frame, uint8_t idx);

/** 写字段期间设备回的 0x80/0x81；true = 已被字段状态机消费 */
bool lora_field_on_ack(const lora_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* LORA_INTERNAL_H */
