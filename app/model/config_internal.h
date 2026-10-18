/**
 * @file  config_internal.h
 * @brief model/config_store 内部接口（config_store.c / config_parse.c / config_serialize.c）
 *
 * 只允许 app/model/config_*.c include；对外 API 一律在 config_store.h。
 */
#ifndef CONFIG_INTERNAL_H
#define CONFIG_INTERNAL_H

#include <stdint.h>
#include "config_store.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 序列化整份配置（含两遍打印：先算 CRC，再补 meta）。0 = 失败。 */
uint16_t cfg_serialize(const config_store_t *c, char *out, uint16_t cap,
                       uint32_t *crc_out);

/* 枚举字段的"线上文本 <-> 下标"表：解析与序列化共用同一份定义，防止两边漂。 */
extern const char *const cfg_measure_names[];
extern const char *const cfg_color_names[];
extern const char *const cfg_rulemode_names[];
extern const char *const cfg_offline_names[];
extern const char *const cfg_policy_names[];

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_INTERNAL_H */
