/**
 * @file  cloud_internal.h
 * @brief cloud/ 内部接口：下行路由（cloud_proto.c）与上行构造（cloud_uplink.c）之间
 *
 * 只允许 app/cloud/*.c include。对外 API 一律在 cloud_proto.h。
 */
#ifndef CLOUD_INTERNAL_H
#define CLOUD_INTERNAL_H

#include <stdint.h>
#include <stdbool.h>
#include "cJSON.h"
#include "cloud_proto.h"      /* cloud_status_t */

#ifdef __cplusplus
extern "C" {
#endif

/** cJSON 池化打印（原 cloud_proto_print）：成功返回长度，0 = 失败/溢出 */
uint16_t cloud_json_print(char *out, uint16_t cap, cJSON *root);

/** 把 cloud_status_t 写成 status 对象的字段（status.report / sys.query_status 共用） */
bool cloud_fill_status_json(cJSON *obj, const cloud_status_t *st);

#ifdef __cplusplus
}
#endif

#endif /* CLOUD_INTERNAL_H */
