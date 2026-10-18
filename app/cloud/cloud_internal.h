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
#include "config_store.h"    /* config_store_t */

#ifdef __cplusplus
extern "C" {
#endif

/* 命令处理函数（按域拆文件；命令表仍在 cloud_proto.c） */
/* 配置域个数（域名表的顺序 = 文档 3.2） */
#define CLOUD_CFG_SECTION_N 9u
extern const char *const cloud_cfg_section_names[CLOUD_CFG_SECTION_N];

/* cloud_proto.c 提供、命令文件共用的基础设施 */
extern config_store_t cloud_cfg_staging_buf;   /* 整份配置的暂存副本 */
int  cloud_commit_config(const config_store_t *tmp);
int  cloud_cfg_sections_mask(const cloud_req_t *req, uint16_t *mask);
bool cloud_write_section_list(cJSON *obj, const char *key, uint8_t mask);

/* 命令处理函数（按域拆文件；命令表仍在 cloud_proto.c） */
int cloud_cmd_cfg_set(const cloud_req_t *req, cJSON *data);
int cloud_cmd_cfg_query(const cloud_req_t *req, cJSON *data);
int cloud_cmd_sys_factory_reset(const cloud_req_t *req, cJSON *data);

int cloud_cmd_sys_query_status(const cloud_req_t *req, cJSON *data);
int cloud_cmd_sys_time_sync(const cloud_req_t *req, cJSON *data);
int cloud_cmd_sys_reboot(const cloud_req_t *req, cJSON *data);

int cloud_cmd_ota_start(const cloud_req_t *req, cJSON *data);
int cloud_cmd_ota_query(const cloud_req_t *req, cJSON *data);
int cloud_cmd_ota_abort(const cloud_req_t *req, cJSON *data);

/** cJSON 池化打印（原 cloud_proto_print）：成功返回长度，0 = 失败/溢出 */
uint16_t cloud_json_print(char *out, uint16_t cap, cJSON *root);

/** 把 cloud_status_t 写成 status 对象的字段（status.report / sys.query_status 共用） */
bool cloud_fill_status_json(cJSON *obj, const cloud_status_t *st);

#ifdef __cplusplus
}
#endif

#endif /* CLOUD_INTERNAL_H */
