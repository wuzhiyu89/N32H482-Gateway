#ifndef __JSON_PROTO_H__
#define __JSON_PROTO_H__

#include <stdint.h>

/**
 * \file  json_proto.h
 * \brief 云端 JSON 协议解析 + 打印(对接 doubao.com/thread/xMIcJs7hJVZPXr7s9)
 *
 * 协议结构:
 *   公共字段: device_id, target_dev_type, target_dev_no, cmd, seq, data{}
 *   下行 topic: ★2026-10-08 起为 rt/gw/{gwId}/cmd(旧的 mod/pc_down 已删除)★
 *     cmd: set_sensor_param / set_alarm_light / ota_start / ota_data / ota_end /
 *          ack_ok / ack_err_retrans
 *   上行 topic: ★rt/gw/{gwId}/cmdresp · /telemetry · /event · /status★
 *     (旧的 mod/mod_upload 那条 hex 上行通道已整体删除)
 *     cmd: report_sensor_data / ack_ok / ack_err_retrans / ota_ack / ota_error
 *
 * ⚠️ 本文件描述的是早期(doubao 线程那版)协议字段, 与现在的 JSON 信封
 *    (cmd/msgId/ts/auth/data)【不是一套】。看现网的协议口径请以
 *    `云端对接网关说明书` 与 `cloud_proto.c` 为准, 这里只作历史参考。
 *
 * 本模块职责: 解析下行 JSON, 打印各字段值, 供测试验证。
 */

/**
 * @brief 处理云端下发的 JSON 报文(解析 + 打印)
 * @param topic MQTT 主题
 * @param data  JSON 原始数据(无需 null 结尾)
 * @param len   数据长度
 */
void json_proto_handle(const char *topic, const uint8_t *data, uint16_t len);

#endif /* __JSON_PROTO_H__ */
