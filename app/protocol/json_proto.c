/**
 * \file  json_proto.c
 * \brief 云端 JSON 协议解析 + 打印实现
 *
 * 轻量级 JSON 解析: 用 strstr 定位 key, strtoul/手动提取 value。
 * 不依赖外部 JSON 库, 适配 MCU 有限 RAM。
 */
#include "json_proto.h"
#include "log.h"
#include <string.h>
#include <stdlib.h>

/* ==================== JSON 辅助函数 ==================== */

/**
 * @brief 在 JSON 中查找 key 对应的数值(uint32)
 * @param json       null 结尾的 JSON 字符串
 * @param key_quoted 带引号的 key, 如 "\"seq\""
 * @return 解析到的数值; 未找到返回 0
 */
static uint32_t json_get_uint(const char *json, const char *key_quoted)
{
    const char *p = strstr(json, key_quoted);
    if (p == NULL) return 0;
    p += strlen(key_quoted);
    p = strchr(p, ':');
    if (p == NULL) return 0;
    return (uint32_t)strtoul(p + 1, NULL, 10);
}

/**
 * @brief 在 JSON 中查找 key 对应的字符串值
 * @param json       null 结尾的 JSON 字符串
 * @param key_quoted 带引号的 key
 * @param out        输出缓冲
 * @param out_sz     输出缓冲大小
 * @return 字符串长度; 未找到返回 -1
 */
static int json_get_str(const char *json, const char *key_quoted,
                        char *out, int out_sz)
{
    const char *p, *end;
    int len;

    p = strstr(json, key_quoted);
    if (p == NULL) return -1;
    p += strlen(key_quoted);
    p = strchr(p, ':');
    if (p == NULL) return -1;
    p = strchr(p, '"');
    if (p == NULL) return -1;
    p++;                        /* 跳过开头引号 */
    end = strchr(p, '"');
    if (end == NULL) return -1;
    len = (int)(end - p);
    if (len >= out_sz) len = out_sz - 1;
    memcpy(out, p, len);
    out[len] = '\0';
    return len;
}

/* ==================== 各 cmd 处理 ==================== */

/* 1.1 set_sensor_param: 设置液位传感器参数 + 时间同步 */
static void handle_set_sensor_param(const char *json)
{
    log_info("  [set_sensor_param]\r\n");
    log_info("  detect_interval_min=%u\r\n", json_get_uint(json, "\"detect_interval_min\""));
    log_info("  report_interval_min=%u\r\n", json_get_uint(json, "\"report_interval_min\""));
    log_info("  report_same_level=%u\r\n",    json_get_uint(json, "\"report_same_level\""));
    log_info("  alarm_level_threshold_mm=%u\r\n", json_get_uint(json, "\"alarm_level_threshold_mm\""));
    log_info("  time_sync: yy=%u mm=%u dd=%u ww=%u hh=%u mi=%u\r\n",
             json_get_uint(json, "\"yy\""),
             json_get_uint(json, "\"mm\""),
             json_get_uint(json, "\"dd\""),
             json_get_uint(json, "\"ww\""),
             json_get_uint(json, "\"hh\""),
             json_get_uint(json, "\"mi\""));
}

/* 1.2 set_alarm_light: 设置声光报警器 */
static void handle_set_alarm_light(const char *json)
{
    char led_text[64] = {0};
    char tts_text[128] = {0};

    log_info("  [set_alarm_light]\r\n");
    log_info("  power_ctrl=%u\r\n",       json_get_uint(json, "\"power_ctrl\""));
    log_info("  light_ctrl=%u\r\n",       json_get_uint(json, "\"light_ctrl\""));
    log_info("  volume=%u\r\n",           json_get_uint(json, "\"volume\""));
    log_info("  led_mode=%u\r\n",         json_get_uint(json, "\"led_mode\""));
    log_info("  led_brightness=%u\r\n",   json_get_uint(json, "\"led_brightness\""));
    log_info("  led_scroll_speed=%u\r\n", json_get_uint(json, "\"led_scroll_speed\""));
    (void)json_get_str(json, "\"led_text\"", led_text, sizeof(led_text));
    (void)json_get_str(json, "\"tts_text\"", tts_text, sizeof(tts_text));
    log_info("  led_text=\"%s\"\r\n", led_text);
    log_info("  tts_text=\"%s\"\r\n", tts_text);
}

/* 1.3.1 ota_start: OTA 升级开始 */
static void handle_ota_start(const char *json)
{
    char fw_ver[32] = {0};

    log_info("  [ota_start]\r\n");
    (void)json_get_str(json, "\"firmware_version\"", fw_ver, sizeof(fw_ver));
    log_info("  firmware_version=%s\r\n", fw_ver);
    log_info("  pkg_total=%u\r\n",       json_get_uint(json, "\"pkg_total\""));
    log_info("  firmware_crc32=%u\r\n",  json_get_uint(json, "\"firmware_crc32\""));
}

/* 1.3.2 ota_data: OTA 分片数据 */
static void handle_ota_data(const char *json)
{
    /* pkg_data 是 base64 字符串, 可能很长, 只打印 index 和长度 */
    char pkg_data[16] = {0};   /* 只取前几个字符预览 */
    int data_len;

    log_info("  [ota_data]\r\n");
    log_info("  pkg_index=%u\r\n", json_get_uint(json, "\"pkg_index\""));
    data_len = json_get_str(json, "\"pkg_data\"", pkg_data, sizeof(pkg_data));
    log_info("  pkg_data_len=%d, preview=\"%s...\"\r\n", data_len, pkg_data);
}

/* 1.3.3 ota_end: OTA 升级结束 */
static void handle_ota_end(const char *json)
{
    (void)json;
    log_info("  [ota_end]\r\n");
}

/* 1.4 ack_ok: 应答 OK */
static void handle_ack_ok(const char *json)
{
    (void)json;
    log_info("  [ack_ok]\r\n");
}

/* 1.5 ack_err_retrans: 应答错误重传 */
static void handle_ack_err_retrans(const char *json)
{
    char err_msg[64] = {0};

    log_info("  [ack_err_retrans]\r\n");
    log_info("  err_code=%u\r\n", json_get_uint(json, "\"err_code\""));
    (void)json_get_str(json, "\"err_msg\"", err_msg, sizeof(err_msg));
    log_info("  err_msg=\"%s\"\r\n", err_msg);
}

/* ==================== 主入口 ==================== */

void json_proto_handle(const char *topic, const uint8_t *data, uint16_t len)
{
    char buf[512];
    uint16_t copy_len;
    char cmd[32] = {0};
    uint32_t dev_type, dev_no, seq;
    char device_id[32] = {0};

    copy_len = (len < (sizeof(buf) - 1u)) ? len : (uint16_t)(sizeof(buf) - 1u);
    memcpy(buf, data, copy_len);
    buf[copy_len] = '\0';

    /* 解析公共字段 */
    dev_type = json_get_uint(buf, "\"target_dev_type\"");
    dev_no   = json_get_uint(buf, "\"target_dev_no\"");
    seq      = json_get_uint(buf, "\"seq\"");
    (void)json_get_str(buf, "\"cmd\"", cmd, sizeof(cmd));
    (void)json_get_str(buf, "\"device_id\"", device_id, sizeof(device_id));

    log_info("\r\n[JSON] ======== 下行报文 ========\r\n");
    log_info("[JSON] topic=%s\r\n", topic);
    log_info("[JSON] device_id=%s type=%u no=%u seq=%u cmd=%s\r\n",
             device_id, dev_type, dev_no, seq, cmd);

    /* 按 cmd 分发 */
    if (strcmp(cmd, "set_sensor_param") == 0)
    {
        handle_set_sensor_param(buf);
    }
    else if (strcmp(cmd, "set_alarm_light") == 0)
    {
        handle_set_alarm_light(buf);
    }
    else if (strcmp(cmd, "ota_start") == 0)
    {
        handle_ota_start(buf);
    }
    else if (strcmp(cmd, "ota_data") == 0)
    {
        handle_ota_data(buf);
    }
    else if (strcmp(cmd, "ota_end") == 0)
    {
        handle_ota_end(buf);
    }
    else if (strcmp(cmd, "ack_ok") == 0)
    {
        handle_ack_ok(buf);
    }
    else if (strcmp(cmd, "ack_err_retrans") == 0)
    {
        handle_ack_err_retrans(buf);
    }
    else
    {
        log_info("[JSON] 未知 cmd=\"%s\"\r\n", cmd);
    }

    log_info("[JSON] ======== 解析完成 ========\r\n");
}
