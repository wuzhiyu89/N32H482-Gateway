/**
 * @file  config_serialize.c
 * @brief config_store 的序列化侧：9 个配置域 -> JSON（cJSON + 静态池）+ 诊断打印
 *
 * CRC 的覆盖范围是"全量配置不含 meta、且不含根对象结尾的 '}'"，所以打印两遍：
 * 第一遍拿长度算 CRC，第二遍把 meta 挂上再打。PrintPreallocated 不额外分配，
 * 所以"打两遍"只花时间、不花 RAM（详见 cfg_serialize() 上方的说明）。
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "cJSON.h"
#include "config_store.h"
#include "config_internal.h"
#include "utils.h"       /* crc32_ieee */
#include "log.h"

static char s_json_crc[CFG_JSON_MAX];            /* CRC 复算专用（与保存载荷分开） */

/* ==================== 序列化 ==================== */

const char *const cfg_measure_names[] = { "level", "distance" };
/* ★2026-10-08: 顺序必须与 cfg_led_color_t 一致, 也就是与【空口位定义】一致:
 *     00 无色 / 01 红 / 10 绿 / 11 黄
 *   cfg_get_enum() 是"按下标查名字、查到就返回下标", 所以这里的顺序【直接决定】
 *   JSON 字符串 -> 内部枚举的映射。改了枚举就必须同步改这一行, 否则红绿会反。 */
const char *const cfg_color_names[]   = { "off", "red", "green", "yellow" };
const char *const cfg_rulemode_names[]= { "auto", "manual" };
const char *const cfg_offline_names[] = { "keep", "alarm", "off" };
const char *const cfg_policy_names[]  = { "immediate", "idle" };

/** 写 system..ota(即"全量配置不含 meta"), 这一段正是 CRC32 的覆盖范围
 *
 *  【v33-2: 从自写 cjw_* 迁到 cJSON】
 *  ★ 键序 / 字段名 / 枚举名表【一个都不能动】: 序列化出来的字节必须与迁移前逐字节
 *    一致 —— 否则(1) Flash 里已存的 CRC 全部失效, 现场配置会被判成损坏;
 *    (2) 主机单测里那几条定长断言(n=1203 / n=1862 / n=3288)会立刻抓到差异。
 *  ★ 任何一步失败(池满)都返回 false, 由调用方把整棵树丢掉 —— 绝不发半条配置出去。 */
static bool cfg_write_domains(cJSON *root, const config_store_t *c)
{
    cJSON  *o;
    cJSON  *arr;
    uint8_t i, j;

    /* ---------------- system ---------------- */
    o = cJSON_CreateObject();
    if (o == NULL) { return false; }
    cJSON_AddItemToObject(root, "system", o);
    if ((cJSON_AddStringToObject(o, "gwId",     c->system.gw_id) == NULL) ||
        (cJSON_AddStringToObject(o, "password", c->system.password) == NULL) ||
        (cJSON_AddNumberToObject(o, "timezone", (double)c->system.timezone) == NULL))
    { return false; }

    /* ---------------- network ---------------- */
    o = cJSON_CreateObject();
    if (o == NULL) { return false; }
    cJSON_AddItemToObject(root, "network", o);
    if ((cJSON_AddStringToObject(o, "host", c->network.host) == NULL) ||
        (cJSON_AddNumberToObject(o, "port", (double)c->network.port) == NULL) ||
        (cJSON_AddBoolToObject(o, "tls", (c->network.tls != 0u) ? 1 : 0) == NULL) ||
        (cJSON_AddStringToObject(o, "clientId", c->network.client_id) == NULL) ||
        (cJSON_AddStringToObject(o, "username", c->network.username) == NULL) ||
        (cJSON_AddStringToObject(o, "password", c->network.mqtt_password) == NULL) ||
        (cJSON_AddNumberToObject(o, "keepAliveSeconds",
            (double)c->network.keep_alive_s) == NULL) ||
        (cJSON_AddNumberToObject(o, "reconnectMinSeconds",
            (double)c->network.reconnect_min_s) == NULL) ||
        (cJSON_AddNumberToObject(o, "reconnectMaxSeconds",
            (double)c->network.reconnect_max_s) == NULL) ||
        (cJSON_AddNumberToObject(o, "statusIntervalSeconds",
            (double)c->network.status_interval_s) == NULL))
    { return false; }

    /* ---------------- sensors[] ---------------- */
    arr = cJSON_CreateArray();
    if (arr == NULL) { return false; }
    cJSON_AddItemToObject(root, "sensors", arr);
    for (i = 0u; i < c->sensor_count && i < CFG_SENSOR_MAX; i++)
    {
        const cfg_sensor_t *s = &c->sensors[i];

        o = cJSON_CreateObject();
        if (o == NULL) { return false; }
        cJSON_AddItemToArray(arr, o);
        if ((cJSON_AddStringToObject(o, "uid", s->uid) == NULL) ||
            (cJSON_AddStringToObject(o, "serial", s->serial) == NULL) ||
            (cJSON_AddStringToObject(o, "name", s->name) == NULL) ||
            (cJSON_AddStringToObject(o, "measureMode",
                cfg_measure_names[(s->measure_mode < 2u) ? s->measure_mode : 0u]) == NULL) ||
            (cJSON_AddNumberToObject(o, "installHeightCm",
                (double)s->install_height_cm) == NULL) ||
            (cJSON_AddNumberToObject(o, "calibOffsetMm",
                (double)s->calib_offset_mm) == NULL) ||
            (cJSON_AddNumberToObject(o, "reportPeriodSeconds",
                (double)s->report_period_s) == NULL) ||
            (cJSON_AddNumberToObject(o, "lowBatteryPercent",
                (double)s->low_battery_pct) == NULL) ||
            (cJSON_AddNumberToObject(o, "offlineTimeoutSeconds",
                (double)s->offline_timeout_s) == NULL) ||
            (cJSON_AddBoolToObject(o, "enabled", (s->enabled != 0u) ? 1 : 0) == NULL))
        { return false; }
    }

    /* ---------------- thresholds ---------------- */
    o = cJSON_CreateObject();
    if (o == NULL) { return false; }
    cJSON_AddItemToObject(root, "thresholds", o);
    if ((cJSON_AddNumberToObject(o, "alarmCm",
            (double)c->thresholds.alarm_cm) == NULL) ||
        (cJSON_AddNumberToObject(o, "warningCm",
            (double)c->thresholds.warning_cm) == NULL) ||
        (cJSON_AddNumberToObject(o, "resolveCm",
            (double)c->thresholds.resolve_cm) == NULL) ||
        (cJSON_AddNumberToObject(o, "minChangeCm",
            (double)c->thresholds.min_change_cm) == NULL) ||
        (cJSON_AddNumberToObject(o, "eventMinIntervalSeconds",
            (double)c->thresholds.event_min_interval_s) == NULL) ||
        (cJSON_AddNumberToObject(o, "confirmCount",
            (double)c->thresholds.confirm_count) == NULL))
    { return false; }

    /* ---------------- gears[] ---------------- */
    arr = cJSON_CreateArray();
    if (arr == NULL) { return false; }
    cJSON_AddItemToObject(root, "gears", arr);
    for (i = 0u; i < CFG_GEAR_COUNT; i++)
    {
        const cfg_gear_t *g = &c->gears[i];

        o = cJSON_CreateObject();
        if (o == NULL) { return false; }
        cJSON_AddItemToArray(arr, o);
        if ((cJSON_AddNumberToObject(o, "gear", (double)g->gear) == NULL) ||
            (cJSON_AddStringToObject(o, "name", g->name) == NULL))
        { return false; }

        /* targets: "all" 或 [addr,...](文档 3.5) */
        if (g->targets_all != 0u)
        {
            if (cJSON_AddStringToObject(o, "targets", "all") == NULL) { return false; }
        }
        else
        {
            cJSON *ta = cJSON_CreateArray();

            if (ta == NULL) { return false; }
            cJSON_AddItemToObject(o, "targets", ta);
            for (j = 0u; j < g->target_count && j < CFG_DEVICE_MAX; j++)
            {
                /* CreateString 池满会返回 NULL, 此时 AddItemToArray 也返回 0 -> 整段失败 */
                if (cJSON_AddItemToArray(ta, cJSON_CreateString(g->target_uid[j])) == 0)
                { return false; }
            }
        }

        /* ★2026-10-08: 两个"双值"字段按 §1.9 回写成逗号分隔的【字符串】★
         *   ledTextShowMode: "1,2"       (两个半屏各一个 0/1)
         *   ledColor:        "red,yellow"
         * 【必须用字符串】: JSON 不允许裸的 `1,2`(那不是合法 JSON), 平台侧照抄会解析失败。 */
        {
            char colorbuf[24];
            char alignbuf[8];

            (void)snprintf(colorbuf, sizeof(colorbuf), "%s,%s",
                cfg_color_names[(g->led_color[0] <= (uint8_t)CFG_COLOR_YELLOW) ? g->led_color[0] : 0u],
                cfg_color_names[(g->led_color[1] <= (uint8_t)CFG_COLOR_YELLOW) ? g->led_color[1] : 0u]);
            (void)snprintf(alignbuf, sizeof(alignbuf), "%u,%u",
                (unsigned)g->led_text_show_mode[0], (unsigned)g->led_text_show_mode[1]);

            if ((cJSON_AddNumberToObject(o, "soundMode", (double)g->sound_mode) == NULL) ||
                (cJSON_AddNumberToObject(o, "ledShowMode", (double)g->led_show_mode) == NULL) ||
                (cJSON_AddStringToObject(o, "ledTextShowMode", alignbuf) == NULL) ||
                (cJSON_AddStringToObject(o, "ledColor", colorbuf) == NULL) ||
                (cJSON_AddStringToObject(o, "ledTextTemplate1", g->led_text1) == NULL) ||
                (cJSON_AddStringToObject(o, "ledTextTemplate2", g->led_text2) == NULL) ||
                (cJSON_AddStringToObject(o, "ttsTextTemplate", g->tts_text) == NULL))
            { return false; }
        }
    }

    /* ---------------- devices[] ---------------- */
    arr = cJSON_CreateArray();
    if (arr == NULL) { return false; }
    cJSON_AddItemToObject(root, "devices", arr);
    for (i = 0u; i < c->device_count && i < CFG_DEVICE_MAX; i++)
    {
        const cfg_device_t *d = &c->devices[i];

        o = cJSON_CreateObject();
        if (o == NULL) { return false; }
        cJSON_AddItemToArray(arr, o);
        if ((cJSON_AddStringToObject(o, "uid", d->uid) == NULL) ||
            (cJSON_AddStringToObject(o, "serial", d->serial) == NULL) ||
            (cJSON_AddStringToObject(o, "name", d->name) == NULL) ||
            (cJSON_AddNumberToObject(o, "type", (double)d->type) == NULL) ||
            (cJSON_AddNumberToObject(o, "volume", (double)d->volume) == NULL) ||
            (cJSON_AddNumberToObject(o, "flashMode", (double)d->flash_mode) == NULL) ||
            (cJSON_AddBoolToObject(o, "enabled", (d->enabled != 0u) ? 1 : 0) == NULL))
        { return false; }
    }

    /* ---------------- rules ---------------- */
    o = cJSON_CreateObject();
    if (o == NULL) { return false; }
    cJSON_AddItemToObject(root, "rules", o);
    if ((cJSON_AddStringToObject(o, "mode",
            cfg_rulemode_names[(c->rules.mode <= CFG_RULE_MANUAL) ? c->rules.mode : 0u]) == NULL) ||
        (cJSON_AddNumberToObject(o, "manualTimeoutSeconds",
            (double)c->rules.manual_timeout_s) == NULL) ||
        (cJSON_AddStringToObject(o, "sensorOfflinePolicy",
            cfg_offline_names[(c->rules.sensor_offline_policy <= CFG_OFFLINE_OFF)
                            ? c->rules.sensor_offline_policy : 0u]) == NULL))
    { return false; }

    /* ---------------- ota ---------------- */
    o = cJSON_CreateObject();
    if (o == NULL) { return false; }
    cJSON_AddItemToObject(root, "ota", o);
    if ((cJSON_AddStringToObject(o, "fwVersion", c->ota.fw_version) == NULL) ||
        (cJSON_AddStringToObject(o, "upgradePolicy",
            cfg_policy_names[(c->ota.upgrade_policy <= CFG_OTA_IDLE)
                           ? c->ota.upgrade_policy : 0u]) == NULL))
    { return false; }

    return true;
}

/** 写 meta(CRC 由调用方传入, 因为 CRC 覆盖的是它前面那一段) */
static bool cfg_write_meta(cJSON *obj, const config_store_t *c, uint32_t crc)
{
    cJSON *o;
    char   hex[9];

    (void)snprintf(hex, sizeof(hex), "%08lX", (unsigned long)crc);

    o = cJSON_CreateObject();
    if (o == NULL) { return false; }
    cJSON_AddItemToObject(obj, "meta", o);

    return ((cJSON_AddNumberToObject(o, "configVersion",
                (double)c->meta.config_version) != NULL) &&
            (cJSON_AddStringToObject(o, "crc32", hex) != NULL) &&  /* 文档: HEX 大写字符串 */
            (cJSON_AddNumberToObject(o, "updatedAt",
                (double)c->meta.updated_at) != NULL));
}

/**
 * @brief 序列化并回填 CRC
 * @param crc_out 可选, 输出算出的 CRC(供 bank 头部使用, 免得再算一遍)
 *
 * 【v33-2 迁到 cJSON 后为什么是"打两遍"】
 *   CRC 的覆盖范围是"全量配置不含 meta, 且【不含】根对象结尾的 '}'"
 *   ——也就是旧实现里"根对象还没闭合"时那一段。cJSON 是整树打印, 关不关根不由我们
 *   控制, 所以:
 *     第一遍: 只建 system..ota 的树 -> PrintPreallocated 拿到长度 n
 *             -> meta_start = n - 1(去掉 cJSON 自己补的那个 '}')
 *             -> crc = cfg_crc32(out, meta_start)
 *   这样 CRC 的输入与迁移前【逐字节一致】, Flash 里已存的 CRC 才不会集体失效。
 *     第二遍: 把 meta 挂到同一棵树上再打一遍(PrintPreallocated 直接覆盖 out)。
 *   PrintPreallocated 不需要额外缓冲, 所以"打两遍"只花时间、不花 RAM。
 */
uint16_t cfg_serialize(const config_store_t *c, char *out, uint16_t cap,
                              uint32_t *crc_out)
{
    cJSON   *root;
    uint16_t meta_start;
    uint32_t crc;
    uint16_t n;

    if ((c == NULL) || (out == NULL) || (cap < 64u)) { return 0u; }

    root = cJSON_CreateObject();
    if (root == NULL) { return 0u; }
    if (!cfg_write_domains(root, c))
    {
        cJSON_Delete(root);
        return 0u;
    }

    if (cJSON_PrintPreallocated(root, out, (int)cap, 0) == 0)
    {
        cJSON_Delete(root);
        out[0] = '\0';
        return 0u;
    }
    n = (uint16_t)strlen(out);
    if (n < 2u)
    {
        cJSON_Delete(root);
        out[0] = '\0';
        return 0u;
    }
    meta_start = (uint16_t)(n - 1u);        /* 去掉结尾的 '}' —— 与旧实现同一覆盖范围 */
    crc        = crc32_ieee((const uint8_t *)out, (uint32_t)meta_start);

    if (!cfg_write_meta(root, c, crc))
    {
        cJSON_Delete(root);
        out[0] = '\0';
        return 0u;
    }
    if (cJSON_PrintPreallocated(root, out, (int)cap, 0) == 0)
    {
        cJSON_Delete(root);
        out[0] = '\0';
        return 0u;
    }
    cJSON_Delete(root);

    if (crc_out != NULL) { *crc_out = crc; }
    return (uint16_t)strlen(out);
}

uint16_t config_store_to_json(const config_store_t *c, char *out, uint16_t cap)
{
    return cfg_serialize(c, out, cap, NULL);
}

uint32_t config_store_calc_crc(const config_store_t *c)
{
    uint32_t crc = 0u;
    uint16_t n;

    /* 走专用缓冲 s_json_crc: 不能碰调用方(可能是 save 的载荷缓冲) */
    n = cfg_serialize(c, s_json_crc, (uint16_t)sizeof(s_json_crc), &crc);
    return (n == 0u) ? 0u : crc;
}

/* ==================== cfg.query 用: 把 9 个域写进已开好的应答 data ====================
 *
 * 【为什么不复用 config_store_to_json()】那个函数会自己建【一整棵以根对象为顶的树】
 * 并打印出一个完整的顶层对象; 而 cfg.query 的应答 data 只是信封里的一个成员:
 *     {"cmd":"cfg.query",...,"data":{ ...配置对象... }}
 * 若把它当"字符串值"塞进 data, 会得到 "data":"{...}" 或 "data":{ {...} } ——
 * 形状不对(这正是任务清单里标注的"数据形状冲突"之一)。所以这里只往调用方
 * 已经开好的对象里挂成员, 由调用方决定外层容器。
 *
 * 【crc32 必须重算】文档 meta.crc32 = "全量配置(不含 meta)的 CRC32"。这里写出来的
 * 就是当前内存配置, 所以直接调 config_store_calc_crc() —— 与存 Flash 时用的是
 * 同一个函数, 保证"cfg.query 看到的 CRC" == "Flash 里存的 CRC"。
 */
bool config_store_write_members(cJSON *obj, const config_store_t *c)
{
    uint32_t crc;

    if ((obj == NULL) || (c == NULL)) { return false; }

    if (!cfg_write_domains(obj, c)) { return false; }        /* system .. ota */
    crc = config_store_calc_crc(c);
    return cfg_write_meta(obj, c, crc);                     /* meta(configVersion/crc32/updatedAt) */
}

/* ==================== 诊断 ==================== */

void config_store_dump(const config_store_t *c)
{
    if (c == NULL) { return; }

    log_info("[CFG] ver=%lu gwId=%s tz=%d host=%s:%u\r\n",
             (unsigned long)c->meta.config_version,
             c->system.gw_id, (int)c->system.timezone,
             (c->network.host[0] != '\0') ? c->network.host : "(空)",
             (unsigned)c->network.port);
    log_info("[CFG] 传感器=%u 联动设备=%u | 阈值 告警%u/预警%u/解除%u confirm=%u\r\n",
             (unsigned)c->sensor_count, (unsigned)c->device_count,
             (unsigned)c->thresholds.alarm_cm,
             (unsigned)c->thresholds.warning_cm,
             (unsigned)c->thresholds.resolve_cm,
             (unsigned)c->thresholds.confirm_count);
    log_info("[CFG] rules=%s 手动超时=%us 传感器失联策略=%s | ota=%s 固件=%s\r\n",
             cfg_rulemode_names[(c->rules.mode <= CFG_RULE_MANUAL) ? c->rules.mode : 0u],
             (unsigned)c->rules.manual_timeout_s,
             cfg_offline_names[(c->rules.sensor_offline_policy <= CFG_OFFLINE_OFF)
                             ? c->rules.sensor_offline_policy : 0u],
             cfg_policy_names[(c->ota.upgrade_policy <= CFG_OTA_IDLE)
                            ? c->ota.upgrade_policy : 0u],
             c->ota.fw_version);
}
