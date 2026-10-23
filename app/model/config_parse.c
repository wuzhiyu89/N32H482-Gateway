/**
 * @file  config_parse.c
 * @brief config_store 的解析侧：JSON 取值助手 + 9 个配置域的解析 / 逐域增量合并
 *
 * 解析一律走 cloud_json 的零堆接口（cj_get_* / cfg_get_*），只读调用方的缓冲。
 * 事务语义：
 *   · config_store_merge_json()     全局事务——任一域失败即整体回滚（上电加载用）；
 *   · config_store_merge_sections() 逐域事务——每个域单独 copy/validate/提交（cfg.set 用）。
 * 两者都用静态副本 s_merge_tmp：配置结构体约 1.7 KB，4 KB 的栈放不下第二份。
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>      /* strtoul */

#include "config_store.h"
#include "config_internal.h"
#include "cloud_json.h"
#include "log.h"

static config_store_t s_merge_tmp;               /* 增量合并的事务副本 */

/* ==================== 取值辅助 ====================
 * 三个结果, 必须区分:
 *   缺失   -> 增量合并时"不动这个字段"(保持现状); 全量解析时"回落出厂默认"
 *   越界   -> 直接报错, 不能悄悄截断(截断会让平台以为设置成功了) */

#define CFG_GET_MISSING   0
#define CFG_GET_OK        1
#define CFG_GET_RANGE    (-1)

static int cfg_get_u32(const char *json, const char *path,
                       uint32_t *dst, uint32_t lo, uint32_t hi)
{
    cj_type_t t = cj_get(json, path, NULL, 0);
    uint32_t  v;

    if (t == CJ_NOTFOUND) { return CFG_GET_MISSING; }
    if ((t != CJ_NUM) || (t == CJ_NULL)) { return CFG_GET_RANGE; }

    v = cj_get_uint(json, path, 0xFFFFFFFFu);
    if ((v < lo) || (v > hi)) { return CFG_GET_RANGE; }

    *dst = v;
    return CFG_GET_OK;
}

static int cfg_get_u16(const char *json, const char *path,
                       uint16_t *dst, uint16_t lo, uint16_t hi)
{
    uint32_t v = 0u;
    int      r = cfg_get_u32(json, path, &v, lo, hi);

    if (r == CFG_GET_OK) { *dst = (uint16_t)v; }
    return r;
}

static int cfg_get_u8(const char *json, const char *path,
                      uint8_t *dst, uint8_t lo, uint8_t hi)
{
    uint32_t v = 0u;
    int      r = cfg_get_u32(json, path, &v, lo, hi);

    if (r == CFG_GET_OK) { *dst = (uint8_t)v; }
    return r;
}

static int cfg_get_i16(const char *json, const char *path,
                       int16_t *dst, int32_t lo, int32_t hi)
{
    cj_type_t t = cj_get(json, path, NULL, 0);
    int32_t   v;

    if (t == CJ_NOTFOUND) { return CFG_GET_MISSING; }
    if ((t != CJ_NUM) || (t == CJ_NULL)) { return CFG_GET_RANGE; }

    v = cj_get_int(json, path, 0);
    if ((v < lo) || (v > hi)) { return CFG_GET_RANGE; }

    *dst = (int16_t)v;
    return CFG_GET_OK;
}

static int cfg_get_i8(const char *json, const char *path,
                      int8_t *dst, int32_t lo, int32_t hi)
{
    int16_t v = 0;
    int     r = cfg_get_i16(json, path, &v, lo, hi);

    if (r == CFG_GET_OK) { *dst = (int8_t)v; }
    return r;
}

static int cfg_get_bool(const char *json, const char *path, uint8_t *dst)
{
    cj_type_t t = cj_get(json, path, NULL, 0);

    if (t == CJ_NOTFOUND) { return CFG_GET_MISSING; }
    if (t == CJ_NULL)     { return CFG_GET_RANGE; }

    *dst = cj_get_bool(json, path, false) ? 1u : 0u;
    return CFG_GET_OK;
}

static int cfg_get_str(const char *json, const char *path, char *dst, uint16_t max)
{
    /* ★2026-10-08 修正★ 原来这里是 `CFG_LEDTEXT_MAX + 8`。
     *   以前 LED 和 TTS 上限都是 64, 所以"按 LED 的尺寸开缓冲"刚好够用;
     *   现在 TTS 升到 88、LED 降到 48 —— 再沿用 LED 就会**把 88 字节的语音文本
     *   在 56 字节处截断**, 而且 cfg_get_str 返回的还是 OK(它只按 max 判),
     *   表现为"平台配了长语音, 设备只念半句", 极难查。
     *   ⇒ 一律按【最大的那个字符串字段】开缓冲。 */
    char  tmp[CFG_TTSTEXT_MAX + 8];
    size_t n;

    if (cj_get(json, path, NULL, 0) == CJ_NOTFOUND) { return CFG_GET_MISSING; }
    if (!cj_get_str(json, path, tmp, (uint16_t)sizeof(tmp))) { return CFG_GET_RANGE; }

    n = strlen(tmp);
    if (n > (size_t)max) { return CFG_GET_RANGE; }   /* 文档给了长度上限, 超了要报错 */

    (void)memcpy(dst, tmp, n + 1u);
    return CFG_GET_OK;
}

/** 枚举字符串 -> 序号; 缺失/不匹配分别返回 MISSING/RANGE */
static int cfg_get_enum(const char *json, const char *path,
                        const char *const *names, uint8_t n, uint8_t *dst)
{
    char tmp[24];
    uint8_t i;

    if (cj_get(json, path, NULL, 0) == CJ_NOTFOUND) { return CFG_GET_MISSING; }
    if (!cj_get_str(json, path, tmp, (uint16_t)sizeof(tmp))) { return CFG_GET_RANGE; }

    for (i = 0u; i < n; i++)
    {
        if (strcmp(tmp, names[i]) == 0) { *dst = i; return CFG_GET_OK; }
    }
    return CFG_GET_RANGE;
}

/* ★★★2026-10-08 新增: 两个"双值"字段的解析 ★★★
 *
 * §1.9 把 `ledColor` 和 `ledTextShowMode` 都改成了【逗号分隔的两个值】,
 * 而且明确要求写成【字符串】(JSON 不允许裸的 `1,2`)。
 * 下面两个 helper 各自要求"必须恰好一个逗号 + 两段都合法":
 *   · 少一个逗号 / 多一个逗号 / 任一段非法  -> 一律 CFG_GET_RANGE
 *   · 于是 cfg.set 会【整域拒掉 gears】(与其它字段错误的语义一致, 不半信半疑)
 * 缺失(字段整个没给)仍然返回 MISSING = 保留原值, 与所有 cfg_get_* 一致。 */

/** 解析 "a,b" 形式的两个枚举名(如 ledColor: "red,yellow") */
static int cfg_get_enum_pair(const char *json, const char *path,
                             const char *const *names, uint8_t n, uint8_t *a, uint8_t *b)
{
    char  tmp[24];
    char *comma;
    uint8_t i;

    if (cj_get(json, path, NULL, 0) == CJ_NOTFOUND) { return CFG_GET_MISSING; }
    if (!cj_get_str(json, path, tmp, (uint16_t)sizeof(tmp))) { return CFG_GET_RANGE; }

    comma = strchr(tmp, ',');
    if ((comma == NULL) || (strchr(comma + 1, ',') != NULL)) { return CFG_GET_RANGE; }
    *comma = '\0';

    for (i = 0u; i < n; i++) { if (strcmp(tmp, names[i]) == 0) { *a = i; break; } }
    if (i >= n) { return CFG_GET_RANGE; }

    for (i = 0u; i < n; i++) { if (strcmp(comma + 1, names[i]) == 0) { *b = i; break; } }
    if (i >= n) { return CFG_GET_RANGE; }

    return CFG_GET_OK;
}

/** 解析 "a,b" 形式的两个 0~max 十进制数字(如 ledTextShowMode: "1,2") */
static int cfg_get_u8_pair(const char *json, const char *path, uint8_t max,
                           uint8_t *a, uint8_t *b)
{
    char          tmp[24];
    char         *comma;
    char         *end;
    unsigned long v;

    if (cj_get(json, path, NULL, 0) == CJ_NOTFOUND) { return CFG_GET_MISSING; }
    if (!cj_get_str(json, path, tmp, (uint16_t)sizeof(tmp))) { return CFG_GET_RANGE; }

    comma = strchr(tmp, ',');
    if ((comma == NULL) || (strchr(comma + 1, ',') != NULL)) { return CFG_GET_RANGE; }
    *comma = '\0';

    /* 两段都必须是"纯十进制数字且不超上界" —— strtoul 会把 "1x" 解析成 1,
     * 所以必须检查 end 指向结尾, 否则 "1x,2" 会被悄悄接受。 */
    v = strtoul(tmp, &end, 10);
    if ((tmp[0] == '\0') || (*end != '\0') || (v > (unsigned long)max)) { return CFG_GET_RANGE; }
    *a = (uint8_t)v;

    v = strtoul(comma + 1, &end, 10);
    if ((comma[1] == '\0') || (*end != '\0') || (v > (unsigned long)max)) { return CFG_GET_RANGE; }
    *b = (uint8_t)v;

    return CFG_GET_OK;
}

/* ==================== 反序列化(按域) ==================== */

#define CFG_RET_ON_RANGE(r)  do { if ((r) == CFG_GET_RANGE) { return CFG_ERR_RANGE; } } while (0)

/* 字符串超长必须报错, 不能当"没给"悄悄放过 —— 否则平台会以为设置成功了。
 * 文档每个字符串字段都标了长度上限(如 gwId ≤20 字节、ledTextTemplate ≤64)。 */
#define CFG_STR_ON_RANGE(json, path, dst, max)                            \
    do {                                                                  \
        if (cfg_get_str((json), (path), (dst), (max)) == CFG_GET_RANGE)   \
        {                                                                 \
            return CFG_ERR_RANGE;                                         \
        }                                                                 \
    } while (0)

/* 数组元素的缺省值(文档 3.3 / 3.6 的"默认"列)。
 * 必须在解析【之前】铺好: 数组元素是整体替换的, 不铺就是 0, 而
 * reportPeriodSeconds=0 连校验都过不去(文档要求 5~3600)。 */
static void cfg_sensor_defaults(cfg_sensor_t *s)
{
    (void)memset(s, 0, sizeof(*s));
    s->measure_mode      = CFG_MEASURE_LEVEL;   /* level */
    s->report_period_s   = 300u;                /* 与绑定默认一致 */
    s->offline_timeout_s = 900u;                /* 3 倍上报周期 */
    s->low_battery_pct   = 80u;                 /* 电量 ≤80% 上报 low_voltage */
    s->enabled           = 1u;
}

static void cfg_device_defaults(cfg_device_t *d)
{
    (void)memset(d, 0, sizeof(*d));
    d->volume     = 50u;
    d->flash_mode = 1u;
    d->enabled    = 1u;
}

static cfg_result_t cfg_parse_system(const char *json, cfg_system_t *s, bool *present)
{
    int r;

    if (cj_get(json, "system", NULL, 0) != CJ_OBJ) { *present = false; return CFG_OK; }
    *present = true;

    CFG_STR_ON_RANGE(json, "system.gwId",     s->gw_id,    CFG_GWID_MAX);
    CFG_STR_ON_RANGE(json, "system.password", s->password, CFG_SYSPWD_MAX);
    r = cfg_get_i8(json, "system.timezone", &s->timezone, -12, 14);
    CFG_RET_ON_RANGE(r);

    return CFG_OK;
}

static cfg_result_t cfg_parse_network(const char *json, cfg_network_t *n, bool *present)
{
    int r;
    uint8_t b = 0u;

    if (cj_get(json, "network", NULL, 0) != CJ_OBJ) { *present = false; return CFG_OK; }
    *present = true;

    CFG_STR_ON_RANGE(json, "network.host",     n->host,      CFG_HOST_MAX);
    CFG_STR_ON_RANGE(json, "network.clientId", n->client_id, CFG_CLIENTID_MAX);
    CFG_STR_ON_RANGE(json, "network.username", n->username,  CFG_MQTTUSER_MAX);

    /* ★★2026-10-09: password 的取值要挡掉掩码值★★
     * cfg.query 现在把 network.password 固定返回 "*****"(见 cloud_proto 的
     * cloud_cmd_cfg_query)。平台若把整份配置【读回来再原样写回】(很常见的做法),
     * 就会把真密码改成 5 个星号 —— 之后 MQTT 再也连不上, 而且现场极难看出原因。
     * ⇒ 恰为 "*****" 时【当作"不修改"跳过】, 保留原值。
     * ⚠️ 代价: 真想把密码设成 "*****" 就做不到 —— 这是可接受的取舍(没人会这么设)。
     * 其余情况照常覆盖(所以设空串 "" = 清空密码, 仍然可用)。 */
    {
        /* 缓冲要比 CFG_MQTTPWD_MAX 大 1 —— cfg_get_str 在 n == max 时会
         * memcpy(dst, tmp, n+1)(连结尾 NUL), 刚好 max 字节会溢出 1 字节。 */
        char pwd[CFG_MQTTPWD_MAX + 2];
        int  pr;

        pr = cfg_get_str(json, "network.password", pwd, CFG_MQTTPWD_MAX);
        if (pr == CFG_GET_RANGE)
        {
            /* 超长: 照旧【拒绝整条命令】(与原 CFG_STR_ON_RANGE 的行为一致),
             * 不要在这里静默跳过 —— 那会让平台以为设置成功了。 */
            return CFG_ERR_RANGE;
        }
        if ((pr == CFG_GET_OK) && (strcmp(pwd, "*****") != 0))
        {
            (void)strncpy(n->mqtt_password, pwd, sizeof(n->mqtt_password) - 1u);
            n->mqtt_password[sizeof(n->mqtt_password) - 1u] = '\0';
        }
        /* pr == CFG_GET_MISSING(没带这个字段) 或 恰为 "*****"(掩码回传)
         * -> 都【不动】 n->mqtt_password, 保留原密码。 */
    }

    r = cfg_get_u16(json, "network.port", &n->port, 1u, 65535u);              CFG_RET_ON_RANGE(r);
    r = cfg_get_u16(json, "network.keepAliveSeconds", &n->keep_alive_s, 30u, 300u);
    CFG_RET_ON_RANGE(r);
    r = cfg_get_u16(json, "network.reconnectMinSeconds", &n->reconnect_min_s, 1u, 65535u);
    CFG_RET_ON_RANGE(r);
    r = cfg_get_u16(json, "network.reconnectMaxSeconds", &n->reconnect_max_s, 1u, 65535u);
    CFG_RET_ON_RANGE(r);
    r = cfg_get_u16(json, "network.statusIntervalSeconds", &n->status_interval_s, 10u, 600u);
    CFG_RET_ON_RANGE(r);

    r = cfg_get_bool(json, "network.tls", &b);
    CFG_RET_ON_RANGE(r);
    if (r == CFG_GET_OK) { n->tls = b; }

    return CFG_OK;
}

static cfg_result_t cfg_parse_sensors(const char *json, config_store_t *c, bool *present,
                                      bool merge)
{
    cj_type_t t;
    uint16_t cnt;
    uint16_t i;
    uint8_t  slot;

    t = cj_get(json, "sensors", NULL, 0);
    if (t == CJ_NOTFOUND) { *present = false; return CFG_OK; }
    /* 出现了就先把 present 置上: 逐域合并要靠它区分"没这个域"和"这个域坏了",
     * 后者必须计入 rejected 而不是被当成"没提到"悄悄跳过。 */
    *present = true;
    if (t != CJ_ARR) { return CFG_ERR_JSON; }

    cnt = cj_count(json, "sensors");
    if (cnt > CFG_SENSOR_MAX) { return CFG_ERR_RANGE; }

    if (!merge)
    {
        /* 全量语义: 整数组替换(存进去的就是完整数组) */
        (void)memset(c->sensors, 0, sizeof(c->sensors));
        c->sensor_count = (uint8_t)cnt;
        for (slot = 0u; slot < c->sensor_count; slot++)
        {
            cfg_sensor_defaults(&c->sensors[slot]);
        }
    }

    for (i = 0u; i < cnt; i++)
    {
        cfg_sensor_t *s;
        char    p[40];
        char    want_uid[CFG_UID_STR_MAX];
        int     r;
        uint8_t e = 0u;
        uint8_t k;
        bool    found = false;

        /* ---- 先取 uid, 用它决定"写到哪个槽" ----
         * 缺 uid / 格式非法 -> 拒该域: 没有地址就无法定位改的是哪一台
         * (新增设备应走 sen.bind / dev.bind, 不该从 cfg.set 混进来) */
        (void)snprintf(p, sizeof(p), "sensors[%u].uid", (unsigned)i);
        if (cfg_get_str(json, p, want_uid, CFG_UID_HEX_LEN) != CFG_GET_OK) { return CFG_ERR_RANGE; }
        if (!cfg_uid_valid(want_uid)) { return CFG_ERR_RANGE; }

        if (merge)
        {
            for (k = 0u; k < c->sensor_count && k < CFG_SENSOR_MAX; k++)
            {
                if (strcmp(c->sensors[k].uid, want_uid) == 0) { found = true; break; }
            }
            if (found)
            {
                slot = k;                       /* 命中: 只覆盖来包里出现过的字段 */
            }
            else
            {
                if (c->sensor_count >= CFG_SENSOR_MAX) { return CFG_ERR_RANGE; }
                slot = c->sensor_count;         /* 未命中: 追加, 先铺文档默认值 */
                cfg_sensor_defaults(&c->sensors[slot]);
                c->sensor_count++;
            }
        }
        else
        {
            slot = (uint8_t)i;
        }

        s = &c->sensors[slot];
        (void)strcpy(s->uid, want_uid);

        (void)snprintf(p, sizeof(p), "sensors[%u].serial", (unsigned)i);
        CFG_STR_ON_RANGE(json, p, s->serial, CFG_SERIAL_MAX);
        (void)snprintf(p, sizeof(p), "sensors[%u].name", (unsigned)i);
        CFG_STR_ON_RANGE(json, p, s->name, CFG_NAME_MAX);

        (void)snprintf(p, sizeof(p), "sensors[%u].measureMode", (unsigned)i);
        r = cfg_get_enum(json, p, cfg_measure_names, 2u, &e);    CFG_RET_ON_RANGE(r);
        if (r == CFG_GET_OK) { s->measure_mode = e; }

        (void)snprintf(p, sizeof(p), "sensors[%u].installHeightCm", (unsigned)i);
        r = cfg_get_u16(json, p, &s->install_height_cm, 0u, 2000u);  CFG_RET_ON_RANGE(r);
        (void)snprintf(p, sizeof(p), "sensors[%u].calibOffsetMm", (unsigned)i);
        r = cfg_get_i16(json, p, &s->calib_offset_mm, -5000, 5000);  CFG_RET_ON_RANGE(r);
        (void)snprintf(p, sizeof(p), "sensors[%u].reportPeriodSeconds", (unsigned)i);
        r = cfg_get_u16(json, p, &s->report_period_s, 5u, 3600u);    CFG_RET_ON_RANGE(r);
        (void)snprintf(p, sizeof(p), "sensors[%u].lowBatteryPercent", (unsigned)i);
        r = cfg_get_u8(json, p, &s->low_battery_pct, 0u, 100u);      CFG_RET_ON_RANGE(r);
        (void)snprintf(p, sizeof(p), "sensors[%u].offlineTimeoutSeconds", (unsigned)i);
        r = cfg_get_u16(json, p, &s->offline_timeout_s, 0u, 65535u); CFG_RET_ON_RANGE(r);

        (void)snprintf(p, sizeof(p), "sensors[%u].enabled", (unsigned)i);
        r = cfg_get_bool(json, p, &e);                               CFG_RET_ON_RANGE(r);
        if (r == CFG_GET_OK) { s->enabled = e; }
    }

    return CFG_OK;
}

static cfg_result_t cfg_parse_thresholds(const char *json, cfg_thresholds_t *t, bool *present)
{
    int r;
    uint8_t b = 0u;

    if (cj_get(json, "thresholds", NULL, 0) != CJ_OBJ) { *present = false; return CFG_OK; }
    *present = true;

    r = cfg_get_u16(json, "thresholds.alarmCm",   &t->alarm_cm,   0u, 65535u); CFG_RET_ON_RANGE(r);
    r = cfg_get_u16(json, "thresholds.warningCm", &t->warning_cm, 0u, 65535u); CFG_RET_ON_RANGE(r);
    r = cfg_get_u16(json, "thresholds.resolveCm", &t->resolve_cm, 0u, 65535u); CFG_RET_ON_RANGE(r);
    r = cfg_get_u16(json, "thresholds.minChangeCm", &t->min_change_cm, 0u, 65535u);
    CFG_RET_ON_RANGE(r);
    r = cfg_get_u16(json, "thresholds.eventMinIntervalSeconds", &t->event_min_interval_s,
                    0u, 65535u);
    CFG_RET_ON_RANGE(r);
    r = cfg_get_u8(json, "thresholds.confirmCount", &b, 1u, 10u);              CFG_RET_ON_RANGE(r);
    if (r == CFG_GET_OK) { t->confirm_count = b; }

    return CFG_OK;
}

/* ==================== gears[]: 按 【gear 字段】 定位槽位 ====================
 *
 * 【为什么不能按下标】来包里 `gears` 数组的下标与档位号【没有关系】:
 * 平台只想改 2 号档时下发的是 `{"gears":[{"gear":2,...}]}` —— 数组下标 0。
 * 若按下标写, 就会把 2 号档的内容写到 0 号档槽里; 而 validate 要求
 * gears[i].gear == i, 于是要么被拒(改不动), 要么把 gear 也改成 0(串档)。
 * 所以: 一律先读该元素的 `gear`, 再用它决定写哪个槽。
 *
 * 【gear 缺失/越界 -> CFG_ERR_RANGE】没有档位号就无法定位改的是哪一档。
 * 这条对"全量加载"和"增量合并"同样成立, 所以 gears 只需要这一个函数。
 *
 * 【槽位不变量】由于写入时 gear 与槽位索引始终一致, validate 里
 * `gears[i].gear == i` 这条检查天然成立; 全量加载时未出现在数组里的档位
 * 保持出厂默认(from_json 会先铺默认值), 不会出现半截档位。
 */
static cfg_result_t cfg_parse_gears(const char *json, cfg_gear_t *gears, bool *present)
{
    cj_type_t t;
    uint16_t cnt;
    uint8_t  i, j;
    int      r;

    t = cj_get(json, "gears", NULL, 0);
    if (t == CJ_NOTFOUND) { *present = false; return CFG_OK; }
    *present = true;
    if (t != CJ_ARR) { return CFG_ERR_JSON; }

    cnt = cj_count(json, "gears");
    if (cnt > CFG_GEAR_COUNT) { return CFG_ERR_RANGE; }

    for (i = 0u; i < (uint8_t)cnt; i++)
    {
        cfg_gear_t *g;
        char p[48];
        uint8_t e = 0u;
        uint8_t gg = 0u;

        /* ---- 先取 gear, 由它决定槽位 ----
         * CFG_RET_ON_RANGE 管"越界"; 这里还要单独管"缺失"(MISSING) ——
         * 两者都不能放行, 但原因不同, 所以合并成 r != CFG_GET_OK。 */
        (void)snprintf(p, sizeof(p), "gears[%u].gear", (unsigned)i);
        r = cfg_get_u8(json, p, &gg, 0u, 3u);
        CFG_RET_ON_RANGE(r);
        if (r != CFG_GET_OK) { return CFG_ERR_RANGE; }      /* 缺 gear: 无法定位 */

        g       = &gears[gg];                               /* ← 按档位号, 不是下标 i */
        g->gear = gg;

        (void)snprintf(p, sizeof(p), "gears[%u].name", (unsigned)i);
        CFG_STR_ON_RANGE(json, p, g->name, CFG_NAME_MAX);
        (void)snprintf(p, sizeof(p), "gears[%u].soundMode", (unsigned)i);
        /* ★2026-10-08: 上界 2 -> 3★ 0关闭/1常亮/2慢闪/3快闪 */
        r = cfg_get_u8(json, p, &e, 0u, 3u);             CFG_RET_ON_RANGE(r);
        if (r == CFG_GET_OK) { g->sound_mode = e; }

        (void)snprintf(p, sizeof(p), "gears[%u].ledShowMode", (unsigned)i);
        r = cfg_get_u8(json, p, &e, 0u, 1u);             CFG_RET_ON_RANGE(r);
        if (r == CFG_GET_OK) { g->led_show_mode = e; }   /* 0横 / 1竖 */

        /* ---- ★两个"双值"字段: 必须是 "a,b" 字符串★ ---- */
        (void)snprintf(p, sizeof(p), "gears[%u].ledTextShowMode", (unsigned)i);
        {
            uint8_t a = g->led_text_show_mode[0];
            uint8_t b = g->led_text_show_mode[1];
            /* ★上界是 3, 不是 1★ 云端 JSON 口径是 §1.9 的 0~3
             *   (0默认/1上对齐|左对齐/2居中/3下对齐|右对齐), 例如 "1,2"。
             *   空口上只有 1 位/半屏, 由 level_eval.c 的 led_align_bit() 折算 —— 
             *   【校验必须按 JSON 口径收 0~3】, 否则 "1,2" 会被这里误拒成 1001。 */
            r = cfg_get_u8_pair(json, p, 3u, &a, &b);    CFG_RET_ON_RANGE(r);
            if (r == CFG_GET_OK) { g->led_text_show_mode[0] = a; g->led_text_show_mode[1] = b; }
        }

        (void)snprintf(p, sizeof(p), "gears[%u].ledColor", (unsigned)i);
        {
            uint8_t a = g->led_color[0];
            uint8_t b = g->led_color[1];
            r = cfg_get_enum_pair(json, p, cfg_color_names, 4u, &a, &b); CFG_RET_ON_RANGE(r);
            if (r == CFG_GET_OK) { g->led_color[0] = a; g->led_color[1] = b; }
        }

        /* ★ledTextTemplate 拆成 1/2★ (原来是单字段 ledTextTemplate) */
        (void)snprintf(p, sizeof(p), "gears[%u].ledTextTemplate1", (unsigned)i);
        CFG_STR_ON_RANGE(json, p, g->led_text1, CFG_LEDTEXT_MAX);
        (void)snprintf(p, sizeof(p), "gears[%u].ledTextTemplate2", (unsigned)i);
        CFG_STR_ON_RANGE(json, p, g->led_text2, CFG_LEDTEXT_MAX);
        (void)snprintf(p, sizeof(p), "gears[%u].ttsTextTemplate", (unsigned)i);
        CFG_STR_ON_RANGE(json, p, g->tts_text, CFG_TTSTEXT_MAX);

        /* targets: "all" 或 [addr,...] */
        (void)snprintf(p, sizeof(p), "gears[%u].targets", (unsigned)i);
        if (cj_get(json, p, NULL, 0) == CJ_STR)
        {
            char tbuf[8];
            if (cj_get_str(json, p, tbuf, (uint16_t)sizeof(tbuf)) &&
                (strcmp(tbuf, "all") == 0))
            {
                g->targets_all  = 1u;
                g->target_count = 0u;
            }
            else
            {
                return CFG_ERR_RANGE;       /* 只认 "all" 这个字符串 */
            }
        }
        else if (cj_get(json, p, NULL, 0) == CJ_ARR)
        {
            uint16_t tc = cj_count(json, p);
            if (tc > CFG_DEVICE_MAX) { return CFG_ERR_RANGE; }

            g->targets_all  = 0u;
            g->target_count = (uint8_t)tc;
            for (j = 0u; j < (uint8_t)tc; j++)
            {
                char pa[56];
                (void)snprintf(pa, sizeof(pa), "gears[%u].targets[%u]", (unsigned)i, (unsigned)j);
                CFG_STR_ON_RANGE(json, pa, g->target_uid[j], CFG_UID_HEX_LEN);
                if (!cfg_uid_valid(g->target_uid[j])) { return CFG_ERR_RANGE; }
            }
        }
    }

    return CFG_OK;
}

/* ==================== devices[]: 与 sensors[] 完全对应的两种语义 ====================
 *
 *   merge=false -> 整数组替换(全量加载: 存进去的就是完整数组, 先铺文档默认值)
 *   merge=true  -> 按 uid 逐条合并:
 *                    命中  -> 只覆盖来包里【出现过】的字段, 其余保持原值
 *                    未命中-> 追加(受 CFG_DEVICE_MAX 约束), 追加前先铺
 *                            volume=50 / flashMode=1 / enabled=1
 *
 * 【为什么 merge=true 必须存在】cfg.set 的语义是"增量": 平台只发一条
 * {"devices":[{"uid":"310001","name":"东门报警器"}]} 时, 绝不能把另外几台设备
 * 从列表里删掉 —— 那正是整数组替换会干的事(与 sensors 曾经踩过的坑完全相同)。
 *
 * 【为什么"未命中"要报满而不是覆盖】设备只能从 dev.bind 进来(要配对/标识),
 * 从 cfg.set 偷偷塞一台新设备会让云和网关的绑定表不一致; 但文档又要求
 * "数组未出现的条目不变" —— 所以容量满时宁可整域拒绝(rejected), 让平台看得见。
 */
static cfg_result_t cfg_parse_devices(const char *json, config_store_t *c, bool *present,
                                     bool merge)
{
    cj_type_t t;
    uint16_t  cnt;
    uint16_t  i;
    uint8_t   slot;
    int       r;
    uint8_t   e = 0u;

    t = cj_get(json, "devices", NULL, 0);
    if (t == CJ_NOTFOUND) { *present = false; return CFG_OK; }
    *present = true;                    /* 同 sensors: 出现了就要计入 applied/rejected */
    if (t != CJ_ARR) { return CFG_ERR_JSON; }

    cnt = cj_count(json, "devices");
    if (cnt > CFG_DEVICE_MAX) { return CFG_ERR_RANGE; }

    if (!merge)
    {
        /* 全量语义: 整数组替换 */
        (void)memset(c->devices, 0, sizeof(c->devices));
        c->device_count = (uint8_t)cnt;

        /* 先铺文档 3.6 的默认值(volume=50 / flashMode=1 / enabled=true) */
        for (slot = 0u; slot < c->device_count; slot++)
        {
            cfg_device_defaults(&c->devices[slot]);
        }
    }

    for (i = 0u; i < cnt; i++)
    {
        cfg_device_t *d;
        char    p[40];
        char    want_uid[CFG_UID_STR_MAX];
        uint8_t k;
        bool    found = false;

        /* ---- 先取 uid, 用它决定"写到哪个槽" ----
         * 缺 uid / 格式非法 -> 拒该域: 没有地址就无法定位改的是哪一台 */
        (void)snprintf(p, sizeof(p), "devices[%u].uid", (unsigned)i);
        if (cfg_get_str(json, p, want_uid, CFG_UID_HEX_LEN) != CFG_GET_OK) { return CFG_ERR_RANGE; }
        if (!cfg_uid_valid(want_uid)) { return CFG_ERR_RANGE; }

        if (merge)
        {
            for (k = 0u; k < c->device_count && k < CFG_DEVICE_MAX; k++)
            {
                if (strcmp(c->devices[k].uid, want_uid) == 0) { found = true; break; }
            }
            if (found)
            {
                slot = k;                       /* 命中: 只覆盖来包里出现过的字段 */
            }
            else
            {
                if (c->device_count >= CFG_DEVICE_MAX) { return CFG_ERR_RANGE; }
                slot = c->device_count;         /* 未命中: 追加, 先铺文档默认值 */
                cfg_device_defaults(&c->devices[slot]);
                c->device_count++;
            }
        }
        else
        {
            slot = (uint8_t)i;
        }

        d = &c->devices[slot];
        (void)strcpy(d->uid, want_uid);

        (void)snprintf(p, sizeof(p), "devices[%u].serial", (unsigned)i);
        CFG_STR_ON_RANGE(json, p, d->serial, CFG_SERIAL_MAX);
        (void)snprintf(p, sizeof(p), "devices[%u].name", (unsigned)i);
        CFG_STR_ON_RANGE(json, p, d->name, CFG_NAME_MAX);

        (void)snprintf(p, sizeof(p), "devices[%u].type", (unsigned)i);
        r = cfg_get_u8(json, p, &d->type, 1u, 3u);             CFG_RET_ON_RANGE(r);
        (void)snprintf(p, sizeof(p), "devices[%u].volume", (unsigned)i);
        r = cfg_get_u8(json, p, &d->volume, 0u, 100u);         CFG_RET_ON_RANGE(r);
        (void)snprintf(p, sizeof(p), "devices[%u].flashMode", (unsigned)i);
        r = cfg_get_u8(json, p, &d->flash_mode, 0u, 3u);       CFG_RET_ON_RANGE(r);

        (void)snprintf(p, sizeof(p), "devices[%u].enabled", (unsigned)i);
        r = cfg_get_bool(json, p, &e);                         CFG_RET_ON_RANGE(r);
        if (r == CFG_GET_OK) { d->enabled = e; }
    }

    return CFG_OK;
}

static cfg_result_t cfg_parse_rules(const char *json, cfg_rules_t *r0, bool *present)
{
    int r;
    uint8_t e = 0u;

    if (cj_get(json, "rules", NULL, 0) != CJ_OBJ) { *present = false; return CFG_OK; }
    *present = true;

    r = cfg_get_enum(json, "rules.mode", cfg_rulemode_names, 2u, &e); CFG_RET_ON_RANGE(r);
    if (r == CFG_GET_OK) { r0->mode = e; }

    r = cfg_get_enum(json, "rules.sensorOfflinePolicy", cfg_offline_names, 3u, &e);
    CFG_RET_ON_RANGE(r);
    if (r == CFG_GET_OK) { r0->sensor_offline_policy = e; }

    r = cfg_get_u16(json, "rules.manualTimeoutSeconds", &r0->manual_timeout_s, 0u, 65535u);
    CFG_RET_ON_RANGE(r);

    return CFG_OK;
}

static cfg_result_t cfg_parse_ota(const char *json, cfg_ota_t *o, bool *present)
{
    int r;
    uint8_t e = 0u;

    if (cj_get(json, "ota", NULL, 0) != CJ_OBJ) { *present = false; return CFG_OK; }
    *present = true;

    /* fwVersion 只读(文档 3.8): 平台下发也不改, 由固件自己填 —— 这里刻意不解析 */
    r = cfg_get_enum(json, "ota.upgradePolicy", cfg_policy_names, 2u, &e); CFG_RET_ON_RANGE(r);
    if (r == CFG_GET_OK) { o->upgrade_policy = e; }

    return CFG_OK;
}

/* ==================== 增量合并 / 全量解析 ==================== */

cfg_result_t config_store_merge_json(config_store_t *c, const char *json,
                                     uint8_t *changed_mask)
{
    uint8_t      mask = 0u;
    bool         pres = false;
    cfg_result_t r;

    if ((c == NULL) || (json == NULL)) { return CFG_ERR_PARAM; }

    /* 在副本上改: 任何一步失败都让 *c 保持原样, 免得留下改了一半的配置 */
    (void)memcpy(&s_merge_tmp, c, sizeof(*c));

    r = cfg_parse_system(json, &s_merge_tmp.system, &pres);
    if (r != CFG_OK) { return r; }
    if (pres) { mask |= CFG_CH_SYSTEM; }

    r = cfg_parse_network(json, &s_merge_tmp.network, &pres);
    if (r != CFG_OK) { return r; }
    if (pres) { mask |= CFG_CH_NETWORK; }

    r = cfg_parse_sensors(json, &s_merge_tmp, &pres, false);
    if (r != CFG_OK) { return r; }
    if (pres) { mask |= CFG_CH_SENSORS; }

    r = cfg_parse_thresholds(json, &s_merge_tmp.thresholds, &pres);
    if (r != CFG_OK) { return r; }
    if (pres) { mask |= CFG_CH_THRESHOLDS; }

    r = cfg_parse_gears(json, s_merge_tmp.gears, &pres);
    if (r != CFG_OK) { return r; }
    if (pres) { mask |= CFG_CH_GEARS; }

    r = cfg_parse_devices(json, &s_merge_tmp, &pres, false);
    if (r != CFG_OK) { return r; }
    if (pres) { mask |= CFG_CH_DEVICES; }

    r = cfg_parse_rules(json, &s_merge_tmp.rules, &pres);
    if (r != CFG_OK) { return r; }
    if (pres) { mask |= CFG_CH_RULES; }

    r = cfg_parse_ota(json, &s_merge_tmp.ota, &pres);
    if (r != CFG_OK) { return r; }
    if (pres) { mask |= CFG_CH_OTA; }

    /* 合并后必须整体仍合法(例如只改 alarmCm 可能破坏 alarm>warning) */
    r = config_store_validate(&s_merge_tmp);
    if (r != CFG_OK) { return r; }

    (void)memcpy(c, &s_merge_tmp, sizeof(*c));
    if (changed_mask != NULL) { *changed_mask = mask; }
    return CFG_OK;
}

cfg_result_t config_store_apply_json(config_store_t *c, const char *json)
{
    return config_store_merge_json(c, json, NULL);
}

/* ==================== 逐域增量合并(cfg.set 用) —— 任务 P2-4 ==================== */

/** 在 s_merge_tmp 上只应用一个域; *present 置位 = 来包里确实出现了这个域 */
static cfg_result_t cfg_apply_one_section(const char *json, uint8_t ch, bool *present)
{
    switch (ch)
    {
        case CFG_CH_SYSTEM:     return cfg_parse_system(json, &s_merge_tmp.system, present);
        case CFG_CH_NETWORK:    return cfg_parse_network(json, &s_merge_tmp.network, present);
        case CFG_CH_SENSORS:    return cfg_parse_sensors(json, &s_merge_tmp, present, true);
        case CFG_CH_THRESHOLDS: return cfg_parse_thresholds(json, &s_merge_tmp.thresholds, present);
        case CFG_CH_GEARS:      return cfg_parse_gears(json, s_merge_tmp.gears, present);
        case CFG_CH_DEVICES:    return cfg_parse_devices(json, &s_merge_tmp, present, true);
        case CFG_CH_RULES:      return cfg_parse_rules(json, &s_merge_tmp.rules, present);
        case CFG_CH_OTA:        return cfg_parse_ota(json, &s_merge_tmp.ota, present);
        default:                *present = false; return CFG_OK;
    }
}

cfg_result_t config_store_merge_sections(config_store_t *c, const char *json,
                                         cfg_sections_result_t *res)
{
    /* 固定顺序 = 文档 5.2 的域列表(system..ota)。
     * 【meta 不在表里, 这是刻意的】meta(configVersion/crc32/updatedAt)是系统维护
     * 字段, 平台只读; 所以即使来包里带了 meta, 也既不算 applied 也不算 rejected。
     *
     * 【已知局限】域名拼错(如 "sensor")同样落在"不在表里"这一档, 会被静默忽略。
     * 想把它变成 rejected 需要能遍历对象的键, 而 cloud_json.h 目前没有这个接口 ——
     * 与其在这加一套半吊子扫描, 不如等真需要时给 cloud_json 补一个 cj_key_count/
     * cj_key_at。当前行为对平台是"这个域没生效"而不是"改错了别的域", 不算危险。 */
    static const uint8_t order[8] =
    {
        CFG_CH_SYSTEM, CFG_CH_NETWORK, CFG_CH_SENSORS, CFG_CH_THRESHOLDS,
        CFG_CH_GEARS, CFG_CH_DEVICES, CFG_CH_RULES, CFG_CH_OTA
    };
    cfg_sections_result_t out;
    uint8_t i;

    if (res != NULL) { res->applied = 0u; res->rejected = 0u; }
    if ((c == NULL) || (json == NULL)) { return CFG_ERR_PARAM; }

    out.applied  = 0u;
    out.rejected = 0u;

    for (i = 0u; i < (uint8_t)(sizeof(order) / sizeof(order[0])); i++)
    {
        bool         pres = false;
        cfg_result_t r;

        /* 每个域都从【当前 *c】重新拷贝一份副本, 保证失败域的改动不会污染下一个域,
         * 也不会污染 *c。s_merge_tmp 是静态的: 配置结构体 ~1.7KB, 不能在 4KB 栈上再来一份。 */
        (void)memcpy(&s_merge_tmp, c, sizeof(*c));

        r = cfg_apply_one_section(json, order[i], &pres);
        if (!pres)
        {
            continue;                       /* 来包里没有这个域 -> 原样不动 */
        }

        /* 应用完还要整体校验: 只改 alarmCm 可能破坏 alarm>warning 这类域内/域间约束 */
        if (r == CFG_OK) { r = config_store_validate(&s_merge_tmp); }

        if (r == CFG_OK)
        {
            (void)memcpy(c, &s_merge_tmp, sizeof(*c));      /* 成功 -> 提交这一个域 */
            out.applied |= order[i];
        }
        else
        {
            out.rejected |= order[i];                       /* 失败 -> 丢弃该域, 继续下一个 */
        }
    }

    if (res != NULL) { *res = out; }

    /* 全部域都被拒也返回 CFG_OK: 域级失败是【业务结果】, 写在 res 里给平台看;
     * 用错误码返回会让平台以为整条命令根本没被处理。只有入参非法才是 CFG_ERR_PARAM。 */
    return CFG_OK;
}

cfg_result_t config_store_from_json(config_store_t *c, const char *json)
{
    cfg_result_t r;
    cj_type_t    t;

    if ((c == NULL) || (json == NULL)) { return CFG_ERR_PARAM; }

    config_store_factory_default(c);

    /* meta 单独取密: configVersion / updatedAt 是系统维护字段, 平台只读,
     * 但上电从 Flash 读回时必须恢复, 否则版本号会一直归零。 */
    t = cj_get(json, "meta", NULL, 0);
    if (t == CJ_OBJ)
    {
        c->meta.config_version = cj_get_uint(json, "meta.configVersion", 0u);
        c->meta.updated_at     = cj_get_uint(json, "meta.updatedAt", 0u);
        c->meta.crc32          = 0u;
    }

    r = config_store_merge_json(c, json, NULL);
    if (r != CFG_OK) { return r; }

    /* ★必须在这里恢复 ota.fwVersion★
     *   它是"固件自己维护"的只读字段: cfg.set 的增量合并里【故意不解析】它
     *   (平台不可写), 但它【参与 CRC 覆盖】。全量解析(上电读 Flash / save 回读校验)
     *   如果不恢复它, 重算 CRC 时它仍是出厂默认 "1.0.0", 而 Flash 里存的是上一版
     *   固件写下的 "1.0.3" ⇒ 每次升级后配置都判 CRC 错:
     *     · 两区轮流报废 -> 回出厂 -> 绑定丢失;
     *     · 连 save 的回读校验都失败 -> A 区永远写不进去(只写了 B)。
     *   (2026-10-19 实机: 1.0.3 -> 1.0.4 升级后 "两区都不可用(code=4)")
     *   cfg.set 那条路径不受影响: 它走 merge_sections, 不会到达这里。 */
    {
        char fw[CFG_FWVER_MAX + 1u];

        /* 注意: cj_get_str() 找不到字段时会把目标【清空】—— 所以必须先取到临时缓冲,
         * 取到且非空才覆盖; 否则 {} 这种空配置会把出厂默认的 fwVersion 也清掉。 */
        if (cj_get_str(json, "ota.fwVersion", fw, (uint16_t)sizeof(fw)) && (fw[0] != '\0'))
        {
            (void)strcpy(c->ota.fw_version, fw);
        }
    }

    /* CRC 校验必须在 merge 之后: 此时 c 才是这份 JSON 的完整解析结果。
     * 先重新序列化一次拿到"应有"的 CRC, 再和 JSON 里声明比 —— 这一步同时
     * 验证了【序列化器与解析器互为逆运算】: 任何一边丢字段/截断都会在这里暴露。 */
    if ((t == CJ_OBJ) && (cj_get(json, "meta.crc32", NULL, 0) == CJ_STR))
    {
        char     hex[12];
        uint32_t want;
        uint32_t have;

        if (cj_get_str(json, "meta.crc32", hex, (uint16_t)sizeof(hex)))
        {
            have = (uint32_t)strtoul(hex, NULL, 16);
            want = config_store_calc_crc(c);
            if ((want == 0u) || (want != have)) { return CFG_ERR_CRC; }
            c->meta.crc32 = want;
        }
    }

    return CFG_OK;
}
