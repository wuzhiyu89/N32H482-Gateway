/**
 * @file  cloud_uplink.c
 * @brief 上行数据段构造：status / telemetry / event.* / buffered / 断线补报渲染
 *
 * 与 cloud_proto.c 的分工：
 *   cloud_proto.c  —— 下行：解析信封、鉴权、幂等、21 条命令分发、应答组包
 *   cloud_uplink.c —— 上行：把业务结构写成文档要求的 data JSON（用 cJSON 静态池）
 *
 * 这里不碰 MQTT / 4G / Flash；调用方负责把结果套信封并发布。
 * 补报的紧凑记录来自 app/service/event_cache.h，渲染成完整事件对象后再分组。
 */
#include <stdio.h>
#include <string.h>

#include "cloud_proto.h"
#include "cloud_internal.h"
#include "config_store.h"
#include "cjson_port.h"
#include "event_cache.h"
#include "level_eval.h"
#include "log.h"
#include "utils.h"

/* ---- 断线补报的一页组装 (v67) ---- */
#define CLOUD_BACKFILL_MAX      16u     /* 一批最多 16 条(见头文件: 栈只有 4 KB) */
#define CLOUD_BACKFILL_EV_MAX   256u    /* 单个事件对象文本上限 */

/* ★ 必须是 static ★ —— 16 × 256 B = 4 KB, 放栈上一次就把 4 KB 栈吃掉了。 */
static char        s_bf_text[CLOUD_BACKFILL_MAX][CLOUD_BACKFILL_EV_MAX];
static const char *s_bf_ptr[CLOUD_BACKFILL_MAX];
static char                  s_bf_uid[CLOUD_BACKFILL_MAX][CFG_UID_STR_MAX];
/* 缓存里的 uid 是 13 字节二进制, 而协议层的 results[] 要的是 hex 字符串
 * (这个缝是编译器抓出来的 —— 两边 uid 表示法本来就不一样)。 */
static cloud_action_result_t s_bf_res[CLOUD_BACKFILL_MAX];

/**
 * @brief 按缓存记录重建事件的 msgId（v12，2026-09-30）
 *
 * 【为什么要有它】原来补报一律调 cloud_proto_next_backfill_msg_id() 生成 `b<序号>`,
 * 于是同一个事件在线时是 `s007`、补报时变成 `b012` —— 平台按 msgId 做幂等去重
 * 时对不上。现在入缓存时就把序号存进记录（event_cache.h 的 `msg_seq`, 2 B/条），
 * 这里按 kind 选 salt、用同一个生成器重建，与首次分配时逐字节相同。
 *
 * @return true = 用记录里的原序号重建成功;
 *         false = 记录没带序号(msg_seq==0, 老记录)或 kind 不认
 *                 -> 调用方回落到旧的 `b<自增>` 行为(宁可换个 id, 也不发假 id)
 */
static bool cloud_proto_msg_id_of_record(char *out, uint16_t cap, const ec_record_t *rec)
{
    uint8_t salt;

    if ((out == NULL) || (cap == 0u) || (rec == NULL) || (rec->msg_seq == 0u))
    {
        return false;
    }

    switch (rec->kind)
    {
    case (uint8_t)EC_KIND_STATUS_CHANGE: salt = (uint8_t)'s'; break;   /* 与在线路径一致 */
    case (uint8_t)EC_KIND_DEVICE:        salt = (uint8_t)'v'; break;
    case (uint8_t)EC_KIND_ACTION:        salt = (uint8_t)'a'; break;
    default:                             return false;
    }

    return utils_gen_msg_id_seq(out, cap, salt, rec->msg_seq);
}

uint16_t cloud_proto_build_backfill_data(char *out, uint16_t cap, uint8_t batch_max,
                                         uint8_t *out_consumed, uint8_t *out_events,
                                         uint8_t *out_skipped)
{
    static ec_action_pair_t pairs[CLOUD_BACKFILL_MAX];  /* 动作组最多 16 台 */
    cloud_buffered_event_t  be;
    uint16_t                len_ret;
    uint8_t                 n;
    uint8_t                 cnt  = 0u;
    uint8_t                 skip = 0u;
    uint8_t                 i;

    if ((out == NULL) || (cap == 0u) || (out_consumed == NULL) ||
        (out_events == NULL) || (out_skipped == NULL))
    {
        return 0u;
    }
    *out_consumed = 0u;
    *out_events   = 0u;
    *out_skipped  = 0u;

    if (batch_max == 0u)
    {
        return 0u;
    }
    if (batch_max > (uint8_t)CLOUD_BACKFILL_MAX)
    {
        batch_max = (uint8_t)CLOUD_BACKFILL_MAX;
    }

    n = (uint8_t)event_cache_page_count((uint16_t)batch_max);
    if (n == 0u)
    {
        return 0u;                      /* 缓存空 */
    }

    for (i = 0u; (i < n) && (cnt < (uint8_t)CLOUD_BACKFILL_MAX); )
    {
        ec_record_t rec;
        uint16_t    len;
        uint8_t     step = 1u;
        uint8_t     gcnt = 0u;
        char        msg_id[CLOUD_MSGID_MAX];

        if (!event_cache_peek_at(i, &rec))
        {
            break;
        }

        /* 动作组 -> 渲染成一条事件(带完整 results[])。
         * 这里要把 (3 字节 uid, 执行码) 转成协议层要的 (hex 串, code)。 */
        /* ★★2026-10-08 新增: "甲"方案的单台动作记录(seq == 0)★★
         * 补报驱动原来【只认成组的】(下面那个 seq != 0 的分支), 因为早期设计是
         * "一次动作的多台结果攒成一条 event.action"。用户 2026-10-08 选定甲方案:
         * 每台设备【单独一条】, 结果码就存在这条记录的 aux 里, 不需要攒组。
         * ⇒ 这里先给它造一个单元素 results[], 后面那段原封不动 ——
         *   两条路最终都归结为 (gcnt, s_bf_res[0..gcnt-1]), 渲染层不需要知道区别。
         * ⚠️ 结果码必须走 ec_action_code_of() 取, 不要直接读 aux —— 那个字段
         *   的含义随 kind 变, 直接读会变成靠人记的隐性知识(见 event_cache.h)。 */
        if ((rec.kind == (uint8_t)EC_KIND_ACTION) && (rec.seq == 0u))
        {
            cfg_uid_from_bytes(rec.uid, s_bf_uid[0]);
            s_bf_res[0].uid  = s_bf_uid[0];
            s_bf_res[0].code = (int)ec_action_code_of(&rec);
            gcnt = 1u;
        }

        if ((rec.kind == (uint8_t)EC_KIND_ACTION) && (rec.seq != 0u))
        {
            uint8_t g;

            gcnt = event_cache_group_at(i, pairs, (uint8_t)CLOUD_BACKFILL_MAX);
            if (gcnt == 0u)
            {
                i++;                    /* 组收不齐 -> 跳过这一台(计数) */
                skip++;
                continue;
            }
            step = gcnt;
            for (g = 0u; g < gcnt; g++)
            {
                cfg_uid_from_bytes(pairs[g].uid, s_bf_uid[g]);
                s_bf_res[g].uid  = s_bf_uid[g];
                s_bf_res[g].code = (int)pairs[g].code;
            }
        }

        /* v12: 优先用记录里存的【原序号】重建 msgId;
         * 记录没带序号(老记录)时才回落到旧的 `b<自增>` 分配。 */
        if ((!cloud_proto_msg_id_of_record(msg_id, (uint16_t)sizeof(msg_id), &rec))
            && (!cloud_proto_next_backfill_msg_id(msg_id, (uint16_t)sizeof(msg_id))))
        {
            i = (uint8_t)(i + step);    /* 连 id 都拿不到: 跳过并计数 */
            skip++;
            continue;
        }

        len = cloud_proto_render_cached_event(s_bf_text[cnt],
                                              (uint16_t)CLOUD_BACKFILL_EV_MAX,
                                              &rec, msg_id,
                                              (gcnt > 0u) ? s_bf_res : NULL, gcnt);
        if (len == 0u)
        {
            i = (uint8_t)(i + step);    /* 渲染不出来 -> 跳过并计数, 绝不截断 */
            skip++;
            continue;
        }

        s_bf_ptr[cnt] = s_bf_text[cnt];
        cnt++;
        i = (uint8_t)(i + step);
    }

    /* 无论有没有内容, 都要如实告诉调用方推进了多少条:
     * 返回 0 但有推进时, 调用方必须照常 drop —— 否则渲染不出来的记录会永远
     * 堵在队头, 把后面所有事件都堵死。 */
    *out_consumed = i;
    *out_skipped  = skip;
    if (cnt == 0u)
    {
        return 0u;
    }

    be.count       = cnt;
    be.events_json = s_bf_ptr;

    len_ret = cloud_proto_build_buffered_data(out, cap, &be);
    if (len_ret == 0u)
    {
        /* 整页装不下 cap: 什么都没发出去, 那么一条都不能算 consumed,
         * 否则这些事件就凭空丢了(让调用方下次再试, 或换更大的 cap)。 */
        *out_consumed = 0u;
        *out_events   = 0u;
        *out_skipped  = 0u;
        return 0u;
    }

    *out_events = cnt;
    return len_ret;
}


/* ==================================================================
 * 断线补报: 紧凑记录 -> 完整事件对象文本
 * ==================================================================
 * 事件缓存存的是 16 B 紧凑记录(见 app/service/event_cache.h), 补报时才还原成
 * 事件对象。这里只做**单条**渲染; 分页、按 seq 收集动作组、组装 buffered 数据段、
 * 发布、成功后的 drop —— 都是调用方的事(见交接文档 13.19 节的调用序列)。
 *
 * 【码值 -> 字符串】这些字符串就是文档 7.x 里的取值(见 cloud_device_event_t 的注释),
 * 与 event_cache.h 的枚举一一对应。*刻意放在协议层*: 业务层不该知道线上文本长什么样。
 * ================================================================== */
static const char *s_ec_state_name(uint8_t code)
{
    switch (code)
    {
        case (uint8_t)EC_ST_RESOLVED: return "Resolved";
        case (uint8_t)EC_ST_NORMAL:   return "Normal";
        case (uint8_t)EC_ST_WARNING:  return "Warning";
        case (uint8_t)EC_ST_ALARM:    return "Alarm";
        default:                      return NULL;      /* 不认识 -> 让上层返回 0 */
    }
}

const char *cloud_proto_state_name(uint8_t code)
{
    return s_ec_state_name(code);       /* 只保留这一份实现(见头文件说明) */
}

static const char *s_ec_trigger_name(uint8_t code)
{
    switch (code)
    {
        case (uint8_t)EC_TRIG_AUTO:     return "auto";
        case (uint8_t)EC_TRIG_MANUAL:   return "manual";
        case (uint8_t)EC_TRIG_FAILSAFE: return "failsafe";
        default:                        return NULL;
    }
}

static const char *s_ec_dev_event_name(uint8_t code)
{
    switch (code)
    {
        case (uint8_t)EC_DEV_EV_BOUND:       return "bound";
        case (uint8_t)EC_DEV_EV_BIND_FAILED: return "bind_failed";
        case (uint8_t)EC_DEV_EV_UNBOUND:     return "unbound";
        case (uint8_t)EC_DEV_EV_ONLINE:      return "online";
        case (uint8_t)EC_DEV_EV_OFFLINE:     return "offline";
        case (uint8_t)EC_DEV_EV_LOW_VOLTAGE: return "low_voltage";
        default:                             return NULL;
    }
}

static const char *s_ec_dev_type_name(uint8_t code)
{
    switch (code)
    {
        case (uint8_t)EC_DEV_SENSOR: return "sensor";
        case (uint8_t)EC_DEV_DEVICE: return "device";
        default:                     return NULL;
    }
}

/**
 * @brief 按 uid 找设备的序列号（v12: event.device 要带 serial）
 *
 * 【为什么现查、不是存进事件记录】事件记录只有 20 B/条(512 条), 塞不下 8~16 字节的
 * serial —— 那会让缓存再涨好几 KB。而 serial 本来就在 config_store 里, 按 uid 找一下
 * 就有(最多 5 台设备, 线性查找的代价可以忽略)。
 *
 * @param uid_hex   26 字符 hex 串
 * @param is_sensor true = sensors[], false = devices[]
 * @return 指向 config_store 内部的 serial(本次渲染期间有效); NULL = 没找到
 * @note  ⚠️ "解绑(unbound)"事件这里【查不到】—— 那时记录已经从配置里删掉了。
 *        所以 unbound 的 event.device 不带 serial(字段直接不输出)。
 *        要连它也带, 得在解绑【之前】把 serial 抄进事件记录 —— 那要扩记录, 未做。
 */
const char *cloud_proto_serial_of_uid(const char *uid_hex, bool is_sensor)
{
    const config_store_t *cfg = config_store_get();
    uint8_t i;

    if ((cfg == NULL) || (uid_hex == NULL) || (uid_hex[0] == '\0'))
    {
        return NULL;
    }

    if (is_sensor)
    {
        for (i = 0u; (i < cfg->sensor_count) && (i < (uint8_t)CFG_SENSOR_MAX); i++)
        {
            if (strcmp(cfg->sensors[i].uid, uid_hex) == 0) { return cfg->sensors[i].serial; }
        }
    }
    else
    {
        for (i = 0u; (i < cfg->device_count) && (i < (uint8_t)CFG_DEVICE_MAX); i++)
        {
            if (strcmp(cfg->devices[i].uid, uid_hex) == 0) { return cfg->devices[i].serial; }
        }
    }
    return NULL;
}

uint16_t cloud_proto_render_cached_event(char *out, uint16_t cap,
                                         const ec_record_t *rec,
                                         const char *msg_id,
                                         const cloud_action_result_t *results,
                                         uint8_t result_count)
{
    /* 数据段工作缓冲。★ 必须 static ★ —— 本工程栈只有 4 KB,
     * 放栈上的话一次补报就把栈吃掉了。 */
    static char data[CLOUD_DATA_MAX];
    uint16_t    n;

    if ((out == NULL) || (rec == NULL) || (msg_id == NULL))
    {
        return 0u;
    }
    /* 码值合法性统一走 v60 那个校验: 渲染层不该自己再判一遍 */
    if (!event_cache_validate(rec))
    {
        return 0u;
    }

    (void)memset(data, 0, sizeof(data));

    switch (rec->kind)
    {
        case (uint8_t)EC_KIND_STATUS_CHANGE:
        {
            cloud_status_change_t sc;
            const char           *from = s_ec_state_name(rec->b);
            const char           *to   = s_ec_state_name(rec->a);

            if ((from == NULL) || (to == NULL))
            {
                return 0u;
            }
            sc.from     = from;
            sc.to       = to;
            /* 档位不在记录里(16 B 装不下), 从迁移后的状态推出来:
             * ec_state_t 的取值与 level_state_t 前四项*刻意对齐*(见 event_cache.h),
             * 所以这里直接转, 不必再存一份。 */
            sc.gear     = level_state_to_gear((level_state_t)rec->a);
            sc.level_mm = rec->aux;

            n = cloud_proto_build_status_change_data(data, (uint16_t)sizeof(data), &sc);
            if (n == 0u)
            {
                return 0u;
            }
            return cloud_proto_build_uplink(out, cap, "event.status_change",
                                            msg_id, rec->ts, data);
        }

        case (uint8_t)EC_KIND_ACTION:
        {
            cloud_action_event_t  ae;
            const char           *trig = s_ec_trigger_name(rec->b);

            /* 动作事件没有执行结果就没有意义: 宁可渲染失败让上层跳过, 也不发空 results */
            if ((trig == NULL) || (results == NULL) || (result_count == 0u))
            {
                return 0u;
            }
            ae.gear         = rec->a;       /* 档位 0~3, 由校验保证 */
            ae.trigger      = trig;
            ae.result_count = result_count;
            ae.results      = results;

            n = cloud_proto_build_action_data(data, (uint16_t)sizeof(data), &ae);
            if (n == 0u)
            {
                return 0u;                  /* 设备太多装不下 -> 跳过并计数, 不截断 */
            }
            return cloud_proto_build_uplink(out, cap, "event.action",
                                            msg_id, rec->ts, data);
        }

        case (uint8_t)EC_KIND_DEVICE:
        {
            cloud_device_event_t de;
            const char          *ev  = s_ec_dev_event_name(rec->a);
            const char          *typ = s_ec_dev_type_name(rec->b);
            char                 uid_hex[CFG_UID_STR_MAX];

            if ((ev == NULL) || (typ == NULL))
            {
                return 0u;
            }
            cfg_uid_from_bytes(rec->uid, uid_hex);   /* 13 字节 -> 26 位 hex 串 */
            de.uid      = uid_hex;
            de.dev_type = typ;
            de.event    = ev;
            /* v12: 带上序列号, uid 现查; 解绑事件查不到 -> NULL -> 不输出 */
            de.serial   = cloud_proto_serial_of_uid(uid_hex, (rec->b == (uint8_t)EC_DEV_SENSOR));

            n = cloud_proto_build_device_event_data(data, (uint16_t)sizeof(data), &de);
            if (n == 0u)
            {
                return 0u;
            }
            return cloud_proto_build_uplink(out, cap, "event.device",
                                            msg_id, rec->ts, data);
        }

        default:
            return 0u;
    }
}

bool cloud_proto_next_backfill_msg_id(char *out, uint16_t cap)
{
    return utils_gen_msg_id(out, cap);
}

/* ==================================================================
 * cfg.query 的 sections 参数 —— 文档里有, 此前只认 "all", 其余一律 1001
 * ==================================================================
 * 【9 个域名的来源】就是 config_store_write_members() 写出的那 9 个键,
 * 不另立一套名字 —— 否则"配置改了、查询名字没跟着"是迟早的事。
 * 键序也保持写入器的顺序(gears 在 rules 之前), 因此 sections:"all" 的输出
 * 与以前逐字节相同, 不会打断任何客户端。
 * ================================================================== */

/*!
 * \brief 把 status 的 11 个字段按【文档 7.7 的顺序】挂进 obj
 * \note  键序不能变: 单测 [14] 是拿整串与文档逐字符比对的。
 *        online 是 bool -> cJSON 的 true/false; 其余整数走 %d(整数)分支。 */
bool cloud_fill_status_json(cJSON *obj, const cloud_status_t *st)
{
    return ((cJSON_AddBoolToObject(obj, "online", st->online ? 1 : 0) != NULL) &&
            (cJSON_AddStringToObject(obj, "fwVersion",
                (st->fw_version != NULL) ? st->fw_version : "0.0.0") != NULL) &&
            (cJSON_AddStringToObject(obj, "slot",
                (st->slot != NULL) ? st->slot : "A") != NULL) &&
            (cJSON_AddNumberToObject(obj, "configVersion",
                (double)st->config_version) != NULL) &&
            /* 见 cloud_proto.h 的说明: 这里是 CSQ 换算出的 RSSI, 不是 RSRP */
            (cJSON_AddNumberToObject(obj, "rsrp", (double)st->rsrp_dbm) != NULL) &&
            (cJSON_AddNumberToObject(obj, "uptimeSeconds",
                (double)st->uptime_s) != NULL) &&
            (cJSON_AddStringToObject(obj, "mode",
                (st->mode != NULL) ? st->mode : "auto") != NULL) &&
            (cJSON_AddNumberToObject(obj, "currentGear",
                (double)st->current_gear) != NULL) &&
            (cJSON_AddNumberToObject(obj, "sensorsOnline",
                (double)st->sensors_online) != NULL) &&
            (cJSON_AddNumberToObject(obj, "sensorsTotal",
                (double)st->sensors_total) != NULL) &&
            (cJSON_AddNumberToObject(obj, "devicesOnline",
                (double)st->devices_online) != NULL) &&
            (cJSON_AddNumberToObject(obj, "devicesTotal",
                (double)st->devices_total) != NULL));
}

uint16_t cloud_proto_build_status_data(char *out, uint16_t cap, const cloud_status_t *st)
{
    cJSON   *root;
    uint16_t n;

    if ((out == NULL) || (cap < 32u) || (st == NULL))
    {
        return 0u;
    }

    if (!cjson_port_is_ready())
    {
        out[0] = '\0';                  /* 护栏: 见 cloud_proto_parse_req 里的说明 */
        return 0u;
    }

    root = cJSON_CreateObject();
    if (root == NULL)
    {
        out[0] = '\0';
        return 0u;
    }

    if (!cloud_fill_status_json(root, st))
    {
        cJSON_Delete(root);
        out[0] = '\0';
        return 0u;
    }

    n = cloud_json_print(out, cap, root);
    cJSON_Delete(root);
    return n;
}

/* ==================================================================
 * telemetry.report 数据段(文档 7.1) —— 7 条上行报文里的第一个
 * ================================================================== */
uint16_t cloud_proto_build_telemetry_data(char *out, uint16_t cap,
                                          const cloud_telemetry_t *t)
{
    cJSON   *root;
    cJSON   *arr;
    cJSON   *eff;
    uint8_t  i;
    uint16_t n;

    if ((out == NULL) || (cap < 32u) || (t == NULL))
    {
        return 0u;
    }
    if ((t->sensor_count > 0u) && (t->sensors == NULL))
    {
        return 0u;                      /* 参数自相矛盾: 别去解引用 */
    }
    if (!cjson_port_is_ready())
    {
        out[0] = '\0';
        return 0u;
    }

    root = cJSON_CreateObject();
    if (root == NULL)
    {
        out[0] = '\0';
        return 0u;
    }

    /* sensors[] —— 键序/字段名按文档 7.1 的例子, 只把 addr 换成 uid(用户决策) */
    arr = cJSON_CreateArray();
    if (arr == NULL)
    {
        cJSON_Delete(root);
        out[0] = '\0';
        return 0u;
    }
    cJSON_AddItemToObject(root, "sensors", arr);

    for (i = 0u; i < t->sensor_count; i++)
    {
        const cloud_telemetry_sensor_t *s = &t->sensors[i];
        cJSON *o = cJSON_CreateObject();

        if (o == NULL)
        {
            cJSON_Delete(root);
            out[0] = '\0';
            return 0u;
        }
        cJSON_AddItemToArray(arr, o);

        if ((cJSON_AddStringToObject(o, "uid",
                (s->uid != NULL) ? s->uid : "") == NULL) ||
            (cJSON_AddNumberToObject(o, "levelCm",
                (double)s->level_mm / 10.0) == NULL) ||
            (cJSON_AddNumberToObject(o, "rawMm", (double)s->raw_mm) == NULL) ||
            (cJSON_AddNumberToObject(o, "batteryPercent", (double)s->battery_pct) == NULL) ||
            (cJSON_AddNumberToObject(o, "rssi", (double)s->rssi_dbm) == NULL) ||
            (cJSON_AddNumberToObject(o, "ts", (double)s->ts) == NULL))
        {
            cJSON_Delete(root);
            out[0] = '\0';
            return 0u;
        }
    }

    /* effective —— 有效水位与它对应的状态/档位(状态边界见文档 3.4) */
    eff = cJSON_CreateObject();
    if (eff == NULL)
    {
        cJSON_Delete(root);
        out[0] = '\0';
        return 0u;
    }
    cJSON_AddItemToObject(root, "effective", eff);

    if ((cJSON_AddNumberToObject(eff, "levelCm",
            (double)t->effective_level_mm / 10.0) == NULL) ||
        (cJSON_AddStringToObject(eff, "status",
            (t->effective_status != NULL) ? t->effective_status : "Normal") == NULL) ||
        (cJSON_AddNumberToObject(eff, "gear", (double)t->effective_gear) == NULL))
    {
        cJSON_Delete(root);
        out[0] = '\0';
        return 0u;
    }

    n = cloud_json_print(out, cap, root);
    cJSON_Delete(root);
    return n;
}

/* ==================================================================
 * event.status_change 数据段(文档 7.2) / event.device 数据段(文档 7.4)
 * ================================================================== */
uint16_t cloud_proto_build_status_change_data(char *out, uint16_t cap,
                                              const cloud_status_change_t *e)
{
    cJSON   *root;
    uint16_t n;

    if ((out == NULL) || (cap < 16u) || (e == NULL))
    {
        return 0u;
    }
    if (!cjson_port_is_ready())
    {
        out[0] = '\0';
        return 0u;
    }

    root = cJSON_CreateObject();
    if (root == NULL)
    {
        out[0] = '\0';
        return 0u;
    }

    /* 键序按文档 7.2: from / to / gear / levelCm */
    if ((cJSON_AddStringToObject(root, "from",
            (e->from != NULL) ? e->from : "Normal") == NULL) ||
        (cJSON_AddStringToObject(root, "to",
            (e->to != NULL) ? e->to : "Normal") == NULL) ||
        (cJSON_AddNumberToObject(root, "gear", (double)e->gear) == NULL) ||
        (cJSON_AddNumberToObject(root, "levelCm",
            (double)e->level_mm / 10.0) == NULL))
    {
        cJSON_Delete(root);
        out[0] = '\0';
        return 0u;
    }

    n = cloud_json_print(out, cap, root);
    cJSON_Delete(root);
    return n;
}

uint16_t cloud_proto_build_device_event_data(char *out, uint16_t cap,
                                             const cloud_device_event_t *e)
{
    cJSON   *root;
    uint16_t n;

    if ((out == NULL) || (cap < 16u) || (e == NULL))
    {
        return 0u;
    }
    if (!cjson_port_is_ready())
    {
        out[0] = '\0';
        return 0u;
    }

    root = cJSON_CreateObject();
    if (root == NULL)
    {
        out[0] = '\0';
        return 0u;
    }

    /* 键序按文档 7.4: uid(本工程口径, 文档里写 addr) / devType / event / serial */
    if ((cJSON_AddStringToObject(root, "uid",
            (e->uid != NULL) ? e->uid : "") == NULL) ||
        (cJSON_AddStringToObject(root, "devType",
            (e->dev_type != NULL) ? e->dev_type : "sensor") == NULL) ||
        (cJSON_AddStringToObject(root, "event",
            (e->event != NULL) ? e->event : "offline") == NULL))
    {
        cJSON_Delete(root);
        out[0] = '\0';
        return 0u;
    }

    /* serial(v12): 可空 —— NULL 时整个字段不输出, 不编空串冒充。
     * (文档 7.4 的示例里没有它, 是用户清单追加要求的; 附加字段不破坏平台解析) */
    if ((e->serial != NULL) && (e->serial[0] != '\0'))
    {
        if (cJSON_AddStringToObject(root, "serial", e->serial) == NULL)
        {
            cJSON_Delete(root);
            out[0] = '\0';
            return 0u;
        }
    }

    n = cloud_json_print(out, cap, root);
    cJSON_Delete(root);
    return n;
}

/* ==================================================================
 * event.action(文档 7.3) / event.buffered(文档 7.5)
 * ================================================================== */
uint16_t cloud_proto_build_action_data(char *out, uint16_t cap,
                                       const cloud_action_event_t *e)
{
    cJSON   *root;
    cJSON   *arr;
    uint8_t  i;
    uint16_t n;

    if ((out == NULL) || (cap < 16u) || (e == NULL))
    {
        return 0u;
    }
    if ((e->result_count > 0u) && (e->results == NULL))
    {
        return 0u;
    }
    if (!cjson_port_is_ready())
    {
        out[0] = '\0';
        return 0u;
    }

    root = cJSON_CreateObject();
    if (root == NULL)
    {
        out[0] = '\0';
        return 0u;
    }

    /* 键序按文档 7.3: gear / trigger / results[] */
    if ((cJSON_AddNumberToObject(root, "gear", (double)e->gear) == NULL) ||
        (cJSON_AddStringToObject(root, "trigger",
            (e->trigger != NULL) ? e->trigger : "auto") == NULL))
    {
        cJSON_Delete(root);
        out[0] = '\0';
        return 0u;
    }

    arr = cJSON_CreateArray();
    if (arr == NULL)
    {
        cJSON_Delete(root);
        out[0] = '\0';
        return 0u;
    }
    cJSON_AddItemToObject(root, "results", arr);

    for (i = 0u; i < e->result_count; i++)
    {
        cJSON *o = cJSON_CreateObject();

        if (o == NULL)
        {
            cJSON_Delete(root);
            out[0] = '\0';
            return 0u;
        }
        cJSON_AddItemToArray(arr, o);

        if ((cJSON_AddStringToObject(o, "uid",
                (e->results[i].uid != NULL) ? e->results[i].uid : "") == NULL) ||
            (cJSON_AddNumberToObject(o, "code",
                (double)e->results[i].code) == NULL))
        {
            cJSON_Delete(root);
            out[0] = '\0';
            return 0u;
        }
    }

    n = cloud_json_print(out, cap, root);
    cJSON_Delete(root);
    return n;
}

uint16_t cloud_proto_build_buffered_data(char *out, uint16_t cap,
                                         const cloud_buffered_event_t *e)
{
    cJSON   *root;
    cJSON   *arr;
    uint8_t  i;
    uint16_t n;

    if ((out == NULL) || (cap < 16u) || (e == NULL))
    {
        return 0u;
    }
    /* 不静默截断: 超过 50 条让调用方自己分页, 文档 7.5 就是这么要求的 */
    if ((e->count == 0u) || (e->count > (uint8_t)CLOUD_BUFFERED_MAX) ||
        (e->events_json == NULL))
    {
        return 0u;
    }
    if (!cjson_port_is_ready())
    {
        out[0] = '\0';
        return 0u;
    }

    root = cJSON_CreateObject();
    if (root == NULL)
    {
        out[0] = '\0';
        return 0u;
    }

    arr = cJSON_CreateArray();
    if (arr == NULL)
    {
        cJSON_Delete(root);
        out[0] = '\0';
        return 0u;
    }
    cJSON_AddItemToObject(root, "events", arr);

    for (i = 0u; i < e->count; i++)
    {
        /* cJSON_Raw: 原样内嵌 —— 每个事件对象自带 event/msgId/ts/data,
         * 补报时不能改写成重连时刻(文档第 3 章明确要求) */
        if (cJSON_AddItemToArray(arr, cJSON_CreateRaw(e->events_json[i])) == 0)
        {
            cJSON_Delete(root);
            out[0] = '\0';
            return 0u;
        }
    }

    n = cloud_json_print(out, cap, root);
    cJSON_Delete(root);
    return n;
}
