/**
 * @file  cloud_proto.c
 * @brief 云平台协议路由层实现(见 cloud_proto.h)
 *
 * 当前阶段(2026-10-02): 21 条下行命令【全部】接上真实实现 —— 最后三条
 * ota.start / ota.query / ota.abort 走"OTA over HTTP(S) + MD5"这条线
 * (数据源是 ML307R 原生 AT+MHTTP*, 见 app/ota/ota_http.c; 此前恒回 1006)。
 * 命令表 s_cmd_table[] 的排列顺序 = 文档第 4 章的命令编号。
 */
#include "cloud_proto.h"
#include "cloud_internal.h"
#include "cloud_profile.h"
#include "lora_business.h"      /* P5: sen.bind/dev.bind/sen.unbind/dev.unbind 异步对接 */
#include "config_store.h"       /* P2-4: cfg.set / cfg.query; P2-5: sys.factory_reset */
#include "last_value.h"         /* v53: sen.query / dev.query 要回"朢�近一次上� */
#include "event_cache.h"        /* v59: sys.query_status �bufferedEvents 给真�
                                * v62: 断线补报要把紧凑记录渲染成事件对象 */
#include "level_eval.h"         /* v62: 补报 status_change 时由"迁移后状�推档�*/
#include "level_supervisor.h"   /* �6 �dev.ctrl: �当前水位"渲染档位模板 */
#include "sensor_offline.h"     /* �10 � sen.query/dev.query �online 判据(与失联检测同丢�� */
#include "bsp_tim.h"            /* get_tick_ms() —��上面那个判据�当前时刻" */
#include "cjson_port.h"         /* v33: 信封层改�cJSON —���hooks 必须�cjson_port_init() */
#include "log.h"                /* P2-5: 恢复出厂的失败原因必须留下日志 */
#include "ota_http.h"           /* 19~21 OTA over HTTP: ota.start/query/abort */
#include "ota_port.h"           /* ota_port_get_version() —— "与当前版本相同"判据 */
#include <stdio.h>              /* snprintf —— 拼 sections[i] 这样的数组路径 */
#include <string.h>

/* ==================================================================
 * cfg.set / cfg.query 的工作缓�—��【必�static, 不能放栈上��P2-4)
 * ==================================================================
 * 实测完整配置 JSON: 默认配置 1203 B, 满配朢��3 传感�+ 5 设备 + 满长文本)
 * 3584 B。��本工程栈只�0x1000 = 4096 B(� * firmware/CMSIS/device/startup/startup_n32h47x_48x.s �Stack_Size),
 * 把 4KB 级的缓冲放栈上会直接爆栈 —— 而且是在"云平台下发 cfg.query"这条
 * 没法本地复现的路径上� 极难定位� *
 * 扢�� 改为文件�static 并放大到 CFG_JSON_MAX + 64(信封外的 data �� * 安全� 本工程是【单线程轮询� cloud_proto_handle() 只在主循环里被调�
 * 不会重入(命令处理里也不会再��归调用�, 扢��static 暂存是安全的� * 原来�CLOUD_DATA_MAX=320 �默认配置"都装不下, cfg.query 必然溢出失败� */
static char s_data_buf[CFG_JSON_MAX + 64u];

/* cloud_proto_handle() 传进来的"网关当前时间", �cfg.set �meta.updatedAt� * 不用下行报文里的 ts: 那是平台的时� 网关的时间戳该用网关自己的��*/
static uint32_t s_gw_now_ts;

uint32_t cloud_proto_get_now_ts(void)
{
    return s_gw_now_ts;
}

/* ==================================================================
 * 21 条下行命�文档 4 �
 * ================================================================== */

typedef int (*cloud_cmd_fn)(const cloud_req_t *req, cJSON *data);

typedef struct
{
    const char   *name;
    cloud_cmd_fn  fn;
} cloud_cmd_entry_t;

/* 配置域名表：cfg.set/cfg.query 的 sections 过滤用（顺序 = 文档 3.2） */
const char *const cloud_cfg_section_names[] =
{
    "system", "network", "sensors", "thresholds", "gears", "devices", "rules", "ota", "meta"
};


/* 21 条命令已全部接真实实现; 占位函数 cloud_cmd_not_impl 于 2026-10-02 删除。 */

/* ==================================================================
 * cfg.set / cfg.query 真实�文档 gateway-protocol 5.2 �
 * ================================================================== */

/** �-> 域名。顺�= 文档 5.2 的固定顺� 也是应答�applied/rejected 的排列顺�*/
typedef struct
{
    uint8_t     bit;                    /* CFG_CH_* */
    const char *name;
} cfg_section_name_t;

static const cfg_section_name_t s_section_names[] =
{
    { CFG_CH_SYSTEM,     "system"     },
    { CFG_CH_NETWORK,    "network"    },
    { CFG_CH_SENSORS,    "sensors"    },
    { CFG_CH_THRESHOLDS, "thresholds" },
    { CFG_CH_GEARS,      "gears"      },
    { CFG_CH_DEVICES,    "devices"    },
    { CFG_CH_RULES,      "rules"      },
    { CFG_CH_OTA,        "ota"        }
};

#define CFG_SECTION_NAME_COUNT (sizeof(s_section_names) / sizeof(s_section_names[0]))

/** 把位掩码写成【域名字符串数组� ["thresholds","sensors"] (文档 5.2 应答形状) */
bool cloud_write_section_list(cJSON *obj, const char *key, uint8_t mask)
{
    cJSON  *arr = cJSON_CreateArray();
    uint8_t i;

    if (arr == NULL) { return false; }
    cJSON_AddItemToObject(obj, key, arr);

    for (i = 0u; i < (uint8_t)CFG_SECTION_NAME_COUNT; i++)
    {
        if ((mask & s_section_names[i].bit) != 0u)
        {
            /* CreateString 池满会返回 NULL, 此时 AddItemToArray 也返回 0 -> 整段失败 */
            if (cJSON_AddItemToArray(arr, cJSON_CreateString(s_section_names[i].name)) == 0)
            {
                return false;
            }
        }
    }
    return true;
}

/* cfg.set 的工作副� config_store_t ~1.7KB, 同样不能放栈上�� * 【为仢�么要在副本上做��先合并 -> 校验 -> �Flash, 全部成功才提交回"当前配置"
 * 实例。万丢� Flash 写失� RAM �Flash 不会出现"丢�个变了��一个没�的分裂状态��*/
config_store_t cloud_cfg_staging_buf;

/* ★ 2026-09-30 用户决定: 【不做 rejectedDetail(拒绝原因)】★
 *   拒绝的可能太多(每个域 x 每条校验规则), 要把"哪个域、哪条规则"的原因从
 *   config_store 逐级带出来, 成本远大于收益。
 *   所以 cfg.set 应答里的 rejected 只回【域名数组】(就是文档示例的格式):
 *   非空即表示"这一域整域被拒"(文档 5.2 的语义), 具体原因看网关日志。
 *   (曾短暂实现过 rejectedDetail + effectiveMode 回显, 按用户决定已移除。) */



/**
 * @brief cfg.query —— 文档 5.2
 *   参数: sections(字符串数� 缺省 ["all"])
 *   应答 data = 配置 JSON(�meta), 即配置对象本� 指定�sections 就只留那几个� *
 * 【关�sections�v55 起已按域过滤 —��下面这段注释以前写的�本期只支持全�,
 * 与实现不� 2026-09-30 �9 条一并订�
 *   实现: cloud_cfg_sections_mask() 把域名翻成位掩码, 再用
 *   cJSON_DeleteItemFromObject() 把没要的域从应答里删�—��语义与全量一� 只是省流量�� *   入参口径: 缺省 / ["all"] = 全量; 单个域名或域名数组都支持; 不认识的名字� *   空数组��类型不对��超长名字一�1001(绝不"悄悄少给丢�个域")� *   �数组里混了坏名字时��不部分生效】★ —��要么全给, 要么明确报错� */


bool cloud_proto_unix_to_calendar(uint32_t unix_ts, int32_t tz_offset_sec,
                                  uint16_t *year, uint8_t *month, uint8_t *day,
                                  uint8_t *hour, uint8_t *minute, uint8_t *second)
{
    int64_t  local;
    int64_t  days;
    int64_t  secs;
    int64_t  z;
    int64_t  era;
    uint32_t doe;
    uint32_t yoe;
    uint32_t doy;
    uint32_t mp;
    uint32_t y;

    if ((year == NULL) || (month == NULL) || (day == NULL) ||
        (hour == NULL) || (minute == NULL) || (second == NULL))
    {
        return false;
    }

    /* 先加上时区偏移再拆日/秒��用 64 位是为了让负偏移(西半�也能正确落地 —��     * unix_ts �uint32, 加上负偏移后可能变成负数�*/
    local = (int64_t)unix_ts + (int64_t)tz_offset_sec;

    /* ★ 向负无穷取整的除法 ★ 不能直接用 C 的 "/"(它向 0 取整),
     * 否则 1970-01-01 之前的时刻会差一天。 */
    days = local / 86400;
    secs = local % 86400;
    if (secs < 0)
    {
        secs += 86400;
        days -= 1;
    }

    /* civil_from_days: 天数 -> 年月�不查表��不�libc)�     * 719468 = 0000-03-01 �1970-01-01 的天� 146097 = 400 年天数��*/
    z   = days + 719468;
    era = (z >= 0) ? (z / 146097) : ((z - 146096) / 146097);
    doe = (uint32_t)(z - era * 146097);                                 /* [0,146096] */
    yoe = (doe - doe / 1460u + doe / 36524u - doe / 146096u) / 365u;    /* [0,399]   */
    y   = yoe + (uint32_t)era * 400u;
    doy = doe - (365u * yoe + yoe / 4u - yoe / 100u);                   /* [0,365]   */
    mp  = (5u * doy + 2u) / 153u;                                       /* [0,11] Mar=0 */
    *day   = (uint8_t)(doy - (153u * mp + 2u) / 5u + 1u);
    *month = (uint8_t)((mp < 10u) ? (mp + 3u) : (mp - 9u));
    if (*month <= 2u)
    {
        y += 1u;                    /* 1/2 月属�上一年的 3 月起� */
    }
    *year = (uint16_t)y;

    *hour   = (uint8_t)(secs / 3600);
    *minute = (uint8_t)((secs % 3600) / 60);
    *second = (uint8_t)(secs % 60);
    return true;
}


/* 取一个域名对应的� 不认识返�-1 */
static int cloud_cfg_section_bit(const char *name)
{
    uint8_t i;

    for (i = 0u; i < (uint8_t)CLOUD_CFG_SECTION_N; i++)
    {
        if (strcmp(cloud_cfg_section_names[i], name) == 0)
        {
            return (int)i;
        }
    }
    return -1;
}

/**
 * @brief 解析 sections, 得出"该保留哪些域"的位掩码
 * @return CLOUD_EC_OK / CLOUD_EC_PARAM
 * @note  没给 sections -> 全都�与以前一�;
 *        "all" -> 全都� 单个域名 �域名数组 -> 只要这些;
 *        空数�/ 名字不认�/ 长度超缓�-> 1001(不猜、不静默当成 all)� */
int cloud_cfg_sections_mask(const cloud_req_t *req, uint16_t *mask)
{
    cj_type_t t;
    char      s[16];
    uint8_t   i;
    uint8_t   cnt;

    *mask = (uint16_t)((1u << CLOUD_CFG_SECTION_N) - 1u);   /* 默认: 全都要 */

    if ((req->data == NULL) || (cj_get(req->data, "sections", NULL, 0) == CJ_NOTFOUND))
    {
        return CLOUD_EC_OK;                 /* 不带参数 = 全量, 保持老行为 */
    }

    t = cj_get(req->data, "sections", NULL, 0);
    if (t == CJ_STR)
    {
        int bit;

        if (!cj_get_str(req->data, "sections", s, (uint16_t)sizeof(s)))
        {
            return CLOUD_EC_PARAM;          /* 超长, 不截断后当合法 */
        }
        if (strcmp(s, "all") == 0)
        {
            return CLOUD_EC_OK;
        }
        /* ★ 必须用 int 接返回值 ★ 先转 uint8_t 再判 <0 会把 -1 变成 255,
         *   于是未知域名被当成合� 还会产生 1u<<255 这种未定义移位��*/
        bit = cloud_cfg_section_bit(s);
        if (bit < 0)
        {
            return CLOUD_EC_PARAM;          /* 不认识的名字 */
        }
        *mask = (uint16_t)(1u << (uint8_t)bit);
        return CLOUD_EC_OK;
    }

    if (t != CJ_ARR)
    {
        return CLOUD_EC_PARAM;              /* 既不是字符串也不是数组 */
    }

    cnt = (uint8_t)cj_count(req->data, "sections");
    if ((cnt == 0u) || (cnt > (uint8_t)CLOUD_CFG_SECTION_N))
    {
        return CLOUD_EC_PARAM;              /* 空数组 / 条数超上限 */
    }

    *mask = 0u;
    for (i = 0u; i < cnt; i++)
    {
        char path[24];
        int  bit;

        (void)snprintf(path, sizeof(path), "sections[%u]", (unsigned)i);
        if (!cj_get_str(req->data, path, s, (uint16_t)sizeof(s)))
        {
            return CLOUD_EC_PARAM;
        }
        if (strcmp(s, "all") == 0)
        {
            *mask = (uint16_t)((1u << CLOUD_CFG_SECTION_N) - 1u);   /* "all" 压过其它 */
            return CLOUD_EC_OK;
        }
        bit = cloud_cfg_section_bit(s);
        if (bit < 0)
        {
            return CLOUD_EC_PARAM;
        }
        *mask |= (uint16_t)(1u << (uint8_t)bit);
    }
    return CLOUD_EC_OK;
}


/* ==================================================================
 * sys.factory_reset 真实�文档 gateway-protocol 5.1 �4 � —��P2-5
 * ==================================================================
 * 文档原文(参数表 + 响应):
 *   "4. `sys.factory_reset` — 恢复出厂
 *    参数: keepNetwork  bool  默认 true   保留 network �避免恢复后失�
 *    响应: code 0。其余配置域恢复出厂默认，sensors[]/devices[] 清空，configVersion 归零�
 *
 * 实现要点:
 *   1. keepNetwork 默认 true —���不带这个参数时保�network �;
 *   2. 其余 8 个域回����config_store 的出厂默�config_store_factory_default),
 *      sensors[]/devices[] 清空、configVersion 归零(工厂默认函数里就是如�;
 *   3. 【一�Flash 提交� 朢�终要落盘的那份配置先在副本里拼好再写丢��Flash,
 *      绝不出现"Flash 里短暂处�network 被清�的中间��
 *   4. 【失败就�1007� 落盘失败绝不能回 code=0 —��平台�code=0 认为已经恢复
 *      出厂, 现场丢�重启配置又回来了, 属于朢�难查�答应了却没做�;
 *   5. 落盘成功才提交给"当前配置"实例, 否则紧接睢��cfg.query 会读到旧� *      (cfg.set 在同丢�处踩过坑, 见下�cloud_cmd_cfg_set 的注�� */


/* ==================================================================
 * v34+: sen.unbind / dev.unbind / rule.mode(文档 5.3/5.4/5.5)
 * ==================================================================
 * 【统丢�范式: 事务式改配置】与 cfg.set 丢�模一� 新命令一律照这个�
 *     在副本上改 -> config_store_validate() -> config_store_touch()(版本+1)
 *     -> config_store_save() �Flash -> 成功�memcpy �当前配置"实例
 * 这样 Flash 写失败时 RAM �Flash 不会出现"丢�个变了��一个没�的分裂状态�� * 副本复用 cloud_cfg_staging_buf(1.7KB, 放栈上会�4KB 的栈); 本工程单线程轮询, 不会重入� * ================================================================== */

/** 把配置副本落盘并提交; 失败时当前配置一个字都不动 */
int cloud_commit_config(const config_store_t *tmp)
{
    config_store_t *live;

    if (config_store_validate(tmp) != CFG_OK)
    {
        return CLOUD_EC_PARAM;              /* 改完反而不合法: 当成参数错 */
    }
    config_store_touch((config_store_t *)tmp, s_gw_now_ts);

    if (config_store_save(tmp) != CFG_OK)
    {
        log_info("[CLOUD] !! 配置落 Flash 失败, 回 1007, 当前配置未改动\r\n");
        return CLOUD_EC_INTERNAL;
    }

    live = config_store_get_mut();
    (void)memcpy(live, tmp, sizeof(*live));
    return CLOUD_EC_OK;
}



/* ==================== 命令表(顺序 = 文档 4 章) ==================== */
static const cloud_cmd_entry_t s_cmd_table[] =
{
    /* 1~4 系统�P2-5 已把 sys.factory_reset 换成真实� */
    { "sys.query_status",  cloud_cmd_sys_query_status },
    { "sys.time_sync",     cloud_cmd_sys_time_sync },
    { "sys.reboot",        cloud_cmd_sys_reboot },
    { "sys.factory_reset", cloud_cmd_sys_factory_reset },
    /* 5~6 配置�P2-4 已接真实� */
    { "cfg.set",           cloud_cmd_cfg_set   },
    { "cfg.query",         cloud_cmd_cfg_query },
    /* 7~10 传感器类 */
    { "sen.bind",          cloud_cmd_sen_bind },
    { "sen.unbind",        cloud_cmd_sen_unbind },
    { "sen.set",           cloud_cmd_sen_set },
    { "sen.query",         cloud_cmd_sen_query },
    /* 11~15 联动设备类 */
    { "dev.bind",          cloud_cmd_dev_bind },
    { "dev.unbind",        cloud_cmd_dev_unbind },
    { "dev.set",           cloud_cmd_dev_set },
    { "dev.ctrl",          cloud_cmd_dev_ctrl },
    { "dev.query",         cloud_cmd_dev_query },
    /* 16~18 联动规则类 */
    { "rule.set",          cloud_cmd_rule_set },
    { "rule.query",        cloud_cmd_rule_query },
    { "rule.mode",         cloud_cmd_rule_mode },
    /* 19~21 升级类 —— 2026-10-02 起接真实实现(此前恒回 1006) */
    { "ota.start",         cloud_cmd_ota_start },
    { "ota.query",         cloud_cmd_ota_query },
    { "ota.abort",         cloud_cmd_ota_abort }
};

#define CLOUD_CMD_TABLE_LEN (sizeof(s_cmd_table) / sizeof(s_cmd_table[0]))

static const cloud_cmd_entry_t *cloud_cmd_find(const char *name)
{
    uint8_t i;

    for (i = 0u; i < (uint8_t)CLOUD_CMD_TABLE_LEN; i++)
    {
        if (strcmp(s_cmd_table[i].name, name) == 0)
        {
            return &s_cmd_table[i];
        }
    }
    return NULL;
}

uint8_t cloud_proto_cmd_count(void)
{
    return (uint8_t)CLOUD_CMD_TABLE_LEN;
}

bool cloud_proto_is_known_cmd(const char *cmd)
{
    if (cmd == NULL)
    {
        return false;
    }
    return (cloud_cmd_find(cmd) != NULL);
}

const char *cloud_proto_ec_msg(int code)
{
    switch (code)
    {
        case CLOUD_EC_OK:           return "ok";
        case CLOUD_EC_PARAM:        return "param error";
        case CLOUD_EC_VER_CONFLICT: return "config version conflict";
        case CLOUD_EC_NO_DEVICE:    return "device not found";
        case CLOUD_EC_DEV_OFFLINE:  return "device offline";
        case CLOUD_EC_LORA_TIMEOUT: return "lora timeout";
        case CLOUD_EC_STATE:        return "not allowed in current state";
        case CLOUD_EC_INTERNAL:     return "internal error";
        case CLOUD_EC_AUTH:         return "auth failed";
        case CLOUD_EC_LIMIT:        return "device limit reached";
        case CLOUD_EC_NO_SPACE:     return "no space";
        case CLOUD_EC_OTA_URL:      return "ota url invalid";
        case CLOUD_EC_OTA_DOWNLOAD: return "ota download failed";
        case CLOUD_EC_OTA_VERIFY:   return "ota verify failed";
        default:                    return "error";
    }
}

/* ==================================================================
 * 信封解析 / 生成
 * ================================================================== */

/*!
 * \brief �cJSON 字符串节点拷进定长缓�绝不截断)
 * \return true = 拷成� false = 缺失/非字符串/装不� * \note  v33 迁到 cJSON 后新增��原来用 cj_get_str() 是��截断��语� 而截断在 cmd � *        是危险的: "cfg.setXXXX" 会被截成 "cfg.set" 而��静默命中另丢�条命令������ *        那比"�1001"严重得多。所以这里改�装不下就失败", 由调用方判成 1001� */
static bool cloud_proto_take_str(const cJSON *item, char *dst, uint16_t cap)
{
    const char *s;
    size_t      len;

    if ((dst == NULL) || (cap == 0u))
    {
        return false;
    }
    dst[0] = '\0';
    if (!cJSON_IsString(item))
    {
        return false;
    }
    s = cJSON_GetStringValue(item);
    if (s == NULL)
    {
        return false;
    }
    len = strlen(s);
    if (len >= (size_t)cap)
    {
        return false;
    }
    (void)memcpy(dst, s, len + 1u);
    return true;
}

int cloud_proto_parse_req(const char *json, cloud_req_t *out)
{
    cJSON *root;
    cJSON *it;
    int    rc = 0;

    if ((json == NULL) || (out == NULL))
    {
        return CLOUD_EC_PARAM;
    }
    (void)memset(out, 0, sizeof(*out));

    /* 护栏: hooks 没装(忘了 cjson_port_init())就绝不能�cJSON —��它会去用�     * 512 字节的堆, 而且是��没有任何编�链接错误】的静默故障�*/
    if (!cjson_port_is_ready())
    {
        return CLOUD_EC_INTERNAL;
    }

    /* v33: 信封解析改用 cJSON。整棵树只用�取那几个字段", 取完立刻 Delete —��     * 本工�RAM 是静态的(�24 KB, �cjson_port.h), 不留树��*/
    root = cJSON_Parse(json);
    if ((root == NULL) || !cJSON_IsObject(root))
    {
        if (root != NULL)
        {
            cJSON_Delete(root);
        }
        return CLOUD_EC_PARAM;
    }

    /* cmd 必填 */
    if (!cloud_proto_take_str(cJSON_GetObjectItemCaseSensitive(root, "cmd"),
                              out->cmd, (uint16_t)sizeof(out->cmd)) ||
        (out->cmd[0] == '\0'))
    {
        rc = CLOUD_EC_PARAM;
    }

    /* msgId: 文档规定必带�<=16 字节。缺�非字符串【宽容��上层与幂等层自己决定),
     * 但超�16 字节直接�—��否则应答里带不回原�� 平台对不上��*/
    if (rc == 0)
    {
        it = cJSON_GetObjectItemCaseSensitive(root, "msgId");
        if (cJSON_IsString(it) &&
            (!cloud_proto_take_str(it, out->msg_id, (uint16_t)sizeof(out->msg_id)) ||
             (strlen(out->msg_id) > 16u)))
        {
            rc = CLOUD_EC_PARAM;
        }
    }

    if (rc == 0)
    {
        /* auth: 超长按��截断��处�截断后必然和密码对不�-> 1008), 不为此单独报�*/
        it = cJSON_GetObjectItemCaseSensitive(root, "auth");
        if (cJSON_IsString(it))
        {
            const char *s   = cJSON_GetStringValue(it);
            size_t      len = (s != NULL) ? strlen(s) : 0u;

            if (len >= (size_t)sizeof(out->auth))
            {
                len = (size_t)sizeof(out->auth) - 1u;
            }
            if (len > 0u)
            {
                (void)memcpy(out->auth, s, len);
            }
            out->auth[len] = '\0';
        }

        it = cJSON_GetObjectItemCaseSensitive(root, "ts");
        if (cJSON_IsNumber(it))
        {
            out->ts     = (uint32_t)cJSON_GetNumberValue(it);
            out->has_ts = true;
        }

        /* data 仍然交出【原文指针�� 配置�config_store)这一步还在用 cloud_json �         * cj_* 取字� 直接给它 cJSON 树会牵动丢�大片。指针指向调用方�json 缓冲,
         * 生命周期与迁移前完全丢�致��         * TODO(v33-2): 配置层也迁到 cJSON 之后, 这里改成交出 data 节点�*/
        out->data = cj_get_sub(json, "data");
    }

    cJSON_Delete(root);
    return rc;
}

/* ==================================================================
 * 信封生成(改用 cJSON)
 * ==================================================================
 * 【为仢�么一律用 cJSON_PrintPreallocated, 不用 cJSON_Print/PrintUnformatted� *   1) Print/PrintUnformatted 霢�要额外一�边打印边扩容"的缓�—��实测多花
 *      7.5 KB 池空��cjson_port.h 的实测表);
 *   2) 更要命的是池里没�realloc, 扩容�新分�+ 拷贝 + 释放旧块"; 池被解析� *      切碎之后很可能找不到足够大的连续�—��实测 24 KB 池下【直接失败���� *   PrintPreallocated 直接打进入调用方的静态缓� 零额外分� 也没有碎片问题�� * ================================================================== */

/*!
 * \brief 把一�cJSON 树以【无格式】打进入调用方的静��缓� * \return 写入长度; 0 = 失败(缓冲不足/树有问题)
 * \note  失败时把 out 置成空串 —��绝不能发半条应答出去� */
uint16_t cloud_json_print(char *out, uint16_t cap, cJSON *root)
{
    if ((out == NULL) || (cap < 2u) || (root == NULL))
    {
        return 0u;
    }
    if (cJSON_PrintPreallocated(root, out, (int)cap, 0) == 0)
    {
        out[0] = '\0';
        return 0u;
    }
    return (uint16_t)strlen(out);
}

/*!
 * \brief 按【文档 3 章示例的键序】往应答对象里填字段
 * \note  键序不能随手调换: 单测 [7] 是拿整条应答与文档示例����字符��比对的� *        data �cJSON_Raw 原样嵌入 —��它在 cJSON �print_value 里是 memcpy,
 *        不加引号也不二次转义, 与迁移前 cjw_raw 的行为一致�� */
static bool cloud_proto_add_resp(cJSON *root, const cloud_req_t *req, int code,
                                 const char *msg, const char *data_json, uint32_t ts)
{
    if (cJSON_AddStringToObject(root, "cmd", req->cmd) == NULL)       { return false; }
    if (cJSON_AddStringToObject(root, "msgId", req->msg_id) == NULL)  { return false; }
    if (cJSON_AddNumberToObject(root, "code", (double)code) == NULL)  { return false; }
    if (cJSON_AddStringToObject(root, "msg",
            (msg != NULL) ? msg : cloud_proto_ec_msg(code)) == NULL)  { return false; }
    if (cJSON_AddNumberToObject(root, "ts", (double)ts) == NULL)      { return false; }

    if ((data_json != NULL) && (data_json[0] != '\0'))
    {
        if (cJSON_AddRawToObject(root, "data", data_json) == NULL)    { return false; }
    }
    else
    {
        if (cJSON_AddObjectToObject(root, "data") == NULL)            { return false; }
    }
    return true;
}

uint16_t cloud_proto_build_resp(char *out, uint16_t cap, const cloud_req_t *req,
                                int code, const char *msg, const char *data_json,
                                uint32_t ts)
{
    cJSON   *root;
    uint16_t n;

    if ((out == NULL) || (cap < 16u) || (req == NULL))
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
        out[0] = '\0';                  /* 池满: 明确失败, 不睁一只眼闭一只眼 */
        return 0u;
    }

    if (!cloud_proto_add_resp(root, req, code, msg, data_json, ts))
    {
        cJSON_Delete(root);             /* 任何丢�步失�池满) -> 整条应答作废 */
        out[0] = '\0';
        return 0u;
    }

    n = cloud_json_print(out, cap, root);
    cJSON_Delete(root);
    return n;
}

uint16_t cloud_proto_build_uplink(char *out, uint16_t cap, const char *event,
                                  const char *msg_id, uint32_t ts, const char *data_json)
{
    cJSON   *root;
    uint16_t n;

    if ((out == NULL) || (cap < 16u) || (event == NULL))
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

    /* 键序 = 文档 3 章的 {event,msgId,ts,data} */
    if ((cJSON_AddStringToObject(root, "event", event) == NULL) ||
        (cJSON_AddStringToObject(root, "msgId",
            (msg_id != NULL) ? msg_id : "") == NULL) ||
        (cJSON_AddNumberToObject(root, "ts", (double)ts) == NULL) ||
        (((data_json != NULL) && (data_json[0] != '\0'))
            ? (cJSON_AddRawToObject(root, "data", data_json) == NULL)
            : (cJSON_AddObjectToObject(root, "data") == NULL)))
    {
        cJSON_Delete(root);
        out[0] = '\0';
        return 0u;
    }

    n = cloud_json_print(out, cap, root);
    cJSON_Delete(root);
    return n;
}

bool cloud_proto_looks_like_json(const uint8_t *data, uint16_t len)
{
    uint16_t i = 0u;
    uint16_t j;

    if ((data == NULL) || (len == 0u))
    {
        return false;
    }
    /* 跳过前导空白 */
    while ((i < len) && ((data[i] == ' ') || (data[i] == '\t') ||
                         (data[i] == '\r') || (data[i] == '\n')))
    {
        i++;
    }
    if ((i >= len) || (data[i] != '{'))
    {
        return false;
    }
    /* 必须还能找到 '}' �'"' —��免得�恰好�'{' 弢��的二进制帧误判成 JSON */
    for (j = (uint16_t)(i + 1u); j < len; j++)
    {
        if ((data[j] == '}') || (data[j] == '"'))
        {
            return true;
        }
    }
    return false;
}

/* ==================================================================
 * status.report 数据段 + 时间换算
 * ================================================================== */



/**
 * @brief 公历 -> 儒略日数(Hinnant 算法), 返回 1970-01-01 起算的天� * @note  �1970 之前也能给负� 本工程用不到但算法本身支持�� */
static int32_t cloud_days_from_civil(int32_t y, int32_t m, int32_t d)
{
    int32_t  era;
    uint32_t yoe;
    uint32_t doy;
    uint32_t doe;

    y -= (m <= 2) ? 1 : 0;
    era  = (y >= 0) ? (y / 400) : ((y - 399) / 400);
    yoe  = (uint32_t)(y - (era * 400));                       /* [0, 399] */
    doy  = (uint32_t)(((153 * (m + ((m > 2) ? -3 : 9))) + 2) / 5 + d - 1);  /* [0, 365] */
    doe  = (yoe * 365u) + (yoe / 4u) - (yoe / 100u) + doy;    /* [0, 146096] */
    return (era * 146097) + (int32_t)doe - 719468;
}

uint32_t cloud_proto_calendar_to_unix(uint16_t year, uint8_t month, uint8_t day,
                                      uint8_t hour, uint8_t minute, uint8_t second,
                                      int32_t tz_offset_sec)
{
    int32_t  days;
    int64_t  secs;

    if ((month < 1u) || (month > 12u) || (day < 1u) || (day > 31u) ||
        (hour > 23u) || (minute > 59u) || (second > 60u))
    {
        return 0u;                      /* 字段非法: 返回 0 让上层能看出来 */
    }

    days = cloud_days_from_civil((int32_t)year, (int32_t)month, (int32_t)day);
    secs = ((int64_t)days * 86400) + ((int64_t)hour * 3600) +
           ((int64_t)minute * 60) + (int64_t)second - (int64_t)tz_offset_sec;

    if (secs < 0)
    {
        return 0u;
    }
    return (uint32_t)secs;
}

/* ==================================================================
 * P1-6 msgId 幂等 —��"同一�msgId 不重复执�
 * ==================================================================
 * 【为仢�么必须有】云端可能把同一条命令重发一�应答在路上丢�/ MQTT QoS1 重传 /
 * 平台自己等应答超时后重试)。在此之前重发会【真的再执行丢�遍�� cfg.set 会再写一� * Flash �configVersion �+1, sys.factory_reset 会再擦一次配置��P1-6 的验收判� * 就是"重复 msgId 不重复执�� *
 * 【判据与做法� *   记住朢��CLOUD_DEDUP_SLOTS 条��已通过解析与鉴权��的 msgId, 连同它产生的应答
 *   正文原样存下来��再见到同一�msgId �
 *     · 丢�个命令处理函数都不调�—��扢�以不会有第二次副作用;
 *     · 把上次那条应答����字节��重发出�—��扢�以平台前后看到的答案是同丢�份�� *   重发时保留的是应答里【原本的 ts�同一条��辑应答), 刻意不改成当前时间�� *
 * 【为仢�么单条应答只缓存 CLOUD_DEDUP_RESP_MAX(512 B)� *   实测只有 cfg.query 的应答会超过 512 B(完整配置对象 1572 B, 满配朢���.7 KB),
 *   而它是��只读��没有副作用】的。所以缓存不下时逢�化成"照常执行丢��是��正确��的:
 *   平台照样拿到完整正文, 不会看到被截断的配置 —���为了缓存把它截断"好得多�� *   �这条推理的前提是"超过 512 B 的应答一定来自只读命�。将来若有��带副作用��的
 *     命令产生 >512 B 的应� 必须同步�CLOUD_DEDUP_RESP_MAX 调大, 否则那条命令� *     重发会被真的执行第二遍�� *
 * 【刻意不做的两件事�� *   · 不做掉电持久� 网关重启后记忆就没了, 重启前的重发会再执行丢�次��写 Flash 会带� *     磨损�重启即写"的问� 不��得 —��作为已知限制写进 v31 的说�txt� *   · 不清空在 MQTT 重连� �cloud_proto_dedup_reset() 的说明�� *
 * 【必须放在鉴权之后��重复命中检查要放在 cloud_profile_check_auth() 之后: 否则丢�� *   不带 auth(�auth �的报文只要抄丢�个见过的 msgId, 就能直接拿到上次的成功应�—�� *   那是丢�个鉴权绕过��见主机单测 [21] �重放不绕过鉴�用例� * ================================================================== */

#define CLOUD_DEDUP_SLOTS      4u      /* 记住最近 4 条命令 */
#define CLOUD_DEDUP_RESP_MAX   512u    /* 单条应答缓存上限(理由见上) */

/* ★★2026-10-08: 幂等的【时间窗口】★★
 *
 * 【为什么必须加 —— 现场真的踩了】
 *   14:01:06 下发 ota.start msgId=ota-02 → 受理, 但升级失败(2002)。
 *   14:34:37 用户隔 33 分钟再次下发【同名】ota-02(想重跑一次):
 *       [CLOUD] msgId=ota-02 重复(P1-6 幂等命中): 不执行 ota.start, 直接重发上次应答
 *   ⇒ 网关什么都没干, 却回 code 0(看起来像成功), 用户完全不知道为什么。
 *
 * 【根因】老实现只记"最近 4 条 msgId", 没有任何时间限制 ——
 *   一条命令只要还留在槽里就永远算"重复"。
 *
 * 【需求文档的本意不是这样】《gateway-protocol》第 3 章:
 *   "平台下发命令后【10 秒】未收到应答可重发(同 msgId)"
 *   ⇒ 同 msgId 是【短窗口内的重发】语义(防网络重复投递), 不是"永久记住这个名字"。
 *     隔半小时再发同一个名字, 在平台的模型里就是一条新命令。
 *
 * 【窗口取 60s】文档的重试窗口是 10s; 60s 比平台实际重试节奏宽裕得多
 *   (不会漏挡真正的重复投递), 又远小于"人隔几十分钟手动重发"的间隔。
 *
 * 【时钟没同步时】now_ts 可能还是 0(没校时), 此时 now-ts==0 ⇒ 落在窗口内 ⇒
 *   退回老行为(一律去重)。刻意的: 宁可多挡一次重复, 也不要因时钟不可用丢掉防重复能力。 */
#ifndef CLOUD_DEDUP_WINDOW_S
#define CLOUD_DEDUP_WINDOW_S   60u
#endif

typedef struct
{
    char     msg_id[CLOUD_MSGID_MAX];   /* 空串 = 该槽位还没用过 */
    uint32_t ts;                        /* ★2026-10-08: 处理这条命令的时刻(Unix秒), 用于时间窗 */
    int      code;                      /* 上次的返回码 */
    uint16_t len;                       /* 上次应答长度(逐字节重发用) */
    char     resp[CLOUD_DEDUP_RESP_MAX];
} cloud_dedup_slot_t;

static cloud_dedup_slot_t s_dedup[CLOUD_DEDUP_SLOTS];
static uint8_t            s_dedup_next;     /* 环形写指� 指向朢�旧的�下次覆盖� */
static uint32_t           s_dedup_hits;     /* 累计命中次数(只增不减, 诊断� */

void cloud_proto_dedup_reset(void)
{
    (void)memset(s_dedup, 0, sizeof(s_dedup));
    s_dedup_next = 0u;
    s_dedup_hits = 0u;
}

uint32_t cloud_proto_dedup_hit_count(void)
{
    return s_dedup_hits;
}

/*!
 * \brief 在记忆里找 msg_id; 找不到返回 NULL
 * \note  调用方保�msg_id 非空(�msgId 不参与去� �cloud_proto_handle)
 */
static const cloud_dedup_slot_t *cloud_dedup_find(const char *msg_id, uint32_t now_ts)
{
    uint8_t i;

    for (i = 0u; i < (uint8_t)CLOUD_DEDUP_SLOTS; i++)
    {
        if ((s_dedup[i].msg_id[0] != '\0') &&
            (strcmp(s_dedup[i].msg_id, msg_id) == 0))
        {
            /* ★2026-10-08: 超出时间窗口 = 这是一条"同名的新命令", 不再算重复★ */
            if ((uint32_t)(now_ts - s_dedup[i].ts) > (uint32_t)CLOUD_DEDUP_WINDOW_S)
            {
                log_info("[CLOUD] msgId=%s 虽见过, 但已隔 %lus(>窗口 %us) —— "
                         "按【新命令】执行, 不再当成重发\r\n",
                         msg_id,
                         (unsigned long)(uint32_t)(now_ts - s_dedup[i].ts),
                         (unsigned)CLOUD_DEDUP_WINDOW_S);
                return NULL;
            }
            return &s_dedup[i];
        }
    }
    return NULL;
}

/*!
 * \brief 记住丢��msgId 与它产生的应�供重发时逐字节重�
 * \note  装不�> CLOUD_DEDUP_RESP_MAX)就不�—��见文件上�为什�丢�节�� *        环形覆盖: 槽满时覆盖最旧的丢�条�� */
static void cloud_dedup_remember(const char *msg_id, int code,
                                 const char *resp, uint16_t len, uint32_t now_ts)
{
    cloud_dedup_slot_t *slot;
    size_t              idlen;

    if ((msg_id == NULL) || (msg_id[0] == '\0') ||
        (resp == NULL) || (len == 0u) ||
        (len > (uint16_t)CLOUD_DEDUP_RESP_MAX))
    {
        return;
    }

    idlen = strlen(msg_id);
    if (idlen >= sizeof(s_dedup[0].msg_id))
    {
        return;                             /* 防御: 超长 msgId 不记(正常到不� */
    }

    slot = &s_dedup[s_dedup_next];
    (void)memcpy(slot->msg_id, msg_id, idlen + 1u);
    slot->ts   = now_ts;                /* ★2026-10-08: 时间窗判据(见 cloud_dedup_find)★ */
    slot->code = code;
    slot->len  = len;
    (void)memcpy(slot->resp, resp, len);

    s_dedup_next = (uint8_t)((s_dedup_next + 1u) % (uint8_t)CLOUD_DEDUP_SLOTS);
}

/* ==================================================================
 * 命令处理
 * ================================================================== */

int cloud_proto_handle(const char *json, uint32_t now_ts,
                       char *resp, uint16_t resp_cap, uint16_t *resp_len)
{
    cloud_req_t              req;
    const cloud_cmd_entry_t *entry;
    const char              *msg;
    uint16_t                 data_len = 0u;
    uint16_t                 n;
    int                      rc;

    if (resp_len != NULL)
    {
        *resp_len = 0u;
    }
    if ((resp == NULL) || (resp_cap == 0u))
    {
        return CLOUD_EC_INTERNAL;
    }
    resp[0] = '\0';

    /* ---- 1. 解析信封 ---- */
    rc = cloud_proto_parse_req(json, &req);
    if (rc != 0)
    {
        cloud_req_t empty;
        (void)memset(&empty, 0, sizeof(empty));
        n = cloud_proto_build_resp(resp, resp_cap, &empty, CLOUD_EC_PARAM,
                                   "bad request", NULL, now_ts);
        if (resp_len != NULL)
        {
            *resp_len = n;
        }
        return CLOUD_EC_PARAM;
    }

    /* ---- 2. 应用层鉴�文档: 启用后不带正�auth 应答 1008) ---- */
    if (!cloud_profile_check_auth(req.auth))
    {
        n = cloud_proto_build_resp(resp, resp_cap, &req, CLOUD_EC_AUTH,
                                   cloud_proto_ec_msg(CLOUD_EC_AUTH), NULL, now_ts);
        if (resp_len != NULL)
        {
            *resp_len = n;
        }
        return CLOUD_EC_AUTH;
    }

    /* ---- 3. P1-6 幂等: 这个 msgId 是不是已经处理过了? ----
     * 位置刻意放在【鉴权之后�� 若放在鉴权之� 丢�条不�auth(�auth �的报�     * 只要抄一个见过的 msgId 就能直接拿到上次的成功应�—��那是鉴权绕过�     * msgId 为空则不去重: parse_req �msgId 是宽容的(允许�, 而空 id 之间没有
     * 区分� 若参与去重会�扢�有没�msgId 的命�当成同一条互相顶掉��*/
    if (req.msg_id[0] != '\0')
    {
        const cloud_dedup_slot_t *hit = cloud_dedup_find(req.msg_id, now_ts);

        if (hit != NULL)
        {
            uint16_t copy_len = hit->len;

            s_dedup_hits++;

            /* 调用方缓冲比缓存上限(512 B)大得� 正常到不了这� 万一真到了也
             * 【绝不能】改去执行一遍命�那就会有第二次副作用), 而是逢�化成
             * "只回上次的 code、不带 data", 至少保证不重复执行。 */
            if (((uint32_t)copy_len + 1u) > (uint32_t)resp_cap)
            {
                copy_len = 0u;
            }

            if (copy_len > 0u)
            {
                (void)memcpy(resp, hit->resp, copy_len);
            }
            resp[copy_len] = '\0';
            if (resp_len != NULL)
            {
                *resp_len = copy_len;
            }

            log_info("[CLOUD] msgId=%s 重复(P1-6 幂等命中): 不执行 %s, 直接重发上次应答 %u 字节\r\n",
                     req.msg_id, req.cmd, (unsigned)copy_len);
            return hit->code;
        }
    }

    /* ---- 4. 查表分发 ---- */
    entry = cloud_cmd_find(req.cmd);
    if (entry == NULL)
    {
        rc  = CLOUD_EC_PARAM;
        msg = "unknown cmd";
    }
    else
    {
        /* ---- v33-2: data 段改用 cJSON ----
         * 处理函数徢�我们建好的对象里挂成� 挂完再��打进入】文件级 static �         * s_data_buf (= CFG_JSON_MAX+64, 不能放栈�—��cfg.query �data 是整份配�
         * 满配实测 3584 B, �4KB 的栈上会�, 朢�后交给下�build_resp �cJSON_Raw
         * 原样嵌进信封�         * 【为仢�么打完再� 而不是把对象直接挂进信封】build_resp 的对外契约是
         * "data 是一段 JSON 文本"(单测 [7] 逐字符比对就建立在这个契约上), 这样
         * 它的签名与测试都不用� 代价只是多打丢��用的�PrintPreallocated, 不花 RAM)�*/
        cJSON *dobj = cJSON_CreateObject();

        /* cfg.set 要用"网关当前时间"写 meta.updatedAt, 而处理函数契约里没有
         * now_ts 参数, 扢�以在这里先记下来(单线� 无重入问��*/
        s_gw_now_ts = now_ts;

        if (dobj == NULL)
        {
            rc       = CLOUD_EC_INTERNAL;       /* 池满 */
            data_len = 0u;
        }
        else
        {
            rc = entry->fn(&req, dobj);

            if (cJSON_PrintPreallocated(dobj, s_data_buf,
                                        (int)sizeof(s_data_buf), 0) == 0)
            {
                rc       = CLOUD_EC_INTERNAL;   /* 池满/缓冲不足: 明确失败 */
                data_len = 0u;
            }
            else
            {
                data_len = (uint16_t)strlen(s_data_buf);
            }
            cJSON_Delete(dobj);
        }
        msg = cloud_proto_ec_msg(rc);
    }

    /* ---- 文档 §8: 1002 的应答必须带 data.currentVersion ----
     * 错误码表原文: "1002 | 配置版本冲突（expectVersion 不一� data.currentVersion
     * 返回当前版本� —��平台据此把本地补到对的版本再重发, 少了这个字段它只能瞎猜��     * 【为仢�么按错误码统丢�处理�002 只有"版本冲突"这一种含� 放在分发出口比让
     * 每个命令各自记得填更可靠(目前只有 cfg.set 会返�1002, 以后别的命令也可能用)�*/
    if (rc == CLOUD_EC_VER_CONFLICT)
    {
        int m = snprintf(s_data_buf, sizeof(s_data_buf), "{\"currentVersion\":%lu}",
                         (unsigned long)config_store_version());

        data_len = ((m > 0) && ((uint16_t)m < (uint16_t)sizeof(s_data_buf)))
                       ? (uint16_t)m : 0u;
    }

    /* ---- 5. 组装应答 ---- */
    n = cloud_proto_build_resp(resp, resp_cap, &req, rc, msg,
                               (data_len > 0u) ? s_data_buf : NULL, now_ts);
    if (resp_len != NULL)
    {
        *resp_len = n;
    }
    if (n == 0u)
    {
        return CLOUD_EC_INTERNAL;
    }

    /* P1-6: 把这�msgId 与应答记� 供云端重发时逐字节重放��     * 只有【走到这里��的报文才占槽位 —��解析失败 / 鉴权失败的报文不� 它们本来
     * 就没有副作用, 也没有一�值得重放的应��*/
    cloud_dedup_remember(req.msg_id, rc, resp, n, now_ts);

    return rc;
}
