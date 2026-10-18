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

/**
 * @brief sen.unbind / dev.unbind —— 文档 5.3 / 5.4
 *   参数: uid(本工程口� 文档写的�addr, 按用户决策一律映射成 uid)
 *   响应: code 0(没有 data 字段)
 *
 * 【P5 改为异步�2026-09-29):
 *   不再立刻�config_store —��先调 lora_business_unbind() 向设备发 0x05 解绑�
 *   网关在收到设备回�0x80 ACK(或重试��尽)时才真正删除 config_store + 从机�
 *   并��过事件回调触发 event.device{unbound} 上报云端。本函数只负责发起解绑请�
 *   立即返回 code 0(异步), 让云端靠"unbound 事件"确认完成, 而不是靠命令响应� *
 *   uid 没绑�-> 1003(目标设备不存�; 已有解绑在进�-> 1006(状��不允许)� *
 *   is_sensor 在异步化后已不需�lora_business 内部按从机表 dev_type 自查),
 *   保留参数是为�sen.unbind / dev.unbind 两个分发表项共用本函数�� */
static int cloud_cmd_unbind(const cloud_req_t *req, cJSON *data, bool is_sensor)
{
    char uid[CFG_UID_STR_MAX];

    (void)data;                             /* 文档这条响应没有 data */
    (void)is_sensor;                        /* 异步化后 lora_business 自查 dev_type */

    if (req->data == NULL)
    {
        return CLOUD_EC_PARAM;
    }
    if (!cj_get_str(req->data, "uid", uid, (uint16_t)sizeof(uid)))
    {
        return CLOUD_EC_PARAM;
    }
    if (!cfg_uid_valid(uid))
    {
        return CLOUD_EC_PARAM;
    }

    int r = lora_business_unbind(uid);
    switch (r)
    {
    case 0:
        return CLOUD_EC_OK;                 /* 异步发起, 真正删除在收到 ACK 后 */
    case -1:
        return CLOUD_EC_PARAM;              /* uid 非法(理论上前面已校验� */
    case -2:
        return CLOUD_EC_NO_DEVICE;          /* 1003: 不在从机表 */
    case -3:
        return CLOUD_EC_STATE;              /* 1006: 已有解绑在进行 */
    default:
        return CLOUD_EC_INTERNAL;
    }
}

static int cloud_cmd_sen_unbind(const cloud_req_t *req, cJSON *data)
{
    return cloud_cmd_unbind(req, data, true);
}

static int cloud_cmd_dev_unbind(const cloud_req_t *req, cJSON *data)
{
    return cloud_cmd_unbind(req, data, false);
}


/*!
 * \brief 某一行的字段是否被给�没给就��不动��该字段)
 * \note  不能直接�cj_get_str() 的存在��判� �找不到就�out 置空�,
 *        直接调会把没给的字段清成�—��那是静默丢数据�� */
static bool cloud_field_given(const char *json, const char *key)
{
    return (cj_get(json, key, NULL, 0u) != CJ_NOTFOUND);
}

/*!
 * \brief 取一个字符串字段, 且��超长就报错】��不是截� *
 * \note  �这里有个� 实测踩过 �cloud_json �cj_get_str() 在装不下时是
 *        【截断并返回 true��_host_test/test_cloud.c 的用�[3]), 扢�以直接用它会
 *        �平台�name 写成 40 字节"被静默截�32 字节 —��平台以为设成功了� *        做法: 先取进一个足够大的临时缓� 再按目标缓冲的真实上限判长度� *        tmp �static: 本工程栈只有 4KB; 单线程轮� 不会重入� */
static bool cloud_take_str_field(const char *json, const char *key,
                                 char *dst, uint16_t dst_cap)
{
    static char tmp[192];

    if (!cj_get_str(json, key, tmp, (uint16_t)sizeof(tmp)))
    {
        return false;                       /* 缺失/非字符串 */
    }
    if (strlen(tmp) >= (size_t)dst_cap)
    {
        return false;                       /* 超长(或已被临时缓冲截� -> 让调用方�1001 */
    }
    (void)strcpy(dst, tmp);
    return true;
}

/** �"level"/"distance" 映射成枚� 非法返回 -1 */
static int cloud_measure_mode_of(const char *s)
{
    if (strcmp(s, "level") == 0)    { return (int)CFG_MEASURE_LEVEL; }
    if (strcmp(s, "distance") == 0) { return (int)CFG_MEASURE_DISTANCE; }
    return -1;
}

/**
 * @brief sen.set / dev.set —��文档 5.3 �9 �/ 5.4 �13 � *   参数: uid(本工程口� + 该域的可写字�给了才改, 没给的不�
 *   响应: code 0(文档这两条没有 data 字段)
 *   uid 没绑�-> 1003; 值非�越界/超长/枚举不认� -> 1001
 *
 * \note 范围校验不在这里重写丢�� 改完交给 config_store_validate() —��它已经覆� *       文档 3.3 / 3.6 的全部取值范�安装高度/校准偏移/周期/低电/超时/音量/闪光�� *       校验不过就当参数错返� 副本丢掉, 配置丢�个字都不动�� */
/* ==================== ★2026-10-05: sen.set -> LoRa 0x01 的默认值 ★ ====================
 * 用户口径: 平台只下发一个字段(上报周期), 另外几个用默认值。
 *   检测间隔 = 10 分钟 = 600 秒      (用户明确指定)
 *   相同液位也上报 = 1               (与 LORA_LEVEL_DEFAULT_DATA 一致)
 *   液位告警阈值 = 500mm             (同上; 现场设备侧一直在用这个量级)
 * ⚠ 这三个是【网关侧填给设备】的值, 不是平台的配置字段 —— 想改就改这里。
 *   ⚠ 单位是【秒】(2026-10-05 起协议字段就是秒, 网关不换算)。 */
#define CLOUD_SENSOR_DETECT_DEFAULT_S   600u
#define CLOUD_SENSOR_ALARM_DEFAULT_MM   500u

static int cloud_cmd_set_entry(const cloud_req_t *req, cJSON *data, bool is_sensor)
{
    char    uid[CFG_UID_STR_MAX];
    uint8_t i;
    int     idx  = -1;

    /* ★2026-10-05: 是否给了"上报周期"、是否给了"离线超时" —— 决定要不要下发 0x01、
     *   以及要不要自动补 offlineTimeoutSeconds。 */
    bool     report_given  = false;
    bool     offline_given = false;

    (void)data;                             /* 文档这两条响应没有 data */

    if (req->data == NULL)
    {
        return CLOUD_EC_PARAM;
    }
    if (!cj_get_str(req->data, "uid", uid, (uint16_t)sizeof(uid)) || !cfg_uid_valid(uid))
    {
        return CLOUD_EC_PARAM;
    }

    (void)memcpy(&cloud_cfg_staging_buf, config_store_get(), sizeof(cloud_cfg_staging_buf));

    if (is_sensor)
    {
        for (i = 0u; i < cloud_cfg_staging_buf.sensor_count; i++)
        {
            if (strcmp(cloud_cfg_staging_buf.sensors[i].uid, uid) == 0) { idx = (int)i; break; }
        }
    }
    else
    {
        for (i = 0u; i < cloud_cfg_staging_buf.device_count; i++)
        {
            if (strcmp(cloud_cfg_staging_buf.devices[i].uid, uid) == 0) { idx = (int)i; break; }
        }
    }

    if (idx < 0)
    {
        return CLOUD_EC_NO_DEVICE;          /* 1003: set 只能改已绑定� 不能凭空造一�*/
    }

    if (is_sensor)
    {
        cfg_sensor_t *s = &cloud_cfg_staging_buf.sensors[idx];

        /* 【字符串】给了才� 超长要报错��不是悄悄截�平台会以为设成功� */
        if (cloud_field_given(req->data, "name") &&
            !cloud_take_str_field(req->data, "name", s->name, (uint16_t)sizeof(s->name)))
        {
            return CLOUD_EC_PARAM;
        }
        /* serial 是��只读��字�文档 3.3: 绑定时录�。给了就明确拒绝 —��         * 静默忽略会让平台以为改成功了, 那是更坏的结果��*/
        if (cloud_field_given(req->data, "serial"))
        {
            return CLOUD_EC_PARAM;
        }
        if (cloud_field_given(req->data, "measureMode"))
        {
            char mm[12];
            int  m;

            if (!cj_get_str(req->data, "measureMode", mm, (uint16_t)sizeof(mm)))
            {
                return CLOUD_EC_PARAM;
            }
            m = cloud_measure_mode_of(mm);
            if (m < 0) { return CLOUD_EC_PARAM; }
            s->measure_mode = (uint8_t)m;
        }

        /* 【数值��先�uint32 � 再夹回目标宽�—��越界交给 validate() � 不在这里�*/
        if (cloud_field_given(req->data, "installHeightCm"))
        {
            uint32_t v = cj_get_uint(req->data, "installHeightCm", 0u);
            if (v > 0xFFFFu) { return CLOUD_EC_PARAM; }
            s->install_height_cm = (uint16_t)v;
        }
        if (cloud_field_given(req->data, "calibOffsetMm"))
        {
            int32_t v = cj_get_int(req->data, "calibOffsetMm", 0);
            if ((v < -32768) || (v > 32767)) { return CLOUD_EC_PARAM; }
            s->calib_offset_mm = (int16_t)v;
        }
        if (cloud_field_given(req->data, "reportPeriodSeconds"))
        {
            uint32_t v = cj_get_uint(req->data, "reportPeriodSeconds", 0u);
            if (v > 0xFFFFu) { return CLOUD_EC_PARAM; }
            s->report_period_s = (uint16_t)v;
            report_given = true;        /* ★ 决定改完要不要下发 0x01 给传感器 ★ */
        }
        if (cloud_field_given(req->data, "lowBatteryPercent"))
        {
            uint32_t v = cj_get_uint(req->data, "lowBatteryPercent", 0u);
            if (v > 0xFFu) { return CLOUD_EC_PARAM; }
            s->low_battery_pct = (uint8_t)v;
        }
        if (cloud_field_given(req->data, "offlineTimeoutSeconds"))
        {
            uint32_t v = cj_get_uint(req->data, "offlineTimeoutSeconds", 0u);
            if (v > 0xFFFFu) { return CLOUD_EC_PARAM; }
            s->offline_timeout_s = (uint16_t)v;
            offline_given = true;
        }
        if (cloud_field_given(req->data, "enabled"))
        {
            s->enabled = cj_get_bool(req->data, "enabled", true) ? 1u : 0u;
        }
    }
    else
    {
        cfg_device_t *d = &cloud_cfg_staging_buf.devices[idx];

        if (cloud_field_given(req->data, "name") &&
            !cloud_take_str_field(req->data, "name", d->name, (uint16_t)sizeof(d->name)))
        {
            return CLOUD_EC_PARAM;
        }
        /* serial / type 都是【只读��字�文档 3.6): serial 绑定时录� type �dev.bind
         * 的 type 参数决定。给了就明确拒绝, 不静默忽略。 */
        if (cloud_field_given(req->data, "serial") || cloud_field_given(req->data, "type"))
        {
            return CLOUD_EC_PARAM;
        }
        if (cloud_field_given(req->data, "volume"))
        {
            uint32_t v = cj_get_uint(req->data, "volume", 0u);
            if (v > 0xFFu) { return CLOUD_EC_PARAM; }
            d->volume = (uint8_t)v;
        }
        if (cloud_field_given(req->data, "flashMode"))
        {
            uint32_t v = cj_get_uint(req->data, "flashMode", 0u);
            if (v > 0xFFu) { return CLOUD_EC_PARAM; }
            d->flash_mode = (uint8_t)v;
        }
        if (cloud_field_given(req->data, "enabled"))
        {
            d->enabled = cj_get_bool(req->data, "enabled", true) ? 1u : 0u;
        }
    }

    /* ---- ★2026-10-05: 平台只给一个字段(reportPeriodSeconds)时, 自动把
     *      offlineTimeoutSeconds 填成 2 倍★
     *
     * 【为什么要有这一步】用户口径是"平台只下发上报周期, 另一个字段用默认值"。
     *   而 offlineTimeoutSeconds 在本工程里【没有任何消费者】—— 失联判定用的是
     *   【3 × reportPeriodSeconds】(sensor_offline.c)。所以它只是个"给人看"的字段,
     *   但留着一个和上报周期明显不匹配的旧值(比如报 3600s 却写着 offline 90s)
     *   会让人误判失联逻辑, 这里按用户明确要求(2 倍)自动对齐。
     *   ★平台【显式给了】offlineTimeoutSeconds 就完全以平台为准, 这里一个字不改。★
     *   (config_store_validate 里"offline 必须 > report"那条校验已经去掉,
     *    所以这里不是"为了过校验", 纯粹是不留误导性的旧值。) */
    if (is_sensor && report_given && !offline_given)
    {
        /* report_period_s 上限 3600(config_store_validate), ×2 = 7200, uint16 装得下 */
        cloud_cfg_staging_buf.sensors[idx].offline_timeout_s =
            (uint16_t)(cloud_cfg_staging_buf.sensors[idx].report_period_s * 2u);
    }

    {
        int rc = cloud_commit_config(&cloud_cfg_staging_buf);

        if (rc != CLOUD_EC_OK)
        {
            return rc;      /* 落盘失败: 配置一个字没动, 更不该去改设备 */
        }
    }

    /* ---- ★2026-10-05: 网关负责把参数打包成 LoRa 0x01 帧下发给传感器本身★
     *   顺序很关键: 必须【落盘成功之后】才动设备 —— 否则会出现
     *   "设备参数改了、网关配置没改"的分裂状态, 而且平台收到 1007 以为没生效。
     *   只在上报周期被给到时下发: 改 name/enabled 这类【网关侧才有意义】的字段
     *   不下发(传感器只有 3 字节 UID, 它连自己的 name 都不知道)。
     *   单位【秒】, 网关不换算。真正上空中继在传感器下次上报的应答窗口里(异步)。 */
    if (is_sensor && report_given)
    {
        const cfg_sensor_t *s = &config_store_get()->sensors[idx];

        if (!lora_business_set_sensor_params(s->uid,
                                             (uint16_t)CLOUD_SENSOR_DETECT_DEFAULT_S,
                                             s->report_period_s,
                                             true,
                                             (uint16_t)CLOUD_SENSOR_ALARM_DEFAULT_MM))
        {
            log_info("[CLOUD] !! sen.set: %s 的参数已落盘, 但 0x01 投递给 LoRa 失败"
                     "(见上一行 [CLOUD->LoRa] 的原因) —— 设备侧参数没变\r\n", s->uid);
        }
        else
        {
            log_info("[CLOUD] sen.set: %s 已落盘, 且 0x01 参数已投入下发缓存"
                     "(检测=%us 上报=%us 相同上报=1 阈值=%umm), 等它下次上报时搭车下发\r\n",
                     s->uid,
                     (unsigned)CLOUD_SENSOR_DETECT_DEFAULT_S,
                     (unsigned)s->report_period_s,
                     (unsigned)CLOUD_SENSOR_ALARM_DEFAULT_MM);
        }
    }

    return CLOUD_EC_OK;
}

/* ★★2026-10-09 新增: 挡掉只读字段 serial★★
 *
 * 【为什么】`serial` 是【网关在绑定/注册时分配的】只读字段(配置里 devices[i].serial),
 *   平台不该写它 —— 写了我们也不认, 但**必须明确回 1001**, 不能静默忽略:
 *   静默忽略的话平台会以为设置成功了, 然后在 sen.query 里发现 serial 没变,
 *   又回来报"设置不生效"。
 *
 * 【A 层测试计划 A4 就是验这一条】("再带 serial 字段发一次" -> 期望 1001)。
 *
 * ⚠️ 覆盖 sen.set 与 dev.set 两条: serial 对两者都是网关分配的只读字段。
 *   (测试计划只点名 sen.set, 但性质相同, 一起挡更一致。)
 *
 * 【只挡顶层 serial】`data` 是平的: {"uid":"...","name":"...","serial":"..."}。
 *   如果 serial 嵌在子对象里(当前协议没有), 这里挡不住 —— 那时要按实际结构调整。 */
/* ⚠️ 参数是【原始 JSON 文本】不是 cJSON * —— 本工程的 cj_get() 系列工作在文本上
 *    (与 cloud_cfg_sections_mask 里的用法一致: cj_get(req->data, ...))。 */
static int cloud_cmd_check_readonly(const char *json)
{
    if ((json != NULL) && (cj_get(json, "serial", NULL, 0u) != CJ_NOTFOUND))
    {
        log_info("[CLOUD] !! 写命令带了只读字段 serial(网关分配, 平台不可写) -> 回 1001\r\n");
        return CLOUD_EC_PARAM;
    }
    return CLOUD_EC_OK;
}

static int cloud_cmd_sen_set(const cloud_req_t *req, cJSON *data)
{
    int rc = cloud_cmd_check_readonly(req->data);

    if (rc != CLOUD_EC_OK)
    {
        return rc;
    }
    return cloud_cmd_set_entry(req, data, true);
}

static int cloud_cmd_dev_set(const cloud_req_t *req, cJSON *data)
{
    int rc = cloud_cmd_check_readonly(req->data);

    if (rc != CLOUD_EC_OK)
    {
        return rc;
    }
    return cloud_cmd_set_entry(req, data, false);
}


/* ==================================================================
 * sen.query —��文档 5.3 �10 �/ dev.query —��文档 5.4 �15 � * ==================================================================
 * �取本域配�的办法与 rule.query 完全丢�致�� *   复用 config_store_write_members() �9 个域全写出来, 再把本条命令不该带的域删掉�� *   这样 sensors[]/devices[] 的字段名与键序与 cfg.query 里同名对象��同源��
 *   不会因为抄一遍字段表而漂(抄一�= 以后改配置结构要改两� 两处必然不一�� *
 * 【本工程补的运行时字段�� *   文档要求这两条能�朢�近一次上�。该 uid 有上报记录时�
 *       levelCm        朢�近水�cm, = mm/10)   —��只有传感器补
 *       batteryPercent 电量百分�0~100)
 *       lastTs         网关收到该帧时的 Unix � *   �没有上报记录、或网关时钟未同步时, 这几个键【根本不出现�� *     不用 0 冒充: 平台会把 0 读成"水位 0cm / 电量 0% / 时间 1970",
 *     �这个字段没有"更糟, 也和我们别处"不拿默认值冒充真�的做法一致�� *   uid 匹配: 配置里是 hex 串��快照表按键 3 字节, �cfg_uid_to_bytes() �
 *   转不出来就当没匹配上(跳过), 不猜� * ================================================================== */
static void cloud_inject_last_values(cJSON *arr, bool with_level)
{
    cJSON *it;

    if ((arr == NULL) || (arr->type != cJSON_Array))
    {
        return;                         /* 该域不存在或不是数组: 不是错误 */
    }

    for (it = arr->child; it != NULL; it = it->next)
    {
        cJSON       *u;
        uint8_t      uid[CFG_UID_BYTES];
        last_value_t v;

        if (it->type != cJSON_Object)
        {
            continue;
        }
        u = cJSON_GetObjectItemCaseSensitive(it, "uid");
        if ((u == NULL) || (u->type != cJSON_String) || (u->valuestring == NULL))
        {
            continue;
        }
        if (!cfg_uid_to_bytes(u->valuestring, uid))
        {
            continue;                   /* uid 串非� 跳过 */
        }

        /* ---- online(文档 5.3/5.4: sen.query / dev.query 每一项都要带) ----
         * 判据与失联检测��完全同丢�套�� 距上次上�< 3 × 该设备上报周�即在线��         * 以前这里没有 online 字段 —��平台只能�有没�lastTs"去猜设备死活�         * 从没上报过的设备也要回一�online=false + 只有配置字段), 不能整项不出�
         * 平台�列表里没有它"会以为是配置丢了�*/
        {
            uint16_t period_s = sensor_offline_period_for_uid(config_store_get(),
                                                              u->valuestring);
            bool     have     = last_value_get(uid, &v);
            bool     is_on    = false;

            if (have)
            {
                is_on = ((uint32_t)(get_tick_ms() - v.rx_tick_ms)
                         < sensor_offline_threshold_ms(period_s));
            }
            else
            {
                /* ★2026-10-18: 没有快照(声光从不发 0x02, 永远不进快照表)
                 *   => 问业务层的真实状态。声光按"是否被判过离线"算,
                 *      否则它的 online 会恒为 false, 平台一直误报掉线。 */
                is_on = (lora_business_device_state(u->valuestring)
                         == LORA_DEV_STATE_ONLINE);
            }
            (void)cJSON_AddBoolToObject(it, "online", is_on ? 1 : 0);
            if (!have)
            {
                continue;               /* 从没上报� 只有 online=false + 配置字段 */
            }
        }

        if (with_level)
        {
            /* 字段名按文档 5.3 第 10 条: 【lastLevelCm】
             * (2026-09-30 用户确认: 对云端来说每次收到的都是"最新一条", 所以用 last 前缀。
             *  以前发的是 levelCm —— 那是 telemetry 的字段名, 两处口径不同。) */
            (void)cJSON_AddNumberToObject(it, "lastLevelCm", (double)v.level_mm / 10.0);
        }
        (void)cJSON_AddNumberToObject(it, "batteryPercent", (double)v.bat_pct);
        if (v.rx_unix != 0u)
        {
            (void)cJSON_AddNumberToObject(it, "lastTs", (double)v.rx_unix);
        }
        /* 加不进去(池满)时只丢这几个运行时字� 不让整条命令失败 —��         * 配置部分是主体且已经写好� cJSON_Add* 失败返回 NULL, 不会破坏对象�*/
    }
}

static int cloud_cmd_sen_query(const cloud_req_t *req, cJSON *data)
{
    static const char *const drop[] =
    {
        "system", "network", "devices", "thresholds", "gears", "rules", "ota", "meta"
    };
    uint8_t i;

    (void)req;                          /* 文档这条没有参数 */

    if (!config_store_write_members(data, config_store_get()))
    {
        return CLOUD_EC_INTERNAL;       /* 池满 */
    }

    for (i = 0u; i < (uint8_t)(sizeof(drop) / sizeof(drop[0])); i++)
    {
        cJSON_DeleteItemFromObject(data, drop[i]);
    }

    cloud_inject_last_values(cJSON_GetObjectItemCaseSensitive(data, "sensors"), true);
    return CLOUD_EC_OK;
}

static int cloud_cmd_dev_query(const cloud_req_t *req, cJSON *data)
{
    static const char *const drop[] =
    {
        "system", "network", "sensors", "thresholds", "gears", "rules", "ota", "meta"
    };
    uint8_t i;

    (void)req;                          /* 文档这条没有参数 */

    if (!config_store_write_members(data, config_store_get()))
    {
        return CLOUD_EC_INTERNAL;
    }

    for (i = 0u; i < (uint8_t)(sizeof(drop) / sizeof(drop[0])); i++)
    {
        cJSON_DeleteItemFromObject(data, drop[i]);
    }

    /* 联动设备没有水位, 所以不补 levelCm; 电量/时间照补 */
    cloud_inject_last_values(cJSON_GetObjectItemCaseSensitive(data, "devices"), false);
    return CLOUD_EC_OK;
}





/* ==================================================================
 * sen.bind / dev.bind(文档 5.3 �7 �/ 5.4 �11 � + 配对后端钩子
 * ==================================================================
 * 文档语义:
 *   参数 serial(必填, 8~16 字节) + name(可�� + pairWindowSeconds(可�� 默认 60);
 *   响应 code 0 + data:{"pairWindowSeconds":N} —��只表�窗口已开";
 *   真实绑定结果【异步】经 event.device(bound / bind_failed)上报;
 *   已有 5 个传感器时拒�错误�1009)� * ================================================================== */
static cloud_pair_open_fn s_pair_fn;

void cloud_proto_set_pair_hook(cloud_pair_open_fn fn)
{
    s_pair_fn = fn;
}

static int cloud_cmd_bind(const cloud_req_t *req, cJSON *data, bool is_sensor)
{
    char     serial[CFG_SERIAL_MAX + 1u];
    char     name[CFG_NAME_MAX + 1u];
    uint32_t window_s = 60u;            /* 文档默认 60 */
    uint8_t  dev_type = 0u;             /* dev.bind 的类型(1/2/3); sen.bind 不用 */

    if (req->data == NULL)
    {
        return CLOUD_EC_PARAM;
    }

    /* ★★2026-10-08: `serial` 不再必填 —— 用户口径"这一个字段直接丢"★★
     *
     * 原实现要求 serial 必须存在且 8~16 字节, 缺了直接 1001。但平台侧的绑定请求
     * 里根本不带这个字段(用户给的样例):
     *     {"cmd":"sen.bind","msgId":"f4-01",...,"data":{"name":"西入口液位","pairWindowSeconds":60}}
     * ⇒ 原实现会让【每一次绑定都失败】, 而且错误码只有 1001, 现场很难看出是 serial 的锅。
     *
     * 现在的语义: 给了就当备注存下来(仍然超长报 1001, 不静默截断), 没给就留空。
     * 绑定的身份完全由 `uid` 决定(见 §1.2: addr 与 serial 都不参与寻址)。
     * 注: config_store 里 sensors[].serial / devices[].serial 仍然存在(留作备注,
     *     也兼容旧配置), 只是不再被本命令强制要求。 */
    serial[0] = '\0';
    if (cloud_field_given(req->data, "serial") &&
        (!cloud_take_str_field(req->data, "serial", serial, (uint16_t)sizeof(serial))))
    {
        return CLOUD_EC_PARAM;          /* 给了但超长/非字符串 -> 仍然 1001 */
    }

    name[0] = '\0';                     /* name 可选 */
    if (cloud_field_given(req->data, "name") &&
        !cloud_take_str_field(req->data, "name", name, (uint16_t)sizeof(name)))
    {
        return CLOUD_EC_PARAM;          /* 给了但超�非字符串 -> 报错, 不静默丢 */
    }

    if (cloud_field_given(req->data, "pairWindowSeconds"))
    {
        if (cj_get(req->data, "pairWindowSeconds", NULL, 0u) != CJ_NUM)
        {
            return CLOUD_EC_PARAM;
        }
        window_s = cj_get_uint(req->data, "pairWindowSeconds", 60u);
        if ((window_s == 0u) || (window_s > 0xFFFFu))
        {
            return CLOUD_EC_PARAM;
        }
    }

    /* ---- ��11 � dev.bind �type(文档 5.4 �11 � 额外必填) �----
     * 取��1 声光 / 2 LED / 3 丢��cfg_dev_type_t)。缺了或越界丢��1001 —��     * 【不猜�� 猜错的代价是 devices[].type 写错, 而该字段文档�只读",
     * 平台自己改不� 只能解绑重绑(现场很折��*/
    if (!is_sensor)
    {
        uint32_t t;

        if (cj_get(req->data, "type", NULL, 0u) != CJ_NUM)
        {
            return CLOUD_EC_PARAM;          /* �type */
        }
        t = cj_get_uint(req->data, "type", 0u);
        if ((t < (uint32_t)CFG_DEVTYPE_SOUND_LIGHT) ||
            (t > (uint32_t)CFG_DEVTYPE_ALLINONE))
        {
            return CLOUD_EC_PARAM;          /* 越界 */
        }
        dev_type = (uint8_t)t;
    }

    /* 容量上限: 用户口径"总数� / 液位� / 声光�", 与运行期 0x04 绑定握手里的
     * config_store_can_add 同口径��这条是纯配置判� 不需�LoRa —��先在这里拦住,
     * 别等到配对成功了才发现放不下�     * �注意: 这里只判"还剩不剩位置", 真正绑定�config_store 在收到设�ACK �     *   �lora_business �config_store_bind_device 完成(�0x04 握手路径里再判一��*/
    if (!config_store_can_add(is_sensor))
    {
        return CLOUD_EC_LIMIT;          /* 1009 */
    }

    if (s_pair_fn == NULL)
    {
        /* 配对后端还没�main.c 还没注册): 明确�1006�         * �不做"假装窗口已开" �—��平台会一直等 event.device{bound}, 永远等不到��*/
        log_info("[CLOUD] %s: 本机还没注册配对后端, 回 1006\r\n",
                 is_sensor ? "sen.bind" : "dev.bind");
        return CLOUD_EC_STATE;
    }

    if (!s_pair_fn(serial, name, (uint16_t)window_s, is_sensor, dev_type))
    {
        return CLOUD_EC_INTERNAL;       /* 开窗口失败 -> 1007 */
    }

    if (cJSON_AddNumberToObject(data, "pairWindowSeconds", (double)window_s) == NULL)
    {
        return CLOUD_EC_INTERNAL;       /* 池满 */
    }
    return CLOUD_EC_OK;
}

static int cloud_cmd_sen_bind(const cloud_req_t *req, cJSON *data)
{
    return cloud_cmd_bind(req, data, true);
}

static int cloud_cmd_dev_bind(const cloud_req_t *req, cJSON *data)
{
    return cloud_cmd_bind(req, data, false);
}

/**
 * @brief dev.ctrl —��手动控制联动设备(文档 5.4 �14 � 用户 2026-09-30 �6 �
 *
 *   参数: uid     必填; 本工程口径是 6 �hex � 也接�"all"(=全部启用的联动设�
 *                   —��文档写的�uint16 �addr, 那是偏差 D-4(用户已确认用 uid),
 *                      扢�以字符串 "all" 就是本工程对文档 "all" 的对应写� *         gear    必填 0~3
 *         ledText 可�� 覆盖档位模板里的 LED 文本(给了就用给的)
 *         ttsText 可�� 同上(语音文本)
 *   响应: code 0 + data.results[{uid,code}]
 *   语义: 执行后进�manual 模式(�rules.mode=manual); manualTimeoutSeconds 到期
 *         由主循环自动回归 auto; manual 期间阈��变化��不再自动联动�� *         (抑制点在 level_supervisor.c: �cfg->rules.mode)� *
 * 【与文档的一处有意差� 应答时机� *   文档�网关�LoRa ACK(超时 5 �后应� 个别设备失败�results 中体�� *   本工程是【非阻塞】架� 命令处理不能把主循环�5 秒等空口 —��扢��
 *     · results[].code 反映【受�投����结�
 *         0    = 已投递到下发缓存(空口发��与重试�lora_business �poll 负责)
 *         1003 = 目标设备不存�不在从机表里 / 已停�
 *         1007 = 网关内部错误(uid 非法 / 组帧失败)
 *     · 真正的空口执行结�ACK / 5s 超时 / 重试耗尽)�event.action 异步补报
 *       —��这也正是文档 5.4 末要求的那条("后台重试并经 event.action 补报结果")� */
/* ★2026-10-10: dev.ctrl 的 ledTextShowMode / ledColor 是【逗号串】(与 gears[] 一致)。
 * 这里放两个纯解析小助手, 严格但简单: 格式不对就返回 false -> 调用方回 1001。
 * ⚠️ 不宽容: "1, 2"(带空格)、"1,2,3"、单个值 都判非法 —— 宁可让平台立刻发现写错了,
 *    也不要静默用半个值下发到现场设备上。 */

/** 解析 "a,b" 形式、两个 0~9 的数字 -> out[0]/out[1] */
static bool cloud_parse_u8_pair(const char *s, uint8_t *out)
{
    if ((s == NULL) || (out == NULL)) { return false; }
    if ((s[0] < '0') || (s[0] > '9')) { return false; }
    if (s[1] != ',')                  { return false; }
    if ((s[2] < '0') || (s[2] > '9')) { return false; }
    if (s[3] != '\0')                 { return false; }

    out[0] = (uint8_t)(s[0] - '0');
    out[1] = (uint8_t)(s[2] - '0');
    return true;
}

/** 解析 "off,red" 形式的两个颜色名 -> 0 off / 1 red / 2 green / 3 yellow
 *  ⚠️ 顺序必须与 cfg_led_color_t 一致 —— 那个枚举是【按空口码值重排过的】
 *    (OFF=0, RED=1, GREEN=2, YELLOW=3), 顺序写错会导致红绿互换这种静默 bug。 */
static bool cloud_parse_color_pair(const char *s, uint8_t *out)
{
    static const char *const names[4] = { "off", "red", "green", "yellow" };
    const char *comma;
    uint8_t     h;

    if ((s == NULL) || (out == NULL)) { return false; }
    comma = strchr(s, ',');
    if (comma == NULL) { return false; }

    for (h = 0u; h < 2u; h++)
    {
        const char *b   = (h == 0u) ? s : (comma + 1);
        size_t      n   = (h == 0u) ? (size_t)(comma - s) : strlen(comma + 1u);
        uint8_t     k;
        bool        hit = false;

        for (k = 0u; k < 4u; k++)
        {
            if ((strlen(names[k]) == n) && (strncmp(names[k], b, n) == 0))
            {
                out[h] = k;
                hit    = true;
                break;
            }
        }
        if (!hit) { return false; }
    }
    return true;
}

static int cloud_cmd_dev_ctrl(const cloud_req_t *req, cJSON *data)
{
    char               uid[CFG_UID_STR_MAX];
    /* ★2026-10-08: LED 文本覆盖参数由单段改为两段★
     * 用户选定方案 (A): `ledText` -> `ledText1` + `ledText2`
     * (与 gears[] 的 ledTextTemplate1/2 命名一致)。 */
    char               led1[CFG_LEDTEXT_MAX + 1u];
    char               led2[CFG_LEDTEXT_MAX + 1u];
    char               tts[CFG_TTSTEXT_MAX + 1u];
    const char        *only_uid;
    const config_store_t *cfg = config_store_get();
    lora_gear_result_t res[CFG_DEVICE_MAX];
    uint8_t            total = 0u;
    uint32_t           gear;
    int                sent;
    uint8_t            i;
    cJSON             *arr;

    if ((req->data == NULL) || (cfg == NULL)) { return CLOUD_EC_PARAM; }

    /* ---- 目标: "all" 或具�uid ---- */
    if (!cj_get_str(req->data, "uid", uid, (uint16_t)sizeof(uid)))
    {
        return CLOUD_EC_PARAM;
    }
    if (strcmp(uid, "all") == 0)
    {
        only_uid = NULL;                     /* NULL = 全部启用的联动设�*/
    }
    else
    {
        if (!cfg_uid_valid(uid)) { return CLOUD_EC_PARAM; }
        only_uid = uid;
    }

    /* ---- 档位 ---- */
    if (cj_get(req->data, "gear", NULL, 0u) != CJ_NUM) { return CLOUD_EC_PARAM; }
    gear = cj_get_uint(req->data, "gear", 0u);
    if (gear >= (uint32_t)CFG_GEAR_COUNT) { return CLOUD_EC_PARAM; }

    /* ---- 可覆盖文本: 没给就用档位模板 + 当前水位渲染 ----
     * ★★2026-10-09 用户口径: "给了空串就是空, 不要加默认参数"★★
     * --------------------------------------------------------------------------
     * 【原来错在哪】老代码只看 `led1[0] == '\0'` 来判断"要不要套模板" ——
     *   于是 `ledText1:""`(平台明确要求置空) 会被当成"没给", 又去渲染档位模板,
     *   平台看到的是"我说了空, 结果又冒出模板文字"。
     *
     * 【现在】用 given 标志区分两种语义:
     *     字段【没出现】        -> 用档位模板 + 当前水位渲染
     *     字段出现(含空串 "")   -> 原样使用, 空串就是空, 不套模板、不加默认
     *   ⇒ 这是 JSON 里"缺省"与"显式给空"的区分, 必须靠 given, 不能靠内容是否为空。
     *
     * ⚠️ 超长仍按老规矩回 1001(不截断)。 */
    bool given1 = cloud_field_given(req->data, "ledText1");
    bool given2 = cloud_field_given(req->data, "ledText2");
    bool givenT = cloud_field_given(req->data, "ttsText");

    led1[0] = '\0';
    led2[0] = '\0';
    tts[0]  = '\0';
    if (given1 && (!cloud_take_str_field(req->data, "ledText1", led1, (uint16_t)sizeof(led1))))
    {
        return CLOUD_EC_PARAM;               /* 超长 -> 1001, 不截断 */
    }
    if (given2 && (!cloud_take_str_field(req->data, "ledText2", led2, (uint16_t)sizeof(led2))))
    {
        return CLOUD_EC_PARAM;
    }
    if (givenT && (!cloud_take_str_field(req->data, "ttsText", tts, (uint16_t)sizeof(tts))))
    {
        return CLOUD_EC_PARAM;
    }

    /* 只对【没给】的字段套模板。给的(哪怕是空串)一律不动。 */
    if ((!given1) || (!given2) || (!givenT))
    {
        int32_t lv = 0;
        uint8_t dummy_gear = 0u;

        /* 当前水位由评估模块给(避免在这里再实现"取最大") */
        if (level_supervisor_last(&lv, &dummy_gear))
        {
            if (!given1)
            {
                (void)level_render_template(led1, (uint16_t)sizeof(led1),
                                            cfg->gears[gear].led_text1, lv);
            }
            if (!given2)
            {
                (void)level_render_template(led2, (uint16_t)sizeof(led2),
                                            cfg->gears[gear].led_text2, lv);
            }
            if (!givenT)
            {
                (void)level_render_template(tts, (uint16_t)sizeof(tts),
                                            cfg->gears[gear].tts_text, lv);
            }
        }
        /* 还没给 且 没有过有效水位: 文本留空 —— 只切档位/灯光, 不改文案(而不是把屏幕清空) */
    }

    /* ==========================================================================
     * ★★2026-10-14 修复: 平台【给的】文字也要过 {level} 渲染 ★★
     * --------------------------------------------------------------------------
     * 【原来错在哪】上面那一支只在"字段【没给】"时渲染(用的是 cfg->gears[gear]
     *   的档位模板)。平台【明确给了】ledText1/ledText2/ttsText 时, 文字被
     *   【原样下发】—— 里面的 {level} 一个字都没替换。用户 2026-10-14 实测:
     *       dev.ctrl: {"uid":"310001","gear":3,"ledText1":"路面积水了{level}CM", ...}
     *       [GEAR] 档位 3 已投递 1/1 台联动设备(手动控制)(LED1="路面积水了{level}CM" ...)
     *       [TX] ... E8 B7 AF E9 9D A2 E7 A7 AF E6 B0 B4 E4 BA 86
     *                7B 6C 65 76 65 6C 7D            <-- ★ "{level}" 原样上空口 ★
     *                43 4D ...
     *   ⇒ LED 屏上显示的就是字面的 "{level}"。
     *
     * 【用户口径(2026-10-14, 方案甲)】覆盖文字【生效】(不是退回档位模板),
     *   但必须把 {level} 填成当前水位。
     * 【没有水位数据时】用户定: 按 0 处理(⇒ 渲染成 "0 ", 一位数补空格的规则生效)。
     *
     * ⚠️ 两条实现要点:
     *   1) 必须用【临时缓冲】: level_render_template(out, cap, tmpl, mm) 的 out 与
     *      tmpl 若是同一块内存会自我覆盖 ⇒ 不能原地渲染 led1。
     *   2) 渲染失败(装不下)时 level_render_template 会把 out[0] 置 '\0',
     *      语义是"绝不发半截文本给 LED" ⇒ 这里按 1001 拒绝, 与上面"超长回 1001"
     *      的口径一致(不静默降级成空文案)。
     * ========================================================================== */
    if (given1 || given2 || givenT)
    {
        int32_t lv         = 0;
        uint8_t dummy_gear = 0u;
        bool    have_lv    = level_supervisor_last(&lv, &dummy_gear);

        if (!have_lv) { lv = 0; }       /* 用户定(甲): 还没有过有效水位 -> 按 0 */

        if (given1)
        {
            char t[CFG_LEDTEXT_MAX + 1u];
            if (level_render_template(t, (uint16_t)sizeof(t), led1, lv) == 0u)
            {
                return CLOUD_EC_PARAM;  /* 装不下 -> 1001, 不发半截 */
            }
            (void)memcpy(led1, t, sizeof(led1));
        }
        if (given2)
        {
            char t[CFG_LEDTEXT_MAX + 1u];
            if (level_render_template(t, (uint16_t)sizeof(t), led2, lv) == 0u)
            {
                return CLOUD_EC_PARAM;
            }
            (void)memcpy(led2, t, sizeof(led2));
        }
        if (givenT)
        {
            char t[CFG_TTSTEXT_MAX + 1u];
            if (level_render_template(t, (uint16_t)sizeof(t), tts, lv) == 0u)
            {
                return CLOUD_EC_PARAM;
            }
            (void)memcpy(tts, t, sizeof(tts));
        }
    }

    /* ---- ★★2026-10-10 新增: 样式覆盖 ledShowMode / ledTextShowMode / ledColor ★★
     * --------------------------------------------------------------------------
     * 【原来缺什么】这三字段一直被【静默忽略】—— 上面只解析了 uid/gear/三段文本。
     *   而空口 out[3] 里本来就有位置(见 level_eval.c 的 level_gear_build_payload):
     *       bit7   ledShowMode      0 横 / 1 竖
     *       bit5,4 ledTextShowMode  半屏1/半屏2: 0 居中 / 1 对齐
     *       bit3,2 / bit1,0  ledColor  半屏1/半屏2: off/red/green/yellow
     *   ⇒ 只需要把值传下去, 设备侧一个字都不用改。
     *
     * 【写法与 gears[] 完全一致】两个半屏用【逗号串】: "1,2" / "red,yellow"。
     *   用户 2026-10-10 确认: ledShowMode 只有 0/1; ledTextShowMode 只有 0/1(居中/对齐)。
     *
     * 【语义】临时盖一下 —— 不写 config_store、不持久化; 与文本那边一样用
     *   cloud_field_given() 区分"没给"(用档位配置)和"给了"(用覆盖值)。
     *   解析失败一律回 1001(和文本字段一个口径, 不静默忽略)。 */
    {
        uint8_t style_mask = 0u;
        uint8_t st_show = 0u;
        uint8_t st_text[2] = { 0u, 0u };
        uint8_t st_color[2] = { 0u, 0u };

        /* ---- ledShowMode: 数字, 只有 0/1 ---- */
        if (cloud_field_given(req->data, "ledShowMode"))
        {
            uint32_t v;

            if (cj_get(req->data, "ledShowMode", NULL, 0u) != CJ_NUM) { return CLOUD_EC_PARAM; }
            v = cj_get_uint(req->data, "ledShowMode", 0xFFFFu);
            if (v > 1u) { return CLOUD_EC_PARAM; }
            st_show = (uint8_t)v;
            style_mask |= LORA_STYLE_SHOW_MODE;
        }

        /* ---- ledTextShowMode: "a,b" 两个 0/1 ---- */
        if (cloud_field_given(req->data, "ledTextShowMode"))
        {
            char s[16];

            if (!cj_get_str(req->data, "ledTextShowMode", s, (uint16_t)sizeof(s)))
            {
                return CLOUD_EC_PARAM;
            }
            if (!cloud_parse_u8_pair(s, st_text)) { return CLOUD_EC_PARAM; }
            if ((st_text[0] > 1u) || (st_text[1] > 1u)) { return CLOUD_EC_PARAM; }
            style_mask |= LORA_STYLE_TEXT_MODE;
        }

        /* ---- ledColor: "off,red" 之类 ---- */
        if (cloud_field_given(req->data, "ledColor"))
        {
            char s[24];

            if (!cj_get_str(req->data, "ledColor", s, (uint16_t)sizeof(s)))
            {
                return CLOUD_EC_PARAM;
            }
            if (!cloud_parse_color_pair(s, st_color)) { return CLOUD_EC_PARAM; }
            style_mask |= LORA_STYLE_COLOR;
        }

        if (style_mask != 0u)
        {
            lora_business_set_gear_style(style_mask, st_show,
                                         st_text[0], st_text[1],
                                         st_color[0], st_color[1]);
        }
    }

    /* ---- 下发(投递到缓存; 空口回 ACK 由 lora_business 的 poll 负责) ---- */
    sent = lora_business_apply_gear_ex((uint8_t)gear, only_uid, led1, led2, tts,
                                       res, (uint8_t)CFG_DEVICE_MAX, &total);

    /* ★2026-10-10: 覆盖是"临时盖一下", 用完立刻清 —— 否则会粘到下一次自动联动上。
     * 放在这里(调用之后、任何 return 之前), 成功失败两条路都清。 */
    lora_business_clear_gear_style();
    if (sent < 0) { return CLOUD_EC_PARAM; }
    if (total == 0u)
    {
        /* 丢�台都没��中: 指定的那台不存在/已停� 或压根没有启用的联动设备 */
        return CLOUD_EC_NO_DEVICE;           /* 1003 */
    }

    /* ---- 进入 manual 模式(配置落盘; 抑制点在 level_supervisor) ---- */
    (void)memcpy(&cloud_cfg_staging_buf, cfg, sizeof(cloud_cfg_staging_buf));
    cloud_cfg_staging_buf.rules.mode = (uint8_t)CFG_RULE_MANUAL;
    {
        int rc = cloud_commit_config(&cloud_cfg_staging_buf);
        if (rc != CLOUD_EC_OK)
        {
            /* 配置没落�= manual 没生� 但动作已经投递出去了。如实报�
             * 平台重发丢�次即�幂等: 同样的档�文本, 下发判据不会重复投���*/
            log_info("[CTRL] !! dev.ctrl 动作已投� �manual 模式落盘失败(rc=%d)\r\n", rc);
            return rc;
        }
    }

    /* ---- 应答 data.results[{uid,code}] + configVersion ---- */
    arr = cJSON_AddArrayToObject(data, "results");
    if (arr == NULL) { return CLOUD_EC_INTERNAL; }

    for (i = 0u; (i < total) && (i < (uint8_t)CFG_DEVICE_MAX); i++)
    {
        cJSON *o = cJSON_CreateObject();

        if (o == NULL) { return CLOUD_EC_INTERNAL; }
        if ((cJSON_AddStringToObject(o, "uid", res[i].uid) == NULL) ||
            (cJSON_AddNumberToObject(o, "code", (double)res[i].code) == NULL))
        {
            cJSON_Delete(o);
            return CLOUD_EC_INTERNAL;
        }
        cJSON_AddItemToArray(arr, o);
    }

    if (cJSON_AddNumberToObject(data, "configVersion",
                                (double)config_store_version()) == NULL)
    {
        return CLOUD_EC_INTERNAL;
    }

    log_info("[CTRL] dev.ctrl: uid=%s gear=%u -> 受理 %d/%u � 已进 manual 模式\r\n",
             (only_uid != NULL) ? only_uid : "all", (unsigned)gear, sent, (unsigned)total);
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
