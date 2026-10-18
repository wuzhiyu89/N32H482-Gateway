/**
 * @file  cloud_profile.c
 * @brief 云平台连接档位实现(见 cloud_profile.h 的说明)
 */
#include "cloud_profile.h"
#include "param_save.h"
#include <string.h>

/* ==================================================================
 * 常量表
 * ================================================================== */

/* 文档定义的 topic 后缀(拼成 rt/gw/{gwId}/<后缀>) */
static const char *const s_topic_suffix[CLOUD_TOPIC_COUNT] =
{
    "cmd",          /* CLOUD_TOPIC_CMD       */
    "cmdresp",      /* CLOUD_TOPIC_CMDRESP   */
    "telemetry",    /* CLOUD_TOPIC_TELEMETRY */
    "event",        /* CLOUD_TOPIC_EVENT     */
    "status",       /* CLOUD_TOPIC_STATUS    */
    "lwt"           /* ★2026-10-08 新增★ 遗嘱专用, 与 status 隔离(理由见 .h) */
};

/* 本地/历史档位的 topic 表。
 * ★2026-10-08 起【不再是出厂默认】(默认已切到文档档位 rt/gw/{gwId}/xxx),
 *   保留它只为了"万一要回退"以及主机单测覆盖两套拼装逻辑。
 * 注: 早期这里的下行叫 "mod/pc_down"、四个上行**全塌成** "mod/mod_upload";
 *     v93 已按平台新建的 5 个主题换成下面这组; 那两个遗留主题在 2026-10-08
 *     已按甲方要求【整体删除】(连编译期宏一起), 不要再引用它们。 */
static const char *const s_local_topic[CLOUD_TOPIC_COUNT] =
{
    "mod/cmd",          /* 下行(平台 -> 网关) */
    "mod/cmdresp",      /* 上行(命令应答)     */
    "mod/telemetry",    /* 上行(水位遥测)     */
    "mod/event",        /* 上行(事件)         */
    "mod/status",       /* 上行(状态/心跳)    */
    "mod/lwt"           /* ★2026-10-08 新增★ 遗嘱专用 */
};

/* 文档 2 章: 下行命令/应答/事件/状态 QoS1, 遥测 QoS0。
 * ★lwt 用 QoS1★: 遗嘱只在掉电/断链那一刻发一条, 是最不能丢的消息,
 *   QoS0 的话"发出去就不管了" —— broker 没收到就永久丢了, 平台再也等不到掉线通知。 */
static const uint8_t s_doc_qos[CLOUD_TOPIC_COUNT] =
{
    1u, 1u, 0u, 1u, 1u, 1u
};

/* 文档: 只有 status 和 lwt 是 retained。
 * ★lwt 必须 retained★: 遗嘱是在【网关已经掉线】之后由 broker 代发的,
 *   平台若那一刻不在线(或稍后才订阅), 不 retained 就永远看不到这条掉线通知。
 *   这也正是"lwt 不污染 status"的关键 —— 两者各自 retained, 互不覆盖。 */
static const bool s_doc_retained[CLOUD_TOPIC_COUNT] =
{
    false, false, false, false, true, true
};

/* ==================================================================
 * 内部状态(懒加载)
 * ================================================================== */

static cloud_profile_t s_profile;
static bool            s_loaded = false;

/* 字段是否是可打印 ASCII 且以 '\0' 结尾 */
static bool str_is_printable(const char *s, uint16_t cap)
{
    uint16_t i;

    for (i = 0u; i < cap; i++)
    {
        unsigned char c = (unsigned char)s[i];
        if (c == 0u)
        {
            return true;
        }
        if ((c < 0x20u) || (c > 0x7Eu))
        {
            return false;
        }
    }
    return false;                       /* 字段内找不到 '\0' */
}

void cloud_profile_set_factory_default(cloud_profile_t *p)
{
    if (p == NULL)
    {
        return;
    }
    (void)memset(p, 0, sizeof(*p));

    /* ★2026-10-08: 出厂默认 = 文档档位★
     * 甲方平台(223.75.207.139)约定:
     *   · 主题形如 rt/gw/{gwId}/cmd ... —— 就是文档定义的这套 {prefix}/{gwId}/{suffix};
     *   · {gwId} 取【设备 ID】, 出厂默认 "110001"(= 1xxxxx 网关号 0001), 后续可改;
     *   · 同时这个值也是【MQTT 用户名】(见 main.c: g4_at_set_broker(..., gw_id, ...))。
     *
     * ⚠️ 注意 gw_id 的【真正真相源是 config_store.system.gwId】(出厂也是 "110001"),
     *    main.c 启动时会把配置里的值灌进来(cloud_profile_set_gw_id) ——
     *    这里只是"配置还没加载时的兜底", 两者必须保持一致, 否则平台按用户名认设备、
     *    网关却往另一个主题发, 会连得上但收不到指令。 */
    p->mode           = (uint8_t)CLOUD_MODE_DOC;
    p->id_map_enable  = 0u;
    p->auth_enable    = 0u;

    /* ⚠️ host/port 这两个字段【当前全工程没有消费者】(见偏差记录 D4):
     *    真正连哪个 broker 由 `config_store.network.host`(非空时) 或 g4_at 的编译期
     *    默认值决定。这里填成实际值有两个作用:
     *      ① 让 cloud_profile_validate() 通过(它要求 host 非空);
     *      ② 万一以后有人把它接上, 取到的也是正确的地址, 不会指向一个废弃的 broker。 */
    (void)memcpy(p->host, "223.75.207.139", sizeof("223.75.207.139"));
    p->port = 1883u;

    /* ★2026-10-08: 空 = 【不参与 clientID 决策】★
     * clientID 现在由 g4_at 管: 编译期默认 "110001", 可被 `network.clientId` 覆盖
     * (特殊值 "IMEI" = 用模组 IMEI)。本字段当前【没有消费者】, 留空以免出现
     * "两个地方都能配 clientID、谁生效说不清"。 */
    p->client_id[0] = '\0';
    (void)memcpy(p->gw_id, "11000000000000000000000000", sizeof("11000000000000000000000000"));   /* 2026-10-18: 26 位; 真实值由 config_store 注入 */
    (void)memcpy(p->topic_prefix, "rt/gw", sizeof("rt/gw"));
    p->auth[0] = '\0';
}

int cloud_profile_validate(const cloud_profile_t *p)
{
    if (p == NULL)
    {
        return 1;
    }
    if ((p->mode != (uint8_t)CLOUD_MODE_LOCAL) && (p->mode != (uint8_t)CLOUD_MODE_DOC))
    {
        return 1;
    }
    if ((p->host[0] == '\0') || (!str_is_printable(p->host, CLOUD_HOST_MAX)))
    {
        return 2;
    }
    if (p->port == 0u)
    {
        return 3;
    }
    /* gwId 会被用作 MQTT clientId 与 topic 段: 非空、<=20、不含 topic 通配符 */
    if ((p->gw_id[0] == '\0') || (!str_is_printable(p->gw_id, CLOUD_GWID_MAX)))
    {
        return 4;
    }
    if ((strlen(p->gw_id) > 26u) || (strpbrk(p->gw_id, "/+#") != NULL))
    {
        return 4;
    }
    if ((p->topic_prefix[0] == '\0') || (!str_is_printable(p->topic_prefix, CLOUD_PREFIX_MAX)))
    {
        return 5;
    }
    if (strpbrk(p->topic_prefix, "+#") != NULL)
    {
        return 5;
    }
    if (!str_is_printable(p->auth, CLOUD_AUTH_MAX))
    {
        return 6;
    }
    return 0;
}

static void cloud_profile_ensure_loaded(void)
{
    device_param_t *dp;

    if (s_loaded)
    {
        return;
    }
    s_loaded = true;

    dp = param_get_ptr();
    if (dp != NULL)
    {
        if (cloud_profile_validate(&dp->cloud) == 0)
        {
            s_profile = dp->cloud;
            return;
        }
    }
    cloud_profile_set_factory_default(&s_profile);
}

const cloud_profile_t *cloud_profile_get(void)
{
    cloud_profile_ensure_loaded();
    return &s_profile;
}

int cloud_profile_set(const cloud_profile_t *p)
{
    device_param_t *dp;
    int             rc;

    if (p == NULL)
    {
        return 1;
    }
    rc = cloud_profile_validate(p);
    if (rc != 0)
    {
        return rc;
    }

    s_profile = *p;
    s_loaded  = true;

    dp = param_get_ptr();
    if (dp != NULL)
    {
        dp->cloud = *p;
        (void)param_save_store();       /* 落 Flash */
    }
    return 0;
}

void cloud_profile_reset(void)
{
    cloud_profile_set_factory_default(&s_profile);
    s_loaded = true;
}

/**
 * @brief ★2026-10-08 新增★ 把 Flash 里的"遗留 LOCAL 档"一次性迁移到 DOC 档
 *
 * 【为什么必须做这件事 —— 现场真的踩了】
 *   `cloud_profile` 存在 `param_save` 的 Flash 页里(不在程序区!), 而：
 *     · `param_load_from_flash()` 只要那页 CRC 是好的, 就【用存下来的】参数;
 *     · `cloud_profile_ensure_loaded()` 只要存下来的那份 validate 通过, 就【用它】,
 *       出厂默认只是"没存过/坏了"时的兜底。
 *   ⇒ 旧固件(出厂默认 = LOCAL 档)跑过一次就会把 LOCAL 写进 Flash;
 *     **之后无论怎么重烧 App 都带不走它** —— 网关仍然订阅 mod/cmd,
 *     而甲方平台往 rt/gw/{gwId}/cmd 发, 结果就是"心跳看得到(发到 mod/status)、
 *     指令一条都收不到(订的是 mod/cmd)"。现场表现极具迷惑性。
 *   而且 `cloud_profile_reset()` 在此之前【没有任何调用者】, sys.factory_reset
 *   也清不掉它 —— 现场没有办法恢复。
 *
 * 【为什么是"迁移"而不是每次开机强制】
 *   只在"读到的不是 DOC 档"时改一次并落盘, 之后就不再干预 ——
 *   万一将来要保留 LOCAL 档做台架, 也有地方下手。
 *
 * @return true = 刚刚做了迁移(调用方应该打一条显眼的日志); false = 本来就是 DOC 档
 */
bool cloud_profile_migrate_to_doc(void)
{
    device_param_t *dp;

    if (cloud_profile_is_doc())
    {
        return false;                       /* 已经是文档档位, 什么都不做 */
    }

    s_profile        = *cloud_profile_get();    /* 先确保已加载(保留 gw_id / prefix / auth) */
    s_profile.mode   = (uint8_t)CLOUD_MODE_DOC;
    s_loaded         = true;

    dp = param_get_ptr();
    if (dp != NULL)
    {
        dp->cloud = s_profile;
        (void)param_save_store();               /* 落盘: 下次开机不再需要迁移 */
    }
    return true;
}

int cloud_profile_set_ram(const cloud_profile_t *p)
{
    int rc;

    if (p == NULL)
    {
        return 1;
    }
    rc = cloud_profile_validate(p);
    if (rc != 0)
    {
        return rc;
    }
    s_profile = *p;
    s_loaded  = true;
    return 0;                           /* 刻意不写 Flash */
}

bool cloud_profile_is_doc(void)
{
    return (cloud_profile_get()->mode == (uint8_t)CLOUD_MODE_DOC);
}

/**
 * @brief ★2026-10-08 新增: 用配置里的"设备 ID"(system.gwId) 覆盖档位的 gwId
 *
 * 【为什么需要】topic 里的 {gwId} 与 MQTT 用户名必须是【同一个值】,
 * 而这个值的真相源是 `config_store.system.gwId`(出厂 "110001", 可用 cfg.set 改)。
 * 本模块不依赖 config_store(要保持能在主机上脱机单测), 所以由 main.c 启动时注入,
 * 与既有的 g4_at_set_sub_topic() / g4_at_set_broker() 是同一个套路。
 *
 * @note 只改 RAM, 【不写 Flash】—— 持久化由 config_store 负责, 这里写盘会造成
 *       两个真相源(下次启动谁覆盖谁就不确定了)。
 * @note 只在【下一次 MQTT 建链】时生效(topic 是建链前拼好的)。
 */
void cloud_profile_set_gw_id(const char *gw_id)
{
    if ((gw_id == NULL) || (gw_id[0] == '\0'))
    {
        return;                     /* 空 = 不改, 保留兜底值 */
    }
    s_profile = *cloud_profile_get();       /* 先确保已加载 */
    strncpy(s_profile.gw_id, gw_id, CLOUD_GWID_MAX - 1u);
    s_profile.gw_id[CLOUD_GWID_MAX - 1u] = '\0';
}

bool cloud_profile_auth_required(void)
{
    const cloud_profile_t *p = cloud_profile_get();
    return (p->auth_enable != 0u) && (p->auth[0] != '\0');
}

/* ==================================================================
 * Topic 拼装
 * ================================================================== */

static bool app_str(char *out, uint16_t cap, uint16_t *n, const char *s)
{
    uint16_t l = (uint16_t)strlen(s);

    if ((uint16_t)(*n + l) >= cap)
    {
        return false;
    }
    (void)memcpy(out + *n, s, l);
    *n = (uint16_t)(*n + l);
    out[*n] = '\0';
    return true;
}

uint16_t cloud_profile_build_topic(cloud_topic_t t, char *out, uint16_t cap)
{
    const cloud_profile_t *p;
    uint16_t               n = 0u;

    if ((out == NULL) || (cap == 0u) || (t >= CLOUD_TOPIC_COUNT))
    {
        return 0u;
    }
    out[0] = '\0';
    p = cloud_profile_get();

    if (p->mode == (uint8_t)CLOUD_MODE_LOCAL)
    {
        const char *s = s_local_topic[t];
        uint16_t    l = (uint16_t)strlen(s);
        if (l >= cap)
        {
            return 0u;
        }
        (void)memcpy(out, s, (size_t)l + 1u);
        return l;
    }

    /* 文档模式: {prefix}/{gwId}/{suffix} */
    if (!app_str(out, cap, &n, p->topic_prefix))
    {
        out[0] = '\0';
        return 0u;
    }
    if ((!app_str(out, cap, &n, "/")) ||
        (!app_str(out, cap, &n, p->gw_id)) ||
        (!app_str(out, cap, &n, "/")) ||
        (!app_str(out, cap, &n, s_topic_suffix[t])))
    {
        out[0] = '\0';
        return 0u;
    }
    return n;
}

uint8_t cloud_profile_topic_qos(cloud_topic_t t)
{
    if (t >= CLOUD_TOPIC_COUNT)
    {
        return 0u;
    }
    if (cloud_profile_get()->mode == (uint8_t)CLOUD_MODE_LOCAL)
    {
        return 1u;                      /* 现网全 QoS1, 保持不变 */
    }
    return s_doc_qos[t];
}

bool cloud_profile_topic_retained(cloud_topic_t t)
{
    if (t >= CLOUD_TOPIC_COUNT)
    {
        return false;
    }
    if (cloud_profile_get()->mode == (uint8_t)CLOUD_MODE_LOCAL)
    {
        /* ★ v94: 现场档位也把 status 发成 retained ★（用户 2026-09-27 确认"可以加"）
         * 理由：平台侧的在线状态/心跳 + LWT 遗嘱通常依赖 retained
         *       —— 平台一订阅就能立刻拿到最后状态，不必等下一次心跳。
         * 其余 topic(应答/遥测/事件)仍不 retained：事件与遥测被 broker 存住的话，
         * 重连/重复订阅时会被**再投递一次**，看起来就像设备重复上报。
         * ★2026-10-08: lwt 也 retained★ —— 遗嘱是 broker 在网关掉线后【代发】的,
         * 平台那一刻不在线就永远看不到"掉线"这件事了, 所以必须 retained。 */
        return ((t == CLOUD_TOPIC_STATUS) || (t == CLOUD_TOPIC_LWT));
    }
    return s_doc_retained[t];
}

const char *cloud_profile_topic_name(cloud_topic_t t)
{
    if (t >= CLOUD_TOPIC_COUNT)
    {
        return "?";
    }
    return s_topic_suffix[t];
}

bool cloud_profile_check_auth(const char *auth_from_msg)
{
    const cloud_profile_t *p = cloud_profile_get();

    if (!cloud_profile_auth_required())
    {
        return true;                    /* 未启用 -> 放行 */
    }
    if (auth_from_msg == NULL)
    {
        return false;
    }
    return (strcmp(p->auth, auth_from_msg) == 0);
}
