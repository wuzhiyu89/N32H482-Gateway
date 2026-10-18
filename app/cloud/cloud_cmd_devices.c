/**
 * @file  cloud_cmd_devices.c
 * @brief 传感器 / 联动设备类命令（文档 5.3 / 5.4）
 *
 *   sen.bind / sen.unbind / sen.set / sen.query   —— 液位传感器
 *   dev.bind / dev.unbind / dev.set / dev.query    —— 声光等联动设备
 *   dev.ctrl                                       —— 手动控制（进 manual 模式）
 *
 * 两族共用同一套辅助函数（字段解析、set_entry、last-value 注入、配对钩子），
 * 所以放在同一个文件里，避免再把它们拆成跨文件接口。
 */
#include <string.h>

#include "cloud_proto.h"
#include "cloud_internal.h"
#include "cloud_json.h"
#include "config_store.h"
#include "last_value.h"
#include "level_supervisor.h"
#include "sensor_offline.h"
#include "bsp_tim.h"   /* get_tick_ms */
#include "log.h"
#include "lora_business.h"

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

int cloud_cmd_sen_unbind(const cloud_req_t *req, cJSON *data)
{
    return cloud_cmd_unbind(req, data, true);
}

int cloud_cmd_dev_unbind(const cloud_req_t *req, cJSON *data)
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

int cloud_cmd_sen_set(const cloud_req_t *req, cJSON *data)
{
    int rc = cloud_cmd_check_readonly(req->data);

    if (rc != CLOUD_EC_OK)
    {
        return rc;
    }
    return cloud_cmd_set_entry(req, data, true);
}

int cloud_cmd_dev_set(const cloud_req_t *req, cJSON *data)
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

int cloud_cmd_sen_query(const cloud_req_t *req, cJSON *data)
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

int cloud_cmd_dev_query(const cloud_req_t *req, cJSON *data)
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

int cloud_cmd_sen_bind(const cloud_req_t *req, cJSON *data)
{
    return cloud_cmd_bind(req, data, true);
}

int cloud_cmd_dev_bind(const cloud_req_t *req, cJSON *data)
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

int cloud_cmd_dev_ctrl(const cloud_req_t *req, cJSON *data)
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
