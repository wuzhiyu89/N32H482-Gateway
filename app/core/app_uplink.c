/**
 * @file  app_uplink.c
 * @brief 上行组装与发布：status.report / telemetry / event.* / ota.progress / 补报
 *
 * 这一层只做三件事：
 *   1. 把业务状态（配置 + 从机表 + 最近评估）取出来；
 *   2. 交给 cloud_proto 的构造器变成 JSON 信封；
 *   3. 按 cloud_profile 的 topic/QoS 调 g4_at_mqtt_publish() 登记发布。
 *
 * 不碰 AT 时序（那是 links/g4 的事），也不直接操作 Flash。
 * 断线时的取舍：事件进 event_cache，逐条遥测进 telemetry_cache，在线时直发。
 *
 * ⚠ 整段仍由 MAIN_STATUS_JSON_ENABLE 守卫（=0 时本文件的函数不存在，
 *   目前的 main() 还没做"关掉 JSON 也能编"的适配，所以这个开关实际上必须为 1）。
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "log.h"
#include "app_config.h"
#include "app_core.h"
#include "app_state.h"
#include "bsp_tim.h"
#include "cloud_profile.h"
#include "cloud_proto.h"
#include "config_store.h"
#include "data_cache.h"
#include "event_cache.h"
#include "g4_at.h"
#include "g4_urc.h"
#include "last_value.h"
#include "level_eval.h"
#include "level_supervisor.h"
#include "lora_business.h"
#include "ota_health.h"
#include "ota_http.h"
#include "ota_port.h"
#include "param_save.h"
#include "sensor_offline.h"
#include "utils.h"
#include "telemetry_cache.h"

#if MAIN_STATUS_JSON_ENABLE
/**
 * @brief 组一条 status.report 并按档位发到 status topic(文档 7.7 / 任务 P1-8)
 *
 *   本地档位 -> 发到 "rt/gw/{gwId}/status"(现网正在看的那个), QoS1, 不 retained
 *              —— 不用换 broker 就能在云上看到 JSON
 *   文档档位 -> 发到 "rt/gw/{gwId}/status", QoS1 + retained(文档 2 章)
 *
 * ⚠ 字段口径(与 cloud_proto.h 的说明一致):
 *     rsrp = 用 AT+CSQ 换算的 RSSI(3GPP: rssi=-113+2*csq), 本工程拿不到真 RSRP;
 *     configVersion / mode / currentGear 在 P2(配置) / P3(边缘评估) 落地前是占位值。
 */
/*!
 * \brief 填一份网关状态(供 status.report 与 sys.query_status 共用)
 * \note  ★ 只留这一处 ★ 在线数/信号/运行时长如果两处各算一份, 迟早会对不上
 *        (比如一边按从机表数、另一边按配置数)。sys.query_status 通过
 *        cloud_proto_set_status_provider() 直接调本函数。
 */
void app_fill_status(cloud_status_t *st)
{
    static const char fw_ver[] = MAIN_FW_VERSION;
    lora_slave_t     *sl;
    uint8_t           cnt;
    uint8_t           i;

    (void)memset(st, 0, sizeof(*st));

    /* 子设备在线/总数 —— 来自 LoRa 从机表
     *
     * ★2026-10-18: 判据统一到 lora_business_device_state_bytes() ★
     *   以前这里直接用 sl[i].is_online, 而那个字段对【声光】是"收到过帧/绑定过"
     *   的粘滞标志 —— 声光断电后它仍然是 true(声光平时不收帧, 永远不会被清),
     *   于是出现自相矛盾: dev.query 说 online:false(按"3 条 0x03 都没成功"判),
     *   status.report 的 devicesOnline 却还把它算作在线。
     *   现在两处共用同一份判据(见 lora_business_device_state_bytes):
     *     · 液位 : is_online(由 sensor_offline 状态机维护)
     *     · 声光 : !alarm_offline_reported(3 条 0x03 都没成功才置位)
     *   ⇒ status.report / sys.query_status / sen.query / dev.query 四处的
     *     在线口径从此完全一致。 */
    sl  = lora_business_get_slave_list();
    cnt = lora_business_get_slave_count();
    for (i = 0u; (i < cnt) && (sl != NULL); i++)
    {
        bool on = (lora_business_device_state_bytes(sl[i].dev_uid)
                   == LORA_DEV_STATE_ONLINE);

        if (sl[i].dev_type == SLAVE_TYPE_ALARM_OUT)
        {
            st->devices_total++;
            if (on)
            {
                st->devices_online++;
            }
        }
        else
        {
            st->sensors_total++;
            if (on)
            {
                st->sensors_online++;
            }
        }
    }

    st->online         = true;
    st->fw_version     = fw_ver;
    st->slot           = (ota_port_current_slot() == 1u) ? "B" : "A";
    st->config_version = config_store_version();   /* P2 已落地, 不再是占位的 0 */
    st->rsrp_dbm       = g4_at_get_info()->rssi_dbm;
    st->uptime_s       = get_tick_ms() / 1000u;
    /* ★ 2026-09-30: 不再是占位值 —— 取边缘评估算出的【实时】档位/状态。
     *   以前这里恒填 "auto"/0, 平台看到的档位永远是 0。
     *   mode 暂时仍恒 "auto": manual 模式要等第 6 条(dev.ctrl)落地后才有第二种取值。 */
    st->mode           = "auto";
    st->current_gear   = app_live_gear();
}

sys_err_t app_publish_status_json(void)
{
    static uint16_t   seq;
    cloud_status_t    st;
    char              data[CLOUD_DATA_MAX];
    char              payload[CLOUD_RESP_MAX];
    char              topic[64];
    char              msg_id[20];
    uint16_t          dlen;
    uint16_t          plen;
    uint16_t          tlen;

    app_refresh_epoch();

    /* 状态字段统一由 app_fill_status() 填 —— 与 sys.query_status 共用一处 */
    app_fill_status(&st);

    dlen = cloud_proto_build_status_data(data, (uint16_t)sizeof(data), &st);
    if (dlen == 0u)
    {
        return SYS_ERR;
    }

    seq++;
    (void)utils_gen_msg_id_seq(msg_id, (uint16_t)sizeof(msg_id), (uint8_t)'h',
                              (uint16_t)(seq % 1000u));

    plen = cloud_proto_build_uplink(payload, (uint16_t)sizeof(payload),
                                    "status.report", msg_id, app_now_unix(), data);
    if (plen == 0u)
    {
        return SYS_ERR;
    }

    tlen = cloud_profile_build_topic(CLOUD_TOPIC_STATUS, topic, (uint16_t)sizeof(topic));
    if (tlen == 0u)
    {
        return SYS_ERR;
    }

    /* 【弱网关键】上报正文也打到串口: 离线时 MQTT 发不出去, 但"我们准备发的到底
     * 是什么"必须看得见, 否则无法判断是协议层错了还是网络没通。 */
    log_info("[CLOUD] 上报 %s (%u 字节): %s\r\n", topic, (unsigned)plen, payload);

    return g4_at_mqtt_publish_ex(topic, (const uint8_t *)payload, plen,
                                 cloud_profile_topic_qos(CLOUD_TOPIC_STATUS),
                                 cloud_profile_topic_retained(CLOUD_TOPIC_STATUS));
}

/* ==================================================================
 * telemetry.report —— 水位遥测上行(文档 7.1, 用户 2026-09-30 第 3 条)
 * ==================================================================
 * 【触发条件】★2026-10-18 用户要求: 与心跳【严格同周期】★
 *     · 周期 = config_store.network.statusIntervalSeconds(10~600s, 默认 30);
 *       —— 与 status.report(心跳)用的是【同一个字段、同一个兜底值(30s)】,
 *          所以"心跳一拍, 遥测也一拍", 两者的时间点始终对齐。
 *     · 每条遥测都是【全量】: 把当时所有"在线且启用"的液位【一起】塞进 data.sensors[]。
 *       绑定了 2 台就有 2 个元素, 不会一台一条地发。
 *     · 文档 7.1 里"水位变化超过 minChangeCm 也触发"这条【已停用】
 *       (见 uplink_telemetry_ready 的 ④) —— 否则上报间隔会被水位抖动带偏,
 *       不再等于心跳周期。上电第一条仍然恒发。
 *
 * 【★ 水位怎么算: 取【平均值】, 以及为什么 ★】
 *   用户口径(2026-09-30 第 3 条): 遥测里的水位取各液位传感器的【平均值】
 *   (最多 4 台), 不按文档那套"逐台 + 有效水位取 max + 逐台防抖"来做。
 *   文档 3.3 末的"有效水位 = 取 max"仍然保留给【告警判定/联动】用
 *   (level_eval / level_supervisor 一行没动) —— 也就是:
 *       告警按 max(保守, 有一点淹就报), 遥测按 mean(反映整体水面)。
 *   这个差别是有意的, 写在这里免得以后有人"顺手统一"。
 *
 * 【★ 各路液位不是同一时刻的 —— 处理方式 ★】
 *   这是平均值最容易被忽略的坑: 4 台传感器各自 60s 上报一次, 收到某台的新值时,
 *   另外 3 台手里的是【几十秒前的旧值】。如果直接平均, 就会出现"水面正在涨,
 *   平均却在降"的错觉。本实现这样处理(都是显式规则, 不是"差不多就行"):
 *     ① 只把【在线且 enabled】的算进来 —— 在线判据复用 sensor_offline 那一套
 *        (超过 3×reportPeriodSeconds 没上报 = 离线), 与"失联检测"口径一致;
 *     ② 每台在 data.sensors[] 里【各自带自己的 ts】(文档 7.1 本来就有这个字段)
 *        —— 平台能直接看出这几包是不是同一时刻的, 这是最诚实的做法:
 *        网关不假装它们同时到达, 也不替平台去加权;
 *     ③ 平均值是"快照"(各台的最新值), 不是"同一时刻的插值" —— 所以本字段适合
 *        看趋势/展示, 不适合当"某一瞬间的真实水位"做计量。要更严谨得上时间加权,
 *        那属于平台侧的事(它拿得到每台的 ts)。
 *     ④ 只有一台在线时, 平均值就等于它自己(退化成单点值), 这是可接受的。
 * ================================================================== */

#define MAIN_TELEMETRY_RETRY_MS   5000U   /* 发布登记失败后多久重试(不占满整个周期) */

/* 遥测的时基与状态(纯 RAM) */
static struct
{
    uint32_t attempt_ms;   /* 上次【尝试】发布的 tick */
    uint32_t ok_ms;        /* 上次【成功】发布的 tick(周期基准) */
    int32_t  last_mm;      /* 上次成功发布时的平均值(变化量基准) */
    bool     have;         /* 成功发过至少一条 */
    bool     fail;         /* 上次尝试失败(只影响"提前重试") */
} s_tel;
static uint16_t s_tel_seq;

/**
 * @brief 组装并发布 telemetry.report
 * @param avg_mm 各在线液位传感器的平均水位(mm)
 * @return SYS_OK = 已交给模组
 */
static sys_err_t uplink_publish_telemetry(int32_t avg_mm)
{
    static cloud_telemetry_sensor_t sens[CFG_SENSOR_MAX];
    static char        data[CLOUD_RESP_MAX];
    static char        payload[CLOUD_RESP_MAX];
    static char        topic[64];
    const config_store_t *cfg = config_store_get();
    cloud_telemetry_t  t;
    uint8_t            n = 0u;
    uint8_t            i;
    char               msg_id[20];
    uint16_t           dlen;
    uint16_t           plen;
    uint32_t           now = get_tick_ms();

    if (cfg == NULL) { return SYS_ERR; }

    (void)memset(&t, 0, sizeof(t));
    (void)memset(sens, 0, sizeof(sens));

    /* ---- 逐台填: 只列【启用的 + 有最近值 + 还没超时】的液位传感器 ---- */
    for (i = 0u; (i < cfg->sensor_count) && (i < (uint8_t)CFG_SENSOR_MAX); i++)
    {
        const cfg_sensor_t *s = &cfg->sensors[i];
        uint8_t      uid3[CFG_UID_BYTES];
        last_value_t lv;
        uint32_t     thr;

        if (s->enabled == 0u) { continue; }
        if (!cfg_uid_to_bytes(s->uid, uid3)) { continue; }
        if (!last_value_get(uid3, &lv)) { continue; }        /* 从没上报过 */

        /* 在线判据与"失联检测"完全同一套: 3 × reportPeriodSeconds */
        thr = sensor_offline_threshold_ms(s->report_period_s);
        if ((uint32_t)(now - lv.rx_tick_ms) >= thr) { continue; }   /* 已超时 -> 不参与 */

        sens[n].uid         = s->uid;
        sens[n].level_mm    = (int32_t)lv.level_mm;
        sens[n].raw_mm      = (uint32_t)lv.level_mm;   /* 液位透传(D-1): 网关不做换算, 原始值就是它 */
        sens[n].battery_pct = (uint8_t)((lv.bat_pct > 100u) ? 100u : lv.bat_pct);
        sens[n].rssi_dbm    = lv.rssi_dbm;              /* 第 10 条: 该帧的 LoRa RSSI */
        sens[n].ts          = lv.rx_unix;               /* 各自的时间戳, 不假装同时 */
        n++;
    }

    t.sensor_count        = n;
    t.sensors             = sens;
    t.effective_level_mm  = avg_mm;
    t.effective_status    = app_live_state_name();
    t.effective_gear      = app_live_gear();

    dlen = cloud_proto_build_telemetry_data(data, (uint16_t)sizeof(data), &t);
    if (dlen == 0u)
    {
        log_info("[CLOUD] !! telemetry.report data 段组装失败\r\n");
        return SYS_ERR;
    }

    s_tel_seq++;
    (void)utils_gen_msg_id_seq(msg_id, (uint16_t)sizeof(msg_id), (uint8_t)'m',
                              (uint16_t)(s_tel_seq % 1000u));

    plen = cloud_proto_build_uplink(payload, (uint16_t)sizeof(payload),
                                    "telemetry.report", msg_id, app_now_unix(), data);
    if (plen == 0u)
    {
        log_info("[CLOUD] !! telemetry.report 信封组装失败\r\n");
        return SYS_ERR;
    }

    if (cloud_profile_build_topic(CLOUD_TOPIC_TELEMETRY, topic,
                                  (uint16_t)sizeof(topic)) == 0u)
    {
        log_info("[CLOUD] !! telemetry topic 拼装失败\r\n");
        return SYS_ERR;
    }

    log_info("[CLOUD] 上报 %s (%u 字节): %s\r\n", topic, (unsigned)plen, payload);

    return g4_at_mqtt_publish_ex(topic, (const uint8_t *)payload, plen,
                                 cloud_profile_topic_qos(CLOUD_TOPIC_TELEMETRY),
                                 cloud_profile_topic_retained(CLOUD_TOPIC_TELEMETRY));
}

/**
 * @brief 算平均值 + 判"该不该发遥测"
 * @param out_avg 输出: 平均值(mm); 没有任何有效传感器时不改它
 * @return true = 本次应发; false = 不发
 * @note  只有【在线且 enabled】的传感器参与平均(规则见上面长注释)。
 */
static bool uplink_telemetry_ready(const config_store_t *cfg, uint32_t now_ms,
                                 int32_t *out_avg)
{
    int32_t  sum = 0;
    uint8_t  n   = 0u;
    uint8_t  i;
    int32_t  delta;

    if ((cfg == NULL) || (out_avg == NULL)) { return false; }

    for (i = 0u; (i < cfg->sensor_count) && (i < (uint8_t)CFG_SENSOR_MAX); i++)
    {
        const cfg_sensor_t *s = &cfg->sensors[i];
        uint8_t      uid3[CFG_UID_BYTES];
        last_value_t lv;

        if (s->enabled == 0u) { continue; }
        if (!cfg_uid_to_bytes(s->uid, uid3)) { continue; }
        if (!last_value_get(uid3, &lv)) { continue; }
        if ((uint32_t)(now_ms - lv.rx_tick_ms) >= sensor_offline_threshold_ms(s->report_period_s))
        {
            continue;
        }
        sum += (int32_t)lv.level_mm;
        n++;
    }

    if (n == 0u) { return false; }                 /* 一台有效的都没有 -> 没什么可报 */

    *out_avg = sum / (int32_t)n;                   /* 整数平均, 单位 mm, 不引浮点 */

    /* ① 第一条恒发 */
    if (!s_tel.have) { return true; }

    /* ② 上次登记失败 -> 5s 后重试(不占满整个周期, 弱网下水位能早点上去) */
    if (s_tel.fail
        && ((uint32_t)(now_ms - s_tel.attempt_ms) >= MAIN_TELEMETRY_RETRY_MS))
    {
        return true;
    }

    /* ③ 周期到(周期 = network.statusIntervalSeconds, 与心跳同一个字段)
     *
     * ★2026-10-18 用户要求: 【上报周期与心跳严格一致】★
     *   兜底值原来是 60000, 而心跳(MAIN_GW_ONLINE_MS)的兜底是 30000 ——
     *   只要 statusIntervalSeconds 缺失/越界, 两者间隔就差一倍。
     *   现在统一成 30000(与 MAIN_GW_ONLINE_MS 同值), 保证"心跳一拍、遥测也一拍"。
     *   ⚠️ MAIN_GW_ONLINE_MS 定义在本文件更靠后的 1584 行, 这里用不到,
     *      所以直接写 30000 —— 改那两个数时要【同步维护】。 */
    if ((uint32_t)(now_ms - s_tel.ok_ms)
        >= app_cfg_period_ms(cfg->network.status_interval_s, 30000U))
    {
        return true;
    }

    /* ④ 【已停用】变化量 >= minChangeCm 的提前触发
     *
     * ★2026-10-18 用户要求"网关上报周期应该和心跳一起(间隔一致)"★
     *   原来这条会让遥测在任何一台液位变化超过 minChangeCm 时【提前】发,
     *   于是实际上报间隔取决于水位抖动, 与心跳不同步。
     *   现在只保留周期触发(③) —— 每条遥测都是【整周期、全量、所有液位一起】。
     *   要恢复旧行为, 把下面的 #if 0 改成 #if 1 即可。 */
#if 0
    delta = *out_avg - s_tel.last_mm;
    if (delta < 0) { delta = -delta; }
    if (delta >= (int32_t)cfg->thresholds.min_change_cm * 10) { return true; }
#endif

    (void)delta;                    /* delta 只在上面那段(默认关掉)里用, 消掉未使用告警 */

    return false;
}


/**
 * @brief 遥测的一步(算平均 -> 判据 -> 发布 -> 回填时基)
 * @note  只在主循环 1s 节拍里调; 4G 不在线时直接跳过(模组自己会拒),
 *        但【不】刷新成功基准, 免得"没发出去也算发过"。
 */
void app_telemetry_step(void)
{
    const config_store_t *cfg = config_store_get();
    uint32_t now = get_tick_ms();
    int32_t  avg = 0;

    if ((cfg == NULL) || (!g4_at_is_online())) { return; }
    if (!uplink_telemetry_ready(cfg, now, &avg)) { return; }

    s_tel.attempt_ms = now;

    if (uplink_publish_telemetry(avg) == SYS_OK)
    {
        s_tel.ok_ms   = now;
        s_tel.last_mm = avg;
        s_tel.have    = true;
        s_tel.fail    = false;
    }
    else
    {
        s_tel.fail = true;
        log_info("[CLOUD] !! telemetry.report 发布登记失败, %us 后重试\r\n",
                 (unsigned)(MAIN_TELEMETRY_RETRY_MS / 1000U));
    }
}

/* ==================================================================
 * 离线期间的【逐条液位上报】缓存 + 重连补报 (★2026-10-18)
 * ==================================================================
 * 【用户口径】离线后补报的是【每次传感器上报的水位】, 不是 30 秒聚合遥测。
 *   ⇒ 一条记录 = 一台液位的一次 0x02; 补报时一条记录发一条 telemetry.report
 *     (sensors[] 里就那一个元素)。
 *
 * 【为什么以前会丢】app_telemetry_step() 一开头就
 *   `if (!g4_at_is_online()) return;` —— 断线期间根本不生成遥测。
 *   而 LoRa 与 4G 独立, 液位照常上报 ⇒ 这些数据直接消失了。
 *
 * 【时间戳】rec.ts = 收到这一帧时的 Unix 秒 ⇒ 补报时【原样】用于
 *   信封 ts 与 sensors[].ts —— 用户明确要求"要是当时的时间戳"。
 *
 * 【不重复】只在 4G 离线时捕获; 在线时走原有的 30s 聚合遥测, 两条路互斥。
 * ================================================================== */

/** 每次液位上报(0x02)的回调 —— 4G 离线时存一条, 在线时什么也不做 */
void app_on_level_report(const uint8_t uid[LORA_UID_BYTES],
                                 uint16_t level_mm, uint8_t bat_pct,
                                 int8_t rssi_dbm)
{
    tc_record_t rec;

    if (g4_at_is_online())
    {
        return;                     /* 在线: 走 30s 聚合遥测, 不重复存 */
    }
    if (uid == NULL)
    {
        return;
    }

    (void)memset(&rec, 0, sizeof(rec));
    (void)memcpy(rec.uid, uid, TELEMETRY_UID_BYTES);
    rec.level_mm = level_mm;
    rec.bat_pct  = bat_pct;
    rec.rssi_dbm = rssi_dbm;
    rec.ts       = app_now_unix();      /* ★收到这一帧的时刻(补报不改) */
    rec.state    = app_live_state();
    rec.gear     = app_live_gear();
    s_tel_seq = (uint16_t)((s_tel_seq + 1u) % 1000u);
    rec.msg_seq  = s_tel_seq;            /* 与在线遥测共用同一个序号计数器 */

    (void)telemetry_cache_push(&rec);
}

/**
 * @brief 重连后补报一条"逐条上报"遥测
 * @note  一次只发一条(一条记录 = 一条消息), 由主循环反复调用直到清空。
 *        与 app_backfill_poll 同一条纪律: 【先发后删】, 登记失败一条都不动;
 *        "渲染不出来"的条也出队(否则永远堵在队头), 但会打日志。
 */
static uint32_t _bf_msg_hash(const uint8_t uid[TELEMETRY_UID_BYTES],
                              uint32_t ts, uint16_t seq)
{
    uint32_t h = 2166136261u;
    uint8_t  i;

    for (i = 0u; i < TELEMETRY_UID_BYTES; i++) { h ^= uid[i]; h *= 16777619u; }
    h ^= ts;  h *= 16777619u;
    h ^= seq; h *= 16777619u;
    return (h != 0u) ? h : 1u;
}

void app_telemetry_backfill_poll(void)
{
    /* 一律 static: 栈只有 4 KB */
    static char data[CLOUD_RESP_MAX];
    static char payload[CLOUD_RESP_MAX];
    static char topic[64];
    const tc_record_t *rec;
    cloud_telemetry_sensor_t s1;
    cloud_telemetry_t  t;
    char               uid_hex[CFG_UID_STR_MAX];
    char               msg_id[20];
    uint16_t           dlen;
    uint16_t           plen;
    uint16_t           tlen;

    static uint32_t last_ms;

    if (!g4_at_is_online())
    {
        return;                     /* 还断着: 不补 */
    }
    /* ★2026-10-27: 补报节流 1 条/秒(避免重连后瞬间灌满模组队列)★ */
    if ((last_ms != 0u) && ((uint32_t)(get_tick_ms() - last_ms) < 1000u))
    {
        return;
    }
    last_ms = get_tick_ms();

    rec = telemetry_cache_peek();
    if (rec == NULL)
    {
        last_ms = 0u;               /* 空了 -> 下一条不用等 */
        return;                     /* 没有待补的 */
    }

    (void)cfg_uid_from_bytes(rec->uid, uid_hex);

    (void)memset(&s1, 0, sizeof(s1));
    s1.uid         = uid_hex;
    s1.level_mm    = (int32_t)rec->level_mm;
    s1.raw_mm      = (uint32_t)rec->level_mm;
    s1.battery_pct = rec->bat_pct;
    s1.rssi_dbm    = (int32_t)rec->rssi_dbm;
    s1.ts          = rec->ts;           /* ★该台上报发生的时刻 */

    (void)memset(&t, 0, sizeof(t));
    t.sensor_count       = 1u;          /* 一条记录就一台 */
    t.sensors            = &s1;
    t.effective_level_mm = (int32_t)rec->level_mm;   /* 只有这一个数据点, 有效值就是它 */
    t.effective_status   = (rec->state != 0u) ? cloud_proto_state_name(rec->state)
                                              : "Normal";   /* 与 app_live_state_name 同口径 */
    t.effective_gear     = rec->gear;

    dlen = cloud_proto_build_telemetry_data(data, (uint16_t)sizeof(data), &t);
    if (dlen == 0u)
    {
        log_info("[CLOUD] !! 补报遥测 渲染失败(uid=%s) -> 丢弃这一条(否则堵队头)\r\n", uid_hex);
        telemetry_cache_drop();
        return;
    }

    (void)snprintf(msg_id, sizeof(msg_id), "%08lx",
                    (unsigned long)_bf_msg_hash(rec->uid, rec->ts, rec->msg_seq));

    /* ★信封 ts 也用 record 的 ts —— 不是补报时刻 */
    plen = cloud_proto_build_uplink(payload, (uint16_t)sizeof(payload),
                                    "telemetry.report", msg_id, rec->ts, data);
    if (plen == 0u)
    {
        log_info("[CLOUD] !! 补报遥测 信封失败(uid=%s) -> 丢弃这一条\r\n", uid_hex);
        telemetry_cache_drop();
        return;
    }

    tlen = cloud_profile_build_topic(CLOUD_TOPIC_TELEMETRY, topic, (uint16_t)sizeof(topic));
    if (tlen == 0u)
    {
        return;                     /* topic 拼不出来: 不丢, 下次再试 */
    }

    log_info("[CLOUD] 补报 %s (%u 字节, ts=%lu): %s\r\n",
             topic, (unsigned)plen, (unsigned long)rec->ts, payload);

    if (g4_at_mqtt_publish_ex(topic, (const uint8_t *)payload, plen,
                              cloud_profile_topic_qos(CLOUD_TOPIC_TELEMETRY),
                              cloud_profile_topic_retained(CLOUD_TOPIC_TELEMETRY)) != SYS_OK)
    {
        return;                     /* 登记失败: 不丢, 下次再来 */
    }

    telemetry_cache_drop();
}


/* ==================================================================
 * ota.progress 上报 + 升级就绪后复位(文档 7.6 / 第 6 章) —— 2026-10-02
 * ==================================================================
 * 触发口径(文档: "每 10% 或 10s 一条"):
 *   · 状态变了           -> 立刻一条(idle->downloading、->verifying、->failed 都要看得见)
 *   · 进度跨过 10% 台阶   -> 一条
 *   · 距上一条满 10s      -> 一条(下载卡住时平台也能看出"还在动还是死了")
 *
 * 【为什么由 main.c 发而不是 cloud_proto】: cloud_proto 是纯协议层, 要能在 PC 上
 * 脱机单测; 真正碰 AT/MQTT 的动作按本工程惯例留在 main.c(同 sys.time_sync)。
 *
 * 【READY 之后的重启也在这里】: 协议第 6 章要求"标记启动分区 -> 重启"。写标记页
 * 是 ota_http 在 VERIFYING 里做完的, 复位由本函数倒数 3 拍(≈3s)后执行 —— 留这几
 * 秒是为了让"就绪/即将复位"的日志真的从串口发出去, 现场靠它判断走到哪一步了。
 * ================================================================== */

/* ★2026-10-05 台架验证开关: READY 之后要不要自动复位★
 *
 * 0 = 【当前默认, 台架验证用】校验通过后【不复位】, 停在 READY 只上报。
 *     为什么: 当前分区是【模式 1】(app/common/flash_map.h: FM_MAP_MODE =
 *     FM_MAP_SINGLE_RESERVED, FM_HAS_BOOTLOADER = 0) —— App 就在 0x08000000,
 *     板上【没有 bootloader】,"把暂存区搬到运行区"那件事只有 src/main_bl.c 的
 *     bl_run() 会做, 而它属于另一个 BL 构建、不在 App 里。
 *     所以复位了也只是从 0x08000000 重跑旧固件 —— 白重启一次, 还把串口刷掉,
 *     反而看不清"下载 + MD5 + 写标记"到底成没成。
 *     停在这里的意义: READY = 下载完整、MD5 比对通过、标记页已写, 这三件事都成了。
 *
 * 1 = 恢复正常行为(3s 后复位交给 bootloader 搬运)。
 *
 * ⚠【上线前必须改回 1】—— 走模式 2(双 Slot + 已烧 BL)时, 不改回来升级永远不生效。
 * ⚠ 停在 READY 的副作用: ota.query 会一直回 installing; 期间再下 ota.start 会被
 *   当"正忙"拒掉(设计如此, 主机测试 [12] 钉着), ota.abort 在 READY 也被拒(协议 5.6)。
 *   要重测一次得【断电重上】(或等以后接一个"清 READY"的本地命令)。 */
#ifndef MAIN_OTA_AUTO_REBOOT
#define MAIN_OTA_AUTO_REBOOT     0
#endif

void app_ota_progress_poll(void)
{
    static uint8_t  s_last_state   = 0xFFu;
    static uint8_t  s_last_percent = 0xFFu;
    static uint32_t s_t_last;
    static uint8_t  s_reboot_cnt;
    static uint8_t  s_ready_announced;      /* 非自动复位时, READY 那几行只打一次 */

    static char       payload[CLOUD_RESP_MAX];
    static char       data[160];
    static char       topic[64];
    ota_http_status_t st;
    uint32_t          now = get_tick_ms();
    char              msg_id[20];
    uint16_t          plen;
    uint16_t          tlen;

    ota_http_get_status(&st);

    /* ---- 空闲: 复位节流, 且【什么都不报】----
     * ⚠ 必须【直接 return】: 早期写法只复位 s_last_state = 0xFF 就往下走, 于是
     * "IDLE(0) != 0xFF" 恒成立 -> 设备空闲时每 10s 往云端刷一条 {"state":"idle"}。
     * (现在这条判断本身已经收进 ota_http_progress_due(), 主机侧有测试盯着。) */
    if (st.state == OTA_HTTP_IDLE)
    {
        s_last_state      = 0xFFu;
        s_last_percent    = 0xFFu;
        s_reboot_cnt      = 0u;
        s_ready_announced = 0u;
        return;
    }

    /* ---- 就绪: 先起倒数, 交给 bootloader 搬运 ----
     * 【为什么倒数不放在下面的 send 分支里】: ota_http_progress_due() 对 READY
     * 只会在"首次进入"时返回 true, 放进去就只减一次, 永远数不到 0 -> 永不复位。
     * 所以倒数必须【每一拍】都推进, 与上报节流解耦。 */
    if (st.state == OTA_HTTP_READY)
    {
#if MAIN_OTA_AUTO_REBOOT
        if (s_reboot_cnt == 0u)
        {
#if (FM_HAS_BOOTLOADER != 0)
            {
                ota_boot_rec_t r;
                uint8_t        tgt = ota_port_target_slot();

                if (!ota_port_boot_load(&r)) { r.seq = 0u; }
                r.active    = ota_port_current_slot();
                r.pending   = tgt;
                r.confirmed = 0u;
                r.tries     = OTA_BOOT_TRIES_INIT;
                r.seq++;
                if (!ota_port_boot_save(&r))
                {
                    log_info("[AB] !! 启动记录写入失败, 升级不会切换槽\r\n");
                }
                else
                {
                    log_info("[AB] 已登记 pending=%c(剩 %u 次), 3s 后从该槽启动\r\n",
                             (tgt == 0u) ? 'A' : 'B', (unsigned)r.tries);
                }
            }
#endif
            s_reboot_cnt = 3u;
            log_info("[OTA_HTTP] 校验通过且标记页已写, 3s 后复位交给 bootloader\r\n");
        }
        else
        {
            s_reboot_cnt--;
            if (s_reboot_cnt == 0u)
            {
                log_info("[OTA_HTTP] 复位! bootloader 将把暂存区搬到运行区\r\n");
                ota_port_reboot();          /* 不返回 */
            }
        }
#else
        /* 台架模式: 只宣告一次, 然后一直停在 READY(每一拍都走这里, 但只打一次日志) */
        if (s_ready_announced == 0u)
        {
            s_ready_announced = 1u;
            log_info("[OTA_HTTP] ================================================\r\n");
            log_info("[OTA_HTTP] ★ 校验通过: 下载完整 + MD5 比对成功 + 标记页已写 ★\r\n");
            log_info("[OTA_HTTP]   本固件 MAIN_OTA_AUTO_REBOOT=0, 【不自动复位】。\r\n");
            log_info("[OTA_HTTP]   原因: 当前是分区模式 1(FM_HAS_BOOTLOADER=0), 板上没有\r\n");
            log_info("[OTA_HTTP]   bootloader, 复位也只会从 0x08000000 重跑旧固件。\r\n");
            log_info("[OTA_HTTP]   => 台架验证到此为止【算成功】; 要真装上新固件请走\r\n");
            log_info("[OTA_HTTP]      模式 2 + 先烧 BL, 并把本宏改回 1。\r\n");
            log_info("[OTA_HTTP]   重测一次: 断电重上(READY 期间 ota.start 会被当忙拒掉)。\r\n");
            log_info("[OTA_HTTP] ================================================\r\n");
        }
#endif
        /* 不 return: 下面照常把 installing 那一条报出去 */
    }

    /* ★下载完成必须单独补报一条★
     *   DOWNLOADING -> VERIFYING 可能短于一个轮询周期, 只靠采样云端看不到
     *   下载完成。ota_http 在迁移点设了 latch, 这里强制发 verifying/100%。
     *   st 是本函数的局部副本, 改它只影响这一条上报。 */
    {
        uint8_t force_verify = ota_http_take_download_done() ? 1u : 0u;

        if ((force_verify == 0u) &&
            (!ota_http_progress_due(&st, s_last_state, s_last_percent,
                                    now, s_t_last, MAIN_OTA_PROGRESS_MS)))
        {
            return;
        }
        if (force_verify != 0u)
        {
            st.state   = OTA_HTTP_VERIFYING;
            st.percent = 100u;
        }
    }

    s_last_state   = (uint8_t)st.state;
    s_last_percent = st.percent;
    s_t_last       = now;

    /* 状态名走 ota_http_state_name(): 与 ota.query 的应答必须同口径 */
    if (st.version[0] != '\0')
    {
        (void)snprintf(data, sizeof(data),
                       "{\"state\":\"%s\",\"percent\":%u,\"targetVersion\":\"%s\"}",
                       ota_http_state_name(st.state), (unsigned)st.percent, st.version);
    }
    else
    {
        (void)snprintf(data, sizeof(data), "{\"state\":\"%s\",\"percent\":%u}",
                       ota_http_state_name(st.state), (unsigned)st.percent);
    }

    (void)utils_gen_msg_id(msg_id, (uint16_t)sizeof(msg_id));

    plen = cloud_proto_build_uplink(payload, (uint16_t)sizeof(payload),
                                    "ota.progress", msg_id, app_now_unix(), data);
    if (plen == 0u)
    {
        log_info("[CLOUD] !! ota.progress 信封组装失败\r\n");
        return;
    }

    tlen = cloud_profile_build_topic(CLOUD_TOPIC_EVENT, topic, (uint16_t)sizeof(topic));
    if (tlen == 0u)
    {
        return;                         /* 离线: 进度丢了不致命, 平台会来 ota.query */
    }

    log_info("[CLOUD] 上报 %s (%u 字节): %s\r\n", topic, (unsigned)plen, payload);
    (void)g4_at_mqtt_publish_ex(topic, (const uint8_t *)payload, plen,
                                cloud_profile_topic_qos(CLOUD_TOPIC_EVENT),
                                cloud_profile_topic_retained(CLOUD_TOPIC_EVENT));
}

/* ==================================================================
 * sys.time_sync 的落地：把平台给的 Unix 秒写进 4G 模组时钟 (v91)
 * ==================================================================
 * 云协议层只负责解析/校验并把 Unix 秒交出来(钩子)，真正碰 AT 的在这里 ——
 * 这样协议层仍然能在 PC 上脱机单测。
 * ★ 换算用 v83 的 cloud_proto_unix_to_calendar()，时区取 +28800(北京)：
 *   写进模组的是北京时间 + "+32" 偏移，与模组自己上报的格式一致。★
 * ★ 换算刻意放这里而不是 g4_at：见 g4_at.h 里的说明（否则 test_state 链接失败）。★
 */
bool app_time_sync(uint32_t unix_ts)
{
    /* ============================================================================
     * ★ 2026-09-30 用户拍板的口径: 【网关不接受云端下发的时钟】★
     * ----------------------------------------------------------------------------
     * 收到 sys.time_sync 时, 不拿平台的 ts 去写时钟, 而是【向 4G 模组要时间】,
     * 用它刷新网关自己的 epoch, 然后回成功。
     *
     * 为什么这么定: 网关没有 RTC 电池, 唯一可信的时间源就是 4G 网络给的(模组 AT+CCLK);
     *   平台的时间对网关来说是"另一个系统的时钟", 直接采信等于把两个时钟源混在一起。
     *
     * 旧实现(已废弃): 把平台的 ts 换算成日历后【写进模组】(g4_at_set_clock_ymdhms),
     *   写失败就回 1007 —— 于是"模组当时不可用"会让平台看到"校时失败",
     *   而实际上我们自己的时间本来就可以从模组读到。
     *
     * ⚠️ 关于"上行的 ts 从此完全跟随模组时间": 用户 2026-09-30 确认这是**期望行为**。
     *   曾经看到日志里"网关 ts 比平台 ts 快 82 秒"——那不是时钟偏差, 而是**手工测试
     *   的产物**: 平台侧的 ts 是人工提前抄好、过一会儿才发出来的, 所以它偏旧。
     *   网关的时间是正确的。(这条更正很重要: 不要把手工测试的时间误当成时钟问题。)
     * ========================================================================== */
    (void)unix_ts;                  /* 平台给的 ts 只用于日志对比, 不写时钟 */

    /* 强制下一次 refresh 真的去问模组(不然"没到陈旧阈值"就直接返回, 等于没校) */
    app_invalidate_epoch();
    app_refresh_epoch();

    if (!app_epoch_valid())
    {
        log_info("[CLOUD] sys.time_sync: 向 4G 模组要时间【没要到】(模组不可用?) -> 1007\r\n");
        return false;
    }

    log_info("[CLOUD] sys.time_sync: 已从 4G 模组取时间 -> 本机 epoch=%lu (平台 ts=%lu)\r\n",
             (unsigned long)app_now_unix(), (unsigned long)unix_ts);
    return true;
}

/* ==================================================================
 * 断线补报: 把事件缓存里的一页补发出去 (v70)
 * ==================================================================
 * 只在 4G/MQTT 在线时做。一页最多 16 条(见 cloud_proto.h), 数据段约 3 KB ——
 * **发布通路装得下**: g4_at.c 的 G4_PUB_DATA_MAX = 4352
 * (v28 踩过"发布缓冲 256 B 导致 cfg.query 永远发不出去", 所以这次先查了容量)。
 *
 * ★ 先发后删 ★：发布【登记成功】才 drop；登记失败(离线/队列满)则一条都不动, 下次再来。
 *   与 data_cache 的 peek/drop 是同一条纪律 —— 一次失败不能把事件永久丢掉。
 * ★ 缓存里"渲染不出来"的条也必须出队 ★：否则它们永远堵在队头,
 *   后面的事件全被堵死(v67 的契约)。跳过多少条会打日志 —— 可以丢, 不许静默丢。
 * ★ 2026-10-22: 补报也要套 {event,msgId,ts,data} 信封 ★
 *   此前把 data 段 {"events":[...]} 直接发出去, 平台收到的报文缺 event/msgId/ts,
 *   与文档 7.5 及其它五类上行不一致。现在 data 先组在 buf 的预留区, 再 memmove
 *   到信封头之后 —— 不新增 RAM, 也不用第二个 4 KB 缓冲。
 */

/* 信封头 {"event":"event.buffered","msgId":"01234567","ts":1700000300,"data":
 * 实测 78 B(ts 10 位), 这里留到 96 B 冗余; 尾部再留 "}" + '\0' 两个字节。 */
#define BACKFILL_HDR_MAX   96u
#define BACKFILL_TAIL_MAX  2u

void app_backfill_poll(void)
{
    /* ★ 必须 static: 栈只有 4 KB, 一页正文约 3 KB 放栈上会直接把栈吃掉 ★ */
    static char buf[4096];
    static char topic[64];
    char       *data = buf + BACKFILL_HDR_MAX;   /* data 段先组在预留区, 之后再前移 */
    uint8_t     consumed;
    uint8_t     events;
    uint8_t     skipped;
    uint8_t     i;
    uint16_t    n;
    uint16_t    tlen;

    static uint32_t last_ms;

    if (!g4_at_is_online())
    {
        return;                     /* 断线期间不补报(补了也发不出去) */
    }
    /* 事件补报同样 1 页/秒 */
    if ((last_ms != 0u) && ((uint32_t)(get_tick_ms() - last_ms) < 1000u))
    {
        return;
    }
    last_ms = get_tick_ms();

    n = cloud_proto_build_backfill_data(data,
                                        (uint16_t)(sizeof(buf) - BACKFILL_HDR_MAX - BACKFILL_TAIL_MAX),
                                        (uint8_t)16u,
                                        &consumed, &events, &skipped);
    if (consumed == 0u)
    {
        last_ms = 0u;
        return;                     /* 缓存空, 或整页装不下(下次再来) */
    }
    if (skipped > 0u)
    {
        log_info("[CLOUD] 补报: 跳过 %u 条(渲染不出来), 本页装了 %u 条\r\n",
                 (unsigned)skipped, (unsigned)events);
    }
    if (n == 0u)
    {
        /* 全是渲染不出来的: 照样出队, 否则它们永远堵在队头 */
        for (i = 0u; i < consumed; i++)
        {
            event_cache_drop();
        }
        return;
    }

    /* ★ 套信封: {event,msgId,ts,data} ★
     *   外层 msgId 每次补报现生成(只标识"这一批"); 内层每条事件的 msgId/ts
     *   由 cloud_proto_build_backfill_data 保留原值, 平台按内层做幂等。 */
    {
        char hdr[BACKFILL_HDR_MAX];
        char msg_id[20];
        int  hlen;

        (void)utils_gen_msg_id(msg_id, (uint16_t)sizeof(msg_id));
        hlen = snprintf(hdr, sizeof(hdr),
                        "{\"event\":\"event.buffered\",\"msgId\":\"%s\",\"ts\":%lu,\"data\":",
                        msg_id, (unsigned long)app_now_unix());
        if ((hlen <= 0) || ((uint32_t)hlen >= (uint32_t)sizeof(hdr)))
        {
            log_info("[CLOUD] !! 补报信封组装失败, 这 %u 条留着下次\r\n",
                     (unsigned)consumed);
            return;                 /* 没发布 -> 一条都不 drop */
        }

        (void)memmove(buf, data, (size_t)n + 1u);   /* 含结尾 '\0' */
        (void)memcpy(buf, hdr, (size_t)hlen);
        buf[hlen + (int)n]     = '}';
        buf[hlen + (int)n + 1] = '\0';
        n = (uint16_t)(hlen + (int)n + 1);
    }

    tlen = cloud_profile_build_topic(CLOUD_TOPIC_EVENT, topic, (uint16_t)sizeof(topic));
    if (tlen == 0u)
    {
        log_info("[CLOUD] !! 补报 topic 拼装失败, 这 %u 条留着下次\r\n",
                 (unsigned)consumed);
        return;                     /* 一条都不动 */
    }

    log_info("[CLOUD] 补报一页 %u 条事件 (正文 %u 字节)\r\n", (unsigned)events, (unsigned)n);

    if (g4_at_mqtt_publish_ex(topic, (const uint8_t *)buf, n,
                              cloud_profile_topic_qos(CLOUD_TOPIC_EVENT),
                              cloud_profile_topic_retained(CLOUD_TOPIC_EVENT)) != SYS_OK)
    {
        log_info("[CLOUD] !! 补报登记失败, 这 %u 条留着下次再发\r\n", (unsigned)consumed);
        return;                     /* ★ 登记失败: 一条都不 drop ★ */
    }

    for (i = 0u; i < consumed; i++)
    {
        event_cache_drop();         /* ★ 登记成功才出队 ★ */
    }
}

#endif /* MAIN_STATUS_JSON_ENABLE */
