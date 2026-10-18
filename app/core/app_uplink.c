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
    (void)snprintf(msg_id, sizeof(msg_id), "h%03u", (unsigned)(seq % 1000u));

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
    (void)snprintf(msg_id, sizeof(msg_id), "m%03u", (unsigned)(s_tel_seq % 1000u));

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

/* ==================================================================
 * 液位传感器【全部失联】时的 fail-safe 动作(文档第 5 章, 用户第 7 条)
 * ==================================================================
 * 【什么时候进来】level_supervisor 判出"曾经有水位、现在一台在线的都没有",
 *   并且在【刚跌进去的那一拍】给 failsafe_enter=true(边沿, 只做一次)。
 * 【做什么】按 rules.sensorOfflinePolicy:
 *     keep  -> 保持当前档位(什么都不发, 只记日志)
 *     alarm -> 强制档位 3(告警)
 *     off   -> 执行档位 0(解除)
 * 【文本用什么水位渲染】全失联时没有"当前水位", 用【最后一次已知水位】——
 *   这是唯一合理的选择; 档位 0 的模板通常是空文本, 所以 off 基本只会关屏/关声。
 * 【为什么要 mark_sent】强制改档之后去重基准得跟着走 —— 否则传感器恢复回来时会
 *   拿"上一次自动下发的档位"去比, 该发的不发或白重发一条。
 * ================================================================== */
void app_failsafe_step(const level_decision_t *ld)
{
    const config_store_t *cfg = config_store_get();
    uint8_t policy;
    char    led1[CFG_LEDTEXT_MAX + 1u];
    char    led2[CFG_LEDTEXT_MAX + 1u];
    char    tts[CFG_TTSTEXT_MAX + 1u];
    int32_t lv = 0;
    uint8_t g  = 0u;

    if ((cfg == NULL) || (ld == NULL) || (!ld->failsafe_enter)) { return; }

    policy = cfg->rules.sensor_offline_policy;

    log_info("[LEVEL] !! 液位传感器【全部失联】-> fail-safe 策略=%s, 应执行档位 %u\r\n",
             (policy == (uint8_t)CFG_OFFLINE_ALARM) ? "alarm" :
             (policy == (uint8_t)CFG_OFFLINE_OFF)   ? "off"   : "keep",
             (unsigned)ld->failsafe_gear);

    if (policy == (uint8_t)CFG_OFFLINE_KEEP)
    {
        return;                     /* 保持当前档位 = 什么都不做 */
    }

    led1[0] = '\0';
    led2[0] = '\0';
    tts[0]  = '\0';
    if (level_supervisor_last(&lv, &g) && (ld->failsafe_gear < (uint8_t)CFG_GEAR_COUNT))
    {
        (void)level_render_template(led1, (uint16_t)sizeof(led1),
                                    cfg->gears[ld->failsafe_gear].led_text1, lv);
        (void)level_render_template(led2, (uint16_t)sizeof(led2),
                                    cfg->gears[ld->failsafe_gear].led_text2, lv);
        (void)level_render_template(tts, (uint16_t)sizeof(tts),
                                    cfg->gears[ld->failsafe_gear].tts_text, lv);
    }

    if (lora_business_apply_gear_ex(ld->failsafe_gear, NULL, led1, led2, tts,
                                    NULL, 0u, NULL) > 0)
    {
        level_supervisor_mark_sent(ld->failsafe_gear, led1, led2);   /* 见上方说明 */
    }
}

/* ==================================================================
 * manual 模式窗口(用户第 6 条)
 * ==================================================================
 * 【分工】"manual 期间抑制自动联动"在 level_supervisor.c 里(它直接读 rules.mode);
 *   这里只管【回归】: manualTimeoutSeconds 到期把 mode 写回 auto, 并强制重评估一次
 *   (文档 5.5: "切回 auto 时网关立即按当前水位重评估一次并刷新联动状态")。
 * 【计时基准】进入 manual 的时刻; 若 manual 期间【配置版本变了】(又下发了一次
 *   dev.ctrl / rule.mode), 窗口重新计时 —— 这符合"每次手动控制后重新开始计时"的
 *   直觉, 代价是任何别的配置改动也会顺带刷新窗口(已在此注明)。
 * 【0 = 不回归】(文档 3.7: manualTimeoutSeconds 0 = 不回归)。
 * 【为什么放在这里而不是 cloud_proto】回归是"时间"触发的, 主循环才是时间的主人;
 *   cloud_proto 只负责"用户下命令"这一侧。
 * ================================================================== */
static uint32_t s_man_start_ms;      /* 本次 manual 的起点 */
static uint32_t s_man_cfg_ver;       /* 起点时的配置版本(变了就重新计时) */
static bool     s_man_was;           /* 上一拍是不是 manual(用于检测"回到 auto") */

void app_manual_step(void)
{
    const config_store_t *cfg = config_store_get();
    uint32_t now = get_tick_ms();
    uint32_t ver;

    if (cfg == NULL) { return; }
    ver = config_store_version();

    /* ---- 不在 manual: 若刚从 manual 回来, 强制重评估一次 ---- */
    if (cfg->rules.mode != (uint8_t)CFG_RULE_MANUAL)
    {
        if (s_man_was)
        {
            s_man_was = false;
            log_info("[CTRL] 已回到 auto 模式(自动联动恢复), 强制重评估一次\r\n");
            level_supervisor_force_next();
        }
        return;
    }

    /* ---- 刚进 manual, 或 manual 期间配置又变过 -> (重新)开始计时 ---- */
    if ((!s_man_was) || (ver != s_man_cfg_ver))
    {
        s_man_was      = true;
        s_man_start_ms = now;
        s_man_cfg_ver  = ver;
        log_info("[CTRL] 进入 manual 模式: 自动联动已抑制, manualTimeoutSeconds=%us%s\r\n",
                 (unsigned)cfg->rules.manual_timeout_s,
                 (cfg->rules.manual_timeout_s == 0u) ? "(0=不回归)" : "");
        return;
    }

    if (cfg->rules.manual_timeout_s == 0u) { return; }      /* 不回归 */

    if ((uint32_t)(now - s_man_start_ms)
        >= ((uint32_t)cfg->rules.manual_timeout_s * 1000U))
    {
        config_store_t tmp;

        (void)memcpy(&tmp, cfg, sizeof(tmp));
        tmp.rules.mode = (uint8_t)CFG_RULE_AUTO;

        if (config_store_save(&tmp) == CFG_OK)
        {
            /* ★★2026-10-14 修复: 必须把 tmp 提交给"当前配置"实例 ★★
             * ------------------------------------------------------------------
             * 【漏了什么】config_store_save() 只负责【落 Flash】, 它【不更新】
             *   RAM 里那份"当前配置"。调用方必须自己 memcpy 到
             *   config_store_get_mut() —— 对比 cloud_proto.c 的 sys.factory_reset:
             *   那处 save 之后有一句 "落盘成功才提交给当前配置实例" 的 memcpy, 这里漏了。
             *   那处有, 这里漏了。
             *
             * 【后果(2026-10-14 用户实测, 三条症状同时出现)】
             *   ① 云端 status.report 的 mode 恒为 manual —— 因为它读的是 RAM;
             *   ② manual 抑制一直生效 -> 液位变化不再联动声光设备;
             *   ③ 本次 save 抬了配置版本 -> 下一拍 line 682 的
             *      `ver != s_man_cfg_ver` 成立 -> 走"刚进 manual, 重新计时"分支
             *      -> 每 manual_timeout_s 重复一次, 永远回不到 auto。
             *   ⇒ 一个遗漏解释全部三个现象。
             *
             * 【提交之后】下一拍 line 670 会走"已回到 auto"那支:
             *   打 "[CTRL] 已回到 auto 模式(自动联动恢复), 强制重评估一次"
             *   并调 level_supervisor_force_next() -> 档位按当前水位重算下发。 */
            (void)memcpy(config_store_get_mut(), &tmp, sizeof(tmp));

            log_info("[CTRL] manual 超时(%us) -> 自动回归 auto\r\n",
                     (unsigned)cfg->rules.manual_timeout_s);
            /* 不在这里置 s_man_was=false: 下一拍会发现 mode 已经不是 manual,
             * 走上面那支"回到 auto"并强制重评估 —— 一处收口, 不会漏。 */
        }
        else
        {
            log_info("[CTRL] !! manual 超时, 但写回 auto 失败(Flash), 下一拍重试\r\n");
        }
    }
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

    if (!g4_at_is_online())
    {
        return;                     /* 还断着: 不补 */
    }

    rec = telemetry_cache_peek();
    if (rec == NULL)
    {
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

    (void)snprintf(msg_id, sizeof(msg_id), "m%03u", (unsigned)rec->msg_seq);

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

/**
 * @brief 组一条 event.device(文档 7.4) 发到 event topic —— 液位失联/恢复
 *
 * 文档 7.4 原文:
 *   { "event": "event.device", "msgId": "v001", "ts": 1735689600,
 *     "data": { "addr": 1, "devType": "sensor", "event": "offline" } }
 *   event 取值: bound / bind_failed / unbound / online / offline / low_voltage。
 *
 * 【字段口径】: 本工程子设备寻址是 13 字节 UID 的 26 位 hex 串(偏差 D-8), 文档那个
 * uint16 的 "addr" 装不下, 所以 data 里用 "uid":"210001"; "devType" 与 "event"
 * 两个字段名与取值【严格照文档】。
 *
 * 【这个函数是被 lora_business 回调进来的】: 判定与去重都在
 * lora_business.c + sensor_offline.c 里, 这里只管"组包 + 登记发布"。
 *
 * 【弱网】: 发布登记失败只打日志 —— g4_at_mqtt_publish_ex() 本身是非阻塞登记队列,
 * 而且事件不像配置应答那样必须让平台知道, 绝不能因为它去阻塞主循环。
 *
 * ★★2026-10-18 (丙) 三处改动★★
 *   ① devType 参数化: 以前写死 "sensor", 于是【声光离线】也发成 devType="sensor",
 *      平台会误以为是液位传感器掉了。现在: 液位 -> "sensor", 声光 -> "device"。
 *   ② 4G 不在线时【入事件缓存】(EC_KIND_DEVICE) 等重连补报 —— 原来只有
 *      event.action 会入缓存, event.device 在弱网期间直接丢; 而"声光离线"
 *      恰恰常发生在弱网/掉线时, 丢了平台就永远不知道。
 *   ③ 配合 lora_business 的 alarm_offline_reported: 声光恢复时补报
 *      event.device{"online"}(见 app_on_alarm_result 的 recovered 分支)。
 */
/* ★ 第 12 条(2026-09-30): 事件 msgId 的【唯一序号】—— 在线发送与"断线入缓存"
 * 两条路【共用同一个计数器】。
 * 【为什么必须共用】以前两边各有一个 static seq: 同一个事件在线时叫 `s001`,
 * 断线入缓存、之后补报时被重新分配成 `b001` —— 平台按 msgId 做幂等/去重时对不上。
 * ★2026-10-18: 定义上移到这里(uplink_publish_device_event 也要用它)。 */
static uint16_t s_evt_seq;

static void uplink_publish_device_event(const char *uid_hex, bool online,
                                      const char *dev_type)
{
    /* 一律 static: 栈只有 4KB(见 startup_n32h47x_48x.s 的 Stack_Size) */
    static char     data[96];
    static char     payload[CLOUD_RESP_MAX];
    static char     topic[64];
    static uint16_t seq;
    char            msg_id[20];
    cjw_t           w;
    uint16_t        plen;
    uint16_t        tlen;
    const char     *dt = ((dev_type != NULL) && (dev_type[0] != '\0'))
                             ? dev_type : "sensor";
    bool            is_sensor = (strcmp(dt, "sensor") == 0);

    /* ---- 4G 不在线: 写进事件缓存, 重连后由补报队列发出去 ---- */
    if (!g4_at_is_online())
    {
        ec_record_t rec;
        uint8_t     uid_b[CFG_UID_BYTES];

        if (!cfg_uid_to_bytes(uid_hex, uid_b))
        {
            log_info("[CLOUD] !! %s 不是合法 uid, 设备事件无法入缓存\r\n",
                     (uid_hex != NULL) ? uid_hex : "(null)");
            return;
        }

        (void)memset(&rec, 0, sizeof(rec));
        rec.kind = (uint8_t)EC_KIND_DEVICE;
        (void)memcpy(rec.uid, uid_b, CFG_UID_BYTES);
        rec.a    = online ? (uint8_t)EC_DEV_EV_ONLINE : (uint8_t)EC_DEV_EV_OFFLINE;
        rec.b    = is_sensor ? (uint8_t)EC_DEV_SENSOR : (uint8_t)EC_DEV_DEVICE;
        rec.ts   = app_now_unix();
        rec.seq  = 0U;                          /* 设备事件不按 seq 分组 */
        s_evt_seq = (uint16_t)((s_evt_seq + 1u) % 1000u);
        rec.msg_seq = s_evt_seq;                /* 与其它事件共用同一个序号计数器 */

        if (event_cache_push(&rec))
        {
            log_info("[CLOUD] 4G 不在线 -> 设备事件(uid=%s type=%s %s)入补报缓存\r\n",
                     (uid_hex != NULL) ? uid_hex : "?", dt,
                     online ? "online" : "offline");
        }
        else
        {
            log_info("[CLOUD] !! 设备事件(event_cache_push 拒绝) -> 这条丢了\r\n");
        }
        return;
    }

    cjw_init(&w, data, (uint16_t)sizeof(data));
    cjw_obj_begin(&w);
    cjw_kv_str(&w, "uid", (uid_hex != NULL) ? uid_hex : "");
    cjw_kv_str(&w, "devType", dt);
    cjw_kv_str(&w, "event", online ? "online" : "offline");
    cjw_obj_end(&w);
    if (!cjw_ok(&w))
    {
        log_info("[CLOUD] !! event.device 的 data 段组装失败(缓冲不足?)\r\n");
        return;
    }
    (void)cjw_len(&w);      /* v91: 本函数不再需要长度(信封自带) —— 原来算给 dlen 但没用到 */

    seq++;
    (void)snprintf(msg_id, sizeof(msg_id), "v%03u", (unsigned)(seq % 1000u));

    plen = cloud_proto_build_uplink(payload, (uint16_t)sizeof(payload),
                                    "event.device", msg_id, app_now_unix(), data);
    if (plen == 0u)
    {
        log_info("[CLOUD] !! event.device 信封组装失败\r\n");
        return;
    }

    tlen = cloud_profile_build_topic(CLOUD_TOPIC_EVENT, topic, (uint16_t)sizeof(topic));
    if (tlen == 0u)
    {
        log_info("[CLOUD] !! event topic 拼装失败, 事件丢弃\r\n");
        return;
    }

    /* 发不出去也要把正文打出来: 弱网下串口是唯一能确认"我们准备发什么"的地方
     * (与 app_publish_status_json / app_downlink_try 同一套做法) */
    log_info("[CLOUD] 上报 %s (%u 字节): %s\r\n", topic, (unsigned)plen, payload);

    if (g4_at_mqtt_publish_ex(topic, (const uint8_t *)payload, plen,
                              cloud_profile_topic_qos(CLOUD_TOPIC_EVENT),
                              cloud_profile_topic_retained(CLOUD_TOPIC_EVENT)) != SYS_OK)
    {
        log_info("[CLOUD] !! 事件发布登记失败(离线/队列满), 这条事件只记在本机\r\n");
    }
}

/**
 * @brief 液位失联/恢复的【适配器】—— 固定 devType = "sensor"
 *
 * ★2026-10-18 (丙): uplink_publish_device_event 增加了 devType 参数, 而
 * lora_business 的传感器回调签名只有 (uid, online) ⇒ 用这一层适配。
 * 声光那边直接调 uplink_publish_device_event(..., "device"), 不走这里。
 */
void app_on_sensor_event(const char *uid_hex, bool online)
{
    uplink_publish_device_event(uid_hex, online, "sensor");
}

/**
 * @brief 出厂字段操作的结果(写成功 / 读成功 / 失败) —— 只打串口, 【不上云】
 *
 * ★2026-10-18: 字段是产线/开发者的内部信息(用户明确"不要让云端知道"),
 *   所以这里既不组 JSON、也不进 event_cache, 只落一行日志。
 */
void app_on_field_result(bool ok, const char *field10)
{
    if (!ok)
    {
        log_info("[FIELD] ==> 结果: 失败"
                 "(写: 3 条都没收到 0x80 / 读: 12s 内没收到 0x08)\r\n");
        return;
    }

    if ((field10 != NULL) && (field10[0] != 0x00))
    {
        log_info("[FIELD] ==> 结果: 读取成功, 设备字段 = \"%s\"\r\n", field10);
    }
    else
    {
        log_info("[FIELD] ==> 结果: 写入成功(设备已回 0x80)\r\n");
    }
}

/* ==================================================================
 * event.device: bound / bind_failed / unbound(P5, 设备绑定/解绑)
 * ==================================================================
 * 与上面的 uplink_publish_device_event(online/offline)【同 topic 同 data 结构】,
 * 只是 event 字段不同。由 lora_business_set_bind_event_cb 注册给业务层,
 * 在 0x04 绑定成功/失败、0x05 解绑完成时各调一次。
 *
 * 异步: 0x04/0x05 完成后才触发本回调, 那时云端命令响应(sen.bind/dev.unbind 的
 * 应答 code 0)早就发回去了 —— 云端靠这条事件确认"真绑/真删"完成。
 *
 * 与 uplink_publish_device_event 一样, 静态缓冲 + 发布失败只记本机不阻塞。
 * ★ 不进 event_cache(断线补报队列): 绑定/解绑是即时事件, 平台收不到时会再下发
 *   命令重试, 不需要补报(补报反而可能让平台收到"重复 bound"造成误判)。
 *
 * 【2026-09-29 补回】本函数与下面那两行注册在 14:06 的一次改动里被误删,
 *   导致 sen.bind/dev.bind 恒回 1006(pair_hook 为空)、解绑完成事件也发不出去。
 *   从 v105 归档原样取回。 */
void app_publish_bind_event(const char *uid_hex, bool is_sensor,
                                     lora_bind_event_t event)
{
    static char     data[96];
    static char     payload[CLOUD_RESP_MAX];
    static char     topic[64];
    static uint16_t seq;
    char            msg_id[20];
    cjw_t           w;
    uint16_t        plen;
    uint16_t        tlen;
    const char     *ev_str;

    ev_str =
        (event == LORA_BIND_EVENT_BOUND)          ? "bound"          :
        (event == LORA_BIND_EVENT_BIND_FAILED)    ? "bind_failed"    :
        (event == LORA_BIND_EVENT_UNBOUND)        ? "unbound"        :
        (event == LORA_BIND_EVENT_UNBIND_FAILED)  ? "unbind_failed"  :
        (event == LORA_BIND_EVENT_UNBIND_FAULT)   ? "unbind_fault"   : "unknown";

    cjw_init(&w, data, (uint16_t)sizeof(data));
    cjw_obj_begin(&w);
    cjw_kv_str(&w, "uid",     (uid_hex != NULL) ? uid_hex : "");
    cjw_kv_str(&w, "devType", is_sensor ? "sensor" : "device");
    cjw_kv_str(&w, "event",   ev_str);
    cjw_obj_end(&w);
    if (!cjw_ok(&w))
    {
        log_info("[CLOUD] !! event.device(bind) data 段组装失败\r\n");
        return;
    }

    seq++;
    (void)snprintf(msg_id, sizeof(msg_id), "b%03u", (unsigned)(seq % 1000u));

    plen = cloud_proto_build_uplink(payload, (uint16_t)sizeof(payload),
                                    "event.device", msg_id, app_now_unix(), data);
    if (plen == 0u)
    {
        log_info("[CLOUD] !! event.device(bind) 信封组装失败\r\n");
        return;
    }

    tlen = cloud_profile_build_topic(CLOUD_TOPIC_EVENT, topic, (uint16_t)sizeof(topic));
    if (tlen == 0u)
    {
        log_info("[CLOUD] !! event topic 拼装失败, 绑定事件丢弃\r\n");
        return;
    }

    log_info("[CLOUD] 上报 %s (%u 字节): %s\r\n", topic, (unsigned)plen, payload);

    if (g4_at_mqtt_publish_ex(topic, (const uint8_t *)payload, plen,
                              cloud_profile_topic_qos(CLOUD_TOPIC_EVENT),
                              cloud_profile_topic_retained(CLOUD_TOPIC_EVENT)) != SYS_OK)
    {
        log_info("[CLOUD] !! 绑定事件发布登记失败, 这条事件只记在本机\r\n");
    }
}

/* ★ 第 12 条(2026-09-30): 事件 msgId 的【唯一序号】—— 在线发送与"断线入缓存"
 * 两条路【共用同一个计数器】。
 * 【为什么必须共用】以前两边各有一个 static seq: 同一个事件在线时叫 `s001`,
 * 断线入缓存、之后补报时被重新分配成 `b001` —— 平台按 msgId 做幂等/去重时对不上
 * (该去重的去不掉, 或者把同一条当成两条)。
 * 现在: 入缓存时就把号码记进记录(ec_record_t.msg_seq, 2 B/条), 补报时原样还原。
 * ★2026-10-08: 声明【上移到本文件靠前处】★ —— 声光动作结果(app_on_alarm_result)
 *   也要用它, 那个函数在 status_change 之前。原来它定义在下面, 编译报未声明。 */
/* ★2026-10-18: s_evt_seq 的定义已上移到 uplink_publish_device_event 之前
 *   —— 那个函数(设备在线/离线事件)现在也要入补报缓存, 需要 `msg_seq`。 */

/* ==================================================================
 * event.action：声光联动【执行结果】上报（2026-10-08 用户 A1/A3）
 * ==================================================================
 * 由 lora_business 的声光(0x03)状态机回调 —— 设备对 0x03 会先后回
 * 0x80(收到了) → 0x82(成功) / 0x83(失败), 语义见 lora_proto.h。
 *
 * 【三种回调情形, 只有两种要发 MQTT】
 *   ok=true              -> 收到 0x82: 发 event.action{code 0}
 *   ok=false, gave_up=假 -> 中间失败(还会重发): 【什么都不发】——
 *                           否则平台会看到三次"失败", 而其实第三次成功了
 *   ok=false, gave_up=真 -> 三条都没成功: 发 event.action{失败}
 *                           【并且额外】发一条 event.device{"offline"}
 *                           (用户口径: "三次都没响应需要上报声光报警器离线")
 *
 * 【trigger 怎么来的】声光状态机不知道"谁触发的"(那是 level_supervisor 的事),
 *   这里按当前 rules.mode 反推: manual -> "manual", 其余 -> "auto"。
 *   够用且不撒谎 —— failsafe 触发的动作在 manual 模式下本来也不会发。
 * ================================================================== */
void app_on_alarm_result(const char *uid_hex, uint8_t gear,
                                 bool ok, bool gave_up, bool recovered)
{
    static char            data[192];
    static char            payload[CLOUD_RESP_MAX];
    static char            topic[64];
    static uint16_t        seq;
    char                   msg_id[20];
    cloud_action_result_t  res;
    cloud_action_event_t   ae;
    const config_store_t  *cfg = config_store_get();
    uint16_t               plen;
    uint16_t               tlen;

    if (!ok && !gave_up)
    {
        return;                     /* 中间失败: 不打扰平台 */
    }
    if ((uid_hex == NULL) || (uid_hex[0] == '\0'))
    {
        return;
    }

    res.uid  = uid_hex;
    /* 0 = 成功; 非 0 = 失败。0x83 的 data 为空, 拿不到更细的原因,
     * 所以统一用 3001 表示"设备未能完成动作 / 无响应"。 */
    res.code = ok ? 0 : 3001;

    ae.gear    = gear;
    ae.trigger = ((cfg != NULL) && (cfg->rules.mode == (uint8_t)CFG_RULE_MANUAL))
                     ? "manual" : "auto";
    ae.result_count = 1u;
    ae.results      = &res;

    if (cloud_proto_build_action_data(data, (uint16_t)sizeof(data), &ae) == 0u)
    {
        log_info("[CLOUD] !! event.action data 段组装失败\r\n");
        return;
    }

    /* ★★2026-10-08: 断线时【入缓存补报】, 在线时直发★★
     * 判据与 status_change 完全一致(只有一个: 4G 在线), 不引入第二套"能发吗"。
     * 用户 2026-10-08 选定"甲"方案: 每台设备单独一条记录(seq = 0), 结果码存 aux。
     * 渲染/补报那一侧已支持(见 cloud_proto.c 里 EC_KIND_ACTION 的两个分支)。
     *
     * ⚠️ 为什么联动结果值得补报: 平台靠它判断"灯到底亮没亮"。
     *   状态迁移事件丢了顶多晚一点知道, 这个丢了就永远不知道那次联动失败了。 */
    if (!g4_at_is_online())
    {
        ec_record_t rec;
        uint8_t     uid3[CFG_UID_BYTES];

        if (!cfg_uid_to_bytes(uid_hex, uid3))
        {
            log_info("[CLOUD] !! %s 不是合法 uid, 动作结果无法入缓存\r\n", uid_hex);
            return;
        }

        (void)memset(&rec, 0, sizeof(rec));
        rec.kind = (uint8_t)EC_KIND_ACTION;
        (void)memcpy(rec.uid, uid3, CFG_UID_BYTES);
        rec.a    = gear;                    /* 档位 */
        rec.b    = (uint8_t)(((cfg != NULL) &&
                              (cfg->rules.mode == (uint8_t)CFG_RULE_MANUAL))
                                 ? EC_TRIG_MANUAL : EC_TRIG_AUTO);
        rec.ts   = app_now_unix();
        rec.seq  = 0U;                      /* ★甲方案: 每台一条, 不分组★ */
        ec_action_set_code(&rec, res.code); /* ★结果码必须走访问器★ */
        s_evt_seq = (uint16_t)((s_evt_seq + 1u) % 1000u);
        rec.msg_seq = s_evt_seq;            /* 与在线路径共用同一个序号计数器 */
        (void)event_cache_push(&rec);
        log_info("[CLOUD] 4G 不在线 -> 动作结果(uid=%s gear=%u code=%d)入补报缓存\r\n",
                 uid_hex, (unsigned)gear, (int)res.code);

        /* 离线时"设备离线"这件事同样要报 —— 平台做离线告警靠它。
         * ★2026-10-18 (丙): devType="device"(声光), 且它自己内部也会入缓存。 */
        if (gave_up)
        {
            uplink_publish_device_event(uid_hex, false, "device");
        }
        /* ★2026-10-18 (丙): 之前判过离线, 这次成功 => 补报"恢复" ★ */
        if (recovered)
        {
            uplink_publish_device_event(uid_hex, true, "device");
        }
        return;
    }

    seq++;
    (void)snprintf(msg_id, sizeof(msg_id), "a%03u", (unsigned)(seq % 1000u));

    plen = cloud_proto_build_uplink(payload, (uint16_t)sizeof(payload),
                                    "event.action", msg_id, app_now_unix(), data);
    if (plen == 0u)
    {
        log_info("[CLOUD] !! event.action 信封组装失败\r\n");
        return;
    }

    tlen = cloud_profile_build_topic(CLOUD_TOPIC_EVENT, topic, (uint16_t)sizeof(topic));
    if (tlen == 0u)
    {
        log_info("[CLOUD] !! event topic 拼装失败, 动作结果丢弃\r\n");
        return;
    }

    log_info("[CLOUD] 上报 %s (%u 字节): %s\r\n", topic, (unsigned)plen, payload);

    if (g4_at_mqtt_publish_ex(topic, (const uint8_t *)payload, plen,
                              cloud_profile_topic_qos(CLOUD_TOPIC_EVENT),
                              cloud_profile_topic_retained(CLOUD_TOPIC_EVENT)) != SYS_OK)
    {
        log_info("[CLOUD] !! event.action 发布登记失败, 只记在本机\r\n");
    }

    /* ★重试用尽 ⇒ 额外报一条"设备离线"★
     * 两条都发是刻意的: event.action 给"这次动作的明细", event.device 给
     * "这台设备掉了"这个状态 —— 平台做离线告警只需要盯后者一种形状。
     * ★2026-10-18 (丙): devType 用 "device"(声光), 不再误报成 "sensor";
     *   并且恢复时由下面的 recovered 分支补报 event.device{"online"}。 */
    if (gave_up)
    {
        log_info("[CLOUD] !! %s 连续 3 条 0x03 都没成功 -> 补报设备离线\r\n", uid_hex);
        uplink_publish_device_event(uid_hex, false, "device");
    }
    if (recovered)
    {
        log_info("[CLOUD] %s 之前判过离线, 这次 0x82 成功 -> 补报设备恢复\r\n", uid_hex);
        uplink_publish_device_event(uid_hex, true, "device");
    }
}
/* ==================================================================
 * event.status_change：直发 / 入缓存 (v72)
 * ==================================================================
 * 状态迁移时:
 *   · 4G 在线 -> **直发** `event.status_change`；
 *   · 断线   -> 写进事件缓存, 重连后由 v70 的补报发出去。
 * ★ 两支【互斥】★ —— 都做会让平台对同一件事收到两次。
 *
 * 还有一个"其实发不出去"的第三种情况: 在线判据成立、但发布**登记失败**
 * (队列满/刚掉线)。这时**退回缓存** —— 否则这条迁移就凭空没了。
 * 判据仍只有一个(4G 在线), 只是失败后补救, 不引入第二套"能发吗"。
 */
/* ★ 第 12 条(2026-09-30): 事件 msgId 的【唯一序号】—— 在线发送与"断线入缓存"
 * 两条路【共用同一个计数器】。
 * 【为什么必须共用】以前两边各有一个 static seq: 同一个事件在线时叫 `s001`,
 * 断线入缓存、之后补报时被重新分配成 `b001` —— 平台按 msgId 做幂等/去重时对不上
 * (该去重的去不掉, 或者把同一条当成两条)。
 * 现在: 入缓存时就把号码记进记录(ec_record_t.msg_seq, 2 B/条), 补报时原样还原。 */
/* ★2026-10-08: 声明已上移到 app_on_alarm_result 之前(那里也要用)★ */

void app_cache_status_change(uint8_t from_code, uint8_t to_code,
                                     uint8_t gear, int32_t level_mm)
{
    ec_record_t rec;

    (void)gear;             /* 档位不入记录: 渲染时由"迁移后状态"推出(见 event_cache.h) */

    (void)memset(&rec, 0, sizeof(rec));
    rec.kind     = (uint8_t)EC_KIND_STATUS_CHANGE;
    rec.a        = to_code;                 /* to   */
    rec.b        = from_code;               /* from */
    rec.ts       = app_now_unix();         /* ★ 原始发生时刻 */
    rec.aux = level_mm;
    /* ★ 第 12 条: 现在就把"这条事件的 msgId 序号"定下来(与在线路径同一个计数器),
     * 补报时按 s<序号> 原样重建 —— 不再变成另一个 b<序号>。 */
    s_evt_seq = (uint16_t)((s_evt_seq + 1u) % 1000u);
    rec.msg_seq  = s_evt_seq;
    (void)event_cache_push(&rec);
}

void app_publish_status_change(uint8_t from_code, uint8_t to_code,
                                       uint8_t gear, int32_t level_mm)
{
    /* 一律 static: 栈只有 4 KB */
    static char           data[256];
    static char           payload[CLOUD_RESP_MAX];
    static char           topic[64];
    /* 序号不再用本函数自己的 static —— 与 app_cache_status_change() 共用
     * 文件级的 s_evt_seq(第 12 条: 在线与补报必须是同一个号)。 */
    cloud_status_change_t sc;
    char                  msg_id[20];
    uint16_t              dlen;
    uint16_t              plen;
    uint16_t              tlen;
    const char           *from = cloud_proto_state_name(from_code);
    const char           *to   = cloud_proto_state_name(to_code);

    if ((from == NULL) || (to == NULL))
    {
        /* 码值不认识: 宁可这条不发, 也不编一个状态名出来 */
        log_info("[CLOUD] !! 状态码未知(%u->%u), 不上报 status_change\r\n",
                 (unsigned)from_code, (unsigned)to_code);
        return;
    }

    sc.from     = from;
    sc.to       = to;
    sc.gear     = gear;
    sc.level_mm = level_mm;

    dlen = cloud_proto_build_status_change_data(data, (uint16_t)sizeof(data), &sc);
    if (dlen == 0u)
    {
        log_info("[CLOUD] !! status_change 的 data 段组装失败\r\n");
        return;
    }

    /* ★ 第 12 条: 与"断线入缓存"那条路共用 s_evt_seq, 同一个事件只有一个号 */
    s_evt_seq = (uint16_t)((s_evt_seq + 1u) % 1000u);
    (void)snprintf(msg_id, sizeof(msg_id), "s%03u", (unsigned)s_evt_seq);    plen = cloud_proto_build_uplink(payload, (uint16_t)sizeof(payload),
                                    "event.status_change", msg_id, app_now_unix(), data);
    if (plen == 0u)
    {
        log_info("[CLOUD] !! status_change 信封组装失败\r\n");
        return;
    }

    tlen = cloud_profile_build_topic(CLOUD_TOPIC_EVENT, topic, (uint16_t)sizeof(topic));
    if (tlen == 0u)
    {
        log_info("[CLOUD] !! event topic 拼装失败, status_change 转入缓存\r\n");
        app_cache_status_change(from_code, to_code, gear, level_mm);
        return;
    }

    log_info("[CLOUD] 上报 %s (%u 字节): %s\r\n", topic, (unsigned)plen, payload);

    if (g4_at_mqtt_publish_ex(topic, (const uint8_t *)payload, plen,
                              cloud_profile_topic_qos(CLOUD_TOPIC_EVENT),
                              cloud_profile_topic_retained(CLOUD_TOPIC_EVENT)) != SYS_OK)
    {
        /* 登记失败(刚掉线/队列满): ★ 退回缓存 ★, 否则这条迁移就凭空没了 */
        log_info("[CLOUD] !! status_change 发布登记失败, 转入缓存下次补报\r\n");
        app_cache_status_change(from_code, to_code, gear, level_mm);
    }
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
    static uint16_t s_ota_seq;
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

    /* 发送判据走 ota_http_progress_due() —— 纯函数, 主机侧测得到 */
    if (!ota_http_progress_due(&st, s_last_state, s_last_percent,
                               now, s_t_last, MAIN_OTA_PROGRESS_MS))
    {
        return;
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

    s_ota_seq = (uint16_t)((s_ota_seq + 1u) % 1000u);
    (void)snprintf(msg_id, sizeof(msg_id), "p%03u", (unsigned)s_ota_seq);

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
 */
void app_backfill_poll(void)
{
    /* ★ 必须 static: 栈只有 4 KB, 一页正文约 3 KB 放栈上会直接把栈吃掉 ★ */
    static char buf[4096];
    static char topic[64];
    uint8_t     consumed;
    uint8_t     events;
    uint8_t     skipped;
    uint8_t     i;
    uint16_t    n;
    uint16_t    tlen;

    if (!g4_at_is_online())
    {
        return;                     /* 断线期间不补报(补了也发不出去) */
    }

    n = cloud_proto_build_backfill_data(buf, (uint16_t)sizeof(buf), 16u,
                                        &consumed, &events, &skipped);
    if (consumed == 0u)
    {
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
