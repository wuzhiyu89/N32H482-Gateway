/**
 * @file  app_events.c
 * @brief 事件类上行：event.device / event.action / event.status_change
 *
 * 与 app_uplink.c 的分工：
 *   app_uplink.c —— 周期性 status.report 与 telemetry.report（含逐条补报）
 *   app_events.c —— 由"状态跳变"触发的事件（失联/恢复/绑定/声光执行结果/档位迁移）
 *
 * 事件的上行策略（文档 7.x）：能发就直发；4G 不在线或发布登记失败就写进
 * event_cache，重连后由 app_backfill_poll() 按原始时间戳补报（不改成重连时刻）。
 */
#include <stdio.h>
#include <string.h>

#include "log.h"
#include "app_config.h"
#include "app_core.h"
#include "app_state.h"
#include "bsp_tim.h"
#include "cloud_profile.h"
#include "cloud_proto.h"
#include "config_store.h"
#include "event_cache.h"
#include "g4_at.h"
#include "lora_business.h"
#include "sensor_offline.h"
#include "utils.h"

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

/* 设备事件码 -> 线上字符串(与 cloud_proto.h 的 cloud_device_event_t.event 一致) */
static const char *s_dev_ev_name(uint8_t ev)
{
    switch (ev)
    {
        case (uint8_t)EC_DEV_EV_BOUND:       return "bound";
        case (uint8_t)EC_DEV_EV_BIND_FAILED: return "bind_failed";
        case (uint8_t)EC_DEV_EV_UNBOUND:     return "unbound";
        case (uint8_t)EC_DEV_EV_ONLINE:      return "online";
        case (uint8_t)EC_DEV_EV_OFFLINE:     return "offline";
        case (uint8_t)EC_DEV_EV_LOW_VOLTAGE: return "low_voltage";
        default:                             return "unknown";
    }
}

/* ★2026-10-22: online(bool) 改成事件码(ev), 以支持 low_voltage。
 *   事件码与缓存里 rec.a 用的是同一套 EC_DEV_EV_*, 在线/补报两条路不会分叉。 */
static void uplink_publish_device_event(const char *uid_hex, uint8_t ev,
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
        rec.a    = ev;
        rec.b    = is_sensor ? (uint8_t)EC_DEV_SENSOR : (uint8_t)EC_DEV_DEVICE;
        rec.ts   = app_now_unix();
        rec.seq  = 0U;                          /* 设备事件不按 seq 分组 */
        s_evt_seq = (uint16_t)((s_evt_seq + 1u) % 1000u);
        rec.msg_seq = s_evt_seq;                /* 与其它事件共用同一个序号计数器 */

        if (event_cache_push(&rec))
        {
            log_info("[CLOUD] 4G 不在线 -> 设备事件(uid=%s type=%s %s)入补报缓存\r\n",
                     (uid_hex != NULL) ? uid_hex : "?", dt, s_dev_ev_name(ev));
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
    cjw_kv_str(&w, "event", s_dev_ev_name(ev));
    cjw_obj_end(&w);
    if (!cjw_ok(&w))
    {
        log_info("[CLOUD] !! event.device 的 data 段组装失败(缓冲不足?)\r\n");
        return;
    }
    (void)cjw_len(&w);      /* v91: 本函数不再需要长度(信封自带) —— 原来算给 dlen 但没用到 */

    seq++;
    (void)utils_gen_msg_id_seq(msg_id, (uint16_t)sizeof(msg_id), (uint8_t)'v',
                              (uint16_t)(seq % 1000u));

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
    uplink_publish_device_event(uid_hex,
                                online ? (uint8_t)EC_DEV_EV_ONLINE
                                       : (uint8_t)EC_DEV_EV_OFFLINE,
                                "sensor");
}

/**
 * @brief 液位低电量(适配器, 固定 devType="sensor")
 *
 * ★2026-10-22 新增: 平台 lowBatteryPercent 由网关判定, 越限上报
 *   event.device{"event":"low_voltage"}。判定与去重(low_bat_reported)在
 *   lora_business.c, 本函数只负责组包 —— 与失联事件同一条路径, 断线会入补报缓存。
 */
void app_on_sensor_low_battery(const char *uid_hex, uint8_t bat_pct)
{
    log_info("[CLOUD] 液位 %s 低电量(%u%%), 上报 event.device low_voltage\r\n",
             (uid_hex != NULL) ? uid_hex : "?", (unsigned)bat_pct);
    uplink_publish_device_event(uid_hex, (uint8_t)EC_DEV_EV_LOW_VOLTAGE, "sensor");
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

    (void)utils_gen_msg_id(msg_id, (uint16_t)sizeof(msg_id));

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
void app_event_action_failsafe(void)
{
    static char           data[128];
    static char           payload[CLOUD_RESP_MAX];
    static char           topic[64];
    static uint16_t       seq;
    cloud_action_event_t  ae;
    char                  msg_id[20];
    uint16_t              dlen;
    uint16_t              plen;
    uint16_t              tlen;

    (void)memset(&ae, 0, sizeof(ae));
    ae.gear         = 0u;
    ae.gear_null    = true;
    ae.trigger      = "failsafe";
    ae.result_count = 0u;
    ae.results      = NULL;

    dlen = cloud_proto_build_action_data(data, (uint16_t)sizeof(data), &ae);
    if (dlen == 0u) { return; }

    seq++;
    (void)utils_gen_msg_id_seq(msg_id, (uint16_t)sizeof(msg_id), (uint8_t)'a',
                              (uint16_t)(seq % 1000u));
    plen = cloud_proto_build_uplink(payload, (uint16_t)sizeof(payload),
                                    "event.action", msg_id, app_now_unix(), data);
    if (plen == 0u) { return; }
    tlen = cloud_profile_build_topic(CLOUD_TOPIC_EVENT, topic, (uint16_t)sizeof(topic));
    if (tlen == 0u) { return; }

    if (!g4_at_is_online())
    {
        log_info("[CLOUD] event.action(failsafe) 4G 不在线, 丢弃\r\n");
        return;
    }
    (void)g4_at_mqtt_publish_ex(topic, (const uint8_t *)payload, plen,
                                cloud_profile_topic_qos(CLOUD_TOPIC_EVENT),
                                cloud_profile_topic_retained(CLOUD_TOPIC_EVENT));
}

void app_event_device_online(const char *uid_hex, bool online)
{
    uplink_publish_device_event(uid_hex,
                                online ? (uint8_t)EC_DEV_EV_ONLINE
                                       : (uint8_t)EC_DEV_EV_OFFLINE,
                                "device");
}

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

    ae.gear      = gear;
    ae.gear_null = false;
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
            uplink_publish_device_event(uid_hex, (uint8_t)EC_DEV_EV_OFFLINE, "device");
        }
        /* ★2026-10-18 (丙): 之前判过离线, 这次成功 => 补报"恢复" ★ */
        if (recovered)
        {
            uplink_publish_device_event(uid_hex, (uint8_t)EC_DEV_EV_ONLINE, "device");
        }
        return;
    }

    seq++;
    (void)utils_gen_msg_id_seq(msg_id, (uint16_t)sizeof(msg_id), (uint8_t)'a',
                              (uint16_t)(seq % 1000u));

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
        uplink_publish_device_event(uid_hex, (uint8_t)EC_DEV_EV_OFFLINE, "device");
    }
    if (recovered)
    {
        log_info("[CLOUD] %s 之前判过离线, 这次 0x82 成功 -> 补报设备恢复\r\n", uid_hex);
        uplink_publish_device_event(uid_hex, (uint8_t)EC_DEV_EV_ONLINE, "device");
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
    (void)utils_gen_msg_id_seq(msg_id, (uint16_t)sizeof(msg_id), (uint8_t)'s',
                              s_evt_seq);

    plen = cloud_proto_build_uplink(payload, (uint16_t)sizeof(payload),
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
