/**
 * @file  app_boot.c
 * @brief 正常固件的上电初始化序列（顺序即设计，见 docs/ARCHITECTURE.md 第 5 节）
 *
 * 为什么每一步都打 [BOOT] 日志：现场"上电就重启"时，最后一行就是死在哪一步的证据。
 * 顺序上有两条硬约束：
 *   1. 时基 -> 日志 -> 再动电源（否则重启过程一个字都留不下）；
 *   2. 4G 上电与 LoRa 上电之间要错开（两股浪涌叠加会把电源拉塌）。
 */
#include "n32h47x_48x.h"
#include <string.h>
#include <stdio.h>

#include "log.h"
#include "app_config.h"
#include "app_core.h"
#include "app_state.h"
#include "bsp_gpio.h"
#include "bsp_tim.h"
#include "cjson_port.h"
#include "cloud_profile.h"
#include "cloud_proto.h"
#include "config_store.h"
#include "g4_at.h"
#include "g4_urc.h"
#include "last_value.h"
#include "local_alarm.h"
#include "lora_business.h"
#include "ota_health.h"
#include "ota_http.h"
#include "param_save.h"
#include "serial_proto.h"

void app_boot_run(void)
{
    /* ---------- 0. 先把时基和日志起来 ----------
     * 【顺序改过】: 原来第一件事是给 4G 上电, 但那时候 log 还没起来 —— 结果
     * "上电把板子拉重启"这种问题一个字都留不下, 只能靠猜。
     * 现在改成: 时基 → 日志 → 【每一步都打标记】→ 再动电源。
     * 这样一旦重启, 最后打印的那一行就是"死在哪个动作上"。 */
    bsp_tim1_init_1ms();    /* 必须最先: delay_ms / get_tick_ms 都依赖 ATIM1 时基 */

    log_init();
    app_console_init();

    log_info("\r\n==== N32H482 4G+LoRa 网关 模式=%s  %s %s ====\r\n",
             MAIN_MODE_NAME, __DATE__, __TIME__);

    /* ---------- 0.5 装 cJSON 的静态池分配器(v33) ----------
     * 【必须在这里, 而且必须在这条 #else 分支里】
     *  · 信封层已经改用 cJSON, 而 cJSON 默认走裸 malloc/free —— 本工程堆只有
     *    512 字节, 不装 hooks 就是一跑就挂。装上之后全程无堆分配(见
     *    app/cjson/cjson_port.h);
     *  · 放在 #else 里: 隔离测试固件(MAIN_LORA_RAW_TEST=1)用不到 cJSON,
     *    放外面会让那份固件白背 cJSON 的代码 + 24 KB 池。
     *  · 位置要早于任何可能处理下行命令的地方 —— 所以紧跟日志之后。 */
    cjson_port_init();

    app_report_reset_cause();   /* 上一轮到底是谁把我复位的 */

    /* ---------- 1. 给 4G 模组上电 ----------
     * PA8 控的是模组【供电】(板上没引出 RST, "复位"=断一下电再上电)。
     * 极性已实测确认: ★PA8 低电平 = 上电★(bsp_gpio.h 的 G4_PWR_SW_ON_HIGH=0)。
     * 必须放在 lora_business_init() 之前 —— radio 初始化要几百毫秒,
     * 这段时间模组应该已经在跑了。
     * ⚠️ 若一给模组上电板子就重启(BOR), 用 MAIN_G4_POWER_AT_BOOT=0 隔离验证。 */
#if MAIN_G4_POWER_AT_BOOT
    log_info("[BOOT] 给 4G 模组上电: PA8 -> %s ...\r\n",
             G4_PowerSwitch_GetOnHigh() ? "高电平" : "低电平");
    G4_PowerSwitch_Init();
    log_info("[BOOT] 4G 已上电, PA8=%s; 等 300ms 让上电浪涌过去\r\n",
             G4_PowerSwitch_GetLevel() ? "H" : "L");
    delay_ms(300);
    log_info("[BOOT] 4G 上电后 300ms 仍然活着(没有欠压复位)\r\n");
#else
    /* 注意: 这里只是"不提前上电" —— 4G 最终由 g4_at_init() 上电(并做拔插/预热)。
     * 旧文案写成"不给 4G 上电", 现场看到 4G 明明在跑 AT, 会误以为日志和硬件对不上。 */
    log_info("[BOOT] 4G 暂不提前上电(MAIN_G4_POWER_AT_BOOT=0), 稍后由 g4_at_init() 上电\r\n");
    G4_PowerSwitch_InitNoPower();
#endif

    /* ---------- 2. 本地告警表 ---------- */
    local_alarm_init();

    /* ---------- 3. LoRa 协议层 + 网关业务层 ----------
     * lora_business_init() 内部依次完成:
     *   param_save_init()            读 Flash 参数(含 gw_uid)
     *   lora_proto_init(on_frame)    Radio.Init + 持续 RX + 注册帧回调
     *   data_cache_init()            上行/下发缓存清零
     *   g4_urc_set_recv_cb(...)      注册 MQTT 下行回调 lora_mqtt_recv_cb
     */
#if MAIN_LORA_ENABLE && MAIN_G4_POWER_AT_BOOT
    /* 【错开两股上电浪涌】: 4G 刚上电的这几百毫秒里再给 LoRa 上电, 两个模组的
     * 输入电容同时充电 + 各自内部 LDO 起动, 浪涌叠在一起最容易把电源拉塌 ——
     * 表现正是"MCU 反复 BOR 欠压复位、连 main 都跑不完"。
     * 隔开 500ms 再看: 如果这样就不重启了, 那就确认是供电能力/储能不足。 */
    log_info("[BOOT] 等 500ms(错开两个模组的上电浪涌)再给 LoRa 上电...\r\n");
    delay_ms(500);
#endif
    log_info("[BOOT] LoRa 初始化...\r\n");
#if MAIN_LORA_ENABLE
    lora_business_init();
    {
        device_param_t *p = param_get_ptr();
        log_info("[BOOT] LoRa OK, 网关 UID=%02X%02X%02X\r\n",
                 p->gw_uid[0], p->gw_uid[1], p->gw_uid[2]);

        /* 源地址全 0 的帧协议上非法, 对端会当坏帧丢掉(这个坑踩过: ACK 的
         * gw_uid 留了 00 00 00, 设备一律不回)。这里只大声报警, 不拦。 */
        if ((p->gw_uid[0] == 0U) && (p->gw_uid[1] == 0U) && (p->gw_uid[2] == 0U))
        {
            log_info("[BOOT] !! 警告: 网关 UID = 00 00 00, 外发帧源地址非法,\r\n");
            log_info("[BOOT] !!       对端会直接丢弃 -> 永远收不到应答。\r\n");
            log_info("[BOOT] !!       请检查 param_save(Flash) 里的 gw_uid。\r\n");
        }
    }

    /* P5 设备绑定/解绑(LoRa 0x04/0x05):
     *   - cloud_proto 的 pair_hook 接到 lora_business_open_bind_window, 让云端
     *     sen.bind/dev.bind 命令能打开绑定窗口;
     *   - 业务层在 0x04 绑定成功/失败、0x05 解绑完成时回调 app_publish_bind_event,
     *     由它组 event.device{bound/bind_failed/unbound} 发到 event topic。
     * 这两条注册都要在 lora_business_init() 之后(网关 UID 已读出)、
     * 在云端命令处理之前完成。
     * ⚠️【别删】这两行 + app_publish_bind_event(): 少任何一个, sen.bind/dev.bind
     *    都会恒回 1006(云端再也绑不上设备) —— 2026-09-29 误删过一次。 */
    cloud_proto_set_pair_hook(lora_business_open_bind_window);
    lora_business_set_bind_event_cb(app_publish_bind_event);
    /* ★2026-10-08 新增★ 声光(0x03)执行结果回调 -> event.action(+ 离线时补 event.device)
     * 声光设备对 0x03 回 0x80(收到了) -> 0x82(成功)/0x83(失败), 见 lora_proto.h。
     * ⚠️【别删】少了这一行, 声光状态机照样跑完 15 秒 x3 条的重试, 但结果
     *   一个字都发不出去 —— 平台永远不知道灯亮没亮。 */
    lora_business_set_alarm_result_cb(app_on_alarm_result);
    log_info("[BOOT] 绑定/解绑回调已挂载(pair_hook + bind_event -> event topic)\r\n");
    log_info("[BOOT] 声光执行结果回调已挂载(0x82/0x83 -> event.action)\r\n");

    /* 射频健康自检: BUSY(PB3) 一直高 = LoRa 模组不应答(没电/EN没拉/没焊/线断)。
     * 单独打一行, 免得现场要去数 "wait busy pin timeout" 才知道射频是坏的。 */
    if (lora_business_radio_stuck())
    {
        log_info("[BOOT] !! LoRa 模组不应答(BUSY 一直高): 射频功能不可用, 4G 不受影响\r\n");
    }
    else
    {
        log_info("[BOOT] LoRa 自检 OK (BUSY=%u)\r\n", (unsigned)lora_business_radio_busy());
    }
#else  /* !MAIN_LORA_ENABLE */
    log_info("[BOOT] LoRa 已关闭(MAIN_LORA_ENABLE=0): 只跑 4G, 没有射频收发\r\n");
#endif

    /* ---------- 4. 4G 链路: USART2 + AT/MQTT 状态机 ---------- */
#if MAIN_G4_ENABLE
    log_info("[BOOT] 4G AT 状态机启动...\r\n");
    g4_at_init();

    /* v79: 把配置域 network 的 broker/账号灌给 4G 层 —— 文档 3.2 要求"整域重启生效"。
     * 此前 g4_at 只认编译期常量(broker.emqx.io), 配置里那几个字段是【死配置】:
     * 用 cfg.set 改了也不生效。现在只在配置里 host 非空时才覆盖, 空则沿用编译期默认
     * —— 默认行为不变(出厂配置 host 为空时仍然连原来的 broker)。
     *
     * ★★2026-10-08 用户重新定的口径★★
     *   · clientID = 【固定 "110001"】(不再是 IMEI) —— 编译期默认, 配置可覆盖;
     *   · 用户名   = 【"tuguan"】(固定)     —— 编译期默认, 配置可覆盖;
     *   · 密码     = 【空】;
     *   · topic 里的 {gwId} = 【仍跟 system.gwId 走】(出厂 110001, 可改)。
     *   ⇒ 【三者已经解耦】: 主题的 gwId 可以改, 而 clientID/用户名保持固定值。
     *     (之前是"用户名 = 设备 ID = 主题 gwId"三者同一个值, 现在不是了。)
     *
     *   ⚠️ 唯一要警惕的: 固定 clientID + 同一个 broker = 多台网关互相顶下线。
     *      单台测试没问题; 上第二台时把 `network.clientId` 配成唯一值(或 "IMEI")。 */
    {
        const config_store_t *cfg = config_store_get();
        const char *dev_id = ((cfg != NULL) && (cfg->system.gw_id[0] != '\0'))
                                 ? cfg->system.gw_id : "11000000000000000000000000";   /* 2026-10-18: 兜底也 26 位 */

        /* (1) ★先把遗留档位迁移到文档档位★
         * Flash(param_save 页)里可能存着旧固件写下的 LOCAL 档(mod/cmd ...),
         * 而它【优先于】代码里的出厂默认, 且【重烧 App 带不走】——
         * 现场表现极具迷惑性: 心跳发到 mod/status 看得到, 但平台往
         * rt/gw/110001/cmd 发的指令网关一条都收不到(它订的是 mod/cmd)。
         * 这里一次性改成 DOC 档并落盘, 下次开机不再需要迁移。 */
        if (cloud_profile_migrate_to_doc())
        {
            log_info("[BOOT] ★检测到 Flash 里是遗留的 LOCAL 档(mod/xxx) —— 已迁移到"
                     " DOC 档(rt/gw/{gwId}/xxx)并落盘★\r\n");
            log_info("[BOOT]   原因: 平台用 rt/gw/{gwId}/cmd 等主题, 旧档位会导致"
                     "【心跳能发、指令收不到】\r\n");
        }

        /* (2) 设备 ID -> cloud_profile(决定 rt/gw/{gwId}/xxx 里的 {gwId}) */
        cloud_profile_set_gw_id(dev_id);

        /* (3) clientID: 默认用编译期的 "110001"; 配置里给了 network.clientId 就覆盖它。
         *     "IMEI" 是特殊值 -> 用模组 IMEI(多台网关时的正解), 见 g4_at_set_client_id。 */
        if ((cfg != NULL) && (cfg->network.client_id[0] != '\0'))
        {
            g4_at_set_client_id(cfg->network.client_id);
            log_info("[BOOT] clientID 来自配置 network.clientId = %s\r\n",
                     cfg->network.client_id);
        }

        /* (4) broker host/port/账号。用户名/密码规则:
         *     配置里给了 network.username / network.mqtt_password 就用配置的;
         *     没给就【保持编译期默认】(用户名 "tuguan"、密码空) ——
         *     因为 g4_at_set_broker() 的约定是"空串 = 不覆盖"。 */
        if ((cfg != NULL) && (cfg->network.host[0] != '\0'))
        {
            char portbuf[8];

            (void)snprintf(portbuf, sizeof(portbuf), "%u", (unsigned)cfg->network.port);
            g4_at_set_broker(cfg->network.host, portbuf,
                             cfg->network.username,      /* 空 = 保持 "tuguan" */
                             cfg->network.mqtt_password);/* 空 = 保持 "" */
            /* ★network.tls 接线(2026-10-04 第 3 步)★
             * 这个字段以前是【死配置】: config_store 解析/校验/序列化它, 但全工程
             * 没有任何消费者 —— 于是"配置里开了 TLS"实际上一点用都没有。
             * 【为什么放在这个 if 里面】: 与 broker 其余字段保持同一个语义 ——
             * "配置里给了 host = 整条用配置的一套(含 tls); 没给 = 整条用编译期默认"。
             * 不放进去的话, 出厂默认配置(network.host 为空、tls=false)会把
             * g4_at 里编译期的 TLS=1 给覆盖掉, 于是私有 broker 的 8883 就连不上了。 */
            g4_at_set_tls(cfg->network.tls ? 1u : 0u);
            log_info("[BOOT] broker 来自配置 network: %s:%s user=%s pass=%s tls=%s\r\n",
                     cfg->network.host, portbuf,
                     (cfg->network.username[0] != '\0') ? cfg->network.username : "(默认 tuguan)",
                     (cfg->network.mqtt_password[0] != '\0') ? "已设" : "空",
                     cfg->network.tls ? "开" : "关");
        }
        else
        {
            /* 用编译期默认: host/port 见 g4_at.c 的 G4_MQTT_*, user="tuguan", pass=""
             * —— 这里【不需要】再调 g4_at_set_broker(), 那些默认值本来就是生效值。 */
            log_info("[BOOT] broker 用编译期默认(配置 network.host 为空), "
                     "用户名用默认 tuguan\r\n");
        }

        /* 无论走哪条路径, 都把【真正生效】的值打出来 —— 连不上时第一眼就看这一行。
         * ★2026-10-08: pass 之前这里硬编码成 "空" —— 密码改成 tg123456! 之后
         * 那行日志就是错的(会让人以为没发密码)。改成按实际值打"已设/空",
         * 但仍然【不打印明文】。 */
        log_info("[BOOT] 生效 broker = %s:%s user=%s pass=%s clientID=%s\r\n",
                 g4_at_get_broker_host(), g4_at_get_broker_port(),
                 (g4_at_get_broker_user() != NULL) ? g4_at_get_broker_user() : "-",
                 (g4_at_get_broker_pass() != NULL) && (g4_at_get_broker_pass()[0] != '\0')
                     ? "已设" : "空",
                 (g4_at_get_client_id() != NULL) ? g4_at_get_client_id() : "-");
    }

    /* P1-3: 下行订阅 topic 按档位注入。
     * 本地档位算出来就是 "rt/gw/{gwId}/cmd"(与 g4_at 的默认值相同) -> 行为不变;
     * 文档档位是 "rt/gw/{gwId}/cmd"。 */
    {
        char subt[64];
        if (cloud_profile_build_topic(CLOUD_TOPIC_CMD, subt, (uint16_t)sizeof(subt)) > 0u)
        {
            g4_at_set_sub_topic(subt);
            log_info("[BOOT] 下行订阅 topic = %s (%s档位)\r\n",
                     subt, cloud_profile_is_doc() ? "文档" : "本地");
        }
    }

    /* MQTT 遗嘱(LWT): 正文 {"online":false}。
     * ★★2026-10-08: 遗嘱 topic 由 status 改成【专用主题 rt/gw/{gwId}/lwt】★★
     *
     * 【为什么改 —— 现场真的踩了】
     *   原来遗嘱发到 status, 于是【每次网关上电】平台都会先收到一条
     *   {"online":false}, 然后才是 status.report(online:true), 看起来像
     *   "网关刚掉线又上线"。根因是 MQTT 协议行为: clientID 固定时设备掉电后
     *   broker 侧旧会话要等 keepalive 超时才过期, 网关重连时 broker 踢掉旧会话,
     *   那一刻【旧会话的遗嘱才真正发出】—— 也就是"上电瞬间收到 offline"。
     *   而 status 是 retained 的, 这条遗嘱还会把平台上的状态短暂改成 offline。
     *
     * 【现在】lwt 独立一条主题、也 retained, 两者互不覆盖:
     *   · status 的 retained 只由本机主动上报维护;
     *   · 平台要【多订阅一个】rt/gw/{gwId}/lwt 才能拿到掉线通知
     *     (《云端对接偏差说明》§2.4 提过, 用户 2026-10-08 拍板采用)。
     *
     * 这里只"备好"(并把要用的值打进日志); 是否真的拼进 AT+MQTTCONN 由 g4_at.c 的
     * MAIN_MQTT_LWT_ENABLE 控制(默认 0, 语法待台架确认, 见 MQTT_LWT_台架测试.txt)。 */
    {
        char lwtt[64];
        if (cloud_profile_build_topic(CLOUD_TOPIC_LWT, lwtt, (uint16_t)sizeof(lwtt)) > 0u)
        {
            g4_at_set_lwt(lwtt, "{\"online\":false}");
        }
    }

    /* ---------- 断网判定的第二条腿: 保活停滞看门狗 ----------
     * 【为什么由 main.c 来设】: 这条判据只有在"有周期性上行"时才成立, 所以窗口
     * 必须跟着心跳周期走, 不能用固定值:
     *     心跳开着 -> 5 × 30s = 150s: 连着几次心跳都没发成功才触发 4G_RST 断电重启;
     *     心跳关着 -> 0(= 关闭这条判据): 否则没有周期上行时必然误判, 会一直断电。
     * (第一条腿是 g4_at 里的"连续 3 次发布失败", 见 G4_PUB_FAIL_MAX。)
     *
     * ★2026-09-22 3 × -> 5 × (依据现场日志 78.txt)★
     *   模组自己重启一次要 40~60 秒才回到 ONLINE, 而 90s 的窗口意味着
     *   "它还没回来, 我们就又判它死、再给它断一次电" —— 看门狗本身成了又一个
     *   "自己拆自己链路"的来源。窗口必须明显长于一个完整的重建周期。
     *   150s 仍然远快于"云端一直等不到数据", 不至于让人等太久。 */
#if MAIN_GW_ONLINE_ENABLE
    /* 保活停滞看门狗窗口 = 5 ×【实际配置的】心跳周期(以前用编译期宏, 配置改了就不匹配) */
    g4_at_set_link_watchdog(5U * app_cfg_period_ms(config_store_get()->network.status_interval_s,
                                                    MAIN_GW_ONLINE_MS));
#else
    g4_at_set_link_watchdog(0U);
#endif

    /* ---------- 用上【模组自己】的 MQTT 能力(2026-10-04 第 2 步) ----------
     * ① keepalive: 模组级保活时间。以前 network.keepAliveSeconds 是【死配置】
     *    (只有 config_store 读写, 没有任何消费者) —— 模组一直用它的默认 120s。
     *    现在真正下发; 手册下限 60s, 配置给 30~59 会被 g4_at 夹紧并在日志里说明。
     * ② reconnect min/max: 恢复动作的指数退避。这两个也一直是死配置, 于是
     *    "连续失败就断电"没有任何减速带 —— 现场 1.6 小时断电 27 次就是这么来的。
     * ③ 遗嘱 / MQTT心跳间隔 / 心跳回显 / 模组自带重连 都在 g4_at 的建链流程里
     *    统一下发(MQTTCONN 之前), 不在这里 —— 那几条每条建链都要重发一遍。 */
    g4_at_set_keepalive_s(config_store_get()->network.keep_alive_s);
    g4_at_set_reconnect(config_store_get()->network.reconnect_min_s,
                        config_store_get()->network.reconnect_max_s);

#if MAIN_LORA_ENABLE
    /* P1-3: 给 LoRa 的下行回调挂一个前置过滤器 —— 内容是 JSON 就当云协议命令处理,
     * 其余(二进制帧)照旧走 LoRa 透传。不注册时行为与本改动前完全一致。 */
    lora_business_set_dl_prefilter(app_downlink_try);

    /* 把 sys.query_status 要的运行时状态接上(v40): 协议层不 include 4G/LoRa 的模块,
     * 由这里提供一个回调 —— 这样协议层仍然能在 PC 上脱机单测。
     * ⚠ 不注册的话 sys.query_status 会明确回 1006(而不是拿一堆 0 冒充状态)。 */
    cloud_proto_set_status_provider(app_fill_status);

    /* sys.time_sync 的写时钟钩子(v91)。★ 不注册的话这条命令会明确回 1006 ★ ——
     * 此前正是如此（自 v49 实现该命令起一直没接上，见 v82 说明）。 */
    cloud_proto_set_time_sync_hook(app_time_sync);

    /* sen.query / dev.query 的 lastTs 需要"收到那一帧时的 Unix 秒"(v53)。
     * 用钩子注入 —— last_value 模块要能在 PC 上脱机单测, 不能直接调 app_now_unix()。
     * ⚠ 不注册的话 lastTs 在应答里【根本不出现】, 不拿 0 冒充 1970。 */
    last_value_set_now_hook(app_now_unix);
#if MAIN_STATUS_JSON_ENABLE
    /* 液位失联/恢复事件: lora_business 只负责"什么时候该报"(判定+跳变去重),
     * 真正的组 JSON + 发 event topic 在本文件的 uplink_publish_device_event()。
     * 这样 LoRa 业务层不用依赖云协议层, 主机测试也就不用为它造 4G 桩。 */
    lora_business_set_sensor_event_cb(app_on_sensor_event);   /* 2026-10-18: 经适配器固定 devType="sensor" */

    /* ★2026-10-18: 每次液位上报(0x02)都通知一次 —— 4G 离线时存进
     *   telemetry_cache, 重连后按原时间戳逐条补报(用户口径: 补的是"每次上报",
     *   不是 30s 聚合遥测)。在线时这个回调直接返回, 不影响原有遥测。 */
    lora_business_set_level_report_cb(app_on_level_report);

    /* *2026-10-18: 出厂字段(0x06/0x07)的结果回调 —— 只打串口, 不上云 */
    lora_business_set_field_result_cb(app_on_field_result);
    log_info("[BOOT] 液位失联事件回调已挂载(event.device -> event topic)\r\n");
#endif
    log_info("[BOOT] MQTT 下行回调 = lora_mqtt_recv_cb, 前置过滤器 = 云协议层(JSON)\r\n");
#else
    /* LoRa 关闭时也必须注册下行回调, 否则云端下发的消息会被静默丢掉,
     * 现场会误判成"模组收不到下发"(见上面 app_downlink_log_cb 的说明)。 */
    g4_at_set_recv_cb(app_downlink_log_cb);
    log_info("[BOOT] LoRa 已关闭 -> MQTT 下行回调 = 只打印(JSON 命令仍走云协议层)\r\n");
#endif

#if MAIN_AT_CONSOLE_ENABLE
    /* 把云协议离线自测命令挂到日志口(#json / #status / #topic / #offline) + 射频诊断(#lora)。
     * 弱网/离线时靠它验证协议层; 射频收不到时靠 #lora 看芯片里的实际配置。 */
    g4_at_set_console_ext(app_console_ext, app_console_help);
    log_info("[BOOT] 日志口离线自测命令已挂载: #json #status #topic #offline #lora\r\n");
#endif
#else  /* !MAIN_G4_ENABLE */
    log_info("[BOOT] 4G 状态机已关闭(MAIN_G4_ENABLE=0): 只跑 LoRa, 云端下发收不到\r\n");
#endif

    /* ---------- 5. RS232 / RS485 通道(协议待定, 仅初始化) ---------- */
#if MAIN_SERIAL_INIT_ENABLE
    serial_init_all(MAIN_SERIAL_BAUD);
    log_info("[BOOT] RS232/RS485 通道已初始化 (%lu bps)\r\n",
             (unsigned long)MAIN_SERIAL_BAUD);
#endif

    /* ---------- 6. OTA ----------
     * 【两条通道, 别混】:
     *   ota.c       MQTT 分包版(按包收 + 位图 + CRC32) —— 保留, 但 cloud_proto
     *               已经不再调它了(2026-10-02 起 ota.start 走 HTTP 版)。
     *   ota_http.c  HTTP 拉取版(ML307R 原生 AT+MHTTP* + MD5) —— ★当前通道★
     *
     * ota_http_init() 【无条件调用】, 不放在 MAIN_OTA_ENABLE 里:
     *   它只做 memset + 把自己注册成 g4_urc 的 +MHTTPURC 钩子, 本身不发任何 AT。
     *   注册这件事【必须】做 —— 否则 +MHTTPURC 家族(里面是固件正文的 HEX)
     *   会落到 AT 应答匹配里去, 要么被当垃圾行丢掉, 要么误命中关键字把 AT 流带偏。
     * 注: 它不碰 Flash、不碰分区, 所以与 MAIN_OTA_ENABLE 那个"分区顾虑"无关。 */
    ota_http_init();
    log_info("[BOOT] OTA(HTTP) 已初始化: chunk=%u frag=%u/%ums\r\n",
             (unsigned)OTA_HTTP_CHUNK, (unsigned)OTA_HTTP_FRAG_SIZE,
             (unsigned)OTA_HTTP_FRAG_INTV);

    /* ==========================================================================
     * ★★2026-10-11: A/B 回滚判定(开机一次)★★
     * --------------------------------------------------------------------------
     * 【什么时候做】必须在进主循环之前 —— 越早扣机会越准(晚扣的话, 一个"起来就崩"
     *   的固件可能还没扣到就崩了, 计数永远涨不上去, 回滚就永远不触发)。
     *
     * 【(乙) 方案: 模式 1 下【只记录、不复位】】
     *   回滚动作本身是 bootloader 的职责(把另一槽搬回运行区)。而当前是
     *   FM_HAS_BOOTLOADER=0 的单镜像模式 —— 板上没有 BL, 复位之后跑的【还是同一个
     *   slot】⇒ 会变成无限重启循环, 比不回滚严重得多。
     *   ⇒ 用 FM_HAS_BOOTLOADER 条件编译: 有 BL 才复位。
     *   ⇒ 好处: 现在(模式 1)就能把判定+计数逻辑跑起来、用日志验证;
     *     将来切模式 2 只需改 flash_map.h 一个宏, 这段代码不用动。
     * ======================================================================== */
    /* 无条件编译: 模式 1 下只记录日志, 零副作用(见下面的 (乙) 说明)。
     * (旧的 MQTT 分包版 ota.c 已归档, 不再有 MAIN_OTA_ENABLE 这个开关。) */
    if (ota_health_boot_step() == OTA_HEALTH_BOOT_ROLLBACK)
    {
#if (FM_HAS_BOOTLOADER != 0)
        log_info("[BOOT] !! A/B 回滚: 本槽机会已用完 -> 复位, 让 bootloader 搬回另一槽\r\n");
        NVIC_SystemReset();          /* 不返回 */
#else
        /* ⚠️ 模式 1(无 BL): 复位也是同一个槽 ⇒ 只记录, 不复位。
         * 这一行日志就是"这台设备本该被回滚"的证据, 现场可据此判断固件是否有问题。 */
        log_info("[BOOT] !! A/B 回滚【已判定但不执行】: 当前是模式 1(无 bootloader), "
                 "复位只会重跑同一槽 -> 若切成模式 2 这里会真的回滚\r\n");
#endif
    }
    log_info("[OTAHL] A/B: 槽=%u 剩=%u 次 %s\r\n",
             (unsigned)ota_health_boot_slot(),
             (unsigned)ota_health_tries_left(),
             ota_health_confirmed() ? "(已定案)" : "(待确认)");

    log_info("[BOOT] 进入主循环\r\n");
}
