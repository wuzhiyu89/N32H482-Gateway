/**
 * @file  app_loop.c
 * @brief 主循环（裸机协作式调度，1 ms 时基分频）
 *
 * 节拍表见 docs/ARCHITECTURE.md 第 6 节：
 *   1ms   射频中断/帧分发/重传 · 5ms AT+OTA 状态机 · 10ms 控制台 ·
 *   1s    状态打印/水位评估/联动/补报/心跳准备 · 心跳按配置周期
 *
 * 原则：每一拍都只推进一步，不允许阻塞；末尾与 1 ms 对齐让中断能抢占。
 */
#include "n32h47x_48x.h"
#include <stdint.h>

#include "log.h"
#include "app_config.h"
#include "app_core.h"
#include "app_state.h"
#include "bsp_tim.h"
#include "cloud_proto.h"
#include "config_store.h"
#include "g4_at.h"
#include "g4_urc.h"
#include "level_supervisor.h"
#include "lora_business.h"
#include "ota_health.h"
#include "flash_map.h"
#include "ota_port.h"
#include "ota_http.h"

void app_loop_run(void)
{
    uint32_t now;
#if MAIN_G4_ENABLE
    uint32_t t_g4;
#endif
#if MAIN_G4_ENABLE && MAIN_GW_ONLINE_ENABLE
    uint32_t t_online;
#endif
    uint32_t t_console;
    uint32_t t_status;

    /* ==================== 主循环 ==================== */
    now        = get_tick_ms();
    t_console  = now;
    t_status   = now;
#if MAIN_G4_ENABLE
    t_g4       = now;
#endif
#if MAIN_G4_ENABLE && MAIN_GW_ONLINE_ENABLE
    t_online   = now;
#endif

    while (1)
    {
        now = get_tick_ms();

        /* ★★2026-10-11: 健康判定(每圈都调, 但代价只有 3 次比较)★★
         * 【判据(三条缺一不可)】
         *   ① 4G 在线                      -> g4_at_is_online()
         *   ② 至少一条上行被 broker puback  -> g4_urc_puback_count() > 0
         *      ★ 用 puback 而不是"发布成功": 模组回 OK 只说明它自己收下了,
         *        puback 才是"云端确实收到了"的证据(见上面 [STAT2] 的注释)。
         *   ③ uptime > 60s(OTA_HEALTH_MIN_UPTIME_S) -> 挡"起来就崩"
         * 三条都满足就把本槽"定案"(写一次 Flash), 之后本函数立即变成空操作。
         * 【为什么每圈调而不是挂 1s 节拍】: 定案后 s_done=true 直接 return,
         *   定案前也只是 3 次比较 —— 比"再找一个 1s 节拍插入点"简单且不易错。 */
#if (FM_HAS_BOOTLOADER == 0)
        ota_health_poll(g4_at_is_online(),
                        (g4_urc_puback_count() > 0u),
                        (uint32_t)(now / 1000u));
#else
        /* A/B: pending 槽跑够 60s + 联网 + 有上行确认 -> 转正(由 BL 下次启动沿用) */
        ota_port_boot_poll(g4_at_is_online(),
                           (g4_urc_puback_count() > 0u),
                           (uint32_t)(now / 1000u));
#endif

        /* (A) 每 1ms: LoRa radio 中断处理 + 帧解析分发 + 从机表/重传/上行缓存 */
#if MAIN_LORA_ENABLE
        lora_business_poll();
#endif

        /* (A2) sys.reboot 的延迟重启(v37+): 命令只登记"什么时候重启", 真正的软件复位
         *      在这里做 —— 这样 cloud_proto 不用直接碰 NVIC_SystemReset, 主机单测
         *      也能把这条命令的逻辑验完。纯比较, 每个循环跑一次的开销可以忽略。 */
        if (cloud_proto_reboot_due(app_now_unix()))
        {
            log_info("[SYS] sys.reboot 到期, 执行软件复位(NVIC_SystemReset)...\r\n");
            delay_ms(50);               /* 让上面那行日志从串口发完 */
            NVIC_SystemReset();
        }

        /* (B) 每 5ms: 收包泵 + 云端下行投递 + 非阻塞 AT 状态机 + 发布子流程
         *     ⚠️ 这里【不会阻塞】: 每条 AT 命令每次只推进一步, 收包由 g4_io_pump()
         *        一次性消费干净。模组不应答也只是状态机停在原地重试, 主循环节拍不变。
         *        模组没调通就把 MAIN_G4_ENABLE 改成 0。 */
#if MAIN_G4_ENABLE
        if ((uint32_t)(now - t_g4) >= TICK_G4_POLL_MS)
        {
            t_g4 = now;
            g4_at_poll();

            /* OTA over HTTP 必须跟【同一节拍】推进, 不能放到下面 1s 的 (D) 块:
             * 下载状态机每拍要发一条 AT(HEADER/REQUEST) 并消化 URC 字节队列,
             * 而模组按 fragment 配置以 200 B/块往外吐。若 1s 才推一次, 就变成
             * 200 B/s —— 64 KB 要 5 分钟以上, 还能把 8192 的环形缓冲灌爆。
             * 本函数纯粹是状态机推进, 不阻塞、空闲时什么都不做。 */
            ota_http_poll();
        }
#endif

        /* (C) 每 10ms: 日志口手敲 AT 转发给 4G 模组(台架调试)
         *     注: 手敲命令会插入 AT 流, 可能被状态机的关键字匹配"看到",
         *         排障时尽量避开正在握手/发布的时间窗口。 */
        if ((uint32_t)(now - t_console) >= TICK_CONSOLE_MS)
        {
            t_console = now;
            app_console_poll();
        }

        /* (D) 每 1s: 状态上报 + LED 心跳 */
        if ((uint32_t)(now - t_status) >= TICK_STATUS_MS)
        {
            t_status = now;

            app_status_report();

            /* ---- 边缘水位评估(v54): 复用这同一个 1s 节拍, 【不新增定时器】 ----
             * 文档 4 章的自治核心: 有效水位(max) -> confirmCount 防抖 -> 档位 ->
             * 模板渲染 -> 该不该下发。算法在 level_eval.c, 取数+串接在 level_supervisor.c。
             * ★ 每秒调一次【不会】把 confirmCount 变成"立刻满足": level_supervisor_poll()
             *   只在检测到【新上报】时才推进算法(按 last_value 的更新计数判)。
             *
             * 【2026-09-30 用户第 2 条: 最后一跳接上了】
             *   以前这里只"评估 + 记日志"。现在真的把档位动作下发到联动设备:
             *     level_supervisor_poll() 算出 dispatch -> lora_business_apply_gear()
             *     按 gears[] 模板组帧投递 -> 发【成功】才 level_supervisor_mark_sent()
             *     (失败不 mark, 下次有新上报时会自动重试)。
             *   注意 dispatch 与 migrated 是【两个判据】: migrated 管"要不要上报
             *   status_change", dispatch 管"要不要下发联动"(档位变【或】文案变),
             *   所以下面分成两个 if, 不能合并。 */
            {
                level_decision_t ld;

                level_supervisor_poll(config_store_get(), get_tick_ms(), &ld);

                /* 缓存最近一次【有效】评估结果: status.report / telemetry 要用它取
                 * 实时档位与状态(第 10 条: 以前那里硬编码 0/"auto") */
                if (ld.new_sample && ld.have_level)
                {
                    app_note_decision(&ld);
                }

                /* ---- (D1) 联动下发: 判据是 dispatch ---- */
                if (ld.new_sample && ld.have_level && ld.dispatch)
                {
                    uint8_t total = 0U;
                    int     sent  = lora_business_apply_gear(ld.gear, ld.level_mm,
                                                             ld.led_text1, ld.led_text2,
                                                             ld.tts_text,
                                                             &total);

                    if (sent > 0)
                    {
                        /* 真发出去了 -> 记下档位/文案, 下次同样的内容不再重复下发 */
                        level_supervisor_mark_sent(ld.gear, ld.led_text1, ld.led_text2);
                    }
                    else if (total == 0U)
                    {
                        /* 压根没有"启用的"联动设备: 这是配置事实, 不是发送失败。
                         * 也 mark(否则每来一次上报都会重试一次, 日志被刷满)。 */
                        level_supervisor_mark_sent(ld.gear, ld.led_text1, ld.led_text2);
                        log_info("[LEVEL] 档位 %u 无启用的联动设备, 视为已处理\r\n",
                                 (unsigned)ld.gear);
                    }
                    /* sent==0 且 total>0: 投递失败(载荷过长/从机表里没有) ->
                     * 不 mark, 等下次上报重试。lora_business 那边已打原因。 */
                }

                /* ---- (D1b) 全部失联 fail-safe(第 7 条): 边沿触发, 只做一次 ---- */
                app_failsafe_step(&ld);

                /* ---- (D2) 状态迁移上报: 判据是 migrated ---- */
                if (ld.new_sample && ld.have_level && ld.migrated)
                {
                    log_info("[LEVEL] 迁移 -> state=%u gear=%u level=%ldmm led1=\"%s\" led2=\"%s\" 该下发=%u\r\n",
                             (unsigned)ld.state, (unsigned)ld.gear,
                             (long)ld.level_mm, ld.led_text1, ld.led_text2,
                             (unsigned)ld.dispatch);

                    /* ---- 【第 8 条】事件去抖真正生效 ----
                     * migrated 只说明"状态被确认变了"; 文档 7.2 还要求
                     * event.status_change 受 minChangeCm 与 eventMinIntervalSeconds 约束。
                     * 这两个判据 level_eval 早就算好放在 ld.event_due 里了,
                     * 但这里【以前从来没读它】—— 于是去抖形同不存在。
                     * 现在: 不 due 就不发(也不 mark, 让下一条够格的样本能发出去)。 */
                    if (!ld.event_due)
                    {
                        log_info("[LEVEL] 迁移事件被去抖拦下(变化 < %ucm 或间隔 < %us)\r\n",
                                 (unsigned)config_store_get()->thresholds.min_change_cm,
                                 (unsigned)config_store_get()->thresholds.event_min_interval_s);
                    }
                    else
                    {
                        /* ★ 在线直发 / 断线入缓存, 两支互斥 (v72) ★
                         * 相关函数见上面 app_publish_status_change / app_cache_status_change。
                         * 之前这里只有"入缓存"一支; 现在在线时也真的把 status_change 发出去了。 */
                        if (g4_at_is_online())
                        {
                            app_publish_status_change((uint8_t)ld.prev_state, (uint8_t)ld.state,
                                                       ld.gear, ld.level_mm);
                        }
                        else
                        {
                            app_cache_status_change((uint8_t)ld.prev_state, (uint8_t)ld.state,
                                                     ld.gear, ld.level_mm);
                        }

                        /* 事件【已生成】(直发或入缓存都算) -> 记下基准, 供下一次去抖比较。
                         * ★ 不在这里留"失败重试"的口子: 入缓存那条本来就是断线补报的入口;
                         *   直发失败也只影响这一条, 下一个够格的样本会再发一条新事件。 */
                        level_supervisor_mark_event(ld.level_mm, get_tick_ms());
                    }
                }

                /* ---- (D3) 水位遥测上行(第 3 条): 周期 or 变化量触发 ---- */
                app_telemetry_step();

                /* ---- (D4) manual 模式窗口(第 6 条): 到期自动回归 auto ---- */
                app_manual_step();
            }

            /* ---- 断线补报: 每次最多补一页(16 条), 缓存空了立刻返回 (v70) ---- */
            app_backfill_poll();

            /* ---- ★2026-10-18: 逐条液位上报的补报(一次一条) ----
             * 与上面的"事件补报"分开跑: 两者缓存不同、topic 不同。
             * 先事件后遥测 —— 事件量小、时效性强; 平台侧各自按 ts 排序。 */
            app_telemetry_backfill_poll();

#if MAIN_LORA_ENABLE
            /* ---- 液位失联检测的【定时兜底腿】(2026-09-27 用户拍板加) ----
             * 【为什么光有"上报驱动"不够】原口径是"每收到一路液位上报就顺带检查
             * 全部液位从机", 但那只在【至少有一台还在报】时才成立。现场只挂了一台
             * 液位(210001), 它彻底不报时【没有任何帧能触发检查】—— 而"唯一那台死了"
             * 恰恰是最该报的 case, 于是最需要它的场景反而检测不到。
             * 所以再加这条腿: 复用【已有的 1s 节拍】, 【不新增任何定时器】。
             * check 函数是幂等的(只在"在线->失联"跳变时发一次事件), 每秒调用不会
             * 造成重复上报; 上报驱动那条腿保留不动, 两条腿互为补充。 */
            lora_business_check_sensor_offline();

            /* *2026-10-18: 出厂字段操作(0x06 重发 / 0x07 超时)推进
             * 1 秒粒度足够: 写字段要 5s 才补发一条, 读字段要等 12s。 */
            lora_business_field_poll();
#endif

#if MAIN_HEARTBEAT_LED
            GPIO_TogglePin(GPIOA, GPIO_PIN_8);
#endif

            /* OTA over HTTP 的进度上报 + 就绪后复位。
             * 放在 1s 块里是【刻意的】: 文档 7.6 要求 ota.progress 每 10% 或 10s
             * 一条, 状态机本身的推进在 5ms 块里(见上面 (B)), 两件事分开。
             * 本函数只读状态 + 发 MQTT, 不发 AT、不阻塞; 空闲时第一件事就是 return。
             * 刻意【不】加 #if MAIN_G4_ENABLE: 那样在 4G 关掉时它就成了没人调的
             * static 函数, 编译器会报 unused-function。 */
            app_ota_progress_poll();
        }

        /* (E) 网关在线心跳 —— 往云端 pub 一条 status.report(或裸 "GW_ONLINE")
         *
         * 为什么要有: 云端只认"有没有收到报文"。业务数据都来自下属从机, 从机可能
         * 几十分钟不上报, 这段时间云端会以为网关掉线了。这条心跳就是网关的
         * "我还活着"信号, 同时它也是 4G 链路的探针 —— 发不出去会累加 g4_at 里的
         * 发布失败计数, 连续失败就自动重新建链(见 g4_pub_mark_fail)。
         *
         * 【第 5 条 2026-09-30】周期改成读配置 network.statusIntervalSeconds
         *   (文档 3.2: 10~600s, 默认 60, 生效方式"立即")。以前这里是编译期宏,
         *   平台改了配置【一点反应都没有】。读不到/越界才回落到 MAIN_GW_ONLINE_MS。 */
#if MAIN_G4_ENABLE && MAIN_GW_ONLINE_ENABLE
        {
            uint32_t hb_period_ms = app_cfg_period_ms(config_store_get()->network.status_interval_s,
                                                      MAIN_GW_ONLINE_MS);

            /* ★2026-10-07: 刚上线【立刻】发第一条 retained 状态, 把遗嘱盖掉★
             *
             * 【现场现象】每次上电连上 broker 后, 平台会先收到一条
             *     {"online": false}     (发在 retained 的 mod/status 上)
             * 这不是固件发的, 是 MQTT 的【遗嘱(LWT)】被 broker 代发的 —— 机制是:
             *   本机 client id 就是 IMEI(固定), 上次掉电时 broker 那边那个会话还没
             *   走到 keepalive 超时, 它的遗嘱一直挂着; 这次重连 broker 发现"同名旧
             *   会话还在", 就把它踢掉 —— 【踢掉的那一刻旧会话的遗嘱才真正发出去】。
             *   所以它看起来"跟着上电一起来", 其实是上一次掉线欠下的。
             * 见 g4_at 里的 AT+MQTTCFG="willoption",0,1,1,1 (遗嘱 retain=1) 与
             * willpayload 的 mod/status / {"online":false}。
             *
             * 【为什么必须立刻发】平台按"最后一条 retained 消息"判断在线状态。遗嘱也
             * 是 retained 的, 且和 status.report 用同一个 topic。原先第一条心跳要等
             * 满一个周期(默认 30s)才发, 于是这 30 秒里平台一直显示离线 —— 现场看着
             * 就像"上电后先报离线"。
             * 现在: 一进入 ONLINE 就把 t_online 拨到"已到期", 下一拍立刻发一条
             * retained 的 status.report(online:true), 100ms 级就把遗嘱盖回去。 */
            {
                static bool s_hb_was_online;
                bool        online_now = g4_at_is_online();

                if (online_now && (!s_hb_was_online))
                {
                    t_online = now - hb_period_ms;      /* 让下面的判据立刻成立 */
                }
                s_hb_was_online = online_now;
            }

            /* ★2026-10-07: OTA 下载期间心跳【让路】★
             *
             * 现场(19:09) 实测的死法: 心跳/事件发布和 OTA 共用同一条 AT 引擎, 而
             * 发布一旦撞上 '>' 丢失, 光是等 '>' 就要占住引擎 15 秒
             * (G4_PUB_PROMPT_TIMEOUT_MS); 超时后立刻重发, 又一个 15 秒 —— 连着几轮
             * 就把 OTA 的下一片 AT+MHTTPHEADER 一直挤在门外:
             *   19:09:20  第一片 8192 收满(percent 52)
             *   19:09:20 ~ 19:10:20  【60 秒里一条 AT 都没发出去, 全是 MQTTPUB 在重试】
             *   19:10:20  片内超时(60s) -> 回滚到 7680 重下 7764 字节
             * 而就在这段"抢引擎"的时间里串口还丢了字节(丢字节 1->5),
             * 最终 MD5 对不上(算出 dd4a7606..., 云端 d9c8750b...) —— 前功尽弃。
             *
             * OTA 期间 ota.progress 仍然每 10 秒上报一次, 云端不会以为网关死了,
             * 所以这里让路是安全的; 升级一结束(READY/FAILED)就自动恢复。 */
            if (g4_at_is_online() && (!ota_http_busy()) &&
                ((uint32_t)(now - t_online) >= hb_period_ms))
            {
            sys_err_t hb_res;

#if MAIN_STATUS_JSON_ENABLE
            /* P1-8: 心跳载荷换成文档要求的 status.report JSON(替代裸 "GW_ONLINE") */
            hb_res = app_publish_status_json();
#else
            static const char gw_online[] = "GW_ONLINE";
            hb_res = g4_at_mqtt_publish(NULL, (uint8_t *)gw_online,
                                        (uint16_t)(sizeof(gw_online) - 1u));
#endif
            if (hb_res == SYS_OK)
            {
                t_online = now;
#if MAIN_STATUS_JSON_ENABLE
                log_info("[GW] status.report 已发 (距上次发布成功 %lus)\r\n",
                         (unsigned long)(g4_at_pub_age_ms() / 1000u));
#else
                log_info("[GW] 在线心跳已发: GW_ONLINE (距上次发布成功 %lus)\r\n",
                         (unsigned long)(g4_at_pub_age_ms() / 1000u));
#endif
            }
            else
            {
                /* 【这一条以前是静默的, 而它就是"4G 好像不会保活"最可能的真相】:
                 * 原来失败时什么都不打印, 于是"心跳发不出去"这件事在现场完全看不见,
                 * 只能看到云端那边这台设备慢慢不理人了。
                 * 现在: ① 失败一定打日志; ② 不等满 30s, 2s 后就重试;
                 *       ③ g4_at 里【连续 3 次】(G4_PUB_FAIL_MAX)发布失败就判链路断,
                 *          走 g4_link_recover 直接 4G_RST 断电重启模组重建连接;
                 *       ④ 另有保活停滞看门狗(5 × 心跳周期 = 150s 没成功过一次)兜底。 */
                t_online = now - hb_period_ms + 2000U;
                log_info("[GW] !! 在线心跳【发布登记失败】(上一条还在发 / 状态机不在 ONLINE), "
                         "2s 后重试; 距上次成功 %lus\r\n",
                         (g4_at_pub_age_ms() == 0xFFFFFFFFu)
                             ? 0UL : (unsigned long)(g4_at_pub_age_ms() / 1000u));
            }
            }
        }
#endif

        /* 与 1ms 时基对齐, 避免空转(中断仍可抢占) */
        while (get_tick_ms() == now)
        {
        }
    }
}
