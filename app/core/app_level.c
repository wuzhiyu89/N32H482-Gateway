/**
 * @file  app_level.c
 * @brief 本地自治的时间侧：全部失联 fail-safe + manual 窗口回归
 *
 * 水位"算"在 service/level_eval.c，"取数+串接"在 level_supervisor.c；
 * 这里只做两件需要时钟和配置写入的事：
 *   · 全部失联时的兜底动作（keep / alarm / off）
 *   · manual 模式到期自动回到 auto，并强制重评估一次
 *
 * 不组 JSON、不发 MQTT —— 下发走 lora_business_apply_gear()，上报由主循环决定。
 */
#include <string.h>

#include "log.h"
#include "app_core.h"
#include "bsp_tim.h"
#include "config_store.h"
#include "level_supervisor.h"
#include "lora_business.h"

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
