#ifndef __LEVEL_SUPERVISOR_H__
#define __LEVEL_SUPERVISOR_H__

#include <stdint.h>
#include <stdbool.h>
#include "config_store.h"
#include "level_eval.h"         /* level_state_t */

/* ==========================================================================
 * 边缘水位评估的【取数 + 串接】层
 * ==========================================================================
 * 【为什么需要它】
 *   level_eval.c 只有算法(比较/计数/状态迁移), 它的输入是【数组】, 由调用方给。
 *   于是它一直"没有调用方", 链接器把整段丢掉 —— 文档 4 章的自治核心形同不存在。
 *   本模块就是缺的那一层: 从 config_store 取传感器表、从 last_value 取最近水位、
 *   按 sensor_offline 的同一规则判在线, 然后按文档顺序串起来:
 *
 *     有效水位(max) -> confirmCount 防抖 -> 状态 -> 档位
 *                   -> 档位模板渲染 -> 该不该下发 -> 该不该生成水位事件
 *
 * 【本模块仍然一行 LoRa/4G 都不碰】
 *   真正的"下发"要走 LoRa 下行(现场正在排障), 所以本模块只【算出决定】,
 *   由调用方去发; 发送成功后再回来调 level_supervisor_mark_sent()。
 *   ★ 发送【失败】不要调 mark_sent ★ —— 那样下次会重发(与 level_eval 的约定一致)。
 *
 * 【为什么要 new_sample 这个字段】
 *   poll() 只在【有新上报】时推进算法(见 .c 里的说明), 而没有新上报时它什么都不做。
 *   调用方必须能区分"这次没新样本"和"有样本但一台有效的都没有" ——
 *   否则会把前者误当成 fail-safe。所以 new_sample 先于一切判断给出。
 * ========================================================================== */

typedef struct
{
    bool          new_sample;   /* false = 本次没有新上报, 其余字段无意义 */
    bool          have_level;   /* 是否有有效水位(在线且 enabled 的至少一台) */
    int32_t       level_mm;     /* 有效水位 = 上述集合里取 max */
    level_state_t state;        /* 当前【已确认】状态 */
    level_state_t prev_state;   /* ★ 本次迁移【之前】的状态 ★
                                 * 用途: 产生 `event.status_change` 时要 from/to 两个值,
                                 * 而 `level_eval_feed()` 会把 confirmed 覆盖成新状态 ——
                                 * 所以调用前先存一份。没有迁移时它 == state。 */
    uint8_t       gear;         /* 目标档位 0~3 */
    bool          migrated;     /* 本次确认了状态迁移(该按档位下发 / 该上报 status_change) */
    bool          dispatch;     /* 本次该下发联动(档位变了 或 渲染后文本变了) */
    bool          auto_suppressed;  /* ★ manual 模式把自动联动压住了 ★
                                     * (dispatch=false 是因为"手动模式", 不是"没变化"
                                     *  —— 调用方据此打不同的日志, 见 level_supervisor.c) */
    bool          event_due;    /* 本次该生成一条水位类事件(minChangeCm + 事件间隔) */

    /* ---- 传感器【全部失联】的 fail-safe(文档第 5 章, 用户第 7 条) ----
     * 【为什么要单独给字段】"全部失联"是【时间】条件(3×上报周期没上报), 不是
     * "来了新样本"才成立 —— 所以它在 poll() 里【先于 new_sample 判断】就算出来了。
     * 一台都不在线时 have_level=false, 调用方只凭它分不清
     *   a) 上电至今还没收到过任何上报(没什么可 keep/alarm 的), 和
     *   b) 本来有水位数, 现在全掉线了(必须按策略动作)。
     * 所以这里显式给三者:
     *   failsafe       当前是否处于"全部失联"的 fail-safe 态
     *   failsafe_enter ★边沿★ 刚从"有水位"跌进"全部失联"的那一刻为 true(只一次)
     *   failsafe_gear  按 rules.sensorOfflinePolicy 算出的应执行档位
     *                  (keep -> 保持 s_last 档位, alarm -> 3, off -> 0) */
    bool          failsafe;
    bool          failsafe_enter;
    uint8_t       failsafe_gear;
    /* ★2026-10-08: LED 文本由单段拆成两段★
     * led_text1 = 竖屏左半屏 / 横屏上半屏; led_text2 = 竖屏右半屏 / 横屏下半屏。
     * 任一段为空 => 设备按全屏显示(设备从长度=0 推断)。 */
    char          led_text1[CFG_LEDTEXT_MAX + 1u];
    char          led_text2[CFG_LEDTEXT_MAX + 1u];
    char          tts_text[CFG_TTSTEXT_MAX + 1u];   /* 空 = 无语音 */
} level_decision_t;

/** 清空状态机(上电初始化; 恢复出厂/重新绑定后也该调) */
void level_supervisor_init(void);

/**
 * @brief 评估一次。放在主循环里反复调是安全的(没有新上报就立刻返回)。
 * @param cfg    全量配置(可为 NULL -> 只把 new_sample 置位, 不做判定)
 * @param now_ms 当前 tick(get_tick_ms())
 * @param out    结果(不可为 NULL)
 * @note  只在检测到【新上报】(last_value 的更新计数变化)时才推进 level_eval ——
 *        confirmCount 是"连续几次上报一致", 若按主循环轮次喂, 几微秒就攒够了。
 */
void level_supervisor_poll(const config_store_t *cfg, uint32_t now_ms,
                           level_decision_t *out);

/** 联动【下发成功】之后调用(失败不要调)
 *  ★2026-10-08: 两段 LED 文本都要传★ —— 去重基准是"两段都比",
 *  只传第一段会导致"下半屏变了却不重发"。 */
void level_supervisor_mark_sent(uint8_t gear, const char *led_text1, const char *led_text2);

/** 水位事件【真的生成并发布】之后调用, 记下事件去抖基准 */
void level_supervisor_mark_event(int32_t level_mm, uint32_t now_ms);

/**
 * @brief 取最近一次评估的结果(有效水位 + 档位)
 * @param out_level_mm 输出: 有效水位(mm)
 * @param out_gear     输出: 档位 0~3
 * @return true = 有过有效水位; false = 还没有(两个输出都不改)
 * @note  用途: 手动控制(dev.ctrl)要按档位模板渲染文本, 而模板里的 {level} 得用
 *        "当前水位" —— 这个值只有本模块算得出来。让调用方(cloud_proto)直接取,
 *        比在别处再实现一遍"取 max"要安全(两份实现迟早对不上)。
 */
bool level_supervisor_last(int32_t *out_level_mm, uint8_t *out_gear);

/**
 * @brief 让【下一次】poll 无视"没有新上报"强行评估一次
 * @note  文档 5.5 要求 rule.mode 切回 auto 时"立即按当前水位重评估一次并刷新联动
 *        状态"; 那一刻通常没有新上报, 而 poll 平时只在有新样本时推进(否则
 *        confirmCount 会被主循环的空转瞬间攒满)。所以用这个显式的"强制一次",
 *        而不是把"按轮次推进"变成默认行为。
 */
void level_supervisor_force_next(void);

/** manual -> auto 时强制下一次 dispatch(即使档位/文本没变) */
void level_supervisor_force_dispatch(void);

#endif /* __LEVEL_SUPERVISOR_H__ */
