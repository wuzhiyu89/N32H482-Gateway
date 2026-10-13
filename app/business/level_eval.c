/**
 * @file  level_eval.c
 * @brief 边缘水位评估纯逻辑实现(见 level_eval.h)
 *
 * 本文件【不 include】任何 LoRa / 4G / 日志之外的东西: 输入是数组, 输出是状态,
 * 所以能在主机上把边界情况一次跑完(见 _host_test/test_level.c)。
 */
#include "level_eval.h"
#include "utils.h"      /* utf8_char_len() —— 档位载荷的"汉字 3 字节"文本组帧 */
#include <string.h>     /* strcmp / strncmp / strncpy / memcpy */
#include <stdio.h>      /* snprintf(目标机上有, config_store 也在用) */

/* 阈值是 cm, 水位是 mm -> 比较前统一乘 10。
 * alarmCmd 等是 uint16(最大 65535), *10 后 655350, int32 装得下, 不会溢出。 */
static int32_t lv_cm2mm(uint16_t cm)
{
    return (int32_t)cm * 10;
}

uint8_t level_state_to_gear(level_state_t st)
{
    switch (st)
    {
        case LEVEL_ST_RESOLVED: return 0u;
        case LEVEL_ST_NORMAL:   return 1u;
        case LEVEL_ST_WARNING:  return 2u;
        case LEVEL_ST_ALARM:    return 3u;
        default:                return 0u;   /* UNKNOWN 也按 0 处理(还没测到水位) */
    }
}

level_state_t level_state_of(int32_t level_mm, const cfg_thresholds_t *t)
{
    if (t == NULL)
    {
        return LEVEL_ST_UNKNOWN;             /* 没阈值就不猜 */
    }

    /* 顺序照文档 3.4 的表: >= alarm 先判, 然后是 (warning, alarm), 再 (resolve, warning], 最后 <= resolve */
    if (level_mm >= lv_cm2mm(t->alarm_cm))
    {
        return LEVEL_ST_ALARM;
    }
    if (level_mm > lv_cm2mm(t->warning_cm))
    {
        return LEVEL_ST_WARNING;
    }
    if (level_mm > lv_cm2mm(t->resolve_cm))
    {
        return LEVEL_ST_NORMAL;
    }
    return LEVEL_ST_RESOLVED;
}

bool level_effective(const int32_t *levels_mm, const uint8_t *select,
                     uint8_t n, int32_t *out_mm)
{
    uint8_t i;
    bool    got = false;
    int32_t best = 0;

    if ((levels_mm == NULL) || (select == NULL) || (out_mm == NULL) || (n == 0u))
    {
        return false;
    }

    for (i = 0u; i < n; i++)
    {
        if (select[i] == 0u)
        {
            continue;                       /* 离线或停用的不参与(文档 3.3 末) */
        }
        if (!got || (levels_mm[i] > best))
        {
            best = levels_mm[i];
            got  = true;
        }
    }

    if (!got)
    {
        return false;                       /* 一台有效的都没有 */
    }
    *out_mm = best;
    return true;
}

void level_eval_init(level_eval_t *e)
{
    if (e == NULL)
    {
        return;
    }
    e->confirmed   = LEVEL_ST_UNKNOWN;
    e->candidate   = LEVEL_ST_UNKNOWN;
    e->confirm_cnt = 0u;
    e->have        = false;
}

bool level_eval_feed(level_eval_t *e, const cfg_thresholds_t *t,
                     int32_t level_mm, level_state_t *out_state)
{
    level_state_t cand;
    uint8_t       need;

    if ((e == NULL) || (t == NULL))
    {
        return false;
    }

    cand = level_state_of(level_mm, t);

    /* 连续 N 次候选一致才确认(文档 3.4: confirmCount = 1 时不启用) */
    if ((e->confirm_cnt > 0u) && (cand == e->candidate))
    {
        if (e->confirm_cnt < 0xFFu)
        {
            e->confirm_cnt++;
        }
    }
    else
    {
        e->candidate   = cand;
        e->confirm_cnt = 1u;
    }

    need = (t->confirm_count == 0u) ? 1u : (uint8_t)t->confirm_count;
    if (e->confirm_cnt < need)
    {
        return false;                       /* 还没攒够, 不迁移 */
    }

    if (e->have && (e->confirmed == cand))
    {
        return false;                       /* 已确认的状态没变 -> 文档 4 章"档位未变化时不重复下发" */
    }

    e->confirmed = cand;
    e->have      = true;
    if (out_state != NULL)
    {
        *out_state = cand;
    }
    return true;
}

void level_event_gate_init(level_event_gate_t *g)
{
    if (g == NULL)
    {
        return;
    }
    g->last_mm = 0;
    g->last_ms = 0u;
    g->have    = false;
}

bool level_event_should_fire(const level_event_gate_t *g, const cfg_thresholds_t *t,
                             int32_t level_mm, uint32_t now_ms)
{
    int32_t  delta;
    uint32_t gap_ms;

    if ((g == NULL) || (t == NULL))
    {
        return false;
    }
    if (!g->have)
    {
        return true;                        /* 第一条件总是要出 */
    }

    /* 变化量: 用绝对值, 涨和落都算(文档: 生成新告警事件所需的最小水位变化量) */
    delta = level_mm - g->last_mm;
    if (delta < 0)
    {
        delta = -delta;
    }
    if (delta < lv_cm2mm(t->min_change_cm))
    {
        return false;
    }

    /* 间隔: 无符号差值, 回绕也不会判错(与 bsp 的 tick 同一套口径) */
    gap_ms = now_ms - g->last_ms;
    if (gap_ms < ((uint32_t)t->event_min_interval_s * 1000u))
    {
        return false;
    }
    return true;
}

void level_event_mark(level_event_gate_t *g, int32_t level_mm, uint32_t now_ms)
{
    if (g == NULL)
    {
        return;
    }
    g->last_mm = level_mm;
    g->last_ms = now_ms;
    g->have    = true;
}

/* ---------------- {level} 模板渲染 ---------------- */

uint16_t level_render_template(char *out, uint16_t cap, const char *tmpl, int32_t level_mm)
{
    uint16_t off = 0u;
    int32_t  cm;

    if ((out == NULL) || (cap == 0u) || (tmpl == NULL))
    {
        return 0u;
    }

    cm = level_mm / 10;                     /* 文档 3.5: {level} = 当前水位 cm 整数(截断) */

    while (*tmpl != '\0')
    {
        if ((tmpl[0] == '{') && (strncmp(tmpl, "{level}", 7u) == 0))
        {
            char num[14];
            int  n = snprintf(num, sizeof(num), "%ld", (long)cm);

            /* ★2026-10-14 用户要求: 一位数后面补一个空格 ★
             * 【为什么】LED 屏按定宽格显示, "5" 和 "15" 占位不同会让上下两行
             *   (或同一行的两段)对不齐; 补一个空格后 "5 " 与 "15" 宽度一致。
             * 【补哪些】用户明确: 0~9 都补(0 也补), 两位数/更多位不补。
             * 【两边都要】本函数是 LED 与 TTS 【共用】的渲染器, 用户确认
             *   "都要补" ⇒ 改这一处, LED 和 TTS 同时生效。
             * ⚠️ 判据用 n(字符数)而不是 cm 的数值: 负数会得到 "-5"(n=2) 不补,
             *   这与"两位数不补"的口径一致。 */
            if ((n == 1) && ((uint32_t)n + 1u < (uint32_t)sizeof(num)))
            {
                num[n]     = ' ';
                num[n + 1] = '\0';
                n++;
            }

            if (n <= 0)
            {
                out[0] = '\0';
                return 0u;
            }
            if (((uint32_t)off + (uint32_t)n) >= (uint32_t)cap)
            {
                out[0] = '\0';              /* 装不下就整条作废, 绝不发半截给 LED */
                return 0u;
            }
            (void)memcpy(out + off, num, (size_t)n);
            off  = (uint16_t)(off + (uint16_t)n);
            tmpl += 7;
        }
        else
        {
            if (((uint32_t)off + 1u) >= (uint32_t)cap)
            {
                out[0] = '\0';
                return 0u;
            }
            out[off] = *tmpl;               /* 不认识的花括号原样保留, 不吞不改 */
            off++;
            tmpl++;
        }
    }

    out[off] = '\0';
    return off;
}

/* ---------------- 下发判据 ---------------- */

void level_dispatch_init(level_dispatch_state_t *d)
{
    if (d == NULL)
    {
        return;
    }
    d->have = false;
    d->gear = 0u;
    d->led_text1[0] = '\0';
    d->led_text2[0] = '\0';
}

bool level_dispatch_should_send(const level_dispatch_state_t *d, uint8_t gear,
                                const char *led_text1, const char *led_text2)
{
    if (d == NULL)
    {
        return false;
    }
    if (!d->have)
    {
        return true;                        /* 从没成功发过 -> 必须发(让设备对齐) */
    }
    if (d->gear != gear)
    {
        return true;                        /* 档位变了 -> 必须发 */
    }
    /* 档位没变: 只有【渲染后的文本真的变了】才发(文档 4 章)。
     * ★2026-10-08: 两段都要比★ —— 只比第一段的话, "上半屏没变、下半屏变了"
     * 这种情况会被判成"不用发", 设备就一直显示旧的下半屏。 */
    if (strcmp(d->led_text1, (led_text1 != NULL) ? led_text1 : "") != 0)
    {
        return true;
    }
    return (strcmp(d->led_text2, (led_text2 != NULL) ? led_text2 : "") != 0);
}

void level_dispatch_mark(level_dispatch_state_t *d, uint8_t gear,
                         const char *led_text1, const char *led_text2)
{
    const char *s1;
    const char *s2;

    if (d == NULL)
    {
        return;
    }
    s1 = (led_text1 != NULL) ? led_text1 : "";
    s2 = (led_text2 != NULL) ? led_text2 : "";
    (void)strncpy(d->led_text1, s1, sizeof(d->led_text1) - 1u);
    d->led_text1[sizeof(d->led_text1) - 1u] = '\0';
    (void)strncpy(d->led_text2, s2, sizeof(d->led_text2) - 1u);
    d->led_text2[sizeof(d->led_text2) - 1u] = '\0';
    d->gear = gear;
    d->have = true;
}

/* ============================================================================
 * 档位 -> LoRa 动作载荷(自治"最后一跳"的组帧, 2026-09-30 用户第 2 条)
 * ----------------------------------------------------------------------------
 * 布局与"待声光设备协议确认"的说明见 level_eval.h 里这一段的注释块。
 * 本文件只做【字节怎么排】, 一行射频/4G 代码都不碰 —— 所以能主机单测。
 * ========================================================================== */

/**
 * @brief 把 UTF-8 文本按"汉字 3 字节"塞进定长缓冲, 放不下就【在字符边界停】
 * @return 实际写入字节数
 * @note  与 utils 的 utf8_pack_3byte() 的区别: 那个"放不下就报错不截断",
 *        这个"塞得下多少算多少"。组帧必须用这个 —— 要么整字, 要么不放,
 *        绝不能出现半个汉字(设备侧乱码, 比少几个字难查得多)。
 */
static uint16_t _gear_pack_fit(const char *utf8, uint8_t *out, uint16_t cap)
{
    uint16_t w = 0U;

    if ((utf8 == NULL) || (out == NULL)) { return 0U; }

    while (*utf8 != '\0')
    {
        uint8_t clen = 0U;
        uint8_t i;

        if (!utf8_char_len(utf8, &clen)) { break; }   /* 非法就停, 不猜 */
        if ((uint16_t)(w + clen) > cap)  { break; }   /* 放不下: 整字不要 */
        for (i = 0U; i < clen; i++) { out[w++] = (uint8_t)utf8[i]; }
        utf8 += clen;
    }
    return w;
}

/**
 * @brief ★2026-10-08 新增★ 把云端的 0~3 对齐值折算成空口的 1 位
 *
 * 【为什么需要折算】两个口径不一致:
 *   · 云端 JSON(`ledTextShowMode: "1,2"`) 按 §1.9 是 0~3:
 *       0 默认 / 1 上对齐|左对齐 / 2 居中 / 3 下对齐|右对齐
 *   · 空口 `[3]` 每个半屏只有【1 位】, 文档原文是 "bit5=0居中/对齐":
 *       0 居中 / 1 对齐
 * 【折算规则(我方按语义取的, ★待与云端最终确认★)】
 *   0(默认) 和 2(居中) -> 0 居中
 *   1(上/左对齐) 和 3(下/右对齐) -> 1 对齐
 *   —— 即"是不是被显式要求对齐"。这是把 4 态压成 2 态唯一说得通的读法:
 *   0/2 都表示"居中", 1/3 都表示"靠边"。
 * ⚠ 若云端最终改成"0=居中, 非0=对齐"或别的规则, 只改这一个函数。 */
static uint8_t led_align_bit(uint8_t v)
{
    /* ★2026-10-08 更新: 云端口径已与我们对齐 —— 只有 0/1/2 三个值★
     *   0 = 默认, 1 = 对齐, 2 = 居中 (用户 2026-10-08 确认: "只有 0/1/2,
     *   分别是默认, 对齐, 居中, 我们不用管是左对齐什么的, 交给声光报警器处理")。
     *   空口 `[3]` 每位只有 1 bit(居中/对齐 两态), 所以:
     *       0(默认) -> 0(居中)   1(对齐) -> 1   2(居中) -> 0
     *   ⚠ 3 在云端口径里已不出现; 真收到就按"对齐"处理(保守: 至少是个显式取值),
     *     并且 config_store 的校验仍允许 0~3, 免得旧配置直接判非法。 */
    return ((v == 1U) || (v == 3U)) ? 1U : 0U;
}

/** 取两个 uint16 里的小者。
 *  ⚠ 不能直接写 `(room - 1U < budget) ? (room - 1U) : budget` —— room 是 uint16_t,
 *    参与运算会提升成 int, 与另一个操作数混用会触发 -Wsign-compare
 *    (实测在 ARMClang 上就是这样)。包一层既消警告又读得清楚。 */
static uint16_t min_u16(uint16_t a, uint16_t b)
{
    return (a < b) ? a : b;
}

uint8_t level_gear_build_payload(const cfg_gear_t *g, uint8_t volume,
                                 const char *led_text1, const char *led_text2,
                                 const char *tts_text,
                                 uint8_t *out, uint16_t cap)
{
    uint16_t pos;
    uint16_t n1, n2, nl;
    uint16_t room;
    uint16_t budget;                    /* 三段文本的剩余合计预算(见下方说明) */

    if ((g == NULL) || (out == NULL) || (cap < LEVEL_GEAR_HDR_LEN)) { return 0U; }
    if (cap > LEVEL_GEAR_PAYLOAD_MAX) { cap = LEVEL_GEAR_PAYLOAD_MAX; }

    /* ---- [0] 总电源: 档位 0(解除) 关屏 ---- */
    out[0] = (g->gear == 0U) ? 0U : 1U;

    /* ---- [1] 报警灯光模式: 0关闭 / 1常亮 / 2慢闪 / 3快闪 ----
     * ★2026-10-08 语义变更★ 原来 2 = "闪光+语音"; 现在语音由 tts 长度是否 0 决定,
     * 这一字节【只管灯光】。 */
    out[1] = (uint8_t)(g->sound_mode & 0x03U);

    /* ---- [2] 功放音量: 来自 devices[].volume(用户 2026-10-08 确认) ---- */
    out[2] = volume;

    /* ---- [3] LED屏模式(位定义见下方长注释) ---- */
    out[3] = (uint8_t)(
        ((uint32_t)(g->led_show_mode & 0x01U)        << 7) |   /* bit7 0横 / 1竖      */
        /* bit6 预留, 恒 0 */                                   /*                     */
        ((uint32_t)led_align_bit(g->led_text_show_mode[0]) << 5) |   /* bit5 半屏1 居中/对齐 */
        ((uint32_t)led_align_bit(g->led_text_show_mode[1]) << 4) |   /* bit4 半屏2 居中/对齐 */
        ((uint32_t)(g->led_color[0] & 0x03U)          << 2) |   /* bit3,2 半屏1 颜色    */
        ((uint32_t)(g->led_color[1] & 0x03U)));                 /* bit1,0 半屏2 颜色    */

    /* ---- 三段文本: 每段 = 1 字节长度 + 内容 ----
     * 布局(总长 = 7 + N + M + L, 上限 210):
     *     [4]      N = 文本1长度        (0 = 清屏不显示)
     *     [5..]    文本1 (N 字节 UTF-8)
     *     [5+N]    M = 文本2长度
     *     [6+N..]  文本2 (M 字节)
     *     [6+N+M]  K = TTS长度          (0 = 不播报)
     *     [7+N+M..] TTS (L 字节)
     * ⚠ 文本2 的起点是【6+N】不是 7+N —— 《声光报警器对接》那张表在这里多算了
     *   一个偏移, 用户 2026-10-08 确认为笔误。
     * ⚠ 每段都在【字符边界】截断(绝不出现半个汉字), 放不下就少放几个字。 */
    pos = LEVEL_GEAR_HDR_LEN;           /* = 4, 正好是 [4] 即文本1的长度域 */

    /* ★每段还要受"剩余文本预算"约束★
     * 光看 cap 是不够的: 若文本1 把 210 字节全吃掉, 后面两个长度字节就没地方放,
     * 函数只能返回 0(整帧失败)。所以这里用 LEVEL_GEAR_TEXT_BUDGET 钳住【三段合计】,
     * 保证 4 + 3 个长度字节永远放得下 —— 这也是 CFG_GEAR_TEXT_BUDGET 的本来含义。 */
    budget = LEVEL_GEAR_TEXT_BUDGET;

    /* 文本1 */
    room = (uint16_t)((cap > pos) ? (int)(cap - pos) : 0);
    if (room < 1U) { return 0U; }
    room = min_u16((uint16_t)(room - 1U), budget);
    n1 = _gear_pack_fit(led_text1, &out[pos + 1U], room);
    out[pos] = (uint8_t)n1;
    pos = (uint16_t)(pos + 1U + n1);
    budget = (uint16_t)(budget - n1);

    /* 文本2 */
    room = (uint16_t)((cap > pos) ? (int)(cap - pos) : 0);
    if (room < 1U) { return 0U; }
    room = min_u16((uint16_t)(room - 1U), budget);
    n2 = _gear_pack_fit(led_text2, &out[pos + 1U], room);
    out[pos] = (uint8_t)n2;
    pos = (uint16_t)(pos + 1U + n2);
    budget = (uint16_t)(budget - n2);

    /* TTS: ★空 = 不播报★(长度写 0), 不再看 sound_mode */
    room = (uint16_t)((cap > pos) ? (int)(cap - pos) : 0);
    if (room < 1U) { return 0U; }
    room = min_u16((uint16_t)(room - 1U), budget);
    nl = _gear_pack_fit(tts_text, &out[pos + 1U], room);
    out[pos] = (uint8_t)nl;
    pos = (uint16_t)(pos + 1U + nl);

    return (uint8_t)pos;
}

uint8_t level_failsafe_gear(uint8_t policy, uint8_t cur_gear)
{
    switch (policy)
    {
    case (uint8_t)CFG_OFFLINE_ALARM:  return 3u;   /* 强制升到告警档 */
    case (uint8_t)CFG_OFFLINE_OFF:    return 0u;   /* 执行解除档 */
    case (uint8_t)CFG_OFFLINE_KEEP:
    default:
        return cur_gear;                            /* 保持(含越界值: 不猜不误动) */
    }
}

uint8_t level_gear_collect_targets(const config_store_t *cfg, uint8_t gear,
                                   const char *only_uid,
                                   uint8_t *out_idx, uint8_t cap)
{
    const cfg_gear_t *g;
    uint8_t n = 0U;
    uint8_t i;

    if ((cfg == NULL) || (out_idx == NULL) || (gear >= (uint8_t)CFG_GEAR_COUNT))
    {
        return 0U;
    }
    g = &cfg->gears[gear];

    for (i = 0U; (i < cfg->device_count) && (i < (uint8_t)CFG_DEVICE_MAX); i++)
    {
        if (cfg->devices[i].enabled == 0U) { continue; }   /* 停用的不接收动作 */

        if (only_uid != NULL)
        {
            /* 手动控制(dev.ctrl): 只认指定的那一台 */
            if (strcmp(only_uid, cfg->devices[i].uid) != 0) { continue; }
        }
        else if (g->targets_all == 0U)                     /* 自动联动: 只发给列出来的 */
        {
            uint8_t j;
            bool    hit = false;

            for (j = 0U; (j < g->target_count) && (j < (uint8_t)CFG_DEVICE_MAX); j++)
            {
                if (strcmp(g->target_uid[j], cfg->devices[i].uid) == 0) { hit = true; break; }
            }
            if (!hit) { continue; }
        }

        if (n < cap) { out_idx[n] = i; }
        n++;                                               /* 计数可以超过 cap */
    }
    return n;
}
