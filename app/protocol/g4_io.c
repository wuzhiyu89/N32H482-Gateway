/**
 * \file  g4_io.c
 * \brief 4G 模块 AT 传输层: USART2 (PC8-TX / PC9-RX) + 统一接收泵 + 非阻塞 AT 命令引擎
 *
 * 设计说明见 g4_io.h 顶部。这里只强调三条实现约定:
 *   1) rb_g4 的【唯一消费者】是 g4_io_pump();
 *   2) publish URC 的 payload 是【定长原始字节】, 由字节级状态机按 <payload_len>
 *      收全, 期间完全不理会 CR/LF/0x00 —— 这是"下发被劈开/截断"的根治办法;
 *   3) 收包任何一步出错都进 RESYNC, 丢到下一个行边界重新开始, 绝不把整条 AT 流带偏。
 *
 * 数据通路:
 *   发送  g4_io_send_all  -> g4_uart_send_bytes (阻塞写 USART2)
 *   接收  USART2 RXDNE 中断 -> rb_g4 -> g4_io_pump() -> 行/payload -> URC 回调 + AT 引擎
 */
#include "g4_io.h"
#include "bsp_uart.h"
#include "bsp_tim.h"
#include "ring_buf.h"
#include <string.h>
#include "log.h"

/* ==================== 可调参数 ==================== */

/* G4_IO_LINE_BUF_SZ 现在定义在 g4_io.h(对外约束, ota_http.c 要拿它做编译期自检)。
 * 这里用 #ifndef 兜一下, 免得将来有人只改一处。 */
#ifndef G4_IO_LINE_BUF_SZ
#define G4_IO_LINE_BUF_SZ     4416u    /* 见 g4_io.h: 4352(下行法律上限) + 64 头余量 */
#endif

/* ★★2026-10-14: 行缓冲必须 >= 下行法律上限 ★★
 * 两者是 g4_io(字节泵) 与 g4_urc(下行队列) 之间的隐式契约:
 *   长度超过行缓冲的下行会在下面的字节泵里被【截断】—— 前半段被当成一条完整 URC
 *   ⇒ 截断的 JSON 解析失败(表现为平台看到 1001 bad request),
 *     后半段又不以 '{' 开头, 被判"不像 JSON"而落到 LoRa 透传路径
 *     (危险: 碎片若 <=240 字节就会被真的发到空口)。
 * 2026-10-14 就是这条契约被破坏(512 < 4352)导致平台的大配置下发全坏:
 *   一条 951 字节的 rule.set 被切成 489 + 462 两段。
 *
 * ⚠️ 这里是【数值】而不是符号: G4_URC_PUB_MAX 定义在 g4_urc.c 里(私有),
 *    g4_io 看不到它。曾经试过写 `_Static_assert(G4_IO_LINE_BUF_SZ >=
 *    G4_URC_PUB_MAX + 64u, ...)`, 直接编不过。
 *   ⇒ 所以这个不变式靠【两个文件的注释互指】维护:
 *       本文件 + g4_io.h 说"必须 >= 下行法律上限",
 *       g4_urc.c 的 G4_URC_PUB_MAX 说"改我要同步改 g4_io.h 的 G4_IO_LINE_BUF_SZ"。
 *   当前: 4352(下行上限) + 64(URC 头) = 4416 == G4_IO_LINE_BUF_SZ ✓ */
#define G4_IO_LINE_BUF_MUST_COVER_PUB   4416u
_Static_assert(G4_IO_LINE_BUF_SZ >= G4_IO_LINE_BUF_MUST_COVER_PUB,
               "G4_IO_LINE_BUF_SZ must cover the max downlink payload");
#define G4_UART_BAUD          115200u

/* 一条 "+MATREADY" 离我们最近一次"发送字节"有多近就算"紧跟命令后面"。
 * 这块模组会在每条 AT 命令的应答里再报一次 +MATREADY, 这类不算重启。 */
#ifndef G4_READY_AFTER_TX_MS
#define G4_READY_AFTER_TX_MS  3000u
#endif

/* 下行 publish 原始 payload 上限(AT+MQTTPUB 下行可到 320, 取整留余量) */
#ifndef G4_IO_PUB_BUF_SZ
/* ★★2026-10-09 由 384 提到 6208 —— 384 太小, 平台改联动档位发不进来★★
 *
 * 【现场 bug】平台发一条【格式化(带缩进换行)】的 rule.set, payload 397 字节:
 *     [G4] !! 下行收包中断(payload 长度非法): 已收 0/397 字节(主题 16 字符),
 *          丢弃并重同步; 累计 5 次
 *   同一内容发【紧凑 JSON】时是 310 字节 -> 完全成功。
 *   ⇒ 成败只取决于【大小】, 与换行/格式无关:
 *        397 > 384(G4_IO_PUB_BUF_SZ) -> 被这条 if 拒掉
 *        310 < 384                  -> 正常
 *   ⇒ 后果: 平台想【一次下发多个档位】(4 档 x 两段 LED 文字 + TTS)必然超过 384,
 *     于是"云端改联动档位"永远发不进来 —— 这是平台侧报的阻塞问题。
 *
 * 【为什么定 6208】下行 JSON 最终要进 main.c 的 jsonbuf[MAIN_CLOUD_JSON_MAX+1],
 *   而 MAIN_CLOUD_JSON_MAX = CFG_JSON_MAX(6144) + 64 = 6208。
 *   ⇒ 这里必须【至少】同样大, 否则"收到了却装不下"等于白收。
 *   两处必须同步改(见 config_store.h 的 CFG_JSON_MAX)。
 *
 * 【RAM 代价】+5824 字节(.bss)。本工程 RAM 用约 26%, 宽裕。
 *   这是"下行能收多大命令"的唯一闸门, 不能省。
 *
 * ⚠️ 与上行无关: 单条 MQTT 发布上限是另一个宏(G4_PUB_DATA_MAX, 在 g4_at.c)。 */
#define G4_IO_PUB_BUF_SZ      4352u
#endif
#define G4_IO_TOPIC_MAX       48u

/* 收包中途"多久没有新字节"就判定这条 URC 已残废: 放弃 + 重同步。
 * 115200 下 500ms 能来 5.7KB, 正常一条 payload 最多几十毫秒就收完,
 * 所以只要 500ms 没动静, 一定是丢字节/模组重启/被掐了。 */
#ifndef G4_IO_RX_GAP_TMO_MS
#define G4_IO_RX_GAP_TMO_MS   500u
#endif

#define G4_IO_RESP_MAX        384u     /* 累积应答上限 */
#define G4_IO_HIT_MAX         256u     /* 命中行快照上限 */

/* 收包路径日志限流: 每 G4_IO_RXLOG_WIN_MS 最多打 G4_IO_RXLOG_MAX 行。
 * 【为什么必须限流】115200 下日志是逐字节等 TXC 发出去的, 吞吐与来数据几乎同速,
 * 下行一快, "每条都打"的日志本身就成了瓶颈, 直接把环形缓冲顶爆。 */
#ifndef G4_IO_RXLOG_ENABLE
#define G4_IO_RXLOG_ENABLE    1
#endif
#define G4_IO_RXLOG_WIN_MS    100u
#define G4_IO_RXLOG_MAX       12u

/* "+MQTTURC: \"publish\"," —— 认到这一串就切定长收包 */
static const char G4_PUB_MAGIC[] = "+MQTTURC: \"publish\",";
#define G4_PUB_MAGIC_LEN  (sizeof(G4_PUB_MAGIC) - 1u)

/* ==================== 接收状态机 ==================== */

typedef enum
{
    RXS_LINE = 0,     /* 攒普通行 */
    RXS_PUB_HDR,      /* 正在按字节解析 publish 头(topic / <payload_len>) */
    RXS_PAYLOAD,      /* 正在收定长原始 payload */
    RXS_PROMPT,       /* 刚认到数据模式提示 '>', 吃掉尾随空白 */
    RXS_RESYNC        /* 出错后丢到下一个行边界 */
} g4_rx_state_t;

static g4_rx_state_t s_rx;
static char          s_line[G4_IO_LINE_BUF_SZ];
static uint16_t      s_line_len;
static bool          s_magic_ok;      /* 行首是否还在匹配 publish 头前缀 */

static char          s_pub_topic[G4_IO_TOPIC_MAX];
static uint8_t       s_pub_topic_len;
static uint8_t       s_pub_buf[G4_IO_PUB_BUF_SZ];
static uint16_t      s_pub_need;
static uint16_t      s_pub_have;
/* ★★2026-10-14 新增: 跨 URC 分块重组 ★★
 * 模块把一条长 publish 【按 512 字节左右分包】, 每包一条 URC, 形如:
 *     +MQTTURC: "publish",0,1,"<topic>",<total_len>,<plen>,<payload...>
 * 其中 <total_len> = 整条消息总长(每包都相同), <plen> = 【本包】长度。
 * 实测: 一条 1441 字节的 rule.set 被拆成 plen = 489 / 512 / 440 三包
 *       (第一包 489 是因为前面还有 URC 头, 489+23 = 512)。
 * ⇒ 必须攒够 <total_len> 才交付一次, 否则每一包都会被当成一条独立下行:
 *     第 1 包 -> 截断的 JSON -> 平台收到 1001 bad request
 *     后续包 -> 不以 '{' 开头 -> 被判"不像 JSON" -> 落到 LoRa 透传路径
 *               (危险: 若某包 <=240 字节, 会被真的发到空口)
 * 旧代码在 [g4_io.c 第 3 个逗号处] 把 s_pub_need 直接设成 <plen>, 并在
 * RXS_PAYLOAD 收满 <plen> 就交付 —— 那正是上面这个错误行为的来源。
 * 原注释说"<total_len> 在分包时比分片长度大, 拿错就会收多/收少", 这句本身对,
 * 但结论应是"用 <plen> 读【本包】, 用 <total_len> 判断【攒够了没有】"。 */
static uint16_t      s_pub_total;      /* 本【条消息】总长(URC 第 2 个字段) */
static uint16_t      s_pub_chunk_left; /* 本【包】还剩几个字节没收 */

static uint8_t       s_hdr_quotes;    /* 头里已见到的 '"' 个数(1=主题正文中, 2=主题已结束) */
static uint8_t       s_hdr_commas;    /* 主题结束后的逗号数(1=total_len末, 2=payload_len末, 3=payload起) */
static uint32_t      s_hdr_num;       /* 当前数字字段的累加值 */

static uint32_t      s_rx_last_tick;  /* 收到最近一个字节的时刻 */
static uint32_t      s_last_tx_tick;  /* 最近一次往模组发字节的时刻 */

/* 诊断计数 */
static uint32_t      s_ready_urc_cnt;      /* 无缘无故冒出来的 +MATREADY/RDY */
static uint32_t      s_ready_after_tx_cnt; /* 紧跟命令后面的 +MATREADY/RDY */
static uint32_t      s_dl_abort_cnt;       /* 下行收包中途放弃次数 */
static uint32_t      s_log_drop_cnt;       /* 被限流丢掉的日志行数 */
static uint32_t      s_log_win_tick;
static uint8_t       s_log_win_cnt;

/* URC 回调(由 g4_urc 注册) */
static g4_io_urc_line_cb_t s_urc_line_cb;
static g4_io_urc_pub_cb_t  s_urc_pub_cb;

/* 泵忙标志: 防止 URC 回调里再调 g4_io_pump() 造成重入 */
static bool          s_in_pump;

/* ==================== 非阻塞 AT 命令引擎 ====================
 *
 * 【必须区分"空闲"和"刚完成但结果还没被取走"】
 * 收包泵是在主循环每拍开头跑的, 也就是说命令很可能在【调用方还没来取】的时候
 * 就已经完成了。老写法只有一个 bool:
 *     泵里命中关键字 -> s_cmd_running = false
 *     下一拍调用方再调 g4_io_cmd_run() -> 看到 false -> 又开一条新命令
 *                                  -> 刚拿到的 OK 被覆盖成 RUNNING
 * 结果: 调用方永远看不到 OK, AT_SYNC 里就以【每 5ms 一条】的速度狂发 AT
 * (实测 200 条/秒, 一上电就停不下来)。这就是它。
 *
 * 现在用三态:
 *     ENG_IDLE    没有命令, 下次调用会开一条新的
 *     ENG_RUNNING 在途, 调用方继续等
 *     ENG_DONE    结果已经放在 s_cmd_result 里, 【下一个来问的人拿走它】
 */
typedef enum
{
    G4_ENG_IDLE = 0,
    G4_ENG_RUNNING,
    G4_ENG_DONE
} g4_eng_state_t;

static g4_eng_state_t s_eng = G4_ENG_IDLE;
static g4_cmd_res_t   s_cmd_result = G4_CMD_OK;
static const char   *s_cmd_kw;
static bool          s_cmd_prompt;         /* true = 等严格的 '>' 整行 */
static uint32_t      s_cmd_tick;
static uint32_t      s_cmd_timeout;
static char          s_cmd_resp[G4_IO_RESP_MAX];
static uint16_t      s_cmd_resp_used;
static char          s_cmd_hit[G4_IO_HIT_MAX];

/* ★2026-10-07: 这条命令是【谁】开的 —— 见 g4_io.h 里"命令引擎的属主"整段说明。
 * 只有属主本人能取走结果/触发超时; 外人 poll 一律只拿到 RUNNING。 */
static g4_cmd_owner_t s_cmd_owner = G4_OWNER_NONE;

/* ★2026-10-07: "引擎被真正占用过"的单调计数★
 * 每次 g4_io_cmd_begin_ex() 真的拿到引擎(空闲->忙)时自增。
 * 调用方在发命令前后各读一次就能判断: 【我这条到底发出去没有】。
 * 用途见 ota_http.c 的 conn_cmd() —— 它靠这个挡住"没发出去却拿到结果"那种
 * 会静默跳过一个配置步骤的情况(现场代价: ssl 那步被跳过 -> https 请求回
 * +CME ERROR: 655 = SSL 未使用)。 */
static uint32_t s_cmd_tx_seq;

/* 结果快照: 返回终态后一直有效, 直到下一条命令发出, 调用方可以放心地用 */
static char          s_res_snap[G4_IO_RESP_MAX];
static char          s_hit_snap[G4_IO_HIT_MAX];

/* ==================== 小工具 ==================== */

/** @brief 一行(前 len 个字节)是不是只有空白 */
static bool g4_io_line_all_blank(const char *line, uint16_t len)
{
    uint16_t i;

    for (i = 0u; i < len; i++)
    {
        if ((line[i] != ' ') && (line[i] != '\t'))
        {
            return false;
        }
    }
    return true;
}

/** @brief 一行是不是"数据模式提示 '>'"——只允许 '>' 和空白, 且至少一个 '>' */
static bool g4_io_is_prompt(const char *line)
{
    bool got = false;

    while (*line != '\0')
    {
        if (*line == '>')
        {
            got = true;
        }
        else if ((*line != ' ') && (*line != '\t'))
        {
            return false;
        }
        line++;
    }
    return got;
}

/** @brief 收包路径日志(限流), 避免"日志把接收顶爆" */
static void g4_io_rx_log(const char *line)
{
#if G4_IO_RXLOG_ENABLE
    uint32_t now = get_tick_ms();

    if ((uint32_t)(now - s_log_win_tick) >= G4_IO_RXLOG_WIN_MS)
    {
        s_log_win_tick = now;
        s_log_win_cnt  = 0u;
    }
    if (s_log_win_cnt >= G4_IO_RXLOG_MAX)
    {
        s_log_drop_cnt++;
        return;
    }
    s_log_win_cnt++;
    log_info("[G4_RX] %s\r\n", line);
#else
    (void)line;
#endif
}

/** @brief 把一条行累积进在途命令的应答缓冲(装不下就整行丢) */
static void g4_io_resp_append(const char *line)
{
    uint16_t l = (uint16_t)strlen(line);

    if ((uint16_t)(s_cmd_resp_used + l + 2u) >= G4_IO_RESP_MAX)
    {
        return;
    }
    if (s_cmd_resp_used > 0u)
    {
        s_cmd_resp[s_cmd_resp_used++] = '\n';
    }
    memcpy(&s_cmd_resp[s_cmd_resp_used], line, l);
    s_cmd_resp_used = (uint16_t)(s_cmd_resp_used + l);
    s_cmd_resp[s_cmd_resp_used] = '\0';
}

/** @brief 命令收官: 把应答/命中行做成快照, 结果置为"待取走" */
static void g4_io_cmd_finish(void)
{
    s_eng = G4_ENG_DONE;

    memcpy(s_res_snap, s_cmd_resp, sizeof(s_res_snap));
    s_res_snap[sizeof(s_res_snap) - 1u] = '\0';

    memcpy(s_hit_snap, s_cmd_hit, sizeof(s_hit_snap));
    s_hit_snap[sizeof(s_hit_snap) - 1u] = '\0';
}

/** @brief 记录命中行 */
static void g4_io_cmd_hit(const char *line)
{
    strncpy(s_cmd_hit, line, sizeof(s_cmd_hit) - 1u);
    s_cmd_hit[sizeof(s_cmd_hit) - 1u] = '\0';
}

/* ==================== 下行收包出错: 放弃并重同步 ==================== */

/**
 * @brief 放弃当前这条 URC 并进入 RESYNC
 *
 * 【这是老版本最大的坑的解药】: 老版本收不满就一直在原地等, 结果把后面所有
 * 字节(包括下一条 URC)都当 payload 吃掉, 永久失步。现在只要超时/畸形就立刻
 * 放弃、丢到下一个行边界, 最坏损失一条消息。
 */
static void g4_io_dl_abort(const char *why)
{
    s_dl_abort_cnt++;

    /* 只在头几次和每 20 次打一行, 免得"持续被截断"时日志反过来把接收压死 */
    if ((s_dl_abort_cnt <= 5u) || ((s_dl_abort_cnt % 20u) == 0u))
    {
        log_info("[G4] !! 下行收包中断(%s): 已收 %u/%u 字节(主题 %u 字符), "
                 "丢弃并重同步; 累计 %lu 次\r\n",
                 why, (unsigned)s_pub_have, (unsigned)s_pub_need,
                 (unsigned)s_pub_topic_len, (unsigned long)s_dl_abort_cnt);
    }

    s_rx            = RXS_RESYNC;
    s_line_len      = 0u;
    s_magic_ok      = true;
    s_pub_have      = 0u;
    s_pub_need      = 0u;
    s_pub_topic_len = 0u;
}

/* ==================== 行处理 ==================== */

static void g4_io_process_line(void)
{
    s_line[s_line_len] = '\0';

    /* ---- 模组"起动完成"计数(必须放在分派之前, 与谁消费这行无关) ----
     *
     * ⚠️ 2026-09-22 修正(依据现场日志 78.txt):
     *   老版本把"离最近一次发送 ≤ G4_READY_AFTER_TX_MS"的 +MATREADY 一律归到
     *   "模组应答习惯", 不计入复位计数。可注册阶段的轮询间隔 AT_CEREG_POLL_MS
     *   (2000ms)【比这个窗口(3000ms)还短】—— 于是那段时间里每一条 +MATREADY
     *   都被判成"习惯", 真的复位一次也数不到。现场那 4 分钟里 +MATREADY 明明
     *   出现了 4 次(而且两次是"MQTTCONN 哑了 5 秒后重试才回的"), 而
     *   [STAT2] 的"模组自复位(无缘无故)"从头到尾是 0, 就是它造成的。
     *
     * 现在: 无条件【每次都打一行】, 带上"距上次发送多少毫秒"。
     *   分类(≤3s / >3s)只保留作统计口径, 不再当作"是不是真复位"的判决 ——
     *   判"模组重启"要看的是"有没有第二份证据"(AT 哑掉、注册丢失、IP 没了),
     *   而不是掐秒表。 */
    if ((strstr(s_line, "MATREADY") != NULL) || (strcmp(s_line, "RDY") == 0))
    {
        uint32_t since_tx = (uint32_t)(get_tick_ms() - s_last_tx_tick);

        if (since_tx <= G4_READY_AFTER_TX_MS)
        {
            if (s_ready_after_tx_cnt < 0xFFFFFFFFu) { s_ready_after_tx_cnt++; }
        }
        else if (s_ready_urc_cnt < 0xFFFFFFFFu)
        {
            s_ready_urc_cnt++;
        }

        log_info("[G4] ** 模组报出 \"%s\": 距上次向模组发送 %lums | 累计 %lu 次"
                 "(其中 ≤%ums 的 %lu 次) —— 这是模组 AT 应用层重启过的证据\r\n",
                 s_line, (unsigned long)since_tx,
                 (unsigned long)(s_ready_urc_cnt + s_ready_after_tx_cnt),
                 (unsigned)G4_READY_AFTER_TX_MS,
                 (unsigned long)s_ready_after_tx_cnt);
    }

    /* ---- 1) URC 层先吃: +MQTTURC: 家族一律不参与 AT 应答匹配 ---- */
    if ((s_urc_line_cb != NULL) && s_urc_line_cb(s_line, s_line_len))
    {
        return;     /* URC 层自己会打需要的日志 */
    }

    /* ---- 2) AT 应答匹配 ---- */
    if (s_eng == G4_ENG_RUNNING)
    {
        bool hit = false;

        g4_io_resp_append(s_line);

        if (s_cmd_kw != NULL)
        {
            hit = s_cmd_prompt ? g4_io_is_prompt(s_line)
                               : (strstr(s_line, s_cmd_kw) != NULL);
        }

        if (hit)
        {
            g4_io_cmd_hit(s_line);
            s_cmd_result = G4_CMD_OK;
            g4_io_cmd_finish();
            g4_io_rx_log(s_line);
            return;
        }

        /* 【提示符丢失】: 等数据模式提示 '>' 时, 模组却先回了 OK。
         *
         * "AT+MQTTPUB=...,<len>" 这种二段式发布, 在 '>' 之前【不该】有 OK ——
         * 出现 OK 只可能是模组的 '>' 没送到(固件 URC 与 AT 应答抢串口资源时会丢),
         * 而这时它【很可能已经进了数据模式在等我们发 payload】。
         * 如果我们就这么放着, 后面每发一条 AT 都会被它当 payload 吃掉 —— AT 流
         * 会脏掉, 表现就是"AT 还能应答但什么都不对 / 搜不到网"。
         * 所以这里立刻判失败, 并且让上层【直接断电重启模组】(见 g4_at.c pub_poll):
         * 断电是唯一不依赖 AT 通道的恢复手段。 */
        if (s_cmd_prompt && (strcmp(s_line, "OK") == 0))
        {
            g4_io_cmd_hit(s_line);
            s_cmd_result = G4_CMD_NOPROMPT;
            g4_io_cmd_finish();
            log_info("[G4] !! 数据模式提示 '>' 丢失: 模组直接回了 OK "
                     "(多半是固件 URC 抢了串口) —— AT 数据模式状态已不可信\r\n");
            return;
        }

        /* 【ERROR 必须独立判定, 不能套在 kw != NULL 里面】
         * 等数据模式提示 '>'(g4_io_cmd_run_prompt)时 s_cmd_kw 是 NULL, 老写法于是
         * 既不认提示也不认 ERROR —— 模组明明回了 ERROR, 我们却只能干等满 15s 超时。
         * 发布失败被拖慢、断网判定变迟钝, 这就是一个直接原因。
         * 注意 "ERROR" 也覆盖 "+CME ERROR" / "+CMS ERROR"。 */
        if (strstr(s_line, "ERROR") != NULL)
        {
            g4_io_cmd_hit(s_line);
            s_cmd_result = G4_CMD_ERR;
            g4_io_cmd_finish();
            g4_io_rx_log(s_line);
            return;
        }

        g4_io_rx_log(s_line);
        return;
    }

    /* ---- 3) 没有在途命令: 无请求行, 打出来(限流) ---- */
    g4_io_rx_log(s_line);
}

/* ==================== 字节级状态机 ==================== */

static void g4_io_rx_byte(uint8_t ch)
{
    switch (s_rx)
    {
    /* ---------------- 普通行 ---------------- */
    case RXS_LINE:
        if ((ch == '\r') || (ch == '\n'))
        {
            if (s_line_len > 0u)
            {
                g4_io_process_line();
            }
            s_line_len = 0u;
            s_magic_ok = true;
            return;
        }

        /* 【数据模式提示 '>' —— 必须按字节认, 不能等整行】
         * 模组给的是 "\r\n> ", 后面【没有换行】:
         *     AT+MQTTPUB=...\r\n
         *     \r\n>            <- 到这里它就开始等我们的 payload 了
         * 如果按"整行结束"去匹配 '>', 这条提示永远等不到 → 每次发布都超时 →
         * 连续失败 → 判链路死重建。这正是一个"保活失败"的隐藏成因。
         *
         * 判定条件(两条都要):
         *   1) 该行从行首到现在【全是空白】(所以脏行 "GARBAGE>MORE" 不算);
         *   2) 当前确实有一条"在等 '>' 提示"的命令。
         * 认到之后把尾随的空白吃掉, 免得它们和下一行粘在一起。 */
        if ((ch == '>') && g4_io_line_all_blank(s_line, s_line_len) &&
            (s_eng == G4_ENG_RUNNING) && s_cmd_prompt)
        {
            g4_io_cmd_hit(">");
            s_cmd_result = G4_CMD_OK;
            g4_io_cmd_finish();
            s_line_len = 0u;
            s_magic_ok = true;
            s_rx       = RXS_PROMPT;
            return;
        }

        if (s_line_len < (G4_IO_LINE_BUF_SZ - 1u))
        {
            s_line[s_line_len++] = (char)ch;
        }
        else
        {
            /* 行超长(理论上不该发生): 先交付已有的, 别把整条流卡死 */
            g4_io_process_line();
            s_line_len = 0u;
            s_line[s_line_len++] = (char)ch;
        }

        /* 边收边认 publish 头前缀: 认出来立刻切 PUB_HDR, 这样 payload 的
         * 0x0A/0x0D 永远不会被当行尾(老版本就是死在这) */
        if (s_magic_ok && (s_line_len <= G4_PUB_MAGIC_LEN))
        {
            if (s_line[s_line_len - 1u] != G4_PUB_MAGIC[s_line_len - 1u])
            {
                s_magic_ok = false;
            }
            else if (s_line_len == G4_PUB_MAGIC_LEN)
            {
                s_rx            = RXS_PUB_HDR;
                s_hdr_quotes    = 0u;
                s_hdr_commas    = 0u;
                s_hdr_num       = 0u;
                s_pub_topic_len = 0u;
                s_pub_need      = 0u;
                /* ⚠️2026-10-14: 【不要】在这里清 s_pub_have / s_pub_total ——
                 *   每【包】都会走到这里(每包都是一条独立 URC), 一清就等于把
                 *   上一包攒的字节丢掉, 分块重组就永远做不成。
                 *   清零只发生在两处: ① 交付成功后(RXS_PAYLOAD);
                 *   ② 头里发现 <total_len> 与正在攒的不同(= 新消息)。 */
                s_pub_chunk_left = 0u;
            }
        }
        return;

    /* ---------------- publish 头 ---------------- */
    case RXS_PUB_HDR:
        if ((ch == '\r') || (ch == '\n'))
        {
            g4_io_dl_abort("publish 头里出现换行");
            return;
        }
        if (s_line_len < (G4_IO_LINE_BUF_SZ - 1u))
        {
            s_line[s_line_len++] = (char)ch;
        }
        else
        {
            g4_io_dl_abort("publish 头过长");
            return;
        }

        if (ch == '"')
        {
            s_hdr_quotes++;
            if (s_hdr_quotes == 1u)
            {
                s_pub_topic_len = 0u;       /* 主题正文开始 */
            }
            else if (s_hdr_quotes >= 2u)
            {
                s_hdr_commas = 0u;          /* 主题结束, 开始数后面 3 个逗号 */
            }
            return;
        }

        if (s_hdr_quotes == 1u)
        {
            /* 主题正文(里面可能有逗号, 所以放行) */
            if (s_pub_topic_len < (G4_IO_TOPIC_MAX - 1u))
            {
                s_pub_topic[s_pub_topic_len++] = (char)ch;
            }
            return;
        }

        if (s_hdr_quotes >= 2u)
        {
            if ((ch >= '0') && (ch <= '9'))
            {
                s_hdr_num = (s_hdr_num * 10u) + (uint32_t)(ch - '0');
                return;
            }
            if (ch == ',')
            {
                s_hdr_commas++;
                if (s_hdr_commas == 1u)
                {
                    s_hdr_num = 0u;     /* <total_len> 开始 */
                }
                else if (s_hdr_commas == 2u)
                {
                    /* ★2026-10-14: <total_len> 结束 —— 【不再丢弃】。
                     * 它是"整条消息总长", 是判断"攒够了没有"的唯一依据。
                     * 若与正在攒的那条不同 => 这是一条【新消息】, 上一条残缺, 从头来。 */
                    if (s_pub_total != s_hdr_num)
                    {
                        s_pub_have = 0u;        /* 新消息: 丢掉上一条的残留 */
                        s_pub_total = (uint16_t)s_hdr_num;
                    }
                    s_hdr_num = 0u;     /* 开始收 <payload_len> */
                }
                else
                {
                    /* 第 3 个逗号: <payload_len>(本包长度)结束, 后面紧跟 payload 第一个字节。
                     * ★2026-10-14: s_pub_need 用【总长】(交付判据), 本包长度另存
                     *   s_pub_chunk_left(读满本包后回 RXS_LINE 等下一包的头)。 */
                    uint16_t chunk = (uint16_t)s_hdr_num;
                    s_hdr_num = 0u;

                    s_pub_topic[s_pub_topic_len] = '\0';

                    /* 一致性校验: 总长合法, 且 已攒 + 本包 <= 总长 */
                    if ((s_pub_total == 0u) || (s_pub_total > G4_IO_PUB_BUF_SZ) ||
                        (chunk == 0u) ||
                        (((uint32_t)s_pub_have + (uint32_t)chunk) > (uint32_t)s_pub_total))
                    {
                        g4_io_dl_abort("publish 分块长度不一致/超上限");
                        return;
                    }

                    s_pub_need       = s_pub_total;   /* 攒够这么多才交付 */
                    s_pub_chunk_left = chunk;         /* 本包还要读这么多 */
                    s_rx             = RXS_PAYLOAD;
                }
            }
        }
        return;

    /* ---------------- 定长原始 payload ---------------- */
    case RXS_PAYLOAD:
        /* ★2026-10-14: 按【本包】长度收字节, 但按【总长】决定何时交付。
         * 每包的 payload 追加到 s_pub_buf 的 s_pub_have 偏移处(跨包累加)。 */
        if (s_pub_have < s_pub_need)
        {
            s_pub_buf[s_pub_have++] = ch;
        }
        if (s_pub_chunk_left > 0u)
        {
            s_pub_chunk_left--;
        }
        if (s_pub_chunk_left == 0u)     /* 本包收满 */
        {
            if (s_pub_have >= s_pub_need)   /* 整条消息收满 -> 交付【一次】 */
            {
                if (s_urc_pub_cb != NULL)
                {
                    s_urc_pub_cb(s_pub_topic, s_pub_buf, s_pub_need);
                }
                s_pub_have  = 0u;
                s_pub_total = 0u;
            }
            /* 无论交付与否都回到行模式: 要么等下一条消息, 要么等本条下一包的头。
             * ⚠️ s_pub_have 【不在这里清零】—— 它是跨包累加的进度; 只有交付成功,
             *   或遇到"total 变了"(新消息)时才清。 */
            s_rx       = RXS_LINE;
            s_line_len = 0u;
            s_magic_ok = true;
            s_pub_need = 0u;
        }
        return;

    /* ---------------- 提示尾随空白(吃掉到下一个行边界) ---------------- */
    case RXS_PROMPT:
        if ((ch == '\r') || (ch == '\n'))
        {
            s_rx       = RXS_LINE;
            s_line_len = 0u;
            s_magic_ok = true;
        }
        return;

    /* ---------------- 出错后的重同步 ---------------- */
    case RXS_RESYNC:
    default:
        if (ch == '\n')
        {
            s_rx       = RXS_LINE;
            s_line_len = 0u;
            s_magic_ok = true;
        }
        return;
    }
}

/* ==================== 接收泵 ==================== */

void g4_io_pump(void)
{
    uint8_t ch;

    if (s_in_pump)
    {
        return;     /* URC 回调里又调进来了: 直接返回, 绝不重入 */
    }
    s_in_pump = true;

    while (ring_buf_pop(&rb_g4, &ch))
    {
        s_rx_last_tick = get_tick_ms();
        g4_io_rx_byte(ch);
    }

    /* 排空之后才判"收包卡住": 环形缓冲空着还等不到后续字节, 就是真丢了 */
    if ((s_rx == RXS_PUB_HDR) || (s_rx == RXS_PAYLOAD))
    {
        if ((uint32_t)(get_tick_ms() - s_rx_last_tick) > G4_IO_RX_GAP_TMO_MS)
        {
            g4_io_dl_abort("等后续字节超时");
        }
    }

    s_in_pump = false;
}

bool g4_io_in_pump(void)
{
    return s_in_pump;
}

void g4_io_set_urc_cb(g4_io_urc_line_cb_t on_line, g4_io_urc_pub_cb_t on_pub)
{
    s_urc_line_cb = on_line;
    s_urc_pub_cb  = on_pub;
}

/* ★2026-10-07: 属主"死账"回收窗口★
 *
 * 【为什么必须有它】属主保护(只有属主能取结果/判超时)堵死了"互相偷结果",
 * 但漏了一路: 【属主自己不再来 poll】。那种情况下引擎上那条命令:
 *   · 没人替它判超时(g4_io_cmd_poll 只有属主会调)
 *   · 外人也拿不走(属主保护把他挡住)
 *   -> s_eng 永远不是 IDLE -> 整机【一条 AT 都发不出去】。
 *
 * 2026-10-07 18:21 现场实测就是这个死锁:
 *   18:21:59.603 [OTA_HTTP] 暂存区已擦除, 开始建实例
 *   (之后 2 分多钟: 整个日志再没有一条 [G4_TX])
 *   18:22:29 / 22:59 / 23:29 / 23:59  [G4] 发布放弃(命令引擎被外部流程(OTA)长时间占用)
 *   puback=0 从头到尾; ota.progress 一直 downloading/0
 *
 * 判据: 引擎非空闲, 且【距 s_cmd_tick 已经超过 s_cmd_timeout + 本窗口】。
 * 正常属主每拍都来 poll, 会在 s_cmd_timeout 那一刻就拿到 TIMEOUT 并释放;
 * 结果没人取(DONE)也会在下一拍被取走。所以能闲置到这个窗口的, 一定是死账。
 * 窗口给足余量, 不会误伤正在正常推进的命令。 */
#ifndef G4_IO_OWNER_GRACE_MS
#define G4_IO_OWNER_GRACE_MS   5000u
#endif

/*!
 * \brief 回收"没人推进"的死账命令(见上面 G4_IO_OWNER_GRACE_MS 的整段说明)
 * \note  只清引擎状态, 不产生任何 AT。可以放心地在"准备发命令/查询引擎状态"
 *        的入口处调用 —— 这一条就是把"死锁"变成"最多卡 15 秒"。
 */
static void g4_io_reap_stale(void)
{
    if (s_eng == G4_ENG_IDLE)
    {
        return;
    }
    if ((uint32_t)(get_tick_ms() - s_cmd_tick)
            < (s_cmd_timeout + (uint32_t)G4_IO_OWNER_GRACE_MS))
    {
        return;
    }

    log_info("[G4] !! 命令引擎上的命令(属主=%d)已 %lums 无人推进 —— 判死账, 强制收回"
             "(否则整机一条 AT 都发不出去)\r\n",
             (int)s_cmd_owner,
             (unsigned long)(uint32_t)(get_tick_ms() - s_cmd_tick));
    g4_io_cmd_abort();
}

/* ==================== 非阻塞命令引擎实现 ==================== */

void g4_io_cmd_abort(void)
{
    s_eng         = G4_ENG_IDLE;
    s_cmd_result  = G4_CMD_TIMEOUT;
    s_cmd_resp[0] = '\0';
    s_cmd_hit[0]  = '\0';
    s_cmd_resp_used = 0u;
}

bool g4_io_cmd_active(void)
{
    return (s_eng == G4_ENG_RUNNING);
}

bool g4_io_cmd_idle(void)
{
    /* 顺带清死账: 调用方拿本函数当"引擎能用了吗"的判据, 死账会让它永远 false */
    g4_io_reap_stale();
    return (s_eng == G4_ENG_IDLE);
}

/** @brief 内部: 带"等 '>' 提示"标志和属主的开命令 */
static void g4_io_cmd_begin_ex(const char *cmd, const char *kw, uint32_t timeout_ms,
                               bool prompt, g4_cmd_owner_t owner)
{
    /* ★2026-10-07: 判据从 RUNNING 收紧到 IDLE★
     * 老写法只挡 RUNNING, 于是 DONE(命令已完成、结果还没被调用方取走)会被
     * 【直接覆盖】—— 那个结果就永远看不到了, 而且新命令的归属也说不清。
     * 走 g4_io_cmd_run_owned() / g4_io_cmd_run_prompt() 的路径都在外面
     * 处理过 DONE 了, 所以这里只剩"没确认就硬发"的调用方(public 的
     * g4_io_cmd_begin), 对它们一律不打扰 —— 想发先确认 g4_io_cmd_idle()。 */
    if (s_eng != G4_ENG_IDLE)
    {
        return;     /* 有命令在途【或结果未被取走】: 绝不打断、绝不覆盖 */
    }

    s_eng           = G4_ENG_RUNNING;
    s_cmd_owner     = owner;        /* ★2026-10-07: 记下这条命令归谁★ */
    s_cmd_tx_seq++;                 /* ★"引擎真的被占用了"——调用方靠它判断发没发★ */
    s_cmd_kw        = kw;
    s_cmd_prompt    = prompt;
    s_cmd_timeout   = (timeout_ms == 0u) ? 1u : timeout_ms;
    s_cmd_tick      = get_tick_ms();
    s_cmd_resp_used = 0u;
    s_cmd_resp[0]   = '\0';
    s_cmd_hit[0]    = '\0';
    s_cmd_result    = G4_CMD_RUNNING;

    if (cmd != NULL)
    {
        /* 【可视性】: 每条命令都留痕, 否则"到底发没发、发的什么"现场看不到 */
        log_info("[G4_TX] %s\r\n", cmd);
        g4_io_send_cmd(cmd);
    }
}

void g4_io_cmd_begin(const char *cmd, const char *keyword, uint32_t timeout_ms)
{
    g4_io_cmd_begin_ex(cmd, keyword, timeout_ms, false, G4_OWNER_NONE);
}

g4_cmd_res_t g4_io_cmd_poll(void)
{
    /* 【结果待取走】: 把它交给这次来问的人, 然后回到空闲 */
    if (s_eng == G4_ENG_DONE)
    {
        s_eng = G4_ENG_IDLE;
        return s_cmd_result;
    }
    if (s_eng != G4_ENG_RUNNING)
    {
        return s_cmd_result;        /* 没有在途命令 */
    }

    g4_io_pump();

    if ((s_eng == G4_ENG_RUNNING) &&
        ((uint32_t)(get_tick_ms() - s_cmd_tick) >= s_cmd_timeout))
    {
        s_cmd_result = G4_CMD_TIMEOUT;
        g4_io_cmd_finish();
        s_eng = G4_ENG_IDLE;        /* 超时结果直接交付 */
    }
    return s_cmd_result;
}

/*!
 * \brief 带属主的开命令/收结果 —— 见 g4_io.h 里"命令引擎的属主"整段说明
 *
 * 【一句话】引擎上的命令不属于本人时, 本函数【什么都不做】, 只回 RUNNING:
 *   不偷它的结果、不替它判超时、更不会把它的引擎置成 IDLE。
 *
 * 【为什么要这一条 —— 2026-10-07 OTA 现场实测】
 *   老写法(无属主)下, 两个调用方共用 g4_io_cmd_poll():
 *     OTA 发 AT+MHTTPCFG(超时 10s) -> 发布流程也来 poll
 *       -> 到点 poll 把 s_cmd_result 置 TIMEOUT 并 s_eng = IDLE
 *       -> 发布流程把这个 TIMEOUT 当成【自己】"等 '>' 超时" -> 连 3 次
 *       -> 判"链路已断" -> 4G_RST 断电重启模组
 *       -> 正在配 HTTP 实例的 OTA 拿到 +CME ERROR: 652 -> 失败(2002), percent 恒 0
 *   加了属主之后, 外人的 poll 只会拿到 RUNNING, 上面整条链断在第一步。
 */
g4_cmd_res_t g4_io_cmd_run_owned(const char *cmd, const char *keyword,
                                 uint32_t timeout_ms, g4_cmd_owner_t owner)
{
    /* ★先清死账★: 属主失联的命令绝不能把引擎锁死(见 g4_io_reap_stale) */
    g4_io_reap_stale();

    /* ★属主保护★ 引擎不是空闲(IDLE)时, 先看这条命令是不是自己的 */
    if ((s_eng != G4_ENG_IDLE) && (s_cmd_owner != owner))
    {
        return G4_CMD_RUNNING;      /* 别人的命令: 既不偷结果, 也不替它判超时 */
    }

    if (s_eng == G4_ENG_RUNNING)
    {
        return g4_io_cmd_poll();
    }
    if (s_eng == G4_ENG_DONE)
    {
        /* ★关键★ 上一条命令(可能是在收包泵里完成的)的结果还没被取走:
         * 先把结果交给调用方, 【绝不在这里又开一条新命令】—— 那样会把这个
         * 结果覆盖成 RUNNING, 调用方永远看不到 OK。 */
        s_eng = G4_ENG_IDLE;
        return s_cmd_result;
    }

    g4_io_cmd_begin_ex(cmd, keyword, timeout_ms, false, owner);
    return g4_io_cmd_poll();
}

g4_cmd_res_t g4_io_cmd_run(const char *cmd, const char *keyword, uint32_t timeout_ms)
{
    return g4_io_cmd_run_owned(cmd, keyword, timeout_ms, G4_OWNER_NONE);
}

g4_cmd_owner_t g4_io_cmd_owner(void)
{
    /* ★先清死账★: 否则外人会一直以为"引擎被别人占着", 从而永久停摆 */
    g4_io_reap_stale();

    /* 引擎空闲时【不算任何人持有】—— s_cmd_owner 会留着上一条的值, 不能直接返回它,
     * 否则调用方会把"空闲"误读成"被 OTA 占着"。 */
    return (s_eng == G4_ENG_IDLE) ? G4_OWNER_NONE : s_cmd_owner;
}

uint32_t g4_io_cmd_tx_seq(void)
{
    return s_cmd_tx_seq;
}

void g4_io_cmd_release(g4_cmd_owner_t owner)
{
    if ((s_eng != G4_ENG_IDLE) && (s_cmd_owner == owner))
    {
        g4_io_cmd_abort();
    }
}

g4_cmd_res_t g4_io_cmd_run_prompt(const char *cmd, uint32_t timeout_ms)
{
    /* ★先清死账★: 属主失联的命令绝不能把引擎锁死(见 g4_io_reap_stale)。
     * 2026-10-07 18:21 的死锁里, 发布流程就是在这里被"永久的别人"挡了 2 分钟。 */
    g4_io_reap_stale();

    /* ★属主保护(2026-10-07)★ 与 g4_io_cmd_run_owned() 同一套规则:
     * 引擎上挂的是【别人的】命令时, 本函数只回 RUNNING —— 不偷结果、不替它判超时。
     * 发布子流程就是靠这一条才不会被 OTA 的命令"甩"一个 TIMEOUT 过来。
     * (它没有属主, 所以用 G4_OWNER_NONE 比对。) */
    if (s_eng == G4_ENG_RUNNING)
    {
        if (s_cmd_owner != G4_OWNER_NONE) { return G4_CMD_RUNNING; }
        return g4_io_cmd_poll();
    }
    if (s_eng == G4_ENG_DONE)
    {
        if (s_cmd_owner != G4_OWNER_NONE) { return G4_CMD_RUNNING; }
        s_eng = G4_ENG_IDLE;
        return s_cmd_result;
    }

    g4_io_cmd_begin_ex(cmd, NULL, timeout_ms, true, G4_OWNER_NONE);
    return g4_io_cmd_poll();
}

g4_cmd_res_t g4_io_cmd_run_silent(const char *cmd, uint32_t wait_ms)
{
    return g4_io_cmd_run(cmd, NULL, wait_ms);
}

const char *g4_io_cmd_resp(void)
{
    return s_res_snap;
}

const char *g4_io_cmd_line(void)
{
    return s_hit_snap;
}

/* ==================== 阻塞封装(诊断/必须同步拿结果的地方) ==================== */

bool g4_io_send_expect_resp(const char *cmd, const char *keyword, uint32_t timeout_ms,
                            char **got_line, char *resp, uint16_t resp_sz)
{
    g4_cmd_res_t r;
    uint32_t     wait_start;
    const char  *snap;

    if (resp != NULL)
    {
        resp[0] = '\0';
    }
    if (got_line != NULL)
    {
        *got_line = NULL;
    }

    /* 【不能在 URC 回调里阻塞等 AT】: 那是在收包泵内部, 再跑泵会被拒(重入),
     * 只会白等到超时。直接失败并说清楚, 让调用方走"拿不到就跳过"的分支。 */
    if (s_in_pump)
    {
        log_info("[G4] !! 拒绝在收包回调里阻塞等 AT(\"%s\"): 拿不到结果就跳过\r\n",
                 (cmd != NULL) ? cmd : "-");
        return false;
    }

    if (cmd == NULL)
    {
        return false;
    }

    /* 等引擎【真正空闲】(最多 200ms), 免得把状态机在途的命令打断。
     * ★2026-10-07: 判据从 g4_io_cmd_active()(只报 RUNNING) 换成 g4_io_cmd_idle()
     *   —— DONE(结果还没被取走)也算"不空闲", 否则我们发新命令会把别人的结果覆盖掉。
     * 本函数是【阻塞】的, 归属 CONSOLE 属主: 引擎被 OTA 占着时它抢不到, 也不该抢到。 */
    wait_start = get_tick_ms();
    while ((!g4_io_cmd_idle()) && ((uint32_t)(get_tick_ms() - wait_start) < 200u))
    {
        g4_io_pump();
    }
    if (!g4_io_cmd_idle())
    {
        log_info("[G4] !! 命令引擎忙(或在途/结果未取), 放弃阻塞命令 \"%s\"\r\n", cmd);
        return false;
    }

    g4_io_cmd_begin_ex(cmd, keyword, timeout_ms, false, G4_OWNER_CONSOLE);
    do
    {
        g4_io_pump();
        r = g4_io_cmd_poll();
    } while (r == G4_CMD_RUNNING);

    /* ★★2026-10-07: 本函数【绝不把引擎带走】★★
     *
     * 现场(18:30:22) 实测过最坏的结果: 一条标了 CONSOLE 属主(就是这里)的命令
     * 留在引擎上没人推进 ——
     *   18:30:22:189  [G4_TX] AT+CCLK?          <- 本函数发的
     *   18:30:22:212  [G4_RX] OK                <- 应答到了
     *   ...之后 6 秒引擎一直不是 IDLE...
     *   18:30:27:339  [OTA_HTTP] !! 等了 5000ms 仍拿不到命令引擎(当前属主=2)
     *   -> OTA 建实例被硬生生推迟 6 秒; 再往前一点(18:21)同类泄漏直接把
     *      【整机 AT 通道锁死两分多钟】。
     *
     * 正常路径上上面的 do/while 已经消费掉结果了, 这里是【保险】:
     * 不管从哪条路退出, 只要那条命令还挂在我名下, 就把它收干净。
     * (g4_io_cmd_release 只在"引擎非空闲且属主是我"时才动手, 幂等无害。) */
    g4_io_cmd_release(G4_OWNER_CONSOLE);

    snap = s_res_snap;
    if (resp != NULL && resp_sz > 0u)
    {
        strncpy(resp, snap, (size_t)(resp_sz - 1u));
        resp[resp_sz - 1u] = '\0';
    }
    if (got_line != NULL)
    {
        *got_line = (char *)s_hit_snap;
    }

    if (r == G4_CMD_OK)
    {
        return true;
    }

    /* 超时诊断: 累计收到=0 -> 模组从没开口; >0 -> 它说话了, 看上面的 [G4_RX] */
    log_info("[G4_TIMEOUT] cmd=\"%s\", rb残留=%u字节, 累计收到=%lu字节\r\n",
             cmd, (unsigned)ring_buf_get_len(&rb_g4),
             (unsigned long)g4_io_rx_total());
    return false;
}

bool g4_io_send_expect(const char *cmd, const char *keyword, uint32_t timeout_ms,
                       char **got_line)
{
    return g4_io_send_expect_resp(cmd, keyword, timeout_ms, got_line, NULL, 0u);
}

/* ==================== 传输层基础 ==================== */

void g4_io_init(void)
{
    bsp_g4_uart_init(G4_UART_BAUD);
    ring_buf_reset(&rb_g4);

    s_rx            = RXS_LINE;
    s_line_len      = 0u;
    s_magic_ok      = true;
    s_pub_topic_len = 0u;
    s_pub_need      = 0u;
    s_pub_have      = 0u;
    s_hdr_quotes    = 0u;
    s_hdr_commas    = 0u;
    s_hdr_num       = 0u;
    s_rx_last_tick  = get_tick_ms();
    s_last_tx_tick  = get_tick_ms();

    s_eng           = G4_ENG_IDLE;
    s_cmd_result    = G4_CMD_OK;
    s_cmd_kw        = NULL;
    s_cmd_prompt    = false;
    s_cmd_resp_used = 0u;
    s_cmd_resp[0]   = '\0';
    s_cmd_hit[0]    = '\0';
    s_res_snap[0]   = '\0';
    s_hit_snap[0]   = '\0';

    s_dl_abort_cnt  = 0u;
    s_log_drop_cnt  = 0u;
    s_log_win_tick  = get_tick_ms();
    s_log_win_cnt   = 0u;
    s_in_pump       = false;
}

void g4_io_poll(void)
{
    /* 接收由 USART2 中断 + g4_io_pump() 完成; 保留接口供上层统一调用 */
}

bool g4_io_cdc_ready(void)
{
    return true;    /* 串口随时就绪(函数名保留以兼容 g4_at 状态机) */
}

void g4_io_receive_cb(uint8_t *data)
{
    (void)data;     /* USB CDC 占位: 串口方案不使用, 保留符号以兼容 usbh_user.c */
}

bool g4_io_send_all(const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0u))
    {
        return true;
    }
    g4_uart_send_bytes(data, len);
    s_last_tx_tick = get_tick_ms();
    return true;
}

void g4_io_send_cmd(const char *cmd)
{
    if (cmd == NULL)
    {
        return;
    }
    g4_io_send_all((const uint8_t *)cmd, (uint16_t)strlen(cmd));
    g4_io_send_all((const uint8_t *)"\r\n", 2u);   /* AT 命令以 CRLF 结束 */
}

void g4_io_drain_rx(void)
{
    ring_buf_reset(&rb_g4);         /* 清空接收缓冲 */
    ring_buf_clear_overflow(&rb_g4);

    /* 【必须一起复位收包状态机】: 老版本的坑就是清了缓冲但没复位行拼装进度,
     * 残留半行和下一行粘在一起。 */
    s_rx            = RXS_LINE;
    s_line_len      = 0u;
    s_magic_ok      = true;
    s_pub_topic_len = 0u;
    s_pub_need      = 0u;
    s_pub_have      = 0u;
    s_hdr_quotes    = 0u;
    s_hdr_commas    = 0u;
    s_hdr_num       = 0u;
    s_rx_last_tick  = get_tick_ms();

    g4_io_cmd_abort();
}

/* ==================== 诊断计数 ==================== */

uint32_t g4_io_rx_total(void)
{
    return g4_uart_rx_count();
}

void g4_io_rx_reset(void)
{
    g4_uart_rx_count_reset();
}

uint32_t g4_io_rx_ovf_count(void)
{
    return g4_uart_rx_ovf_count();
}

uint32_t g4_io_dl_abort_count(void)
{
    return s_dl_abort_cnt;
}

uint32_t g4_io_rxlog_drop_count(void)
{
    return s_log_drop_cnt;
}

uint32_t g4_io_ready_urc_count(void)
{
    return s_ready_urc_cnt;
}

uint32_t g4_io_ready_after_tx_count(void)
{
    return s_ready_after_tx_cnt;
}

uint32_t g4_io_ready_total(void)
{
    return s_ready_urc_cnt + s_ready_after_tx_cnt;
}
