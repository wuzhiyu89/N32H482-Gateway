/**
 * \file  g4_urc.c
 * \brief ML307R MQTT URC 解析 + 在线状态维护 + 下行消息队列化投递
 *
 * 依据《MQTT 用户手册 V6.7.0》第 3.9 节,统一从 +MQTTURC: "conn",<id>,<conn_state>
 * 判断连接状态,不再使用不存在的 "closed"/"disconn"。
 *
 * 本文件只做【协议语义】,不做帧拼装:
 *   · 帧怎么切分(行 / 定长二进制 payload / 出错重同步)由 g4_io.c 的接收泵负责,
 *     它盯着 g4_io.h 里解释过的那些坑(0x00 截断、0x0A 劈包、永久失步);
 *   · 这里收到的是"一条完整行"或"一条收满的 publish", 直接判语义。
 *
 * 【下行不在收包路径上做业务】: publish 先入队, 由 g4_urc_poll() 在主循环里投递。
 * 这样上层回调(可能阻塞几百毫秒的 LoRa 组帧/发送)不会卡住收包泵。
 */
#include "g4_urc.h"
#include "g4_io.h"
#include <string.h>
#include <stdlib.h>
#include "log.h"

/* ==================== 下行队列 ====================
 * 深度不用大: 一条下行在主循环里最多几十毫秒就处理完; 4 条足够吸收突发。
 * 满了会【计数并打日志】, 不做静默丢弃 —— 静默丢弃是现场最难查的一类问题。
 */
#ifndef G4_URC_DLQ_DEPTH
/* ★★2026-10-09: 由 4 降到 2 —— 为下行大缓冲腾 RAM★★
 * 队列条目里带 G4_URC_PUB_MAX 字节, 所以 RAM = 深度 x 该值。
 *   6208 x 4 = 24.8KB   -> 整机 RAM 到 92.8%(只剩 13KB 栈), 太危险
 *   4352 x 2 =  8.7KB   -> 安全
 * 【为什么降到 2 是可以的】本队列只用来吸收"云端连发"的突发; 主循环每轮都排空它,
 *   单条命令动辄几百毫秒才处理完, 深度 2 完全够。深度只影响"突发能囤几条",
 *   【不影响单条能不能收】—— 所以 RAM 紧张时该减深度, 不该减 PUB_MAX。 */
#define G4_URC_DLQ_DEPTH   2u
#endif
#ifndef G4_URC_PUB_MAX
/* ★★2026-10-09 由 320 提到 6208 —— 这是下行【三层尺寸闸门】的最里面一层★★
 *
 * 【现象】把外层 G4_IO_PUB_BUF_SZ 从 384 放大后, 322 字节的命令立刻撞上这层:
 *     [URC] !! 下行 322 字节, 长度非法(上限 320), 丢弃
 *
 * 【三层必须同时够大, 否则永远是"剥洋葱"】
 *     ① g4_io.c  的 G4_IO_PUB_BUF_SZ   —— AT 层收原始字节的缓冲
 *     ② 本文件   的 G4_URC_PUB_MAX     —— URC 队列条目里存这一条的缓冲 ← 本层
 *     ③ main.c   的 MAIN_CLOUD_JSON_MAX = CFG_JSON_MAX(6144)+64  —— 解析用
 *   任何一层小, 命令都在那一层被丢掉, 前面的层放大也白搭。
 *   ⇒ 三者都按"最大可能的云端命令"定尺寸。这里取 6208 与 ③ 一致。
 *
 * 【RAM 代价】4(队列深度) x 6208 = 24832 字节(.bss)。
 *   这是"下行能收多大命令"的闸门, 不能省; 若 RAM 紧张, 减 G4_URC_DLQ_DEPTH
 *   比减本值安全 —— 队列深度只影响"突发时能囤几条", 不影响单条能不能收。
 *
 * ⚠️ 改这个值时要一起看那三层, 别只改一处。 */
#ifndef G4_URC_PUB_MAX
#define G4_URC_PUB_MAX     4352u
#endif
#endif
#define G4_URC_TOPIC_MAX   48u

typedef struct
{
    char     topic[G4_URC_TOPIC_MAX];
    uint16_t len;
    uint8_t  data[G4_URC_PUB_MAX];
    bool     used;
} g4_urc_dlq_item_t;

static g4_urc_dlq_item_t s_dlq[G4_URC_DLQ_DEPTH];
static uint8_t           s_dlq_wr;
static uint8_t           s_dlq_rd;
static uint8_t           s_dlq_cnt;

/* ==================== 状态 ==================== */
static volatile bool     g_online;
static g4_mqtt_recv_cb_t g_recv_cb;
static g4_urc_http_cb_t  g_http_cb;   /* OTA over HTTP: +MHTTPURC 处理者 */

/* 最近一次 +MQTTURC: "conn",<id>,<state> 里的 <state>, 0xFF = 还没收到过。
 * 建链失败时 g4_at 会把它打出来 —— "服务器拒绝(3)"和"ping超时(5)"
 * 完全是两个方向的排查(账号/clientID vs 网络不通)。 */
static volatile uint8_t  g_conn_state = 0xFFu;

/* 收到过多少条 conn URC(单调递增, 不清零)。
 * 建链时用它区分"本次的 conn 结果"和"上一轮残留的在线标志", 详见 g4_urc.h。 */
static volatile uint32_t s_conn_urc_seq;

static uint32_t          s_puback_cnt;
static uint32_t          s_pub_rx_cnt;
static uint32_t          s_pub_drop_cnt;

/* ==================== 驻网(注册)与 PS 附着变化的原始证据 ====================
 *
 * 【为什么要有】现场判"4G 到底断没断"一直缺一层证据: 我们只有 MQTT 层的
 * conn URC 和"发布失败计数", 全都在 MQTT 之上。射频掉了但旧 IP 还在时,
 * 这两条都看不见(见 不做与偏差记录_v107 第五、六节)。
 * AT+CEREG=2 / AT+CGEREP=1 打开后, 模组会【主动】把注册状态和 PS 去附着报上来,
 * 那是比 MQTT 更底层、也更权威的断网证据。
 *
 * 【为什么在这里数、而且返回 false】
 *   ① AT+CEREG? 的【轮询应答】与主动上报的 URC 前缀完全一样, 轮询那条必须继续
 *      交给 AT 命令引擎去匹配关键字 —— 所以这里只记账, 不消费(return false)。
 *   ② 两种形态的字段位置【不一样】, 不能用一个解析器硬套:
 *        URC 形态      : +CEREG: <stat>[,<tac>,<ci>,<AcT>]      第一个数就是 stat
 *        轮询应答形态  : +CEREG: <n>,<stat>[,<tac>,<ci>,<AcT>]  第二个数才是 stat
 *      取哪个数取决于"这行是不是我们问出来的", 这里判不了 —— 所以本模块
 *      【只把原始行可靠地打进日志】, 由 g4_at.c 的轮询分支用 at_reg_stat()
 *      按应答形态解析。人工判读看日志里那行 "驻网行(原始)"。
 *
 * 【为什么要单独打一行日志】: g4_io 的收包日志是限流的(每 100ms 最多 12 行),
 * 真掉网那几条恰恰最容易被限流掉。这几行很少见, 值得无条件打出来。
 */
static uint32_t          s_reg_line_cnt;      /* +CEREG/+CGREG/+CREG 行数(含轮询应答) */
static uint8_t           s_reg_first_num = 0xFFu; /* 最近一行的第一个数(仅人工判读用) */
static uint32_t          s_cgev_cnt;          /* +CGEV 行数 */
static uint32_t          s_cgev_detach_cnt;   /* 其中 DETACH 类(网络强制去附着) */

/* ==================== 初始化 ==================== */
void g4_urc_init(void)
{
    g_online       = false;
    g_conn_state   = 0xFFu;
    s_conn_urc_seq = 0u;

    s_dlq_wr = 0u;
    s_dlq_rd = 0u;
    s_dlq_cnt = 0u;
    memset(s_dlq, 0, sizeof(s_dlq));

    s_puback_cnt   = 0u;
    s_pub_rx_cnt   = 0u;
    s_pub_drop_cnt = 0u;

    s_reg_line_cnt     = 0u;
    s_reg_first_num    = 0xFFu;
    s_cgev_cnt         = 0u;
    s_cgev_detach_cnt  = 0u;

    /* 【注意: 这里【不清】 g_recv_cb】
     * 老版本在这里写了 g_recv_cb = NULL, 而 main.c 的执行顺序是
     *     lora_business_init()  → g4_urc_set_recv_cb(lora_mqtt_recv_cb)
     *     g4_at_init()          → g4_urc_init()
     * 于是打开 LoRa 的那一版里, 下行回调【刚注册就被清掉了】:
     * 云端消息收得到、日志里却什么都不发生 —— 现场会误判成"4G 收不到下发"。
     * 回调的注册权归上层, 初始化不该动它。 */

    /* 把自己挂到接收泵上: 从此 URC 解析不再和 AT 收发抢字节流 */
    g4_io_set_urc_cb(g4_urc_input_line, g4_urc_input_publish);
}

void g4_urc_set_recv_cb(g4_mqtt_recv_cb_t cb)
{
    g_recv_cb = cb;
}

void g4_urc_set_http_cb(g4_urc_http_cb_t cb)
{
    g_http_cb = cb;
}

/* ==================== 状态访问 ==================== */
uint8_t g4_urc_get_conn_state(void)
{
    return g_conn_state;
}

void g4_urc_reset_conn_state(void)
{
    g_conn_state = 0xFFu;

    /* 【必须把在线标志一起清掉 —— 这是现场"白烧 30~40 秒"的根因】
     * 原来这里只清 g_conn_state, g_online 还留着上一轮的 true。于是重建时
     * WAIT_CONN 一进来就看到 online=true, 日志打出
     *     [G4] MQTT conn ok (res=255 unknown)
     * —— 而 255 的字面意思正是"本次一条 conn URC 都还没收到" ——
     * 接着就往 MQTT_SUB 走, 连吃 4 个 +CME ERROR: 607 再 rebuild。
     * (现场日志 78.txt 里这段整整出现了两次。) */
    g_online = false;
}

uint32_t g4_urc_conn_seq(void)
{
    return s_conn_urc_seq;
}

bool g4_urc_is_online(void)
{
    return g_online;
}

void g4_urc_set_online(void)
{
    g_online = true;
}

void g4_urc_set_offline(void)
{
    g_online = false;
}

uint32_t g4_urc_puback_count(void)
{
    return s_puback_cnt;
}

uint32_t g4_urc_pub_rx_count(void)
{
    return s_pub_rx_cnt;
}

uint32_t g4_urc_pub_drop_count(void)
{
    return s_pub_drop_cnt;
}

/* ---- 驻网/PS 附着观测(只读诊断, 见文件上方说明) ---- */

uint32_t g4_urc_reg_line_count(void)
{
    return s_reg_line_cnt;
}

uint8_t g4_urc_reg_first_num(void)
{
    return s_reg_first_num;
}

uint32_t g4_urc_cgev_count(void)
{
    return s_cgev_cnt;
}

uint32_t g4_urc_cgev_detach_count(void)
{
    return s_cgev_detach_cnt;
}

/* ==================== URC 行解析 ==================== */

/**
 * @brief 解析 +MQTTURC: "conn",<id>,<conn_state>
 */
static void urc_handle_conn(const char *line)
{
    const char *p = strstr(line, "\"conn\"");
    if (p == NULL) return;
    p += 6u;

    p = strchr(p, ',');
    if (p == NULL) return;
    p = strchr(p + 1u, ',');
    if (p == NULL) return;
    p++;

    int state = atoi(p);

    g_conn_state = (uint8_t)state;   /* 记下来给上层打日志排障 */
    if (s_conn_urc_seq < 0xFFFFFFFFu) { s_conn_urc_seq++; }

    switch (state)
    {
        case 0:     g_online = true;  break;
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 255:
        default:    g_online = false; break;
    }
}

bool g4_urc_input_line(const char *line, uint16_t len)
{
    (void)len;

    if (line == NULL)
    {
        return false;
    }

    /* ---- OTA over HTTP: +MHTTPURC 家族 ----
     * 【必须放在 +MQTTURC 判断之前】: 下面那条 if 对非 +MQTTURC 的行直接
     * return false, 会把 MHTTP 的 URC 放行给 AT 应答匹配 —— 那样 content
     * 数据会被当垃圾行丢掉(甚至误命中关键字), 下载永远收不满。
     * 回调由 ota_http_init() 注册; 没注册(没用 OTA)时退化成"只打日志"。 */
    if (strstr(line, "+MHTTPURC:") != NULL)
    {
        if (g_http_cb != NULL)
        {
            (void)g_http_cb(line);
        }
        else
        {
            log_info("[URC] %s\r\n", line);
        }
        return true;
    }

    if (strstr(line, "+MQTTURC:") == NULL)
    {
        /* ---- 驻网(注册状态)与 PS 附着变化: 【只记账 + 打原始行】, 不消费 ----
         *
         * 【必须放在这个 return false 之前】—— 否则这些行永远不会被统计到。
         * 【必须仍然 return false】—— AT+CEREG? 的轮询应答就长这样, 它还要交给
         *   AT 命令引擎去匹配 "+CEREG:" 关键字; 在这里吃掉它, 轮询会永远超时。
         * 详细论证见文件顶部"驻网(注册)与 PS 附着变化的原始证据"那一节。 */
        if ((strstr(line, "+CEREG") != NULL) || (strstr(line, "+CGREG") != NULL) ||
            (strstr(line, "+CREG")  != NULL))
        {
            const char *c = strchr(line, ':');

            if (s_reg_line_cnt < 0xFFFFFFFFu) { s_reg_line_cnt++; }
            if (c != NULL)
            {
                s_reg_first_num = (uint8_t)atoi(c + 1);
                log_info("[REG] 驻网行(原始): %s\r\n", line);
            }
            return false;
        }

        if (strstr(line, "+CGEV") != NULL)
        {
            if (s_cgev_cnt < 0xFFFFFFFFu) { s_cgev_cnt++; }
            if (strstr(line, "DETACH") != NULL)
            {
                if (s_cgev_detach_cnt < 0xFFFFFFFFu) { s_cgev_detach_cnt++; }
                log_info("[REG] !! PS 去附着(网络/模组强制): %s\r\n", line);
            }
            else
            {
                log_info("[REG] PS 事件: %s\r\n", line);
            }
            return false;
        }

        return false;       /* 不是 URC 家族: 交回给 AT 应答匹配 */
    }

    if (strstr(line, "+MQTTURC: \"conn\"") != NULL)
    {
        urc_handle_conn(line);
        log_info("[URC] conn state=%u\r\n", (unsigned)g_conn_state);
        return true;
    }

    /* 【这才是"云端确实收到了"的证据】
     * QoS 1 时模组会等 broker 的 PUBACK 并上报 +MQTTURC: "puback",<id>,<msgid>。
     * 发布流程用 g4_urc_puback_count() 的变化来确认端到端送达。 */
    if ((strstr(line, "\"puback\"") != NULL) || (strstr(line, "\"pubcomp\"") != NULL))
    {
        s_puback_cnt++;
        log_info("[URC] 发布已被 broker 确认: %s\r\n", line);
        return true;
    }

    /* 其它 URC(suback/timeout/drop/pubnmi/pingresp/...): 只打印, 不影响状态 */
    log_info("[URC] %s\r\n", line);
    return true;
}

/* ==================== publish 入队 ==================== */

void g4_urc_input_publish(const char *topic, const uint8_t *data, uint16_t len)
{
    g4_urc_dlq_item_t *it;

    s_pub_rx_cnt++;

    if ((data == NULL) || (len == 0u) || (len > G4_URC_PUB_MAX))
    {
        log_info("[URC] !! 下行 %u 字节, 长度非法(上限 %u), 丢弃\r\n",
                 (unsigned)len, (unsigned)G4_URC_PUB_MAX);
        return;
    }

    if (s_dlq_cnt >= G4_URC_DLQ_DEPTH)
    {
        s_pub_drop_cnt++;
        log_info("[URC] !! 下行队列满(深度 %u), 丢弃一条; 累计丢 %lu 条"
                 "(说明云端下发比主循环处理还快)\r\n",
                 (unsigned)G4_URC_DLQ_DEPTH, (unsigned long)s_pub_drop_cnt);
        return;
    }

    it = &s_dlq[s_dlq_wr];

    if (topic != NULL)
    {
        strncpy(it->topic, topic, sizeof(it->topic) - 1u);
        it->topic[sizeof(it->topic) - 1u] = '\0';
    }
    else
    {
        it->topic[0] = '\0';
    }

    memcpy(it->data, data, len);
    it->len  = len;
    it->used = true;

    s_dlq_wr = (uint8_t)((s_dlq_wr + 1u) % G4_URC_DLQ_DEPTH);
    s_dlq_cnt++;
}

/* ==================== 主循环投递 ==================== */

bool g4_urc_poll(void)
{
    g4_urc_dlq_item_t *it;

    if (s_dlq_cnt == 0u)
    {
        return false;
    }

    it = &s_dlq[s_dlq_rd];

    if (!it->used)      /* 计数与实际不一致时兜底, 免得卡死 */
    {
        s_dlq_rd = (uint8_t)((s_dlq_rd + 1u) % G4_URC_DLQ_DEPTH);
        if (s_dlq_cnt > 0u) { s_dlq_cnt--; }
        return false;
    }

    if (g_recv_cb != NULL)
    {
        /* 【在收包泵之外执行】: 上层回调里做组帧/LoRa 发送都不会卡住接收 */
        g_recv_cb(it->topic, it->data, it->len);
    }
    else
    {
        log_info("[URC] !! 收到云端下行 %u 字节, 但没有注册回调, 这条丢了"
                 "(检查 g4_urc_set_recv_cb / g4_at_set_recv_cb 有没有被调)\r\n",
                 (unsigned)it->len);
    }

    it->used = false;
    s_dlq_rd = (uint8_t)((s_dlq_rd + 1u) % G4_URC_DLQ_DEPTH);
    s_dlq_cnt--;
    return true;
}
