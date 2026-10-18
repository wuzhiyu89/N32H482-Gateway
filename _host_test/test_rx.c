/**
 * \file  test_rx.c
 * \brief g4_io 接收泵 + g4_urc 下行队列的【主机单元测试】
 *
 * 为什么值得单独测: 这次改的东西全是"字节流边界"逻辑 —— 云端下发的原始二进制
 * payload 里含 0x00/0x0A/0x0D/0x3E, 老版本就是死在这些字节上(被当成字符串结尾、
 * 被当行尾劈开、被当成数据模式提示)。这类 bug 在板子上只能靠"跑一会儿就收不到"
 * 观察到, 在 PC 上却能用确定的字节序列一举复现/证明修好了。
 *
 * 测试项:
 *   1) 普通行 + AT 应答关键字匹配
 *   2) publish URC 的二进制 payload(含 0x00/0x0A/0x0D/0x3E)被【原封不动】收到
 *   3) 两条 publish 连着来, 不串包、不失步
 *   4) 分包: total_len > payload_len 时按 <payload_len> 收
 *   5) payload 被截断 -> 超时放弃 + 重同步, 后面那条依然完整
 *   6) payload_len 非法 -> 放弃 + 重同步
 *   7) '>' 提示必须是严格整行(带 '>' 的脏行不能触发数据模式)
 *   8) AT 命令在途时来的 publish 不会丢(老版本就是在这里丢的)
 *   9) +MQTTURC: "conn",0,0 -> 在线
 *  10) 多行应答累积
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "ring_buf.h"
#include "bsp_uart.h"
#include "bsp_tim.h"
#include "log.h"
#include "g4_io.h"
#include "g4_urc.h"

void host_uart_rx_byte(uint8_t ch);

static int g_fail;
static int g_pass;

#define CHECK(cond, ...)                                        \
    do {                                                        \
        if (cond) { g_pass++; printf("  ok   : "); }             \
        else      { g_fail++; printf("  FAIL : "); }             \
        printf(__VA_ARGS__);                                    \
        printf("\n");                                           \
    } while (0)

/* ---------------- 喂数据 ---------------- */
static void feed(const void *buf, size_t len)
{
    const uint8_t *p = (const uint8_t *)buf;
    size_t i;

    for (i = 0u; i < len; i++)
    {
        host_uart_rx_byte(p[i]);
    }
}

/* ---------------- 捕获下行 ---------------- */
static uint8_t  s_rx_data[512];
static uint16_t s_rx_len;
static char     s_rx_topic[64];
static int      s_rx_cnt;

static void test_recv_cb(const char *topic, const uint8_t *data, uint16_t len)
{
    s_rx_cnt++;
    s_rx_len = len;
    if (len <= sizeof(s_rx_data)) { memcpy(s_rx_data, data, len); }
    if (topic != NULL)
    {
        strncpy(s_rx_topic, topic, sizeof(s_rx_topic) - 1u);
        s_rx_topic[sizeof(s_rx_topic) - 1u] = '\0';
    }
    else { s_rx_topic[0] = '\0'; }
}

static void reset_rx_capture(void)
{
    s_rx_len   = 0u;
    s_rx_cnt   = 0;
    s_rx_topic[0] = '\0';
    memset(s_rx_data, 0, sizeof(s_rx_data));
}

/** 把一条完整 publish URC 的字节喂进去(header + payload + CRLF) */
static void feed_publish(const char *topic, uint32_t total_len, uint32_t plen,
                         const uint8_t *payload, uint32_t real_payload_len)
{
    char hdr[160];
    int  n = snprintf(hdr, sizeof(hdr), "+MQTTURC: \"publish\",0,1,\"%s\",%lu,%lu,",
                      topic, (unsigned long)total_len, (unsigned long)plen);

    feed("\r\n", 2u);
    feed(hdr, (size_t)n);
    if ((payload != NULL) && (real_payload_len > 0u))
    {
        feed(payload, real_payload_len);
    }
    feed("\r\n", 2u);
}

/* ---------------- 用例 ---------------- */
static void test_plain_line_and_at(void)
{
    g4_cmd_res_t r;

    printf("[1] 普通行 + AT 应答匹配\n");
    g4_io_drain_rx();
    host_tick_advance(10u);

    g4_io_cmd_begin("AT", "OK", 1000u);
    feed("\r\n+MATREADY\r\nOK\r\n", 17u);
    g4_io_pump();
    r = g4_io_cmd_poll();

    CHECK(r == G4_CMD_OK, "AT 收到 OK (res=%d)", (int)r);
    CHECK(strstr(host_tx_take(), "AT\r\n") != NULL,
          "命令以 CRLF 结尾: \"%s\"", host_tx_take());
    CHECK(g4_io_ready_total() >= 1u, "+MATREADY 被统计到 (%lu)",
          (unsigned long)g4_io_ready_total());
}

static void test_binary_payload(void)
{
    /* 现场真实下发帧: 含 0x00 / 0x0A / 0x3E */
    static const uint8_t frame[20] = {
        0xAA, 0x55, 0x01, 0x11, 0x00, 0x01, 0x21, 0x00, 0x01, 0x01,
        0x07, 0x05, 0x00, 0x0A, 0x00, 0x3E, 0xDC, 0x05, 0x38, 0x1B
    };

    printf("[2] 二进制 payload(含 0x00/0x0A/0x0D/0x3E)原样收到\n");
    g4_io_drain_rx();
    reset_rx_capture();

    feed_publish("mod/pc_down", 20u, 20u, frame, 20u);
    g4_io_pump();
    (void)g4_urc_poll();

    CHECK(s_rx_cnt == 1, "收到 1 条下行 (实际 %d)", s_rx_cnt);
    CHECK(s_rx_len == 20u, "长度 20 字节 (实际 %u)", (unsigned)s_rx_len);
    CHECK(memcmp(s_rx_data, frame, 20u) == 0, "字节内容完全一致");
    CHECK(strcmp(s_rx_topic, "mod/pc_down") == 0, "topic=%s", s_rx_topic);
    CHECK(g4_io_dl_abort_count() == 0u, "没有发生重同步 (%lu)",
          (unsigned long)g4_io_dl_abort_count());
}

static void test_two_publishes(void)
{
    static const uint8_t f1[6] = { 0xAA, 0x55, 0x0A, 0x00, 0x3E, 0x01 };
    static const uint8_t f2[6] = { 0xAA, 0x55, 0x0D, 0x00, 0x3E, 0x02 };

    printf("[3] 两条 publish 连着来, 不串包\n");
    g4_io_drain_rx();
    reset_rx_capture();
    {
        uint32_t before = g4_urc_pub_rx_count();

        feed_publish("t/a", 6u, 6u, f1, 6u);
        feed_publish("t/b", 6u, 6u, f2, 6u);
        g4_io_pump();
        (void)g4_urc_poll();
        (void)g4_urc_poll();

        CHECK(s_rx_cnt == 2, "收到 2 条 (实际 %d)", s_rx_cnt);
        CHECK((s_rx_len == 6u) && (memcmp(s_rx_data, f2, 6u) == 0),
              "第 2 条内容正确, topic=%s", s_rx_topic);
        CHECK(strcmp(s_rx_topic, "t/b") == 0, "第 2 条 topic=%s", s_rx_topic);
        CHECK(g4_urc_pub_rx_count() == (before + 2u), "本用例新增计数=2 (%lu)",
              (unsigned long)(g4_urc_pub_rx_count() - before));
    }
}

static void test_split_publish(void)
{
    /* 分包: total_len=40, 本条分片 payload_len=10
     * ★★2026-10-14 本用例的期望【改了】★★
     * ---------------------------------------------------------------------------
     * 【原来期望什么】每收到一片就交付一条 10 字节的下行(cnt=1, len=10)。
     * 【为什么那是错的】模块把一条长 publish 按 512 字节左右分包, 每包一条 URC:
     *       +MQTTURC: "publish",0,1,"<topic>",<total_len>,<plen>,<payload...>
     *   ⇒ 每片都当成独立下行的话, 平台看到的是【被截断的 JSON】:
     *       实测一条 1441 字节的 rule.set 被拆成 plen = 489 / 512 / 440,
     *       第 1 片 -> 截断 JSON -> 平台收到 1001 bad request;
     *       后续片 -> 不以 '{' 开头 -> 被判"不像 JSON" -> 落到 LoRa 透传路径
     *                 (危险: 若某片 <=240 字节, 会被真的发到空口)。
     * 【现在期望什么】用 <plen> 读【本片】, 用 <total_len> 判断【攒够了没有】:
     *   收齐 4 片(共 40 字节)之后, 交付【唯一一条】40 字节的下行。
     * ⇒ 这个改动是【先让单测红, 再改实现让它绿】的那个"红"。
     * --------------------------------------------------------------------------- */
    static const uint8_t p[4][10] = {
        {  1,  2,  3,  4,  0, 0x0A,  6,  7,  8,  9 },
        { 11, 12, 13, 0x3E, 15, 16, 17, 18, 19, 20 },
        { 21, 22, 23, 24, 25, 26, 27, 28, 29, 30 },
        { 31, 32, 33, 34, 35, 36, 37, 38, 39, 40 }
    };
    uint8_t expect[40];
    int     i;

    for (i = 0; i < 4; i++)
    {
        (void)memcpy(&expect[i * 10], p[i], 10u);
    }

    printf("[4] 分包(total_len=40, 分 4 片)必须【合成一条 40 字节】\n");
    g4_io_drain_rx();
    reset_rx_capture();

    /* 前 3 片: 一片都不该交付 */
    for (i = 0; i < 3; i++)
    {
        feed_publish("t/split", 40u, 10u, p[i], 10u);
        g4_io_pump();
        (void)g4_urc_poll();
        CHECK(s_rx_cnt == 0, "第 %d 片后【不该】交付(还差字节), cnt=%d", i + 1, s_rx_cnt);
    }

    /* 第 4 片: 攒够 40, 交付唯一一条, 内容逐字节正确 */
    feed_publish("t/split", 40u, 10u, p[3], 10u);
    g4_io_pump();
    (void)g4_urc_poll();
    CHECK(s_rx_cnt == 1, "★ 4 片收齐后【只交付一条】, cnt=%d", s_rx_cnt);
    CHECK(s_rx_len == 40u, "★ 合起来 40 字节 (实际 %u)", (unsigned)s_rx_len);
    CHECK((s_rx_len == 40u) && (memcmp(s_rx_data, expect, 40u) == 0),
          "★ 40 字节内容逐字节正确(含片内的 0x0A/0x3E, 它们不能被当行尾)");
    CHECK(strcmp(s_rx_topic, "t/split") == 0, "topic 正确 (%s)", s_rx_topic);
    CHECK(g4_io_dl_abort_count() == 0u, "★ 正常分包不该计重同步 (%lu)",
          (unsigned long)g4_io_dl_abort_count());
}

static void test_truncated_then_resync(void)
{
    static const uint8_t part[5]  = { 0xAA, 0x55, 0x00, 0x0A, 0x3E };
    static const uint8_t good[7]  = { 0xAA, 0x55, 0x01, 0x02, 0x03, 0x04, 0x05 };

    printf("[5] payload 被截断 -> 超时放弃 + 重同步\n");
    g4_io_drain_rx();
    reset_rx_capture();
    {
        uint32_t before = g4_io_dl_abort_count();
        char     hdr[160];
        int      n = snprintf(hdr, sizeof(hdr),
                              "+MQTTURC: \"publish\",0,1,\"t/trunc\",20,20,");
        feed("\r\n", 2u);
        feed(hdr, (size_t)n);
        feed(part, 5u);                 /* 只给了 5 个字节, 说好是 20 */

        g4_io_pump();
        (void)g4_urc_poll();
        CHECK(s_rx_cnt == 0, "截断的这条没有交付 (cnt=%d)", s_rx_cnt);

        /* 模拟"后续字节永远不来了": 时间推进超过 G4_IO_RX_GAP_TMO_MS */
        host_tick_advance(600u);
        g4_io_pump();
        CHECK(g4_io_dl_abort_count() == (before + 1u),
              "超时后放弃并重同步 (abort=%lu)",
              (unsigned long)g4_io_dl_abort_count());
    }

    /* 重同步之后, 下一条正常的 URC 必须完整收到 */
    reset_rx_capture();
    feed_publish("t/after", 7u, 7u, good, 7u);
    g4_io_pump();
    (void)g4_urc_poll();

    CHECK((s_rx_cnt == 1) && (s_rx_len == 7u) && (memcmp(s_rx_data, good, 7u) == 0),
          "重同步后下一条完整 (cnt=%d len=%u)", s_rx_cnt, (unsigned)s_rx_len);
}

static void test_bad_len_then_resync(void)
{
    static const uint8_t junk[8] = { 1, 2, 0x0A, 4, 0, 6, 0x3E, 8 };
    static const uint8_t good[4] = { 9, 8, 7, 6 };

    printf("[6] payload_len 非法 -> 放弃 + 重同步\n");
    g4_io_drain_rx();
    reset_rx_capture();

    {
        uint32_t before = g4_io_dl_abort_count();
        char     hdr[160];
        int      n = snprintf(hdr, sizeof(hdr),
                              "+MQTTURC: \"publish\",0,1,\"t/bad\",9999,9999,");
        feed("\r\n", 2u);
        feed(hdr, (size_t)n);
        feed(junk, 8u);
        g4_io_pump();
        CHECK(g4_io_dl_abort_count() == (before + 1u), "非法长度被拒 (abort=%lu)",
              (unsigned long)g4_io_dl_abort_count());
    }

    reset_rx_capture();
    feed_publish("t/ok", 4u, 4u, good, 4u);
    g4_io_pump();
    (void)g4_urc_poll();
    CHECK((s_rx_cnt == 1) && (s_rx_len == 4u) && (memcmp(s_rx_data, good, 4u) == 0),
          "之后的下一条完整 (cnt=%d len=%u)", s_rx_cnt, (unsigned)s_rx_len);
}

static void test_prompt_strict(void)
{
    g4_cmd_res_t r;

    printf("[7] '>' 提示必须严格整行\n");
    g4_io_drain_rx();

    r = g4_io_cmd_run_prompt("AT+MQTTPUB=0,\"t\",1,0,0,4", 1000u);
    CHECK(r == G4_CMD_RUNNING, "命令已发出, 等待提示 (res=%d)", (int)r);

    /* 脏行: 含 '>' 但不是纯提示 —— 老版本 strchr 会在这里误判并提前发 payload */
    feed("\r\nGARBAGE>MORE\r\n", 16u);
    g4_io_pump();
    r = g4_io_cmd_poll();
    CHECK(r == G4_CMD_RUNNING, "脏行没有误触发数据模式 (res=%d)", (int)r);

    /* 在等 '>' 期间来一条【原始二进制】云端下发: 必须完整收到, 且不能顶替 '>' */
    {
        static const uint8_t f[4] = { 0xAA, 0x00, 0x0A, 0x3E };

        reset_rx_capture();
        feed_publish("t/while_prompt", 4u, 4u, f, 4u);
        g4_io_pump();
        (void)g4_urc_poll();
        CHECK((s_rx_cnt == 1) && (s_rx_len == 4u) && (memcmp(s_rx_data, f, 4u) == 0),
              "等提示期间的下行完整收到 (cnt=%d len=%u)", s_rx_cnt, (unsigned)s_rx_len);
        r = g4_io_cmd_poll();
        CHECK(r == G4_CMD_RUNNING, "下行没有顶替 '>' 提示 (res=%d)", (int)r);
    }

    /* 真实格式: "\r\n> " —— 【后面没有换行】, 所以绝不能按"整行结束"去等 */
    feed("\r\n> ", 4u);
    g4_io_pump();
    r = g4_io_cmd_poll();
    CHECK(r == G4_CMD_OK, "真正的 '>' 提示(\"\\r\\n> \")被识别 (res=%d)", (int)r);

    /* 提示尾随的空白必须被吃掉, 不能和后面的行粘在一起 */
    feed("\r\nOK\r\n", 6u);
    g4_io_pump();
    CHECK(strstr(host_tx_take(), "AT+MQTTPUB") != NULL,
          "命令已发出: %s", host_tx_take());
}

static void test_publish_during_at(void)
{
    static const uint8_t frame[8] = { 0xAA, 0x55, 1, 2, 3, 0x0A, 0x3E, 8 };
    g4_cmd_res_t r;

    printf("[8] AT 命令在途时来的 publish 不会丢\n");
    g4_io_drain_rx();
    reset_rx_capture();

    g4_io_cmd_begin("AT", "OK", 1000u);
    /* 模组在应答之间插了一条云端下发 */
    feed("\r\n+MATREADY\r\n", 13u);
    feed_publish("t/inter", 8u, 8u, frame, 8u);
    feed("OK\r\n", 4u);
    g4_io_pump();

    r = g4_io_cmd_poll();
    (void)g4_urc_poll();

    CHECK(r == G4_CMD_OK, "AT 应答仍然命中 (res=%d)", (int)r);
    CHECK((s_rx_cnt == 1) && (s_rx_len == 8u) && (memcmp(s_rx_data, frame, 8u) == 0),
          "夹在应答中间的下行完整收到 (cnt=%d len=%u)", s_rx_cnt, (unsigned)s_rx_len);
}

static void test_conn_urc(void)
{
    printf("[9] conn URC 维护在线状态\n");
    g4_io_drain_rx();

    feed("\r\n+MQTTURC: \"conn\",0,0\r\n", 24u);
    g4_io_pump();
    CHECK(g4_urc_is_online(), "state=0 -> 在线");
    CHECK(g4_urc_get_conn_state() == 0u, "conn_state=0 (%u)",
          (unsigned)g4_urc_get_conn_state());

    feed("\r\n+MQTTURC: \"conn\",0,5\r\n", 24u);
    g4_io_pump();
    CHECK(!g4_urc_is_online(), "state=5(ping超时) -> 离线");

    /* puback 计数(上行"真的到了"的证据) */
    {
        uint32_t before = g4_urc_puback_count();
        feed("\r\n+MQTTURC: \"puback\",0,7,0\r\n", 28u);
        g4_io_pump();
        CHECK(g4_urc_puback_count() == (before + 1u), "puback 计数 +1 (%lu)",
              (unsigned long)g4_urc_puback_count());
    }
}

static void test_multi_line_resp(void)
{
    char *line = NULL;
    char  resp[256];

    printf("[10] 多行应答累积\n");
    g4_io_drain_rx();

    feed("\r\n+CGSN: 861234567890123\r\nOK\r\n", 30u);
    CHECK(g4_io_send_expect_resp("AT+CGSN=1", "+CGSN:", 1000u, &line, resp, sizeof(resp)),
          "命中 +CGSN:");
    CHECK(strstr(resp, "861234567890123") != NULL, "应答累积里有 IMEI: \"%s\"", resp);
}

/**
 * 【回归用例】命令结果必须在"下一拍"被交付, 不能被新命令覆盖
 *
 * 这是真实踩过的坑(一上电就一直发 AT, 200 条/秒):
 * 收包泵在主循环每拍开头跑, 命令经常在【调用方还没来取】的时候就完成了。
 * 如果引擎分不清"还没开始"和"刚完成、结果还没被取走", 调用方下一拍再调
 * g4_io_cmd_run() 就会又开一条新命令, 把刚拿到的 OK 覆盖成 RUNNING ——
 * 调用方永远看不到 OK, 于是每 5ms 发一条 AT。
 */
static void test_result_delivered_next_tick(void)
{
    g4_cmd_res_t r1, r2, r3;
    int          at_before, at_after;

    printf("[11] 命令结果必须在下一拍被交付(不能被新命令覆盖)\n");
    g4_io_drain_rx();
    host_tx_reset();

    /* 第 1 拍: 发出 AT, 还没结果 */
    r1 = g4_io_cmd_run("AT", "OK", 1000u);
    CHECK(r1 == G4_CMD_RUNNING, "第 1 拍: 命令在途 (res=%d)", (int)r1);

    /* 第 2 拍: 收包泵(在主循环每拍开头)先把命令完成了 */
    feed("\r\n+MATREADY\r\nOK\r\n", 17u);
    g4_io_pump();

    /* 第 3 拍: 调用方来取结果 —— 必须是 OK, 且【不能再发一条 AT】 */
    at_before = (int)(strstr(host_tx_take(), "AT\r\n") != NULL);
    r2 = g4_io_cmd_run("AT", "OK", 1000u);
    at_after = 0;
    {
        const char *p = host_tx_take();
        while ((p = strstr(p, "AT\r\n")) != NULL) { at_after++; p++; }
    }
    CHECK(r2 == G4_CMD_OK, "下一拍取到的是 OK, 不是 RUNNING (res=%d)", (int)r2);
    CHECK(at_after == at_before, "没有因为取结果而多发一条 AT (AT 条数 %d -> %d)",
          at_before, at_after);

    /* 结果被取走之后, 下一次调用才开始新命令 */
    r3 = g4_io_cmd_run("AT+CSQ", "+CSQ:", 1000u);
    CHECK(r3 == G4_CMD_RUNNING, "结果取走后可以正常开下一条命令 (res=%d)", (int)r3);
}

/**
 * 【回归用例】等 '>' 提示时模组回 ERROR —— 必须【立刻】判失败
 *
 * 真实踩过的坑: ERROR 的判定被套在"关键字 != NULL"里面, 而等 '>' 提示用的是
 * g4_io_cmd_run_prompt()(关键字是 NULL), 于是模组明明回了 ERROR, 我们既不认提示
 * 也不认错误, 只能干等满 15s 超时 —— 发布失败被拖慢、断网判定变迟钝。
 */
static void test_error_while_waiting_prompt(void)
{
    g4_cmd_res_t r;
    uint32_t     t0;

    printf("[12] 等 '>' 提示时回 ERROR 要立刻判失败\n");
    g4_io_drain_rx();
    host_tick_set(10000u);

    r = g4_io_cmd_run_prompt("AT+MQTTPUB=0,\"t\",1,0,0,4", 5000u);
    CHECK(r == G4_CMD_RUNNING, "命令已发出, 等待提示 (res=%d)", (int)r);

    t0 = get_tick_ms();
    feed("\r\nERROR\r\n", 9u);
    g4_io_pump();
    r = g4_io_cmd_poll();

    CHECK(r == G4_CMD_ERR, "★ 立刻返回 ERR, 而不是等满 5s 超时 (res=%d)", (int)r);
    CHECK((get_tick_ms() - t0) < 100u, "判定耗时 %lums (远小于 5000ms)",
          (unsigned long)(get_tick_ms() - t0));
}

/**
 * 【回归用例】等 '>' 提示时模组【直接回了 OK】—— 提示符丢失, 必须立刻返回 NOPROMPT
 *
 * 现场实锤: 发 AT+MQTTPUB 后模组只回 "+MATREADY" + "OK", '>' 被吞了。
 * 这时模组很可能已经进了数据模式在等 payload, 我们后面发的每条 AT 都会被它
 * 当 payload 吃掉 —— 所以不能等满 15s 超时, 也不能原地重试, 必须让上层去断电重启。
 */
static void test_prompt_lost(void)
{
    g4_cmd_res_t r;
    uint32_t     t0;

    printf("[13] 等 '>' 时模组直接回 OK -> 立刻报 NOPROMPT(提示符丢失)\n");
    g4_io_drain_rx();
    host_tick_set(20000u);

    r = g4_io_cmd_run_prompt("AT+MQTTPUB=0,\"mod/mod_upload\",1,0,0,9", 15000u);
    CHECK(r == G4_CMD_RUNNING, "命令已发出, 等待提示 (res=%d)", (int)r);

    t0 = get_tick_ms();
    feed("\r\n+MATREADY\r\nOK\r\n", 17u);
    g4_io_pump();
    r = g4_io_cmd_poll();

    CHECK(r == G4_CMD_NOPROMPT, "★ 立刻返回 NOPROMPT (res=%d)", (int)r);
    CHECK((get_tick_ms() - t0) < 100u, "判定耗时 %lums (远小于 15000ms)",
          (unsigned long)(get_tick_ms() - t0));

    /* 关键字模式(非 prompt)下, "OK" 仍然是正常命中, 不受影响 */
    g4_io_drain_rx();
    g4_io_cmd_begin("AT", "OK", 1000u);
    feed("\r\nOK\r\n", 6u);
    g4_io_pump();
    CHECK(g4_io_cmd_poll() == G4_CMD_OK, "普通命令等 OK 时不受 NOPROMPT 影响");
}

/**
 * 【回归用例★2026-10-07★】带属主的命令: 外人既偷不走结果, 也触发不了它的超时
 *
 * ---------------------------------------------------------------------------
 * 现场证据 log/新建 文本文档.txt(2026-10-07 15:40, OTA 升级):
 *   15:40:03.809  OTA 发 AT+MHTTPCFG="ssl",0,1,0(超时 10s) —— 属主是 OTA
 *   15:40:03.828  发布子流程来发 AT+MQTTPUB: 引擎被占, 【一个字节都没发出去】
 *   15:40:13.814  [G4] PUB fail(等 '>' 超时) —— 正好 03.809 + 10.005s
 *                 = 发布流程拿到了【OTA 那条命令的】TIMEOUT 当成自己的失败
 *   15:40:28 / 43 再来两次 -> 连续 3 次 -> 判"链路已断" -> 4G_RST 断电重启模组
 *   15:40:44~47   OTA 不知情, 继续往正在重启的模组发 CFG -> +CME ERROR: 652
 *                 -> ota.start 失败(2002), percent 恒 0
 *
 * 根因在 g4_io_cmd_poll(): 结果快照和超时判定对两个调用方是【共享】的。谁先来
 * poll, 谁就把对方的超时结果拿走, 还顺手把引擎置成 IDLE(把对方的命令掐掉)。
 *
 * 修法: 命令带属主, 只有属主本人能取结果 / 触发超时。
 * ---------------------------------------------------------------------------
 */
static void test_owner_protects_command(void)
{
    g4_cmd_res_t r;
    uint32_t     t0;
    char         tx[512];

    printf("[14] ★带属主的命令: 外人偷不走结果, 也触发不了它的超时★\n");
    g4_io_drain_rx();
    host_tx_reset();
    host_tick_set(20000u);
    t0 = host_tick_get();

    /* ---- OTA 起一条 10 秒超时的命令 ---- */
    r = g4_io_cmd_run_owned("AT+MHTTPCFG=\"ssl\",0,1,0", "OK", 10000u, G4_OWNER_OTA);
    CHECK(r == G4_CMD_RUNNING, "OTA 发出 CFG 后在途 (res=%d)", (int)r);
    CHECK(g4_io_cmd_owner() == G4_OWNER_OTA,
          "引擎属主 = OTA (实际 %d)", (int)g4_io_cmd_owner());
    tx[0] = '\0';
    (void)snprintf(tx, sizeof(tx), "%s", host_tx_take());
    CHECK(strstr(tx, "MHTTPCFG") != NULL, "CFG 确实写进串口了");

    /* ---- 发布子流程(无属主)来抢: 只能拿到 RUNNING ---- */
    r = g4_io_cmd_run_prompt("AT+MQTTPUB=0,\"mod/status\",1,1,0,5", 15000u);
    CHECK(r == G4_CMD_RUNNING, "发布抢不到引擎, 只拿到 RUNNING (res=%d)", (int)r);
    CHECK(strstr(host_tx_take(), "MQTTPUB") == NULL,
          "★发布那条命令一个字节都没发出去(被正确挡住了)");
    CHECK(g4_io_cmd_owner() == G4_OWNER_OTA, "属主仍然是 OTA, 没被抢走");

    /* ---- 关键: 时间推过 OTA 的 10 秒超时。发布【绝不能】拿到这个 TIMEOUT ---- */
    host_tick_set(t0 + 12000u);
    r = g4_io_cmd_run_prompt("AT+MQTTPUB=0,\"mod/status\",1,1,0,5", 15000u);
    CHECK(r == G4_CMD_RUNNING,
          "★★ 过了 OTA 的超时点, 发布拿到的仍是 RUNNING, 不是 TIMEOUT (res=%d) ★★",
          (int)r);

    /* ---- 属主本人来取: 这才是它自己的超时 ---- */
    r = g4_io_cmd_run_owned(NULL, "OK", 10000u, G4_OWNER_OTA);
    CHECK(r == G4_CMD_TIMEOUT, "OTA 本人取到自己的 TIMEOUT (res=%d)", (int)r);

    /* ---- 引擎回到空闲, 发布随即能用了 ---- */
    CHECK(g4_io_cmd_idle(), "超时被取走后引擎回到空闲");
    r = g4_io_cmd_run_prompt("AT+MQTTPUB=0,\"mod/status\",1,1,0,5", 15000u);
    CHECK(r == G4_CMD_RUNNING, "发布现在能拿到引擎了 (res=%d)", (int)r);
    CHECK(strstr(host_tx_take(), "MQTTPUB") != NULL, "而且真的发出去了");

    /* ---- 交还: 属主不再 poll 时必须能主动放开引擎 ---- */
    g4_io_cmd_abort();
    CHECK(g4_io_cmd_idle(), "准备: 清干净");
    (void)g4_io_cmd_run_owned("AT+MHTTPDEL=0", "OK", 10000u, G4_OWNER_OTA);
    CHECK(!g4_io_cmd_idle(), "OTA 又占着引擎");
    g4_io_cmd_release(G4_OWNER_NONE);
    CHECK(!g4_io_cmd_idle(), "非属主调 release 【不该】放开别人的命令");
    g4_io_cmd_release(G4_OWNER_OTA);
    CHECK(g4_io_cmd_idle(), "★属主交还后引擎空闲(否则 MQTT 会被永久饿死)★");

    /* ---- g4_io_cmd_tx_seq(): ota_http 那道"没发出去就别推进步骤"安全网的判据 ----
     * 2026-10-07 16:06 现场: CONNECTING 的第 1 步(AT+MHTTPCFG="ssl")整条没发出去,
     * 状态机却照样推进 -> 实例上没配 ssl -> https 请求回 +CME ERROR: 655(SSL 未使用)
     * -> 升级失败 2002。conn_cmd() 现在靠这个计数判断"本步到底发出去过没有"。 */
    {
        uint32_t s0;

        /* 先让 OTA 把引擎占住 */
        g4_io_cmd_abort();
        (void)g4_io_cmd_run_owned("AT+MHTTPDEL=1", "OK", 10000u, G4_OWNER_OTA);
        CHECK(!g4_io_cmd_idle(), "准备: OTA 占着引擎");
        s0 = g4_io_cmd_tx_seq();

        /* 另一个属主来发: 抢不到 => 一个字节都没出去 => tx_seq 【不能】变 */
        (void)g4_io_cmd_run_owned("AT+MHTTPDEL=2", "OK", 10000u, G4_OWNER_CONSOLE);
        CHECK(g4_io_cmd_tx_seq() == s0,
              "★抢不到引擎 => tx_seq 不变(调用方据此判断自己这条没发出去)★");
        CHECK(strstr(host_tx_take(), "MHTTPDEL=2") == NULL, "而且串口上确实没有它");

        /* 空闲时真发一条: tx_seq 【必须】自增 */
        g4_io_cmd_abort();
        s0 = g4_io_cmd_tx_seq();
        (void)g4_io_cmd_run_owned("AT+MHTTPDEL=2", "OK", 10000u, G4_OWNER_OTA);
        CHECK(g4_io_cmd_tx_seq() != s0, "★真发出去了 => tx_seq 自增(安全网据此放行)★");
        g4_io_cmd_abort();
    }

    /* ---- ★★2026-10-07 死锁回归★★ 属主失联的命令必须被强制收回 ----
     * 现场(18:21): 引擎上一条"标了属主却再没人推进"的命令, 让【整机 2 分多钟
     * 一条 AT 都发不出去】—— OTA 静默卡在 poll_teardown, 发布每 30s 只打一行
     * "发布放弃(命令引擎被外部流程(OTA)长时间占用)", puback 从头到尾是 0。
     * 只加属主保护、不加死账回收, 就是这个结果。 */
    {
        uint32_t t;

        g4_io_cmd_abort();
        host_tick_set(50000u);
        t = host_tick_get();

        /* 一条"属主标了 CONSOLE、但没人再来 poll"的命令, 超时 1000ms */
        (void)g4_io_cmd_run_owned("AT+CEREG?", "+CEREG:", 1000u, G4_OWNER_CONSOLE);
        CHECK(!g4_io_cmd_idle(), "准备: 引擎被一条属主已失联的命令占着");

        /* 还没到回收窗口: 外人仍然拿不到(证明不会误伤正在推进的命令) */
        host_tick_set(t + 2000u);
        CHECK(g4_io_cmd_owner() != G4_OWNER_NONE, "窗口内: 引擎仍算被占");

        /* 超过 timeout(1000) + 宽限(5000) = 6000ms: 必须被强制收回 */
        host_tick_set(t + 7000u);
        CHECK(g4_io_cmd_idle(), "★超时+宽限后死账被强制收回(不再死锁)★");

        /* 收回之后, 无属主的调用方立刻发得出去 */
        (void)g4_io_cmd_run_prompt("AT+MQTTPUB=0,\"mod/status\",1,1,0,5", 15000u);
        CHECK(strstr(host_tx_take(), "MQTTPUB") != NULL, "★收回后发布立刻发得出去★");

        g4_io_cmd_abort();
        host_tick_set(1000u);
    }
}

int main(void)
{
    host_log_enable(0);
    host_tick_set(1000u);

    g4_io_init();
    g4_urc_init();
    g4_urc_set_recv_cb(test_recv_cb);

    test_plain_line_and_at();
    test_binary_payload();
    test_two_publishes();
    test_split_publish();
    test_truncated_then_resync();
    test_bad_len_then_resync();
    test_prompt_strict();
    test_publish_during_at();
    test_conn_urc();
    test_multi_line_resp();
    test_result_delivered_next_tick();
    test_error_while_waiting_prompt();
    test_prompt_lost();
    test_owner_protects_command();

    printf("\n==== %d passed, %d FAILED ====\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
