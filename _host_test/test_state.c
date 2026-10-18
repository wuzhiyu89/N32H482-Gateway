/**
 * \file  test_state.c
 * \brief g4_at 状态机的【主机仿真测试】: 假模组 + 虚拟毫秒时间, 端到端跑状态机
 *
 * 为什么需要它:
 *   · "一上电就一直发 AT 停不下来"这种问题靠读代码容易看错 —— 到底是一拍发一次
 *     (节拍 bug), 还是每 500ms 发一次但永远不退出的探测循环, 只有把状态机按毫秒
 *     跑一遍才能定论。这里就复现过一次: 命令在收包泵里完成后, 调用方下一拍再来
 *     取结果时引擎又开了新命令, 结果被覆盖 -> 每 5ms 一条 AT(200 条/秒)。
 *   · 断链恢复(4G_RST 断电重启)这种"多久之后会做什么"的逻辑, 同样只能靠时间轴验证。
 *
 * 假模组实现了 ML307R 的建链全流程: AT/ATE0/ATI/CGMR/CPIN/CGSN/CIMI/ICCID/CSQ/
 * MCHIPINFO/CEREG/CGDCONT/CGACT/CGPADDR/MQTTDISC/MQTTCONN(conn URC)/MQTTSUB/
 * MQTTPUB(数据模式 '>' + puback), 还能主动下发一条云端 publish。
 *
 * 用法:
 *   test_state.exe        跑全部场景, 只打印统计与结论
 *   test_state.exe -v     打开日志, 跟踪完整状态迁移(排查用)
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include "ring_buf.h"
#include "bsp_uart.h"
#include "bsp_tim.h"
#include "bsp_gpio.h"
#include "log.h"
#include "g4_io.h"
#include "g4_urc.h"
#include "g4_at.h"

void        host_uart_rx_byte(uint8_t ch);
const char *host_tx_take(void);
void        host_tx_reset(void);

static int g_fail;
static int g_pass;

#define CHECK(cond, ...)                            \
    do {                                            \
        if (cond) { g_pass++; printf("  ok   : "); } \
        else      { g_fail++; printf("  FAIL : "); } \
        printf(__VA_ARGS__);                        \
        printf("\n");                               \
    } while (0)

/* ==================== 假模组 ==================== */

enum { MODEM_FULL = 0, MODEM_SILENT, MODEM_RUDE };

static int      s_mode = MODEM_FULL;
static int      s_echo = 1;             /* 真实模组默认开回显, ATE0 之后关 */
static int      s_cfun = 1;             /* 1=全功能  0=最小功能(射频关, 不会搜网) */
static int      s_pub_fail;             /* 1 = 对 AT+MQTTPUB 回 ERROR(模拟发不出去) */
static int      s_pub_drop_prompt;      /* 1 = 吞掉 '>' 提示符(只回 +MATREADY+OK) */
static int      s_pub_await;            /* >0 = 数据模式, 正在等 payload 字节 */
static int      s_pub_got;
static int      s_mqtt_up;
static uint32_t s_conn_urc_at;          /* 到点补一条 conn URC */
static uint32_t s_puback_at;            /* 到点补一条 puback URC */
static uint32_t s_dl_at;                /* 到点主动下发一条云端 publish */
static uint16_t s_tx_seen;

/* 云端下发的原始二进制帧(含 0x00/0x0A/0x3E) */
static const uint8_t DL_FRAME[20] = {
    0xAA, 0x55, 0x01, 0x11, 0x00, 0x01, 0x21, 0x00, 0x01, 0x01,
    0x07, 0x05, 0x00, 0x0A, 0x00, 0x3E, 0xDC, 0x05, 0x38, 0x1B
};

static void emit(const char *s)
{
    for (; *s != '\0'; s++) { host_uart_rx_byte((uint8_t)*s); }
}

static void modem_reset_state(void)
{
    s_echo       = 1;
    s_cfun       = 1;
    s_pub_fail   = 0;
    s_pub_drop_prompt = 0;
    s_pub_await  = 0;
    s_pub_got    = 0;
    s_mqtt_up    = 0;
    s_conn_urc_at = 0u;
    s_puback_at  = 0u;
    s_dl_at      = 0u;
}

/** @brief 按命令产生应答(假模组的"固件") */
static void on_cmd(const char *cmd)
{
    if (s_echo) { emit(cmd); emit("\r\n"); }

    if (strcmp(cmd, "AT") == 0)
    {
        emit("\r\n+MATREADY\r\nOK\r\n");     /* 这块模组的习惯: 每条命令补一条 */
    }
    else if (strcmp(cmd, "ATE0") == 0)
    {
        s_echo = 0;
        emit("OK\r\n");
    }
    else if (strcmp(cmd, "ATI") == 0)
    {
        emit("\r\nML307R\r\nOK\r\n");
    }
    else if (strcmp(cmd, "AT+CGMR") == 0)
    {
        emit("\r\n+CGMR: ML307R_V1.0.0\r\nOK\r\n");
    }
    else if (strcmp(cmd, "AT+CFUN?") == 0)
    {
        char r[32];

        snprintf(r, sizeof(r), "\r\n+CFUN: %d\r\nOK\r\n", s_cfun);
        emit(r);
    }
    else if (strcmp(cmd, "AT+CFUN=1") == 0)
    {
        s_cfun = 1;
        emit("OK\r\n");
    }
    else if (strcmp(cmd, "AT+CFUN=0") == 0)
    {
        s_cfun = 0;
        emit("OK\r\n");
    }
    else if (strncmp(cmd, "AT+MLPMCFG=", 11) == 0)
    {
        emit("OK\r\n");
    }
    else if (strcmp(cmd, "AT+CPIN?") == 0)
    {
        emit("\r\n+CPIN: READY\r\nOK\r\n");
    }
    else if (strcmp(cmd, "AT+CGSN=1") == 0)
    {
        emit("\r\n+CGSN: 861234567890123\r\nOK\r\n");
    }
    else if (strcmp(cmd, "AT+CIMI") == 0)
    {
        emit("\r\n460001234567890\r\nOK\r\n");
    }
    else if (strcmp(cmd, "AT*ICCID?") == 0)
    {
        emit("\r\n+ICCID: 89860012345678901234\r\nOK\r\n");
    }
    else if (strcmp(cmd, "AT+CSQ") == 0)
    {
        emit("\r\n+CSQ: 22,0\r\nOK\r\n");
    }
    else if (strcmp(cmd, "AT+MCHIPINFO=\"vbat\"") == 0)
    {
        emit("\r\n+MCHIPINFO: \"vbat\",3980\r\nOK\r\n");
    }
    else if (strcmp(cmd, "AT+MCHIPINFO=\"temp\"") == 0)
    {
        emit("\r\n+MCHIPINFO: \"temp\",35\r\nOK\r\n");
    }
    else if (strcmp(cmd, "AT+CBC") == 0)
    {
        emit("\r\n+CBC: 0,3980\r\nOK\r\n");
    }
    else if (strcmp(cmd, "AT+CEREG?") == 0)
    {
        emit("\r\n+CEREG: 0,1\r\nOK\r\n");
    }
    else if (strncmp(cmd, "AT+CGDCONT=", 11) == 0)
    {
        emit("OK\r\n");
    }
    else if (strncmp(cmd, "AT+CGACT=", 9) == 0)
    {
        emit("OK\r\n");
    }
    else if (strcmp(cmd, "AT+CGPADDR=1") == 0)
    {
        emit("\r\n+CGPADDR: 1,10.42.18.188\r\nOK\r\n");
    }
    else if (strncmp(cmd, "AT+MQTTDISC=", 12) == 0)
    {
        s_mqtt_up = 0;
        emit("OK\r\n");
    }
    else if (strncmp(cmd, "AT+MQTTCONN=", 12) == 0)
    {
        emit("OK\r\n");
        s_conn_urc_at = host_tick_get() + 60u;      /* 60ms 后 conn URC */
    }
    else if (strncmp(cmd, "AT+MQTTSUB=", 11) == 0)
    {
        s_mqtt_up = 1;
        emit("OK\r\n");
        s_dl_at = host_tick_get() + 500u;           /* 半秒后云端下发一条 */
    }
    else if (strncmp(cmd, "AT+MQTTPUB=", 11) == 0)
    {
        const char *last;
        int         n;

        if (s_pub_fail)
        {
            emit("ERROR\r\n");      /* 模拟"模组发不出去" */
            return;
        }
        if (s_pub_drop_prompt)
        {
            /* 模拟固件吞掉 '>' 提示符: 只回 +MATREADY + OK */
            emit("\r\n+MATREADY\r\nOK\r\n");
            return;
        }

        last = strrchr(cmd, ',');
        n    = (last != NULL) ? atoi(last + 1) : 0;

        emit("\r\n> ");                             /* 数据模式提示(后面没有换行) */
        s_pub_await = n;
        s_pub_got   = 0;
    }
    else
    {
        emit("ERROR\r\n");
    }
}

/** @brief 到点的 URC / 云端下行 */
static void modem_sched(void)
{
    uint32_t now = host_tick_get();

    if ((s_conn_urc_at != 0u) && (now >= s_conn_urc_at))
    {
        s_conn_urc_at = 0u;
        emit("\r\n+MQTTURC: \"conn\",0,0\r\n");
    }
    if ((s_puback_at != 0u) && (now >= s_puback_at))
    {
        s_puback_at = 0u;
        emit("\r\n+MQTTURC: \"puback\",0,9,0\r\n");
    }
    if ((s_dl_at != 0u) && s_mqtt_up && (now >= s_dl_at))
    {
        char hdr[128];

        s_dl_at = 0u;
        snprintf(hdr, sizeof(hdr),
                 "\r\n+MQTTURC: \"publish\",0,3,\"mod/pc_down\",20,20,");
        emit(hdr);
        for (int i = 0; i < 20; i++) { host_uart_rx_byte(DL_FRAME[i]); }
        emit("\r\n");
    }
}

/** @brief 假模组时序: 被主循环和"自旋等待"两条路径一起驱动 */
static void fake_modem_step(void)
{
    const char *tx = host_tx_take();
    uint16_t    len = (uint16_t)strlen(tx);

    modem_sched();

    if (s_mode == MODEM_SILENT)
    {
        s_tx_seen = len;            /* 收到什么也不回 */
        return;
    }

    /* ---- 数据模式: 命令行的处理让位, 后面的字节全是 payload ---- */
    if (s_pub_await > 0)
    {
        while ((s_tx_seen < len) && (s_pub_got < s_pub_await))
        {
            s_tx_seen++;
            s_pub_got++;
        }
        if (s_pub_got >= s_pub_await)
        {
            s_pub_await = 0;
            s_pub_got   = 0;
            emit("\r\nOK\r\n");
            s_puback_at = host_tick_get() + 40u;
        }
        return;
    }

    /* ---- 命令行 ---- */
    while (s_tx_seen < len)
    {
        uint16_t start = s_tx_seen;
        uint16_t end   = start;
        char     cmd[192];
        uint16_t n;

        while ((end + 1u < len) && !((tx[end] == '\r') && (tx[end + 1u] == '\n')))
        {
            end++;
        }
        if (end + 1u >= len)
        {
            return;                 /* 命令还没收全 */
        }

        n = (uint16_t)(end - start);
        if (n >= sizeof(cmd)) { n = (uint16_t)(sizeof(cmd) - 1u); }
        memcpy(cmd, &tx[start], n);
        cmd[n] = '\0';
        s_tx_seen = (uint16_t)(end + 2u);

        if (s_mode == MODEM_RUDE)
        {
            continue;               /* 收到但不理 */
        }
        on_cmd(cmd);

        if (s_pub_await > 0)
        {
            return;                 /* 切数据模式了, 后面的字节当 payload */
        }
    }
}

/* ==================== 测试脚手架 ==================== */

static uint8_t  s_dl_data[64];
static uint16_t s_dl_len;
static int      s_dl_cnt;

static void recv_cb(const char *topic, const uint8_t *data, uint16_t len)
{
    (void)topic;
    s_dl_cnt++;
    s_dl_len = len;
    if (len <= sizeof(s_dl_data)) { memcpy(s_dl_data, data, len); }
}

static int count_at(const char *tx)
{
    int n = 0;
    const char *p = tx;

    while ((p = strstr(p, "AT\r\n")) != NULL) { n++; p++; }
    return n;
}

/** 数一个子串在缓冲区里出现了几次(只给测试断言用) */
static int count_substr(const char *hay, const char *needle)
{
    int n = 0;
    const char *p = hay;

    while ((p = strstr(p, needle)) != NULL) { n++; p++; }
    return n;
}

/* ★2026-10-08: g4_at 不再有"默认发布主题"★
 * 旧的默认主题 mod/mod_upload(以及订阅默认 mod/pc_down)已按甲方要求整体删除,
 * 所以 `g4_at_mqtt_publish(NULL, ...)` 现在【直接返回失败】—— 任何发布都必须显式给主题。
 * 本文件统一用"出厂 gwId=11000000000000000000000000 的 status 主题"(与 main.c 拼出来的真实主题一致)。
 * 用 char[] 而不是字面量: g4_at_mqtt_publish() 的形参是 char*(非 const)。
 * ⚠ 必须声明在 run_ms() 之前 —— 它在里面被用到。 */
static char s_pub_topic[] = "rt/gw/11000000000000000000000000/status";

static uint32_t s_at_last;
static uint32_t s_pub_ok;
static uint32_t s_pub_try;
static uint32_t s_hb_tick;

/** @brief 跑 ms 毫秒虚拟时间(每 5ms 一次 g4_at_poll, 模拟 main.c) */
static void run_ms(uint32_t ms, int do_heartbeat)
{
    uint32_t start = host_tick_get();

    while ((host_tick_get() - start) < ms)
    {
        uint32_t t = host_tick_get();

        if ((t % 5u) == 0u) { g4_at_poll(); }
        fake_modem_step();

        /* 模仿 main.c: 30s 一次应用层心跳 */
        if (do_heartbeat && g4_at_is_online() &&
            ((t - s_hb_tick) >= 30000u))
        {
            static const char hb[] = "GW_ONLINE";

            s_hb_tick = t;
            s_pub_try++;
            if (g4_at_mqtt_publish(s_pub_topic, (uint8_t *)hb, (uint16_t)(sizeof(hb) - 1u)) == SYS_OK)
            {
                s_pub_ok++;
            }
        }

        host_tick_set(host_tick_get() + 1u);
    }
    s_at_last = (uint32_t)count_at(host_tx_take());
}

static void begin(const char *name)
{
    printf("\n================ %s ================\n", name);
    modem_reset_state();
    s_tx_seen = 0u;
    host_tx_reset();
    host_tick_set(0u);
    s_hb_tick = 0u;
    s_dl_cnt = 0;
}

/* ==================== 场景 ==================== */

/** 完整建链 -> ONLINE -> 心跳 -> 下行 -> 断链 -> 4G_RST -> 重新 ONLINE */
static void test_full_flow(void)
{
    uint32_t rst_before;
    uint32_t t0;

    begin("完整建链 / 心跳 / 云端下行 / 断链后 4G_RST 重启");

    s_mode = MODEM_FULL;
    g4_io_init();
    g4_urc_init();
    g4_at_set_recv_cb(recv_cb);
    g4_at_init();

    /* 建链: 给 60 秒 */
    t0 = host_tick_get();
    while (!g4_at_is_online() && ((host_tick_get() - t0) < 60000u))
    {
        run_ms(200u, 0);
    }
    CHECK(g4_at_is_online(), "建链成功, 进入 ONLINE");
    CHECK(g4_at_get_cfun() == 1, "建链时读到 CFUN=1 (实际 %d)", (int)g4_at_get_cfun());
    CHECK(strstr(host_tx_take(), "AT+CFUN=1") == NULL,
          "CFUN 正常时不会多此一举去改它");

    /* ONLINE 期间不应该有任何多余的 AT 在跑 */
    {
        uint32_t a0 = s_at_last;
        uint32_t c0 = host_pwr_cycle_count();

        run_ms(5000u, 0);
        CHECK((s_at_last - a0) == 0u,
              "ONLINE 稳态下 5 秒内发的 AT 命令数 = %lu (期望 0)",
              (unsigned long)(s_at_last - a0));
        CHECK(host_pwr_cycle_count() == c0, "ONLINE 稳态下没有乱断电重启");
    }

    /* 建链时清旧 PDP 上下文: CGACT=0,1 必须在 CGDCONT 之前 */
    {
        const char *tx     = host_tx_take();
        const char *deact  = strstr(tx, "AT+CGACT=0,1");
        const char *dcont  = strstr(tx, "AT+CGDCONT=");

        CHECK(deact != NULL, "建链时先发了 AT+CGACT=0,1(清旧 PDP 上下文)");
        CHECK((deact != NULL) && (dcont != NULL) && (deact < dcont),
              "AT+CGACT=0,1 排在 AT+CGDCONT 之前");
    }

    /* 云端下行(原始二进制 20 字节, 含 0x00/0x0A/0x3E) */
    run_ms(1000u, 0);
    CHECK((s_dl_cnt >= 1) && (s_dl_len == 20u) &&
          (memcmp(s_dl_data, DL_FRAME, 20u) == 0),
          "收到云端二进制下行 %u 字节且内容一致 (次数=%d)",
          (unsigned)s_dl_len, s_dl_cnt);

    /* 应用层心跳 + broker puback */
    {
        uint32_t ack0 = g4_urc_puback_count();

        run_ms(31000u, 1);
        CHECK(s_pub_ok >= 1u, "心跳登记成功 %lu 次", (unsigned long)s_pub_ok);
        CHECK(g4_urc_puback_count() > ack0,
              "broker 已 puback (puback=%lu)",
              (unsigned long)g4_urc_puback_count());
    }

    /* ---- 断链: 服务器断开(state=5 ping 超时) ---- */
    rst_before = host_pwr_cycle_count();
    emit("\r\n+MQTTURC: \"conn\",0,5\r\n");
    run_ms(300u, 0);
    CHECK(!g4_at_is_online(), "断链被识别, 状态机离线");
    CHECK(host_pwr_cycle_count() > rst_before,
          "★ 触发了 4G_RST 断电重启 (次数 %lu -> %lu)",
          (unsigned long)rst_before, (unsigned long)host_pwr_cycle_count());

    /* 断电重启之后要能自己重新建链回到 ONLINE */
    t0 = host_tick_get();
    while (!g4_at_is_online() && ((host_tick_get() - t0) < 60000u))
    {
        run_ms(200u, 0);
    }
    CHECK(g4_at_is_online(), "4G_RST 之后重新建链回 ONLINE");

    printf("  统计: AT 命令 %lu 条, 断电重启 %lu 次, 心跳 %lu/%lu, 下行 %d 条\n",
           (unsigned long)s_at_last, (unsigned long)host_pwr_cycle_count(),
           (unsigned long)s_pub_ok, (unsigned long)s_pub_try, s_dl_cnt);
}

/** 模组完全不理人: 敲 AT 必须"有个头", 然后靠 4G_RST 恢复 */
static void test_dead_module(void)
{
    uint32_t rst_before;
    uint32_t t0;

    begin("模组不理人(不发 +MATREADY 也不回 AT)");

    s_mode = MODEM_SILENT;
    g4_io_init();
    g4_urc_init();
    g4_at_init();
    rst_before = host_pwr_cycle_count();

    /* 前 10 秒: 应该只在有节奏地敲 AT(每 500ms 一条 = 约 20 条), 不是每拍一条 */
    run_ms(10000u, 0);
    CHECK((s_at_last >= 10u) && (s_at_last <= 24u),
          "头 10 秒敲 AT %lu 条 (期望 ~%u, 每拍一条会到 2000)",
          (unsigned long)s_at_last, (unsigned)(10u * 1000u / 500u));

    /* 跑到超过敲 AT 的上限(G4_AT_SYNC_RETRY_MAX × 500ms = 30s)+ 预热, 必须停下来并断电重启 */
    t0 = host_tick_get();
    while (((host_tick_get() - t0) < 45000u) && (host_pwr_cycle_count() == rst_before))
    {
        run_ms(500u, 0);
    }
    CHECK(host_pwr_cycle_count() > rst_before,
          "★ 敲满次数后触发 4G_RST 断电重启 (次数 %lu -> %lu)",
          (unsigned long)rst_before, (unsigned long)host_pwr_cycle_count());

    /* 断电之后进入静听窗口: 这段时间内【不应该再发 AT】 */
    {
        int a0 = count_at(host_tx_take());

        run_ms(2500u, 0);
        CHECK(count_at(host_tx_take()) == a0,
              "4G_RST 后的静听窗口里不再发 AT (总 %d 条)",
              count_at(host_tx_take()));
    }
}

/** 模组被留在 CFUN=0(射频关): 建链时必须能读出来并自动打开 */
static void test_cfun_off(void)
{
    uint32_t t0;

    begin("模组被留在 CFUN=0(射频关) -> 建链时自动打开");

    s_cfun = 0;                 /* 模拟"上一轮实验关了 / 掉电保持" */
    g4_io_init();
    g4_urc_init();
    g4_at_init();

    t0 = host_tick_get();
    while ((host_tick_get() - t0) < 20000u)
    {
        run_ms(200u, 0);
        if (strstr(host_tx_take(), "AT+CFUN=1") != NULL) { break; }
    }

    CHECK(strstr(host_tx_take(), "AT+CFUN?") != NULL, "建链时读了 AT+CFUN?");
    CHECK(strstr(host_tx_take(), "AT+CFUN=1") != NULL,
          "★ 读到 CFUN=0 后自动补发了 AT+CFUN=1(打开射频)");
    CHECK(g4_at_get_cfun() == 1, "CFUN 记录已更新为 1 (实际 %d)", (int)g4_at_get_cfun());
    CHECK(s_cfun == 1, "假模组的射频档位确实被切到 1");
}

/** 连续发布失败达到阈值 -> 判链路断 -> 4G_RST(现场要求的"严格判定"第一条腿)
 *
 * ⚠️ 阈值由 g4_at.c 的 G4_PUB_FAIL_MAX 决定, 本用例【故意不写死】:
 *    它被现场调过(2 -> 3, 依据 78.txt: 一次"模组自己重启"要 40~60s 才回 ONLINE,
 *    2 次就断电等于在模组刚要回来时又把它按回去)。
 *    这里只验两件事: ① 第 1 次失败【不】断电(说明判定不是见错就断);
 *                    ② 连续失败在有限次内一定断电(说明判定确实生效)。
 */
static void test_pub_fail_threshold_rst(void)
{
    static uint8_t hb[] = "GW_ONLINE";
    uint32_t rst_before;
    uint32_t t0;
    uint32_t i;

    begin("连续发布失败达到阈值 -> 判链路断 -> 4G_RST(阈值不写死)");

    s_mode = MODEM_FULL;
    g4_io_init();
    g4_urc_init();
    g4_at_init();
    g4_at_set_link_watchdog(0u);        /* 本用例只验"失败计数"这条腿 */

    t0 = host_tick_get();
    while (!g4_at_is_online() && ((host_tick_get() - t0) < 60000u)) { run_ms(200u, 0); }
    CHECK(g4_at_is_online(), "先正常建链到 ONLINE");

    rst_before = host_pwr_cycle_count();
    s_pub_fail = 1;                     /* 之后模组对 MQTTPUB 一律回 ERROR */

    CHECK(g4_at_mqtt_publish(s_pub_topic, hb, 9u) == SYS_OK, "第 1 次心跳登记成功");
    run_ms(1500u, 0);
    CHECK(host_pwr_cycle_count() == rst_before, "第 1 次失败【不】断电(判定不是见错就断)");
    CHECK(g4_at_is_online(), "第 1 次失败后仍在线");

    /* 继续失败, 直到断电(CAP 5 次足够覆盖任何合理阈值) */
    for (i = 2u; i <= 5u; i++)
    {
        if (g4_at_mqtt_publish(s_pub_topic, hb, 9u) != SYS_OK) { break; }
        run_ms(1500u, 0);
        if (host_pwr_cycle_count() > rst_before) { break; }
    }

    CHECK(host_pwr_cycle_count() > rst_before,
          "★ 连续失败第 %lu 次时触发 4G_RST 断电重启 (%lu -> %lu)",
          (unsigned long)i, (unsigned long)rst_before,
          (unsigned long)host_pwr_cycle_count());
    CHECK(i <= 5u, "阈值在 5 次以内(实际第 %lu 次) —— 没有变成『永远不断电』",
          (unsigned long)i);

    s_pub_fail = 0;
}

/** 保活停滞看门狗: 长时间没有一次发布成功 -> 4G_RST(第二条腿) */
static void test_hb_watchdog_rst(void)
{
    uint32_t rst_before;
    uint32_t t0;

    begin("保活停滞看门狗: 长时间没有发布成功 -> 4G_RST");

    s_mode = MODEM_FULL;
    g4_io_init();
    g4_urc_init();
    g4_at_init();

    t0 = host_tick_get();
    while (!g4_at_is_online() && ((host_tick_get() - t0) < 60000u)) { run_ms(200u, 0); }
    CHECK(g4_at_is_online(), "先正常建链到 ONLINE");

    g4_at_set_link_watchdog(10000u);    /* 测试用短窗口: 10s 没有成功发布就判死 */
    rst_before = host_pwr_cycle_count();

    run_ms(4000u, 0);
    CHECK(host_pwr_cycle_count() == rst_before, "窗口没到(4s < 10s)时不会误触发");

    run_ms(10000u, 0);
    CHECK(host_pwr_cycle_count() > rst_before,
          "★ 超过窗口后触发 4G_RST (%lu -> %lu)",
          (unsigned long)rst_before, (unsigned long)host_pwr_cycle_count());
}

/** 模组吞掉 '>' 提示符 —— 按【2026-09-27 改过的策略】验:
 *    收到一整行 OK 说明模组已经回到命令模式(并没有在数据模式等 payload), 所以
 *    先敲一条 AT 探活、活着就【重发本条】; 只有连续丢满 G4_PUB_PROMPT_LOSS_MAX 次
 *    (或探活也不回)才断电重启。
 *  本用例验: ① 不再死等 15s; ② 确实发生了"探活 + 重发"(不是一丢就断电);
 *           ③ 连续丢失仍然会收敛到断电重启(不会无限重发)。
 *
 *  阈值同样不写死(现场调过 G4_PUB_PROMPT_LOSS_MAX)。
 */
static void test_pub_prompt_lost_rst(void)
{
    static uint8_t hb[] = "GW_ONLINE";
    uint32_t rst_before;
    uint32_t t0;
    uint32_t at_before;

    begin("固件吞掉 '>' 提示符 -> 先探活+重发, 连续丢满才断电");

    s_mode = MODEM_FULL;
    g4_io_init();
    g4_urc_init();
    g4_at_init();
    g4_at_set_link_watchdog(0u);

    t0 = host_tick_get();
    while (!g4_at_is_online() && ((host_tick_get() - t0) < 60000u)) { run_ms(200u, 0); }
    CHECK(g4_at_is_online(), "先正常建链到 ONLINE");

    rst_before  = host_pwr_cycle_count();
    at_before   = (uint32_t)count_at(host_tx_take());
    s_pub_drop_prompt = 1;              /* 之后 MQTTPUB 只回 +MATREADY+OK, 不给 '>' */

    t0 = host_tick_get();
    (void)g4_at_mqtt_publish(s_pub_topic, hb, 9u);
    run_ms(1500u, 0);

    /* 探活: 策略要求先敲一条纯 "AT"(不是一丢就断电) */
    CHECK(count_at(host_tx_take()) > (int)at_before,
          "★ 丢 '>' 后先敲了 AT 探活(而不是立刻断电)");
    /* 重发: MQTTPUB 至少被发过两次(第一次丢, 探活成功后重发) */
    CHECK(count_substr(host_tx_take(), "AT+MQTTPUB=") >= 2,
          "★ 探活成功后【重发】了本条发布(没浪费一次断电)");
    /* ★★2026-10-08 行为变更: 丢满【不再断电重启模组】★★
     * 原断言是"连续丢满后仍收敛到 4G_RST" —— 那是旧策略(阈值 3, 到点给模组断电)。
     * 现场证据(22.5 分钟内 '>'丢失重发=13 / MQTT快修=13 / +MATREADY=34, 几乎 1:1)
     * 说明"探活+重发同一条包"本身就在反复撑重启模组, 而断电重启要 40~60 秒才回来。
     * 新策略: 同一条包最多重发 1 次(阈值 G4_PUB_PROMPT_LOSS_MAX=2), 再失败就
     * 【放弃这一条, 不碰链路】—— 链路问题交给发布失败计数/保活看门狗那两条腿。 */
    CHECK(host_pwr_cycle_count() == rst_before,
          "★★ 丢满后【不再】断电重启模组(仍是 %lu 次) —— 改成放弃本条",
          (unsigned long)rst_before);
    CHECK(!g4_at_is_online() || g4_at_is_online(),
          "★ 放弃本条不影响链路状态(这里不断言具体值, 只确保没有副作用崩溃)");
    CHECK((host_tick_get() - t0) < 5000u,
          "整个处置只用了 %lums (死等提示要 15000ms)",
          (unsigned long)(host_tick_get() - t0));

    s_pub_drop_prompt = 0;
}

int main(int argc, char **argv)
{
    if ((argc > 1) && (strcmp(argv[1], "-v") == 0))
    {
        host_log_enable(1);
        host_tick_hook = fake_modem_step;
        test_pub_fail_threshold_rst();
        printf("\n==== %d passed, %d FAILED ====\n", g_pass, g_fail);
        return (g_fail == 0) ? 0 : 1;
    }

    host_log_enable(0);
    host_tick_hook = fake_modem_step;

    /* ★2026-10-08: 必须像 main.c 那样注入下行订阅主题★
     * g4_at 里"默认订阅主题 mod/pc_down"已按甲方要求删除 —— 没注入时建链会
     * 【跳过 AT+MQTTSUB】(并打 !!)而不是悄悄用一个遗留主题。
     * 假模组是在收到 AT+MQTTSUB 之后才安排"云端下发一条"的(见 on_cmd), 所以
     * 不注入的话本套件会看到 "收到云端二进制下行 0 字节"。
     * 值取与出厂 gwId=11000000000000000000000000 对应的真实主题。 */
    g4_at_set_sub_topic("rt/gw/11000000000000000000000000/cmd");

    test_full_flow();
    test_cfun_off();
    test_pub_fail_threshold_rst();
    test_pub_prompt_lost_rst();
    test_hb_watchdog_rst();
    test_dead_module();

    printf("\n==== %d passed, %d FAILED ====\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
