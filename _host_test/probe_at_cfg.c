/**
 * \file  probe_at_cfg.c
 * \brief 一次性探针: 把 g4_at 状态机从冷启动跑到 ONLINE, 打印它发出的【每一条 AT】。
 *
 * 【为什么要它】2026-10-04 第 2 步的核心是"把模组自己的 MQTT 参数下发下去" ——
 * 那些命令是拼出来的字符串(遗嘱正文还要转义), 拼错了在板上只表现为"连不上",
 * 现场极易误判成信号问题。这个探针把命令原样打出来, 在 PC 上就能核对。
 *
 * 不参与正式套件(没有加进 run.ps1); 手动编译:
 *   gcc -std=c99 -I_host_test/stub -Iapp/protocol -Iapp/business -Iapp/common \
 *       -Iapp/ota -Iapp/bootloader -Iapp/ring_buf -Iapp/cjson -Iapp/radio/llcc68 \
 *       -I_host_test -Iinc \
 *       app/protocol/g4_io.c app/protocol/g4_urc.c app/protocol/g4_at.c \
 *       app/business/local_alarm.c app/ring_buf/ring_buf.c \
 *       _host_test/host_stub.c _host_test/probe_at_cfg.c -o _host_test/probe_at_cfg.exe
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include "ring_buf.h"
#include "bsp_tim.h"
#include "bsp_gpio.h"
#include "log.h"
#include "g4_io.h"
#include "g4_urc.h"
#include "g4_at.h"

void        host_uart_rx_byte(uint8_t ch);
void        host_tx_reset(void);
uint32_t    host_tick_get(void);
void        host_tick_set(uint32_t ms);
void        host_log_enable(int on);
const char *host_tx_take(void);

/* ==================== 极简假模组: 什么都回 OK, 只把 MQTTPUB 做成数据模式 ==================== */

static uint16_t s_seen;          /* TX 缓冲里已处理到哪 */
static int      s_await;         /* 数据模式待收字节数 */
static uint32_t s_conn_at;       /* 到点补 conn URC */
static int      s_cfg_reject;    /* 1 = 对 AT+MQTTCFG 回 ERROR(验证失败不影响建链) */

static void emit(const char *s)
{
    for (; *s; s++) { host_uart_rx_byte((uint8_t)*s); }
}

static void on_cmd(const char *cmd)
{
    printf("   >> %s\n", cmd);
    fflush(stdout);

    if (strncmp(cmd, "AT+MQTTPUB=", 11) == 0)
    {
        const char *last = strrchr(cmd, ',');
        s_await = (last != NULL) ? atoi(last + 1) : 0;
        emit("\r\n> ");              /* 数据模式提示, 后面没有换行 */
        return;
    }

    if ((s_cfg_reject != 0) && (strncmp(cmd, "AT+MQTTCFG=", 11) == 0))
    {
        emit("+CME ERROR: 4\r\n");   /* 模拟"模组不认识这条" */
        return;
    }

    if (strcmp(cmd, "AT+CEREG?") == 0)
    {
        emit("+CEREG: 0,1\r\nOK\r\n");
        return;
    }
    if (strcmp(cmd, "AT+CGPADDR=1") == 0)
    {
        emit("+CGPADDR: 1,\"10.0.0.7\"\r\nOK\r\n");
        return;
    }
    if (strcmp(cmd, "AT+CSQ") == 0)
    {
        emit("+CSQ: 26,99\r\nOK\r\n");
        return;
    }

    emit("OK\r\n");

    if (strncmp(cmd, "AT+MQTTCONN=", 12) == 0) { s_conn_at = host_tick_get() + 30u; }
}

static void modem_step(void)
{
    const char *tx  = host_tx_take();
    uint16_t    len = (uint16_t)strlen(tx);

    /* 到点补 conn URC */
    if ((s_conn_at != 0u) && (host_tick_get() >= s_conn_at))
    {
        s_conn_at = 0u;
        emit("\r\n+MQTTURC: \"conn\",0,0\r\n");
    }

    if (s_await > 0)
    {
        while ((s_seen < len) && (s_await > 0)) { s_seen++; s_await--; }
        if (s_await == 0) { emit("\r\nOK\r\n"); }
        return;
    }

    while (s_seen < len)
    {
        uint16_t start = s_seen, end = start, n;
        char     cmd[192];

        while ((end + 1u < len) && !((tx[end] == '\r') && (tx[end + 1u] == '\n'))) { end++; }
        if (end + 1u >= len) { return; }

        n = (uint16_t)(end - start);
        if (n >= sizeof(cmd)) { n = (uint16_t)(sizeof(cmd) - 1u); }
        memcpy(cmd, &tx[start], n);
        cmd[n] = '\0';
        s_seen = (uint16_t)(end + 2u);

        on_cmd(cmd);
        if (s_await > 0) { return; }
    }
}

static void run_ms(uint32_t ms)
{
    uint32_t t0 = host_tick_get();

    while ((host_tick_get() - t0) < ms)
    {
        g4_at_poll();
        modem_step();
        host_tick_set(host_tick_get() + 1u);
    }
}

int main(int argc, char **argv)
{
    uint32_t t0;
    const char *only = (argc > 1) ? argv[1] : "";

    host_log_enable(1);

    g4_io_init();
    g4_urc_init();

    /* 按 main.c 的顺序注入(main.c 里就是这么接的)。
     * 【故意不调 g4_at_set_broker / set_tls】: 这样打印出来的就是【编译期默认】
     * ——正是要验证的东西(G4_MQTT_USE_PRIVATE_BROKER 选的是哪一套)。
     * 参数用 "private" 时改走私有 broker 那一套做对照。 */
    if (strstr(only, "private") != NULL)
    {
        g4_at_set_broker("w90916ab.ala.cn-shenzhen.emqxsl.cn", "8883",
                         "N32H482_getway", "0809wzy");
        g4_at_set_tls(1u);
    }
    g4_at_set_sub_topic("mod/cmd");
    g4_at_set_lwt("mod/status", "{\"online\":false}");
    g4_at_set_keepalive_s(45u);       /* 故意给 45: 验证"夹到 60" */
    g4_at_set_reconnect(5u, 300u);
    g4_at_set_link_watchdog(150000u);

    if (strstr(only, "reject") != NULL) { s_cfg_reject = 1; }

    printf("\n================ 冷启动建链, 打印每一条 AT ================\n");
    g4_at_init();

    t0 = host_tick_get();
    while (!g4_at_is_online() && ((host_tick_get() - t0) < 90000u))
    {
        run_ms(200u);
    }

    printf("\n---------------- 结果 ----------------\n");
    printf("4G online = %s (耗时 %ums)\n",
           g4_at_is_online() ? "YES" : "NO",
           (unsigned)(host_tick_get() - t0));

    printf("\n================ ONLINE 稳态 5 秒: 应【零 AT】 ================\n");
    host_tx_reset();
    s_seen = 0;
    run_ms(5000u);
    printf("稳态 5s 内 TX 字节数 = %u (期望 0)\n", (unsigned)strlen(host_tx_take()));

    return 0;
}
