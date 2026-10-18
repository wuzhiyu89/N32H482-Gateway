/**
 * \file  host_stub.c
 * \brief 主机单元测试用的底层替身(时基 / USART2 / PA8 电源脚 / 日志)
 *
 * 只为了让 g4_io.c / g4_urc.c / g4_at.c 能在 PC 上编译并跑逻辑测试,
 * 不参与目标板构建。测试脚本见 _host_test/run.ps1。
 */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdint.h>

#include "bsp_tim.h"
#include "bsp_uart.h"
#include "bsp_gpio.h"
#include "ring_buf.h"
#include "log.h"

/* ---------------- 虚拟时基 ----------------
 * 板子上 get_tick_ms() 由 ATIM1 中断推进; 主机上没有中断, 所以这里额外做一件事:
 * 每 HOST_SPIN_CALLS 次调用推进 1ms —— 让【自旋等待】也能走到超时。
 * 否则 g4_io_send_expect_resp() 这类阻塞封装在主机上会死循环(时间永远不动)。
 * 测试主循环每拍都会 host_tick_set(), 所以对状态机的节拍没有影响。 */
#define HOST_SPIN_CALLS  500u

static uint32_t s_tick;
static uint32_t s_tick_calls;

void (*host_tick_hook)(void);

uint32_t get_tick_ms(void)
{
    if (++s_tick_calls >= HOST_SPIN_CALLS)
    {
        s_tick_calls = 0u;
        s_tick++;
        if (host_tick_hook != NULL) { host_tick_hook(); }
    }
    return s_tick;
}

void delay_ms(uint32_t ms) { s_tick += ms; s_tick_calls = 0u; }
void host_tick_set(uint32_t ms) { s_tick = ms; s_tick_calls = 0u; }
void host_tick_advance(uint32_t ms) { s_tick += ms; s_tick_calls = 0u; }
uint32_t host_tick_get(void) { return s_tick; }

/* ---------------- 日志 ---------------- */
static int s_log_on;

void host_log_enable(int on) { s_log_on = on; }

void host_log_printf(const char *fmt, ...)
{
    va_list ap;

    if (!s_log_on) { return; }
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
}

/* ---------------- USART2 替身 ---------------- */
#define HOST_RB_SZ   8192u
static uint8_t s_rb_mem[HOST_RB_SZ];
ring_buf_t     rb_g4;

static uint32_t s_rx_cnt;
static uint32_t s_ovf_cnt;

void bsp_g4_uart_init(uint32_t baud)
{
    (void)baud;
    ring_buf_init(&rb_g4, s_rb_mem, (uint16_t)sizeof(s_rb_mem));
    s_rx_cnt = 0u;
    s_ovf_cnt = 0u;
}

/* 测试用: 模拟 USART2 中断收到一个字节 */
void host_uart_rx_byte(uint8_t ch)
{
    s_rx_cnt++;
    if (!ring_buf_push(&rb_g4, ch)) { s_ovf_cnt++; }
}

uint32_t g4_uart_rx_count(void) { return s_rx_cnt; }
void g4_uart_rx_count_reset(void) { s_rx_cnt = 0u; }
uint32_t g4_uart_rx_ovf_count(void) { return s_ovf_cnt; }
void g4_uart_rx_ovf_reset(void) { s_ovf_cnt = 0u; }

/* 模组侧"收到"的字节(我们发出去的) */
static char     s_tx[4096];
static uint16_t s_tx_len;

void g4_uart_send_bytes(const uint8_t *buf, uint16_t len)
{
    uint16_t i;

    for (i = 0u; (i < len) && (s_tx_len < (sizeof(s_tx) - 1u)); i++)
    {
        s_tx[s_tx_len++] = (char)buf[i];
    }
    s_tx[s_tx_len] = '\0';
}

const char *host_tx_take(void) { return s_tx; }
void host_tx_reset(void) { s_tx_len = 0u; s_tx[0] = '\0'; }

/* ---------------- PA8 电源脚替身 ---------------- */
static uint8_t s_on_high = 1u;      /* 默认高电平上电 */
static uint8_t s_level;
static uint32_t s_pwr_cycle_cnt;

void G4_ModulePWR_GPIO_Init(void) { }
void G4_PowerSwitch_Init(void) { s_level = s_on_high; }
void G4_PowerSwitch_InitNoPower(void) { s_level = (uint8_t)(!s_on_high); }
void G4_PowerSwitch_Set(uint8_t on) { s_level = on ? s_on_high : (uint8_t)(!s_on_high); }
uint8_t G4_PowerSwitch_GetLevel(void) { return s_level; }
uint8_t G4_PowerSwitch_ReadPin(void) { return s_level; }
uint8_t G4_PowerSwitch_GetOnHigh(void) { return s_on_high; }
void G4_PowerSwitch_SwapOnHigh(void) { s_on_high = (uint8_t)(!s_on_high); }
void G4_PowerCycle(uint32_t off_ms) { (void)off_ms; s_pwr_cycle_cnt++; G4_PowerSwitch_Set(1u); }

uint32_t host_pwr_cycle_count(void) { return s_pwr_cycle_cnt; }
