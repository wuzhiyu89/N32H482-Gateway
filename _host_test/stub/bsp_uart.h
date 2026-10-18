#ifndef __BSP_UART_H__
#define __BSP_UART_H__
#include <stdint.h>
#include "ring_buf.h"

/* 主机单元测试用: USART2 的替身 */
extern ring_buf_t rb_g4;

void     bsp_g4_uart_init(uint32_t baud);
void     g4_uart_send_bytes(const uint8_t *buf, uint16_t len);
uint32_t g4_uart_rx_count(void);
void     g4_uart_rx_count_reset(void);
uint32_t g4_uart_rx_ovf_count(void);
void     g4_uart_rx_ovf_reset(void);

/* 测试用: 取出模组侧"收到"的字节(我们发出去的命令) */
const char *host_tx_take(void);
void        host_tx_reset(void);

#endif
