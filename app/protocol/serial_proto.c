/**
 * \file  serial_proto.c
 * \brief 三路串口薄封装实现 (RS232 + RS485_0 + RS485_1)
 *
 * 本层只做通道抽象, 协议待定 (Modbus / 自定义 ASCII帧 / 二进制帧等上层再写)。
 */
#include "serial_proto.h"
#include "bsp_uart.h"
#include "ring_buf.h"
#include <string.h>

/* 外部引用 bsp_uart.c 里初始化好的接收环形缓冲 */
extern ring_buf_t rb_uart6;   /* RS232   UART6 */
extern ring_buf_t rb_usart4;   /* RS485_0 USART4 */
extern ring_buf_t rb_uart7;    /* RS485_1 UART7 */

/* 通道 → ring_buf 映射表 */
static ring_buf_t *s_rx_rb[SERIAL_CH_MAX] = { &rb_uart6, &rb_usart4, &rb_uart7 };

/* ==================== init ==================== */

void serial_init_all(uint32_t baud)
{
    bsp_uart6_init(baud);   /* RS232    */
    bsp_usart4_init(baud);   /* RS485_0  */
    bsp_uart7_init(baud);    /* RS485_1  */
}

/* ==================== 发送 ==================== */

bool serial_send_byte(serial_ch_t ch, uint8_t byte)
{
    if ((uint32_t)ch >= (uint32_t)SERIAL_CH_MAX) return false;

    uint8_t buf[1] = { byte };
    return serial_send_bytes(ch, buf, 1);
}

bool serial_send_bytes(serial_ch_t ch, const uint8_t *buf, uint16_t len)
{
    if ((uint32_t)ch >= (uint32_t)SERIAL_CH_MAX) return false;
    if ((buf == NULL) || (len == 0U)) return false;

    switch (ch)
    {
    case SERIAL_CH_RS232:
        /* RS232: 整帧 EN 拉高一次, 发完拉低 (rs232_send_bytes 内部已处理) */
        rs232_send_bytes((uint8_t *)buf, len);
        break;

    case SERIAL_CH_RS485_0:
        rs485_0_send((uint8_t *)buf, len);    /* 内部已做 DE/RE# 切换 + TXC 等待 */
        break;

    case SERIAL_CH_RS485_1:
        rs485_1_send((uint8_t *)buf, len);    /* 内部已做 DE/RE# 切换 + TXC 等待 */
        break;

    default:
        return false;
    }
    return true;
}

/* ==================== 接收 (从 ring_buf pop) ==================== */

bool serial_recv_byte(serial_ch_t ch, uint8_t *out)
{
    if ((uint32_t)ch >= (uint32_t)SERIAL_CH_MAX) return false;
    if (out == NULL) return false;

    ring_buf_t *rb = s_rx_rb[ch];
    if (rb == NULL) return false;

    return ring_buf_pop(rb, out);
}

uint16_t serial_get_rx_len(serial_ch_t ch)
{
    if ((uint32_t)ch >= (uint32_t)SERIAL_CH_MAX) return 0U;

    ring_buf_t *rb = s_rx_rb[ch];
    if (rb == NULL) return 0U;

    return ring_buf_get_len(rb);
}
