/**
 * \file  serial_proto.h
 * \brief 三路串口薄封装 —— RS232 + RS485_0 + RS485_1
 *
 * 本层只做通道抽象, 不关心上层协议 (Modbus/自定义帧等)。
 * 物理收发 + DE/RE# 控制已由 bsp_uart.c 完成。
 *
 * 通道映射:
 *   SERIAL_CH_RS232   → USART1  PA4-TX PA5-RX PA6-EN
 *   SERIAL_CH_RS485_0 → USART4  PB0-TX PB1-RX PC5-DE PC4-RE#
 *   SERIAL_CH_RS485_1 → UART7  PB11-TX PB10-RX PB2-DE PH3-RE#
 *
 * 接收: 各通道中断自动推进 ring_buf (rb_usart1/rb_usart4/rb_uart7),
 *       业务层通过 serial_recv_byte() 从 ring_buf 取字节。
 * 发送: RS485 自动 DE/RE# 切换 (rs485_x_send 内部已处理),
 *       RS232 通过 rs232_send_byte / uart_send_bytes 阻塞轮询发送。
 */
#ifndef __SERIAL_PROTO_H__
#define __SERIAL_PROTO_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 通道枚举 */
typedef enum
{
    SERIAL_CH_RS232   = 0,
    SERIAL_CH_RS485_0 = 1,
    SERIAL_CH_RS485_1 = 2,
    SERIAL_CH_MAX     = 3
} serial_ch_t;

/**
 * \brief 初始化三路串口 (统一波特率, 8N1)
 * \param baud 波特率 (常用 9600 / 19200 / 115200)
 */
void serial_init_all(uint32_t baud);

/**
 * \brief 发送单字节 (阻塞)
 * \return true 成功; false 通道号非法
 */
bool serial_send_byte(serial_ch_t ch, uint8_t byte);

/**
 * \brief 发送字节块 (阻塞)
 * \return true 成功; false 参数非法
 */
bool serial_send_bytes(serial_ch_t ch, const uint8_t *buf, uint16_t len);

/**
 * \brief 非阻塞从接收缓冲取一字节 (中断投递的, 业务层 poll 调用)
 * \return true 取到, *out=字节值; false 缓冲空或通道非法
 */
bool serial_recv_byte(serial_ch_t ch, uint8_t *out);

/**
 * \brief 查询接收缓冲待读取字节数
 */
uint16_t serial_get_rx_len(serial_ch_t ch);

#ifdef __cplusplus
}
#endif

#endif /* __SERIAL_PROTO_H__ */
