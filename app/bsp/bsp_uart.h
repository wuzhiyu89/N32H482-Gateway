#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "n32h47x_48x.h"
#include "n32h47x_48x_usart.h"


#define UART_BUF_SIZE 256

#include "ring_buf.h"

extern ring_buf_t rb_g4;   /* 4G AT 接收缓冲(USART2 中断填充) */

/*
 * 串口分配(2026-09 调整):
 *   USART1  PA9-TX  PA10-RX   日志打印(log.c)
 *   USART2  PC8-TX  PC9-RX    4G 模组 AT 口
 *   UART6   PA0-TX  PA1-RX    RS232 (PA2-EN)
 *   USART4  PB0/PB1           485_0
 *   UART7   PB11/PB10         485_1
 */
void bsp_uart6_init(uint32_t baud);   /* 232  PA0-TX  PA1-RX */
void bsp_g4_uart_init(uint32_t baud); /* 4G   PC8-TX  PC9-RX, 收中断推 rb_g4 */
void bsp_usart4_init(uint32_t baud);  /* 485_0 PB0-TX PB1-RX */
void bsp_uart7_init(uint32_t baud);   /* 485_1 PB11-TX PB10-RX */

void uart_send_bytes(USART_Module *USARTx, uint8_t *buf, uint16_t len);

/** RS485‑0发送, 自动 DE/RE# 切换 */
void rs485_0_send(uint8_t *buf, uint16_t len);
/** RS485‑1发送, 自动 DE/RE# 切换 */
void rs485_1_send(uint8_t *buf, uint16_t len);
/** RS232 发送单字节(阻塞, EN 自动切换) */
void rs232_send_byte(uint8_t ch);
/** RS232 发送字节块(阻塞, EN 只拉高一次) */
void rs232_send_bytes(uint8_t *buf, uint16_t len);

/* ------- 4G 模组 AT 口(USART2) ------- */

/** 4G AT 口发送一字节(阻塞) */
void g4_uart_send_byte(uint8_t ch);
/** 4G AT 口发送字节块(阻塞, 等待 TXC 完成) */
void g4_uart_send_bytes(const uint8_t *buf, uint16_t len);

/**
 * @brief 上电以来 4G AT 口(USART2)累计收到的字节数, 在接收中断里自增
 *
 * 【为什么要有】: 排"模组不回 AT"时, rb_g4 的残留字节数是【不能作准】的 ——
 * 每次发命令前都会清缓冲, 模组启动时打的 +MATREADY/RDY banner 会被清掉,
 * 于是"收到了但被扔掉"和"一个字节都没收到"看起来一模一样。
 * 这个计数只增不减, 只回答一个问题: 模组到底开口说过话没有。
 *   = 0    → 模组一个字节都没发过: 供电/TX-RX/波特率/模组没起来
 *   > 0    → 它说话了, 那就是内容或波特率对不上(可对比日志里的原始行)
 */
uint32_t g4_uart_rx_count(void);
/** 清零累计接收计数(测试用) */
void g4_uart_rx_count_reset(void);

/**
 * @brief 上电以来 4G AT 口【丢掉的字节数】
 *
 * 两条来源(都在中断里统计):
 *   1) 硬件溢出 OREF: 上一个字节还没被取走, 新的就来了 —— 硬件直接丢;
 *   2) rb_g4 写满: ring_buf_push 失败, 软件丢。
 *
 * 这个数 > 0 就说明接收通路确实被压垮过。它是判断"还要不要加大缓冲/降日志/
 * 让云端放慢下发"的唯一硬证据 —— 不然只能靠"感觉最近老丢包"猜。
 */
uint32_t g4_uart_rx_ovf_count(void);
/** 清零丢字节计数 */
void g4_uart_rx_ovf_reset(void);

/**
 * @brief 把 PC8(USART2_TX) 放开成高阻 —— 给模组断电之前必须先调它
 *
 * 【为什么必须有】: PC8 空闲时输出高。模组没电时, 这个高电平会顺着模组 RX 脚的
 * 保护二极管往它电源轨灌电, 于是:
 *   · 万用表量模组电源脚得到 2~4V 乱跳的水位(既不干净在 0V 也不到 3.8V);
 *   · "断电复位"失效 —— 电刚断就被 TX 喂回来, 模组半死, 永远不打 +MATREADY;
 *   · 现象就是"AT 一个字节都不回 / 累计收到=0", 但一模组又不是完全没电。
 *
 * 调用时机由 bsp_gpio.c 的 G4_PowerSwitch_Set(0) 负责, 一般不用自己调。
 * USART2 不用关: 只是把复用从引脚上摘掉, resume 之后照常收发。
 */
void g4_uart_tx_float(void);
/** 恢复 PC8 = USART2_TX(复用), 与"给模组上电"配对 */
void g4_uart_tx_resume(void);

#endif /* __BSP_UART_H__ */
