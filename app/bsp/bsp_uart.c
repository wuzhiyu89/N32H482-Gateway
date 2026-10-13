#include <stddef.h>
#include <stdint.h>
#include "bsp_uart.h"
#include "bsp_gpio.h"
#include "ring_buf.h"
#include "misc.h"

/* 接收缓冲区内存 */
static uint8_t uart6_rb_mem[256U];      /* RS232 */
/* 【4G 口必须给大】: 云端下行/模组 URC 是断断续续挤进来的, 而主循环里还有
 * 别的阻塞活(LoRa 发一包 ~500ms, 上行组帧, Flash 读写)。115200 下 256 字节
 * 只要 22ms 就灌满, 一溢出 ring_buf 就整字节丢 —— 现象正是"跑一会儿 4G 下发
 * 就收不到"。4096 字节能顶住 356ms 的连续流量, 现在加到 8192(≈711ms),
 * 配合"溢出计数可见"(见 g4_uart_rx_ovf_count), 再溢出就能立刻看见而不是靠猜。 */
static uint8_t usart2_rb_mem[8192U];    /* 4G AT 口 */
static uint8_t usart4_rb_mem[256U];     /* 485_0 */
static uint8_t uart7_rb_mem[256U];      /* 485_1 */

ring_buf_t rb_uart6;
ring_buf_t rb_usart4;
ring_buf_t rb_uart7;
ring_buf_t rb_g4;   /* 4G AT 接收缓冲, USART2 中断填充 */

/* 4G AT 口累计收到的字节数(只增不减)。排"模组不回 AT"时用它区分
 * "一个字节都没收到" 和 "收到了但被清缓冲扔掉了" —— 详见 bsp_uart.h。 */
static volatile uint32_t s_g4_rx_cnt = 0u;
/* 【丢字节计数】两条来源:
 *   1) 硬件溢出(OREF): 中断/DMA 没及时取走, 硬件直接丢;
 *   2) 环形缓冲写满: ring_buf_push 返回 false, 软件丢。
 * 老版本这两条都【静默】发生 —— 现象就是"下行偶尔少一条/收不到", 谁也说不清。
 * 现在计数进状态行: >0 就说明接收确实被压垮过, 是"扩容/降日志/限速"的判据。 */
static volatile uint32_t s_g4_ovf_cnt = 0u;
/* USART2 是否已初始化(决定 g4_uart_tx_float/resume 能不能动 PC8) */
static uint8_t s_g4_uart_ready = 0u;

uint32_t g4_uart_rx_count(void)
{
    return s_g4_rx_cnt;
}

void g4_uart_rx_count_reset(void)
{
    s_g4_rx_cnt = 0u;
}

uint32_t g4_uart_rx_ovf_count(void)
{
    return s_g4_ovf_cnt;
}

void g4_uart_rx_ovf_reset(void)
{
    s_g4_ovf_cnt = 0u;
}

/* ---------------- RS232 (UART6: PA0-TX / PA1-RX / PA2-EN) ---------------- */

void bsp_uart6_init(uint32_t baud)
{
    USART_InitType USART_InitStructure;
    USART_StructInit(&USART_InitStructure);

    RS232_GPIO_Init();
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_UART6, ENABLE);

    ring_buf_init(&rb_uart6, uart6_rb_mem, sizeof(uart6_rb_mem));

    USART_InitStructure.BaudRate            = baud;
    USART_InitStructure.WordLength          = USART_WL_8B;
    USART_InitStructure.StopBits            = USART_STPB_1;
    USART_InitStructure.Parity              = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode                = USART_MODE_RX | USART_MODE_TX;
    USART_InitStructure.OverSampling        = USART_16OVER;
    USART_Init(UART6, &USART_InitStructure);

    USART_ConfigInt(UART6, USART_INT_RXDNE, ENABLE);
    NVIC_SetPriority(UART6_IRQn, NVIC_EncodePriority(5, 3, 0));
    NVIC_EnableIRQ(UART6_IRQn);
    USART_Enable(UART6, ENABLE);
}

/* ---------------- 4G 模组 AT 口 (USART2: PC8-TX / PC9-RX) ---------------- */

void bsp_g4_uart_init(uint32_t baud)
{
    USART_InitType USART_InitStructure;
    USART_StructInit(&USART_InitStructure);

    RS_DEBUG_GPIO_Init();                       /* PC8=TX, PC9=RX (USART2 AF) */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_USART2, ENABLE);

    ring_buf_init(&rb_g4, usart2_rb_mem, sizeof(usart2_rb_mem));

    USART_InitStructure.BaudRate            = baud;
    USART_InitStructure.WordLength          = USART_WL_8B;
    USART_InitStructure.StopBits            = USART_STPB_1;
    USART_InitStructure.Parity              = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode                = USART_MODE_RX | USART_MODE_TX;
    USART_InitStructure.OverSampling        = USART_16OVER;
    USART_Init(USART2, &USART_InitStructure);

    USART_ConfigInt(USART2, USART_INT_RXDNE, ENABLE);
    NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(5, 3, 1));
    NVIC_EnableIRQ(USART2_IRQn);
    USART_Enable(USART2, ENABLE);

    s_g4_uart_ready = 1u;       /* 之后才允许 g4_uart_tx_float/resume 动 PC8 */
}

/**
 * @brief 把 PC8(USART2_TX) 放开成高阻(不复用 USART2)
 *
 * 【为什么必须有】: PC8 空闲时输出高电平。模组被 PA8 断电之后, 这个高电平会顺着
 * 模组 RX 脚的保护二极管往模组的电源轨灌电, 后果:
 *   1) 模组"半供电": 万用表量模组电源脚得到 2~4V 乱跳的水位, 而不是干净的 0V/3.8V;
 *   2) 断电复位失效: 电刚断就被 TX 喂回来, 模组在半死状态下反复布朗起动, 永远
 *      不会打印 +MATREADY; 表现就是"AT 一个字节都不回、累计收到=0";
 *   3) 模组的电源开关管可能被这个回灌电压顶在半导通区, 更加量不准。
 * 所以只要给模组断电, 就要先把这根 TX 放开。
 *
 * @note USART2 本身不用关: 只是把复用从引脚上摘掉, 重新 resume 之后照常收发。
 */
void g4_uart_tx_float(void)
{
    GPIO_InitType GPIO_InitStructure;

    if (s_g4_uart_ready == 0u)
    {
        return;     /* USART2 还没初始化, PC8 本来就是输入态, 不用动 */
    }

    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin            = RS_DEBUG_TX_PIN;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_INPUT;
    GPIO_InitStructure.GPIO_Pull      = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Alternate = GPIO_NO_AF;
    GPIO_InitPeripheral(RS_DEBUG_GPIO_PORT, &GPIO_InitStructure);
}

/**
 * @brief 恢复 PC8 = USART2_TX(复用), 和"给模组上电"配对使用
 */
void g4_uart_tx_resume(void)
{
    if (s_g4_uart_ready == 0u)
    {
        return;
    }
    RS_DEBUG_GPIO_Init();       /* 幂等: 重新把 PC8/PC9 配回 USART2 复用 */
}

void g4_uart_send_byte(uint8_t ch)
{
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXDE) == RESET);
    USART_SendData(USART2, ch);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXC) == RESET);
}

void g4_uart_send_bytes(const uint8_t *buf, uint16_t len)
{
    uint16_t i;

    if ((buf == NULL) || (len == 0U))
    {
        return;
    }
    for (i = 0U; i < len; i++)
    {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXDE) == RESET);
        USART_SendData(USART2, buf[i]);
    }
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXC) == RESET);
}

/* ---------------- 485_0 (USART4) ---------------- */

void bsp_usart4_init(uint32_t baud)
{
    USART_InitType USART_InitStructure;
    USART_StructInit(&USART_InitStructure);

    RS485_0_GPIO_Init();
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_USART4, ENABLE);
    ring_buf_init(&rb_usart4, usart4_rb_mem, sizeof(usart4_rb_mem));

    USART_InitStructure.BaudRate            = baud;
    USART_InitStructure.WordLength          = USART_WL_8B;
    USART_InitStructure.StopBits            = USART_STPB_1;
    USART_InitStructure.Parity              = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode                = USART_MODE_RX | USART_MODE_TX;
    USART_InitStructure.OverSampling        = USART_16OVER;

    USART_Init(USART4, &USART_InitStructure);
    USART_ConfigInt(USART4, USART_INT_RXDNE, ENABLE);
    NVIC_SetPriority(USART4_IRQn, NVIC_EncodePriority(5, 3, 2));
    NVIC_EnableIRQ(USART4_IRQn);
    USART_Enable(USART4, ENABLE);
}

/* ---------------- 485_1 (UART7) ---------------- */

void bsp_uart7_init(uint32_t baud)
{
    USART_InitType USART_InitStructure;
    USART_StructInit(&USART_InitStructure);

    RS485_1_GPIO_Init();
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_UART7, ENABLE);
    ring_buf_init(&rb_uart7, uart7_rb_mem, sizeof(uart7_rb_mem));

    USART_InitStructure.BaudRate            = baud;
    USART_InitStructure.WordLength          = USART_WL_8B;
    USART_InitStructure.StopBits            = USART_STPB_1;
    USART_InitStructure.Parity              = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode                = USART_MODE_RX | USART_MODE_TX;
    USART_InitStructure.OverSampling        = USART_16OVER;

    USART_Init(UART7, &USART_InitStructure);
    USART_ConfigInt(UART7, USART_INT_RXDNE, ENABLE);
    NVIC_SetPriority(UART7_IRQn, NVIC_EncodePriority(5, 3, 2));
    NVIC_EnableIRQ(UART7_IRQn);
    USART_Enable(UART7, ENABLE);
}

/**
 * @brief 普通串口发送字节块（232/调试串口）
 */
void uart_send_bytes(USART_Module *USARTx, uint8_t *buf, uint16_t len)
{
    if ((USARTx == NULL) || (buf == NULL) || (len == 0U))
    {
        return;
    }
    for (uint16_t i = 0U; i < len; i++)
    {
        while (USART_GetFlagStatus(USARTx, USART_FLAG_TXDE) == RESET);
        USART_SendData(USARTx, buf[i]);
    }
    while (USART_GetFlagStatus(USARTx, USART_FLAG_TXC) == RESET);
}

/**
 * @brief RS485‑0发送，自动切换DE/RE#，发送完成切回接收模式
 */
void rs485_0_send(uint8_t *buf, uint16_t len)
{
    if(buf == NULL || len == 0) return;

    RS485_0_SetSendMode();

    for(uint16_t i=0; i<len; i++)
    {
        while(USART_GetFlagStatus(USART4, USART_FLAG_TXDE) == RESET);
        USART_SendData(USART4, buf[i]);
    }
    //必须等待全部移位发送完成！！！
    while(USART_GetFlagStatus(USART4, USART_FLAG_TXC) == RESET);

    RS485_0_SetRecvMode();
}

/**
 * @brief RS485‑1发送，自动切换DE/RE#
 */
void rs485_1_send(uint8_t *buf, uint16_t len)
{
    if(buf == NULL || len == 0) return;

    RS485_1_SetSendMode();

    for(uint16_t i=0; i<len; i++)
    {
        while(USART_GetFlagStatus(UART7, USART_FLAG_TXDE) == RESET);
        USART_SendData(UART7, buf[i]);
    }
    while(USART_GetFlagStatus(UART7, USART_FLAG_TXC) == RESET);

    RS485_1_SetRecvMode();
}

/* ---------------- RS232 发送(UART6, EN 自动切换) ---------------- */

/** RS232 发送单字节(阻塞轮询) */
void rs232_send_byte(uint8_t ch)
{
    rs232_send_bytes(&ch, 1U);
}

/**
 * @brief RS232 发送字节块, 自动 EN 拉高一次 + 发完拉低
 *        避免逐字节切换 EN 导致收发器时序不稳定
 */
void rs232_send_bytes(uint8_t *buf, uint16_t len)
{
    if ((buf == NULL) || (len == 0U)) return;

    GPIO_SetBits(RS232_GPIO_PORT, RS232_EN_PIN);   /* PA2 EN 拉高, 使能 232 收发器发送方向 */

    for (uint16_t i = 0U; i < len; i++)
    {
        while (USART_GetFlagStatus(UART6, USART_FLAG_TXDE) == RESET);
        USART_SendData(UART6, buf[i]);
    }
    while (USART_GetFlagStatus(UART6, USART_FLAG_TXC) == RESET);

    GPIO_ResetBits(RS232_GPIO_PORT, RS232_EN_PIN);  /* EN 拉低, 切回接收方向 */
}

/* ---------------- 中断服务 ---------------- */

void UART6_IRQHandler(void)
{
    if (USART_GetIntStatus(UART6, USART_INT_RXDNE) != RESET)
    {
        uint8_t ch = (uint8_t)(USART_ReceiveData(UART6) & 0xFFU);
        (void)ring_buf_push(&rb_uart6, ch);
        USART_ClrFlag(UART6, USART_FLAG_RXDNE);
    }
}

/* 4G AT 口: 收到字节推 rb_g4
 *
 * 【老版本的坑】: 这里只看 RXDNE, 而 OREF(溢出)被完全忽略 —— 硬件层面丢掉的
 * 字节没有任何痕迹。下游的表现就是"下行偶尔少一条/整条 URC 被劈开", 而日志里
 * 一点异常都看不到。现在:
 *   · 进中断先看 OREF, 计数(OREF 的清除序列就是"读 STS 再读 DAT", 下面读 DAT
 *     时会自然清掉);
 *   · ring_buf_push 返回 false(缓冲写满)也计数, 不再 (void) 掉。 */
void USART2_IRQHandler(void)
{
    if (USART_GetFlagStatus(USART2, USART_FLAG_OREF) != RESET)
    {
        s_g4_ovf_cnt++;         /* 硬件溢出丢的字节, 个数无法精确, 按"次"记 */
    }

    if (USART_GetIntStatus(USART2, USART_INT_RXDNE) != RESET)
    {
        uint8_t ch = (uint8_t)(USART_ReceiveData(USART2) & 0xFFU);

        s_g4_rx_cnt++;
        if (!ring_buf_push(&rb_g4, ch))
        {
            s_g4_ovf_cnt++;     /* 环形缓冲满, 这个字节丢了 */
        }
        USART_ClrFlag(USART2, USART_FLAG_RXDNE);
    }
}

void USART4_IRQHandler(void)
{
    if (USART_GetIntStatus(USART4, USART_INT_RXDNE) != RESET)
    {
        uint8_t ch = (uint8_t)(USART_ReceiveData(USART4) & 0xFFU);
        (void)ring_buf_push(&rb_usart4, ch);
        USART_ClrFlag(USART4, USART_FLAG_RXDNE);
    }
}

void UART7_IRQHandler(void)
{
    if (USART_GetIntStatus(UART7, USART_INT_RXDNE) != RESET)
    {
        uint8_t ch = (uint8_t)(USART_ReceiveData(UART7) & 0xFFU);
        (void)ring_buf_push(&rb_uart7, ch);
        USART_ClrFlag(UART7, USART_FLAG_RXDNE);
    }
}
