#include "ring_buf.h"

void ring_buf_init(ring_buf_t *rb, uint8_t *mem, uint16_t mem_size)
{
    rb->buf = mem;
    rb->size = mem_size;
    rb->wr = 0;
    rb->rd = 0;
    rb->overflow = false;
}

/**
 * @brief 压入一个字节，缓冲区满返回false，并置溢出标志
 */
bool ring_buf_push(ring_buf_t *rb, uint8_t dat)
{
    //缓冲区已满 wr追上rd
    // 正确写法（"浪费一个槽"的满检测）：
    if (((rb->wr + 1) % rb->size) == rb->rd)
    {
        rb->overflow = true;
        return false;
    }
    rb->buf[rb->wr] = dat;
    rb->wr = (rb->wr + 1) % rb->size;
    return true;
}

/**
 * @brief 弹出一个字节，空返回false
 */
bool ring_buf_pop(ring_buf_t *rb, uint8_t *dat)
{
    if(rb->wr == rb->rd)
    {
        return false;
    }
    *dat = rb->buf[rb->rd];
    rb->rd = (rb->rd + 1) % rb->size;
    return true;
}

/**
 * @brief 获取已经存储的字节数量
 */
uint16_t ring_buf_get_len(ring_buf_t *rb)
{
    return (rb->wr + rb->size - rb->rd) % rb->size;
}

/**
 * @brief 获取空闲字节数
 */
uint16_t ring_buf_get_free(ring_buf_t *rb)
{
    return (rb->rd + rb->size - rb->wr - 1) % rb->size;
}

void ring_buf_clear_overflow(ring_buf_t *rb)
{
    rb->overflow = false;
}

bool ring_buf_is_overflow(ring_buf_t *rb)
{
    return rb->overflow;
}

void ring_buf_reset(ring_buf_t *rb)
{
    rb->wr = 0;
    rb->rd = 0;
    rb->overflow = false;
}
