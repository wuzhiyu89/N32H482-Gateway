#ifndef __RING_BUF_H
#define __RING_BUF_H
#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    uint8_t *buf;
    uint16_t size;
    uint16_t wr;
    uint16_t rd;
    bool overflow;  //新增：溢出标记，发生写满置true，读操作清零
}ring_buf_t;

void ring_buf_init(ring_buf_t *rb, uint8_t *mem, uint16_t mem_size);
bool ring_buf_push(ring_buf_t *rb, uint8_t dat);
bool ring_buf_pop(ring_buf_t *rb, uint8_t *dat);
uint16_t ring_buf_get_len(ring_buf_t *rb);
uint16_t ring_buf_get_free(ring_buf_t *rb);
void ring_buf_clear_overflow(ring_buf_t *rb);
bool ring_buf_is_overflow(ring_buf_t *rb);
void ring_buf_reset(ring_buf_t *rb);

#endif /* __RING_BUF_H */
