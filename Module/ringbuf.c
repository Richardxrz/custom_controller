#include "ringbuf.h"

#include <stddef.h>

// 用取模而不是位掩码：允许 size 不是 2 的幂，省得调用方为了对齐容量浪费 RAM。
// 代价是每次多一次除法 —— Cortex-M4 上 16 位取模有硬件指令，几周期，可接受。
static uint16_t rb_next(const RingBuf *rb, uint16_t i) {
  uint16_t n = (uint16_t)(i + 1u);
  return (n >= rb->size) ? 0u : n;
}

void RingBuf_Init(RingBuf *rb, uint8_t *storage, uint16_t size) {
  if (rb == NULL) {
    return;
  }
  rb->buf = storage;
  rb->size = (storage == NULL) ? 0u : size;
  rb->head = 0u;
  rb->tail = 0u;
}

bool RingBuf_Put(RingBuf *rb, uint8_t byte) {
  if (rb == NULL || rb->buf == NULL || rb->size == 0u) {
    return false;
  }
  uint16_t next = rb_next(rb, rb->head);
  if (next == rb->tail) { /* 满：留一个空位区分空/满 */
    return false;
  }
  rb->buf[rb->head] = byte;
  rb->head = next;
  return true;
}

uint16_t RingBuf_PutBlock(RingBuf *rb, const uint8_t *data, uint16_t len) {
  if (rb == NULL || data == NULL) {
    return 0u;
  }
  uint16_t n = 0u;
  while (n < len && RingBuf_Put(rb, data[n])) {
    n++;
  }
  return n;
}

bool RingBuf_Get(RingBuf *rb, uint8_t *byte) {
  if (rb == NULL || byte == NULL || rb->buf == NULL) {
    return false;
  }
  if (rb->tail == rb->head) { /* 空 */
    return false;
  }
  *byte = rb->buf[rb->tail];
  rb->tail = rb_next(rb, rb->tail);
  return true;
}

uint16_t RingBuf_GetBlock(RingBuf *rb, uint8_t *out, uint16_t cap) {
  if (rb == NULL || out == NULL) {
    return 0u;
  }
  uint16_t n = 0u;
  while (n < cap && RingBuf_Get(rb, &out[n])) {
    n++;
  }
  return n;
}

uint16_t RingBuf_Count(const RingBuf *rb) {
  if (rb == NULL || rb->size == 0u) {
    return 0u;
  }
  uint16_t head = rb->head;
  uint16_t tail = rb->tail;
  return (uint16_t)((head >= tail) ? (head - tail) : (uint16_t)(rb->size - tail + head));
}

bool RingBuf_IsEmpty(const RingBuf *rb) { return RingBuf_Count(rb) == 0u; }

void RingBuf_Flush(RingBuf *rb) {
  if (rb == NULL) {
    return;
  }
  rb->tail = rb->head;
}
