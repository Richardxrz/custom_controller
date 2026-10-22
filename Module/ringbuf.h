/**
 * @file    ringbuf.h
 * @brief   单生产者 / 单消费者字节环形缓冲
 *
 * 用途：中断（DMA 空闲回调）往里放字节，主循环往外取。
 *       生产者只写 head，消费者只写 tail —— 两边都不用关中断。
 *
 * 最小用法：
 *     static uint8_t store[256];
 *     RingBuf rb;
 *     RingBuf_Init(&rb, store, sizeof(store));
 *     RingBuf_Put(&rb, byte);          // 中断里
 *     RingBuf_Get(&rb, &byte);         // 主循环
 *
 * 约束：
 *     只允许一个生产者、一个消费者；
 *     head/tail 是 16 位，Cortex-M 上读写原子；size 不要超过 65535；
 *     满了 Put 返回 false（丢新字节），不覆盖旧数据。
 */
#ifndef RINGBUF_H
#define RINGBUF_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint8_t *buf;
  uint16_t size;
  volatile uint16_t head; /* 只由生产者写 */
  volatile uint16_t tail; /* 只由消费者写 */
} RingBuf;

/**
 * @brief  初始化
 * @param  storage 外部提供的存储区，调用方保证生命周期
 * @param  size    存储区字节数，0 表示禁用（此后所有操作都失败）
 */
extern void RingBuf_Init(RingBuf *rb, uint8_t *storage, uint16_t size);

/**
 * @brief  放入一个字节
 * @retval false = 已满，本次丢弃
 */
extern bool RingBuf_Put(RingBuf *rb, uint8_t byte);

/**
 * @brief  批量放入
 * @retval 实际写入的字节数（可能小于 len）
 */
extern uint16_t RingBuf_PutBlock(RingBuf *rb, const uint8_t *data, uint16_t len);

/**
 * @brief  取出一个字节
 * @retval false = 空
 */
extern bool RingBuf_Get(RingBuf *rb, uint8_t *byte);

/**
 * @brief  批量取出
 * @retval 实际取出的字节数；cap 小于可读量时只取 cap 个
 */
extern uint16_t RingBuf_GetBlock(RingBuf *rb, uint8_t *out, uint16_t cap);

/**
 * @brief  当前可读字节数
 */
extern uint16_t RingBuf_Count(const RingBuf *rb);

/**
 * @brief  是否为空
 */
extern bool RingBuf_IsEmpty(const RingBuf *rb);

/**
 * @brief  丢弃全部数据（消费者侧调用）
 */
extern void RingBuf_Flush(RingBuf *rb);

#endif
