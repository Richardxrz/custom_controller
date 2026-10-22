/**
 * @file    rx_dma.h
 * @brief   串口接收模块 —— 把字节流解析成帧
 *
 * 用法(最小可用)：
 *     RxDma_t rx = rx_dma_open(&huart6);    // 开启接收
 *     uint8_t buf[16];
 *     uint16_t n = 0;
 *     if (rx != NULL) {
 *       n = rx_dma_fetch(rx, buf, sizeof(buf));    // 抓取帧的DATA位并返回DATA位长度
 *     }
 *     if (n > 0) {                  // n = 帧长；0 = 没有新帧
 *       HAL_UART_Transmit_IT(&huart6, buf, n);    // 发送
 *     }
 *
 *
 * 协议：
 *     [0xAA][0x55][LEN][DATA][SUM]
 *     SUM = (LEN + sum(DATA)) & 0xFF
 *
 *     各字段长度或长度范围:
 *     [1][1][1][由LEN位决定:1~16][1]
 *
 * 约束：
 *     用户在使用该模块时注意尽量不要在中断里面调用相关API
 */

#ifndef __RX_DMA_H
#define __RX_DMA_H

#include "usart.h"
#include <stdint.h>

#define SM_PAYLOAD_MAX 16
#define RX_RAW_SIZE 128

typedef struct RxDma_t *RxDma_t;

/**
 * @brief  开启接收
 * @param  huart 串口句柄
 * @note   池子满会返回 NULL
 * @retval 实例指针
 */
extern RxDma_t RxDma_open(UART_HandleTypeDef *huart);

/**
 * @brief  抓取一帧数据中的DATA位
 * @param  self     实例指针
 * @param  out      用户层容器
 * @param  max_len  最大接收长度
 * @note   本模块只做单帧缓存；用户取走得比 DMA 搬得慢，就会丢帧（覆盖）
 * @note   rx_dma_fetch 本身除了抓取和返回长度以外，也意味着一次完整的流程的结束或者说一次数据更新
 * @note   一次只取一帧
 * @note   max_len 过小会截断数据
 * @retval DATA位长度
 */
extern uint16_t RxDma_fetch(RxDma_t self, uint8_t *out, uint16_t max_len);

/**
 * @brief  获取错误帧数量
 * @note   错误:
 *         1. 超时错误
 *         2. 校验失败
 * @note   getter 在fetch以后使用保证数据都是最新的
 * @retval 错误帧数量
 */
extern uint8_t RxDma_GetErrCnt(RxDma_t self);

/*
 * @brief  获取一帧数据中的DATA位长度
 * @note   不管帧是否有效，只要捕获到LEN位就会同步更新
 * @retval DATA位长度
 */
extern uint8_t RxDma_GetDATALength(RxDma_t self);

#endif
