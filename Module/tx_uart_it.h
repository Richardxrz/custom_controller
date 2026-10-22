/**
 * @note  最大实例上限是 2
 */

#ifndef TX_IT_H
#define TX_IT_H

#include "usart.h"
#include <stdint.h>

typedef enum {
  TX_OK,    // 发送开启成功
  TX_ERROR, // 发送开启错误
  TX_READY, // 发送准备就绪
  TX_BUSY   // 发送繁忙
} TxIt_State;

/**
 * @brief  注册并初始化模块实例
 * @param   huart 串口句柄
 * @note   模块管理实例 无需手动分配内存
 */
extern void TxIt_Init(UART_HandleTypeDef *huart);

/**
 * @brief  开启发送
 * @param  huart 串口句柄
 * @param  data 数据容器指针
 * @param  length 数据长度
 * @note   length 太大会导致数据截断
 * @retvel 发送状态
 */
extern TxIt_State TxIt_Send(UART_HandleTypeDef *huart, uint8_t *data, uint16_t length);
/**
 * @brief  获取当前发送状态
 * @param  huart 串口句柄
 * @retvel 发送状态(TX_READY/TX_BUSY)
 */
extern TxIt_State TxIt_GetState(UART_HandleTypeDef *huart);

#endif
