/**
 * @file    uart_rx.h
 * @brief   UART 中断接收 + 环形缓冲 + 给总线舵机用的 BusServoPort
 *
 * 为什么不用 DMA+IDLE：
 *     本工程 rx_dma 已经占用了 HAL_UARTEx_RxEventCallback；再用 DMA+IDLE 会和它抢回调。
 *     舵机 6 轴 50 Hz 的字节量很小（~10 kB/s），逐字节中断完全够用。
 *
 * 用法：
 *     UartRx *rx = UartRx_Open(&huart6);
 *     BusServoPort p = ServoPort_Open(rx, &huart6);
 */
#ifndef UART_RX_H
#define UART_RX_H

#include "bus_servo.h"
#include "main.h"

typedef struct UartRx UartRx;

/**
 * @brief  打开一个串口的字节接收（每字节一次中断）
 * @retval NULL = 实例池满（上限 2）
 */
extern UartRx *UartRx_Open(UART_HandleTypeDef *huart);

/**
 * @brief  取走已收到的字节
 * @retval 实际取走数量
 */
extern uint16_t UartRx_Read(UartRx *rx, uint8_t *out, uint16_t cap);

/**
 * @brief  当前缓存字节数
 */
extern uint16_t UartRx_Available(UartRx *rx);

/**
 * @brief  生成 BusServoPort：写走 tx_uart_it，读走环形缓冲，时间走 HAL_GetTick
 * @note   调用前必须 TxIt_Init(huart)
 */
extern BusServoPort ServoPort_Open(UartRx *rx, UART_HandleTypeDef *huart);

#endif
