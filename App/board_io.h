/**
 * @file    board_io.h
 * @brief   自定义控制器用到的板级外设初始化（C 板）
 *
 * 目前 CubeMX 只生成了 USART6（PG14/PG9），本文件补上：
 *     UART1  PA9(TX) / PB7(RX)  —— 主端：PC 链路；机器人端：图传发送端
 *     CAN1   PD0(RX) / PD1(TX)  —— 机器人端：6 个达妙电机
 *
 * ⚠️ 这些引脚/外设没有经过 CubeMX，属于手写初始化；上板前先量电平、再看波形。
 */
#ifndef BOARD_IO_H
#define BOARD_IO_H

#include <stdbool.h>

#include "main.h"
#include "usart.h"

extern UART_HandleTypeDef huart1;
extern CAN_HandleTypeDef hcan1;

/**
 * @brief  初始化 UART1（8N1，无流控）
 * @param  baud 921600（图传/PC 链路）
 */
void Board_Uart1_Init(uint32_t baud);

/**
 * @brief  改 USART6 波特率（CubeMX 默认 115200，主端舵机总线用它）
 */
void Board_Uart6_SetBaud(uint32_t baud);

/**
 * @brief  初始化 CAN1：1 Mbps，接受所有标准帧
 */
void Board_Can1_Init(void);

/**
 * @brief  发一帧标准 CAN（DLC=8）
 * @retval true = 已进邮箱
 */
bool Board_Can1_Send(uint16_t std_id, const uint8_t data[8]);

/**
 * @brief  收一帧（非阻塞）
 * @retval true = 收到
 */
bool Board_Can1_Recv(uint16_t *std_id, uint8_t data[8]);

#endif
