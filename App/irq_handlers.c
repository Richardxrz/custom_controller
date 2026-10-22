/**
 * @file    irq_handlers.c
 * @brief   手写外设的中断入口（CubeMX 只生成了 USART6 的）
 */
#include "main.h"
#include "usart.h"

extern UART_HandleTypeDef huart1;

void USART1_IRQHandler(void) { HAL_UART_IRQHandler(&huart1); }
