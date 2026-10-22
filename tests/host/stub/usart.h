/*
 * 主机侧测试用的 HAL 替身（stub）
 * 目的：让 Module/tx_uart_it.c 能在 Mac 上直接编译运行，不需要板子。
 * 它只提供"类型 + 两个被调用的函数 + 一份调用记录"，不含任何结论。
 */
#ifndef STUB_USART_H
#define STUB_USART_H

#include <stdint.h>

typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 } HAL_StatusTypeDef;

typedef enum {
  HAL_UART_STATE_RESET = 0x00U,
  HAL_UART_STATE_READY = 0x20U,
  HAL_UART_STATE_BUSY = 0x24U,
  HAL_UART_STATE_BUSY_TX = 0x21U,
  HAL_UART_STATE_BUSY_RX = 0x22U,
  HAL_UART_STATE_BUSY_TX_RX = 0x23U,
  HAL_UART_STATE_TIMEOUT = 0xA0U,
  HAL_UART_STATE_ERROR = 0xE0U
} HAL_UART_StateTypeDef;

typedef struct { int dummy; } USART_TypeDef;

typedef struct {
  USART_TypeDef *Instance;
  uint32_t gState;
  uint32_t RxState;
} UART_HandleTypeDef;

/* 被测代码里引用的全局句柄（由测试自己定义） */
extern UART_HandleTypeDef huart6;

/* 替身记录：模块交给 HAL 的每一样东西都记在这里 —— 断言靠它。
   这是测试**唯一**的观测口：模块内部是 static，测试读不到，只能看它交给 HAL 什么。
   注意 tx_data 存的是**指针**（真 HAL 也只存指针、不复制数据），
   所以"调用当时的内容"要另存一份快照 snap，才能和之后的内容对比。 */
typedef struct {
  int tx_calls;
  UART_HandleTypeDef *tx_huart; /* 传进来的句柄 */
  const uint8_t *tx_data;       /* 传进来的数据指针（不复制内容） */
  uint16_t tx_size;             /* 传进来的长度 */
  HAL_StatusTypeDef tx_ret;     /* Transmit_IT 固定返回这个 */
  uint8_t snap[8];              /* 调用当时从 tx_data 抄下来的前几字节 */
  int snap_n;                   /* 快照字节数 */
} stub_hal_t;

extern stub_hal_t g_stub;
void stub_reset(void);

/* 语义与真 HAL 对齐：
   - Transmit_IT 成功 → 把 huart->gState 置 BUSY_TX（stm32f4xx_hal_uart.c:1323）
   - GetState      → 返回 gState | RxState（:2955-2961），不是查表 */
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size);
HAL_UART_StateTypeDef HAL_UART_GetState(const UART_HandleTypeDef *huart);

#endif /* STUB_USART_H */
