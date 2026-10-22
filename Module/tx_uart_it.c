#include "tx_uart_it.h"
#include <string.h>

#define TX_MAX_BUF 320
#define TX_INSTANCE_MAX 2

struct TxIt_t {
  UART_HandleTypeDef *huart;
  TxIt_State state;
  uint8_t data[TX_MAX_BUF];
  uint16_t length;
};

typedef struct TxIt_t *TxIt_t;

static struct TxIt_t instances[TX_INSTANCE_MAX];
static uint8_t instances_cnt = 0;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  TxIt_t instance = NULL;
  for (int i = 0; i < instances_cnt; i++) {
    if (instances[i].huart == huart) {
      instance = &instances[i];
      instance->state = TX_READY;
    }
  }
  // 清空发送缓存区
  memset(instance->data, 0, TX_MAX_BUF);
}

/**
 * @note  C 的函数参数只有值传递，uint8_t buf[128] 只是 uint8_t *buf 的另一种写法（那个 128
 * 会被静默忽略）。所以在函数里 sizeof 拿到的是指针大小 = 4（32 位 MCU)
 */
TxIt_State TxIt_Send(UART_HandleTypeDef *huart, uint8_t *data, uint16_t length) {
  if (huart == NULL || data == NULL) {
    return TX_ERROR;
  }

  TxIt_t instance = NULL;
  for (int i = 0; i < instances_cnt; i++) {
    if (instances[i].huart == huart) {
      instance = &instances[i];
    }
  }
  // 防止未注册实例
  if (instance == NULL) {
    return TX_ERROR;
  }
  // 防止重入
  if (instance->state == TX_BUSY) {
    return TX_BUSY;
  }

  // 同步用户数据
  if (length > 0) {
    if (length > TX_MAX_BUF) { // 如果数据长度过长会发生数据截断
      length = TX_MAX_BUF;
    }
    instance->length = length;
    memcpy(instance->data, data, instance->length);
  } else {
    return TX_ERROR;
  }

  if (instance->state != TX_READY) {
    return TX_BUSY;
  }
  HAL_StatusTypeDef ret = HAL_UART_Transmit_IT(instance->huart, instance->data, instance->length);
  if (ret == HAL_OK) {
    instance->state = TX_BUSY;
    return TX_OK;
  }
  return (ret == HAL_BUSY) ? TX_BUSY : TX_ERROR;
}

void TxIt_Init(UART_HandleTypeDef *huart) {
  if (huart == NULL) {
    return;
  }

  // 防止重复注册
  for (int i = 0; i < instances_cnt; i++) {
    if (huart == instances[i].huart) {
      return;
    }
  }

  // TODO: 防止注册的huart不支持
  //
  // 内存分配
  TxIt_t instance = NULL;
  // 防止越界
  if (instances_cnt < TX_INSTANCE_MAX) {
    instance = &instances[instances_cnt++];
  } else {
    return;
  }

  // 初始化实例
  instance->huart = huart;
  instance->state = TX_READY;
  instance->length = 0;
  for (int i = 0; i < TX_MAX_BUF; i++) {
    instance->data[i] = 0;
  }
}

TxIt_State TxIt_GetState(UART_HandleTypeDef *huart) {
  for (int i = 0; i < instances_cnt; i++) {
    if (huart == instances[i].huart) {
      return instances[i].state;
    }
  }
  return TX_ERROR;
}
