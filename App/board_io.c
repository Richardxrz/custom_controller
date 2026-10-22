#include "board_io.h"

UART_HandleTypeDef huart1;
CAN_HandleTypeDef hcan1;

void Board_Uart1_Init(uint32_t baud) {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_9; /* TX */
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &gpio);

  gpio.Pin = GPIO_PIN_7; /* RX */
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &gpio);

  huart1.Instance = USART1;
  huart1.Init.BaudRate = baud;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }

  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void Board_Uart6_SetBaud(uint32_t baud) {
  huart6.Init.BaudRate = baud;
  if (HAL_UART_Init(&huart6) != HAL_OK) {
    Error_Handler();
  }
  HAL_NVIC_SetPriority(USART6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART6_IRQn);
}

void Board_Can1_Init(void) {
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_CAN1_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1; /* RX / TX */
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF9_CAN1;
  HAL_GPIO_Init(GPIOD, &gpio);

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;      /* APB1 42 MHz / 3 = 14 MHz，tq = 1/14 us */
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ; /* 1 + 11 + 2 = 14 tq → 1 Mbps */
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK) {
    Error_Handler();
  }

  CAN_FilterTypeDef f = {0};
  f.FilterBank = 0;
  f.FilterMode = CAN_FILTERMODE_IDMASK;
  f.FilterScale = CAN_FILTERSCALE_32BIT;
  f.FilterIdHigh = 0;
  f.FilterIdLow = 0;
  f.FilterMaskIdHigh = 0;
  f.FilterMaskIdLow = 0;
  f.FilterFIFOAssignment = CAN_RX_FIFO0;
  f.FilterActivation = ENABLE;
  if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    Error_Handler();
  }
}

bool Board_Can1_Send(uint16_t std_id, const uint8_t data[8]) {
  CAN_TxHeaderTypeDef h = {0};
  h.StdId = std_id;
  h.IDE = CAN_ID_STD;
  h.RTR = CAN_RTR_DATA;
  h.DLC = 8;
  uint32_t mailbox = 0;
  return HAL_CAN_AddTxMessage(&hcan1, &h, (uint8_t *)data, &mailbox) == HAL_OK;
}

bool Board_Can1_Recv(uint16_t *std_id, uint8_t data[8]) {
  if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) == 0u) {
    return false;
  }
  CAN_RxHeaderTypeDef h = {0};
  if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &h, data) != HAL_OK) {
    return false;
  }
  if (std_id != NULL) {
    *std_id = (uint16_t)h.StdId;
  }
  return true;
}
