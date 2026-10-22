#include "tx_uart_it.h"
#include <stdint.h>

#define TX_MAX_BUF 128

// static uint32_t last_tx_time = 0;
static uint8_t usr_txdata[TX_MAX_BUF];
static uint16_t usr_txdata_length = 0;

static void LED_R_Toggle(void);

void Test_TxIt_SetUp(void) {
  TxIt_Init(&huart6);
  usr_txdata_length = 4;
}

void Test_TxIt_Run(void) {
  for (int i = 0; i < usr_txdata_length; i++) {
    usr_txdata[i] = i;
  }

  HAL_Delay(200);
  if (TxIt_GetState(&huart6) == TX_READY) {
    if (TxIt_Send(&huart6, usr_txdata, usr_txdata_length) == TX_OK) {
      LED_R_Toggle();
      return;
    }
  }
}

static void LED_R_Toggle(void) { HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_12); }
