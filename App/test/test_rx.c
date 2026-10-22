#include "test_rx.h"
#include "rx_dma.h"
#include "stdint.h"

#define RX_FRAME_LEN 16

typedef struct {
  uint8_t rx_frame[RX_FRAME_LEN];
  uint8_t rx_framesN;
  uint8_t last_DATA_length;
  uint8_t err_cnt;
} RxDma_Data;

RxDma_Data rxdma_data = {
    .rx_frame = {0},
    .err_cnt = 0,
    .last_DATA_length = 0,
    .rx_framesN = 0,
};

typedef struct usr_rxdata {
  uint8_t longest_DATA_length;
} Usr_RxData;

Usr_RxData usr_rxdata = {
    .longest_DATA_length = 0,
};

typedef struct {
  float ch_data[4];
  unsigned char tail[4];
} TxFrame;

TxFrame tx_frame = {
    .ch_data = {0.0f},
    .tail = {0x00, 0x00, 0x80, 0x7f},
};

static RxDma_t rx6;
static int8_t isRxDataValid = 0;
uint16_t tx_length = 0;
uint32_t wait_next_tx = 0;

static void Update_UsrRxData(RxDma_t self, RxDma_Data *rxdma_data, Usr_RxData *usr_rxdata);
static uint8_t Longest_DATA_length(uint8_t currentL, uint8_t longestL);
static void LED_Dance_With_FramesN(const uint8_t framesN);
static void ScheduleSend_200_ms(TxFrame *tx_frame, uint16_t length);
static void Update_UsrTxData(TxFrame *frame, const RxDma_Data *rxdma_data,
                             const Usr_RxData *usr_rxdata);

void Test_RxDma_SetUp(void) { rx6 = RxDma_open(&huart6); }

void Test_RxDma_Run(void) {
  if (RxDma_fetch(rx6, rxdma_data.rx_frame,
                  sizeof(rxdma_data.rx_frame)) > 0) { // 如果抓到了
    isRxDataValid = 1;
  } else {
    isRxDataValid = 0;
  }
  Update_UsrRxData(rx6, &rxdma_data, &usr_rxdata);
  if (isRxDataValid) {
    LED_Dance_With_FramesN(rxdma_data.rx_framesN);
  }
  Update_UsrTxData(&tx_frame, &rxdma_data, &usr_rxdata);
  ScheduleSend_200_ms(&tx_frame, tx_length);
}

static void Update_UsrRxData(RxDma_t self, RxDma_Data *rxdma_data, Usr_RxData *usr_rxdata) {
  rxdma_data->err_cnt = RxDma_GetErrCnt(self);
  if (isRxDataValid) {
    rxdma_data->rx_framesN++;
    usr_rxdata->longest_DATA_length =
        Longest_DATA_length(RxDma_GetDATALength(self), usr_rxdata->longest_DATA_length);
    rxdma_data->last_DATA_length = RxDma_GetDATALength(self);
  }
}

static uint8_t Longest_DATA_length(uint8_t currentL, uint8_t longestL) {
  if (currentL > longestL) {
    return currentL;
  }
  return longestL;
}

// static void LED_R_Toggle(void) {
//   if (HAL_GetTick() - last_toggle_time > 500) {
//     HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_12);
//     last_toggle_time = HAL_GetTick();
//   }
// }

static void LED_Dance_With_FramesN(const uint8_t framesN) {
  uint8_t whoDance = framesN & 0x7;
  GPIO_PinState led_r_pin_state = (GPIO_PinState)whoDance & 0b100;
  GPIO_PinState led_g_pin_state = (GPIO_PinState)whoDance & 0b010;
  GPIO_PinState led_b_pin_state = (GPIO_PinState)whoDance & 0b001;
  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, led_r_pin_state);
  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, led_g_pin_state);
  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, led_b_pin_state);
}

static void Update_UsrTxData(TxFrame *frame, const RxDma_Data *rxdma_data,
                             const Usr_RxData *usr_rxdata) {
  frame->ch_data[0] = rxdma_data->rx_framesN;
  frame->ch_data[1] = rxdma_data->last_DATA_length;
  frame->ch_data[2] = usr_rxdata->longest_DATA_length;
  frame->ch_data[3] = rxdma_data->err_cnt;
}

static void ScheduleSend_200_ms(TxFrame *tx_frame, uint16_t length) {
  if (HAL_GetTick() - wait_next_tx > 200) {
    wait_next_tx = HAL_GetTick();
    HAL_UART_Transmit_IT(&huart6, (uint8_t *)tx_frame, sizeof(*tx_frame));
  }
}
