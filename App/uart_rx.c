#include "uart_rx.h"

#include "ringbuf.h"
#include "tx_uart_it.h"

#define UART_RX_MAX 2u
#define UART_RX_BUF 256u

struct UartRx {
  UART_HandleTypeDef *huart;
  RingBuf rb;
  uint8_t store[UART_RX_BUF];
  uint8_t byte;
  bool used;
};

static struct UartRx s_rx[UART_RX_MAX];
static uint8_t s_cnt;

static struct UartRx *find(UART_HandleTypeDef *huart) {
  for (uint8_t i = 0; i < s_cnt; i++) {
    if (s_rx[i].used && s_rx[i].huart == huart) {
      return &s_rx[i];
    }
  }
  return NULL;
}

static void arm(struct UartRx *r) { HAL_UART_Receive_IT(r->huart, &r->byte, 1u); }

UartRx *UartRx_Open(UART_HandleTypeDef *huart) {
  if (huart == NULL || s_cnt >= UART_RX_MAX) {
    return NULL;
  }
  struct UartRx *r = &s_rx[s_cnt++];
  r->huart = huart;
  r->used = true;
  RingBuf_Init(&r->rb, r->store, (uint16_t)sizeof(r->store));
  arm(r);
  return r;
}

uint16_t UartRx_Read(UartRx *rx, uint8_t *out, uint16_t cap) {
  if (rx == NULL) {
    return 0u;
  }
  return RingBuf_GetBlock(&rx->rb, out, cap);
}

uint16_t UartRx_Available(UartRx *rx) {
  return (rx == NULL) ? 0u : RingBuf_Count(&rx->rb);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  struct UartRx *r = find(huart);
  if (r != NULL) {
    RingBuf_Put(&r->rb, r->byte); /* 满了就丢新字节；统计留给上层 */
    arm(r);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  struct UartRx *r = find(huart);
  if (r != NULL) {
    arm(r); /* 溢出/噪声后重新武装，否则接收会永久停摆 */
  }
}

/* ---------------------------------------------------------------- 舵机端口 */

typedef struct {
  UartRx *rx;
  UART_HandleTypeDef *huart;
} ServoPortCtx;

static ServoPortCtx s_ctx[UART_RX_MAX];
static uint8_t s_ctx_cnt;

static void sp_write(void *ctx, const uint8_t *data, uint16_t len) {
  ServoPortCtx *c = (ServoPortCtx *)ctx;
  /* 正常节奏下上一条早就发完了；这里最多等 ~1 ms，避免死等 */
  for (uint32_t i = 0; i < 20000u; i++) {
    if (TxIt_Send(c->huart, (uint8_t *)data, len) != TX_BUSY) {
      return;
    }
  }
}

static uint16_t sp_read(void *ctx, uint8_t *buf, uint16_t cap) {
  ServoPortCtx *c = (ServoPortCtx *)ctx;
  return UartRx_Read(c->rx, buf, cap);
}

static uint32_t sp_now(void *ctx) {
  (void)ctx;
  return HAL_GetTick();
}

BusServoPort ServoPort_Open(UartRx *rx, UART_HandleTypeDef *huart) {
  BusServoPort p = {0};
  if (rx == NULL || huart == NULL || s_ctx_cnt >= UART_RX_MAX) {
    return p;
  }
  ServoPortCtx *c = &s_ctx[s_ctx_cnt++];
  c->rx = rx;
  c->huart = huart;
  p.write = sp_write;
  p.read = sp_read;
  p.now_ms = sp_now;
  p.ctx = c;
  return p;
}
