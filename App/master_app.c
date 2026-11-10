#include "master_app.h"

#include <string.h>

#include "app_payload.h"
#include "board_io.h"
#include "bus_servo_group.h"
#include "rm_frame.h"
#include "tx_uart_it.h"
#include "uart_rx.h"
#include "usbd_cdc_if.h"

#define MASTER_JOINTS 6u
#define MASTER_TX_PERIOD_MS 10u /* 100 Hz 上行 */
#define MASTER_SERVO_TIMEOUT_MS 5u

static BusServoGroup s_group_a;
static BusServoGroup s_group_b;
static CcUplink s_up;
static uint8_t s_seq;
static uint32_t s_last_tx;
static bool s_sweeping;
static uint8_t s_frame[64];
static uint8_t s_map_frame[64];

static int16_t map270_cdeg(uint16_t raw) {
  int32_t v = ((int32_t)raw - 500) * 27000 / 2000 - 13500;
  if (v < -13500) v = -13500;
  if (v > 13500) v = 13500;
  return (int16_t)v;
}

void Master_Setup(void) {
  Board_Uart1_Init(115200);   /* 舵机总线 A：3 个 */
  Board_Uart6_SetBaud(115200); /* 舵机总线 B：3 个 */

  TxIt_Init(&huart1);
  TxIt_Init(&huart6);

  UartRx *rx_a = UartRx_Open(&huart1);
  UartRx *rx_b = UartRx_Open(&huart6);
  BusServoPort port_a = ServoPort_Open(rx_a, &huart1);
  BusServoPort port_b = ServoPort_Open(rx_b, &huart6);

  uint8_t ids_a[3] = {1, 2, 3};
  uint8_t ids_b[3] = {4, 5, 6};
  BusServoGroup_Init(&s_group_a, port_a, ids_a, 3, MASTER_SERVO_TIMEOUT_MS);
  BusServoGroup_Init(&s_group_b, port_b, ids_b, 3, MASTER_SERVO_TIMEOUT_MS);
  BusServoGroup_SetPeriod(&s_group_a, 10);
  BusServoGroup_SetPeriod(&s_group_b, 10);

  memset(&s_up, 0, sizeof(s_up));
  s_up.ver_mode = CC_UPLINK_VER;
  for (uint8_t i = 0; i < CC_JOINTS; i++) {
    s_up.joints[i] = 1500;
  }

  /* 上电先全部释力，操作手才扳得动；两路都完成后再开始采样 */
  uint32_t t0 = HAL_GetTick();
  BusServoGroup_ReleaseAll(&s_group_a, t0);
  BusServoGroup_ReleaseAll(&s_group_b, t0);
  s_sweeping = false;
}

void Master_Loop(void) {
  uint32_t now = HAL_GetTick();
  BusServoGroup_Poll(&s_group_a, now);
  BusServoGroup_Poll(&s_group_b, now);

  if (!s_sweeping && BusServoGroup_GetMode(&s_group_a) == GROUP_DONE &&
      BusServoGroup_GetMode(&s_group_b) == GROUP_DONE) {
    BusServoGroup_StartSweep(&s_group_a, now);
    BusServoGroup_StartSweep(&s_group_b, now);
    s_sweeping = true;
  }
  if (!s_sweeping) {
    return;
  }

  if ((uint32_t)(now - s_last_tx) < MASTER_TX_PERIOD_MS) {
    return;
  }
  s_last_tx = now;

  for (uint8_t i = 0; i < 3; i++) {
    uint16_t pwm = 0;
    if (BusServoGroup_GetPos(&s_group_a, i, &pwm)) {
      s_up.joints[i] = pwm;
    }
    if (BusServoGroup_GetPos(&s_group_b, i, &pwm)) {
      s_up.joints[i + 3] = pwm;
    }
  }
  s_up.seq = s_seq++;
  s_up.flags = 0x01; /* 主端有效 */
  /* C 板 KEY（PA0，按下为低）当急停：按下时置 bit2，机器人端会保持当前位置 */
  s_up.buttons = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) ? 0x04u : 0x00u;

  uint8_t payload[CC_UPLINK_LEN];
  CcUplink_Encode(payload, &s_up);

  uint16_t n = RmFrame_Encode(s_frame, sizeof(s_frame), 0x0302, s_seq, payload, CC_UPLINK_LEN);
  if (n > 0u) {
    CDC_Transmit_FS(s_frame, n); /* USB CDC：PC 看到的仍是串口 */
  }

  /* 0x0303：原始 PRAD + C 板算出的 270° 映射角（0.01°），供 PC/3D 用 */
  uint8_t map[26];
  map[0] = 1;
  map[1] = 0;
  for (uint8_t i = 0; i < 6; i++) {
    uint16_t raw = s_up.joints[i];
    memcpy(&map[2 + i * 2], &raw, 2);
    int16_t cdeg = map270_cdeg(raw);
    memcpy(&map[14 + i * 2], &cdeg, 2);
  }
  uint16_t mn = RmFrame_Encode(s_map_frame, sizeof(s_map_frame), 0x0303, s_seq, map, sizeof(map));
  if (mn > 0u) {
    CDC_Transmit_FS(s_map_frame, mn);
  }
}
