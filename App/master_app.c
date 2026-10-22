#include "master_app.h"

#include <string.h>

#include "app_payload.h"
#include "board_io.h"
#include "bus_servo_group.h"
#include "rm_frame.h"
#include "tx_uart_it.h"
#include "uart_rx.h"

#define MASTER_JOINTS 6u
#define MASTER_TX_PERIOD_MS 10u /* 100 Hz 上行 */
#define MASTER_SERVO_TIMEOUT_MS 5u

static BusServoGroup s_group;
static CcUplink s_up;
static uint8_t s_seq;
static uint32_t s_last_tx;
static bool s_sweeping;

void Master_Setup(void) {
  Board_Uart1_Init(921600);   /* PC 链路 */
  Board_Uart6_SetBaud(115200); /* 舵机总线 */

  TxIt_Init(&huart1);
  TxIt_Init(&huart6);

  UartRx *rx = UartRx_Open(&huart6);
  BusServoPort port = ServoPort_Open(rx, &huart6);

  uint8_t ids[MASTER_JOINTS] = {1, 2, 3, 4, 5, 6};
  BusServoGroup_Init(&s_group, port, ids, MASTER_JOINTS, MASTER_SERVO_TIMEOUT_MS);
  BusServoGroup_SetPeriod(&s_group, 10); /* 100 Hz 一轮 */

  memset(&s_up, 0, sizeof(s_up));
  s_up.ver_mode = CC_UPLINK_VER;
  for (uint8_t i = 0; i < CC_JOINTS; i++) {
    s_up.joints[i] = 1500;
  }

  /* 上电先全部释力，操作手才扳得动；释力完成后再开始采样 */
  BusServoGroup_ReleaseAll(&s_group, HAL_GetTick());
  s_sweeping = false;
}

void Master_Loop(void) {
  uint32_t now = HAL_GetTick();
  BusServoGroup_Poll(&s_group, now);

  if (!s_sweeping && BusServoGroup_GetMode(&s_group) == GROUP_DONE) {
    BusServoGroup_StartSweep(&s_group, now);
    s_sweeping = true;
  }
  if (!s_sweeping) {
    return;
  }

  if ((uint32_t)(now - s_last_tx) < MASTER_TX_PERIOD_MS) {
    return;
  }
  s_last_tx = now;

  for (uint8_t i = 0; i < MASTER_JOINTS; i++) {
    uint16_t pwm = 0;
    if (BusServoGroup_GetPos(&s_group, i, &pwm)) {
      s_up.joints[i] = pwm;
    }
  }
  s_up.seq = s_seq++;
  s_up.flags = 0x01; /* 主端有效 */
  /* C 板 KEY（PA0，按下为低）当急停：按下时置 bit2，机器人端会保持当前位置 */
  s_up.buttons = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) ? 0x04u : 0x00u;

  uint8_t payload[CC_UPLINK_LEN];
  CcUplink_Encode(payload, &s_up);

  uint8_t frame[64];
  uint16_t n = RmFrame_Encode(frame, sizeof(frame), 0x0302, s_seq, payload, CC_UPLINK_LEN);
  if (n > 0u) {
    TxIt_Send(&huart1, frame, n);
  }
}
