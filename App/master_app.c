#include "master_app.h"

#include <string.h>

#include "app_payload.h"
#include "board_io.h"
#include "bus_servo_group.h"
#include "rm_frame.h"
#include "tx_uart_it.h"
#include "uart_rx.h"
#include "usbd_cdc_if.h"
#include "usbd_def.h"

#define MASTER_JOINTS 6u
#define MASTER_TX_PERIOD_MS 10u
#define MASTER_SERVO_TIMEOUT_MS 4u
#define MASTER_SCAN_PERIOD_MS 1000u

extern USBD_HandleTypeDef hUsbDeviceFS;

static BusServoPort s_port_a;
static BusServoPort s_port_b;
static BusServoGroup s_group_a;
static BusServoGroup s_group_b;
static uint8_t s_ids_a[6];
static uint8_t s_ids_b[6];
static uint8_t s_na;
static uint8_t s_nb;
static uint16_t s_range_deg = 270u;
static uint8_t s_dir_mask;
static CcUplink s_up;
static uint8_t s_seq;
static uint32_t s_last_tx;
static uint32_t s_last_scan;
static bool s_sweeping;
static uint8_t s_frame[64];
static uint8_t s_map_frame[64];
static uint8_t s_scan_frame[64];
static RmParser s_pc;

static uint8_t scan_bus(BusServoPort port, uint8_t *out, uint8_t cap) {
  uint8_t n = 0;
  for (uint8_t id = 1; id <= 6 && n < cap; id++) {
    BusServo s;
    BusServo_Init(&s, port, id);
    BusServo_SetTimeout(&s, 3u);
    if (!BusServo_RequestAngle(&s)) {
      continue;
    }
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 12u) {
      BusServo_Poll(&s);
      if (BusServo_GetState(&s) != BUS_SERVO_WAIT) {
        break;
      }
      HAL_Delay(1);
    }
    if (BusServo_GetState(&s) == BUS_SERVO_DONE) {
      out[n++] = id;
    }
  }
  return n;
}

static void start_groups(void) {
  uint8_t a[3], b[3];
  uint8_t an = (s_na > 3u) ? 3u : s_na;
  uint8_t bn = (s_nb > 3u) ? 3u : s_nb;
  for (uint8_t i = 0; i < an; i++) a[i] = s_ids_a[i];
  for (uint8_t i = 0; i < bn; i++) b[i] = s_ids_b[i];
  BusServoGroup_Init(&s_group_a, s_port_a, a, an, MASTER_SERVO_TIMEOUT_MS);
  BusServoGroup_Init(&s_group_b, s_port_b, b, bn, MASTER_SERVO_TIMEOUT_MS);
  BusServoGroup_SetPeriod(&s_group_a, 10u);
  BusServoGroup_SetPeriod(&s_group_b, 10u);
  if (an > 0u) BusServoGroup_ReleaseAll(&s_group_a, HAL_GetTick());
  if (bn > 0u) BusServoGroup_ReleaseAll(&s_group_b, HAL_GetTick());
}

static void scan_and_init(void) {
  s_na = scan_bus(s_port_a, s_ids_a, sizeof(s_ids_a));
  s_nb = scan_bus(s_port_b, s_ids_b, sizeof(s_ids_b));
  start_groups();
  s_sweeping = false;
  s_last_scan = HAL_GetTick();
}

static int16_t map_cdeg(uint16_t raw, uint8_t axis) {
  int32_t span = (int32_t)s_range_deg * 100;
  int32_t v = ((int32_t)raw - 500) * span / 2000 - span / 2;
  if (s_dir_mask & (1u << axis)) v = -v;
  if (v < -span / 2) v = -span / 2;
  if (v > span / 2) v = span / 2;
  return (int16_t)v;
}

static void apply_pc_frame(const RmFrame *f) {
  if (f->cmd_id == 0x0301u && f->len >= 4u) {
    uint16_t deg = (uint16_t)(f->data[0] | (f->data[1] << 8));
    s_range_deg = (deg == 360u) ? 360u : 270u;
    s_dir_mask = f->data[2] & 0x3Fu;
  } else if (f->cmd_id == 0x0300u) {
    scan_and_init();
  }
}

static void poll_pc(void) {
  uint8_t buf[32];
  uint16_t n;
  while ((n = CDC_Read(buf, sizeof(buf))) > 0u) {
    for (uint16_t i = 0; i < n; i++) {
      RmFrame f;
      if (RmParser_Feed(&s_pc, buf[i], &f)) {
        apply_pc_frame(&f);
      }
    }
  }
}

static void send_scan_report(void) {
  uint8_t rep[16];
  memset(rep, 0, sizeof(rep));
  rep[0] = 1;
  rep[1] = s_na;
  rep[2] = s_nb;
  memcpy(&rep[3], s_ids_a, 6);
  memcpy(&rep[9], s_ids_b, 6);
  uint16_t n = RmFrame_Encode(s_scan_frame, sizeof(s_scan_frame), 0x0304, s_seq, rep, sizeof(rep));
  if (n > 0u && hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
    CDC_Transmit_FS(s_scan_frame, n);
  }
}

void Master_Setup(void) {
  Board_Uart1_Init(115200);
  Board_Uart6_SetBaud(115200);
  TxIt_Init(&huart1);
  TxIt_Init(&huart6);

  UartRx *rx_a = UartRx_Open(&huart1);
  UartRx *rx_b = UartRx_Open(&huart6);
  s_port_a = ServoPort_Open(rx_a, &huart1);
  s_port_b = ServoPort_Open(rx_b, &huart6);

  memset(&s_up, 0, sizeof(s_up));
  s_up.ver_mode = CC_UPLINK_VER;
  for (uint8_t i = 0; i < CC_JOINTS; i++) s_up.joints[i] = 1500;
  RmParser_Reset(&s_pc);

  scan_and_init();
  send_scan_report();
}

void Master_Loop(void) {
  uint32_t now = HAL_GetTick();
  BusServoGroup_Poll(&s_group_a, now);
  BusServoGroup_Poll(&s_group_b, now);
  poll_pc();

  bool a_done = (s_group_a.n == 0u) || (BusServoGroup_GetMode(&s_group_a) == GROUP_DONE);
  bool b_done = (s_group_b.n == 0u) || (BusServoGroup_GetMode(&s_group_b) == GROUP_DONE);
  if (!s_sweeping && a_done && b_done) {
    if (s_group_a.n > 0u) BusServoGroup_StartSweep(&s_group_a, now);
    if (s_group_b.n > 0u) BusServoGroup_StartSweep(&s_group_b, now);
    s_sweeping = true;
  }

  if ((uint32_t)(now - s_last_scan) >= MASTER_SCAN_PERIOD_MS) {
    s_last_scan = now;
    send_scan_report();
  }

  if ((uint32_t)(now - s_last_tx) < MASTER_TX_PERIOD_MS) {
    return;
  }
  s_last_tx = now;

  for (uint8_t i = 0; i < 3u; i++) {
    uint16_t pwm = 0;
    if (i < s_group_a.n && BusServoGroup_GetPos(&s_group_a, i, &pwm)) s_up.joints[i] = pwm;
    if (i < s_group_b.n && BusServoGroup_GetPos(&s_group_b, i, &pwm)) s_up.joints[i + 3] = pwm;
  }
  s_up.seq = s_seq++;
  s_up.flags = 0x01;
  s_up.buttons = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) ? 0x04u : 0x00u;

  uint8_t payload[CC_UPLINK_LEN];
  CcUplink_Encode(payload, &s_up);
  uint16_t n = RmFrame_Encode(s_frame, sizeof(s_frame), 0x0302, s_seq, payload, CC_UPLINK_LEN);
  if (n > 0u && hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
    CDC_Transmit_FS(s_frame, n);
  }

  uint8_t map[26];
  map[0] = 1;
  map[1] = 0;
  for (uint8_t i = 0; i < 6u; i++) {
    uint16_t raw = s_up.joints[i];
    memcpy(&map[2 + i * 2], &raw, 2);
    int16_t cdeg = map_cdeg(raw, i);
    memcpy(&map[14 + i * 2], &cdeg, 2);
  }
  uint16_t mn = RmFrame_Encode(s_map_frame, sizeof(s_map_frame), 0x0303, s_seq, map, sizeof(map));
  if (mn > 0u && hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
    CDC_Transmit_FS(s_map_frame, mn);
  }
}
