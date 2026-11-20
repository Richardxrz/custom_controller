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
#define MASTER_SERVO_TIMEOUT_MS 8u
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

static TIM_HandleTypeDef s_htim5;
static uint32_t s_led_last;

/* 板载 RGB 共阳：PH12 红(TIM5_CH3)、PH11 绿(CH2)、PH10 蓝(CH1)，高电平点亮。
 * 1 kHz PWM，20 ms 更新一次；三路正弦相差 120°，形成连续渐变流水。 */
static void led_pwm_init(void) {
  GPIO_InitTypeDef gpio = {0};
  TIM_OC_InitTypeDef oc = {0};

  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_TIM5_CLK_ENABLE();
  gpio.Pin = LED_R_Pin | LED_G_Pin | LED_B_Pin;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF2_TIM5;
  HAL_GPIO_Init(GPIOH, &gpio);

  s_htim5.Instance = TIM5;
  s_htim5.Init.Prescaler = 84u - 1u;      /* 84 MHz / 84 = 1 MHz */
  s_htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  s_htim5.Init.Period = 1000u - 1u;       /* 1 kHz */
  s_htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  s_htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&s_htim5) != HAL_OK) {
    return;
  }
  oc.OCMode = TIM_OCMODE_PWM1;
  oc.Pulse = 0u;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&s_htim5, &oc, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&s_htim5, &oc, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&s_htim5, &oc, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&s_htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&s_htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&s_htim5, TIM_CHANNEL_3);
}

static void led_tick(void) {
  uint32_t now = HAL_GetTick();
  if ((uint32_t)(now - s_led_last) < 20u) {
    return;
  }
  s_led_last = now;
  /* 6 s 一圈色环，1536 级；不引入浮点/数学库。 */
  uint32_t hue = (now % 6000u) * 1536u / 6000u;
  uint32_t sector = hue / 256u;
  uint32_t frac = hue % 256u;
  uint32_t v = 1000u;
  uint32_t q = v * (256u - frac) / 256u;
  uint32_t t = v * frac / 256u;
  uint32_t r = 0u, g = 0u, b = 0u;
  switch (sector) {
  case 0: r = v; g = t; b = 0u; break;
  case 1: r = q; g = v; b = 0u; break;
  case 2: r = 0u; g = v; b = t; break;
  case 3: r = 0u; g = q; b = v; break;
  case 4: r = t; g = 0u; b = v; break;
  default: r = v; g = 0u; b = q; break;
  }
  __HAL_TIM_SET_COMPARE(&s_htim5, TIM_CHANNEL_3, r); /* R */
  __HAL_TIM_SET_COMPARE(&s_htim5, TIM_CHANNEL_2, g); /* G */
  __HAL_TIM_SET_COMPARE(&s_htim5, TIM_CHANNEL_1, b); /* B */
}

static uint8_t scan_bus(BusServoPort port, uint8_t *out, uint8_t cap) {
  uint8_t n = 0;
  for (uint16_t id = 1; id <= 253 && n < cap; id++) {
    BusServo s;
    BusServo_Init(&s, port, (uint8_t)id);
    BusServo_SetTimeout(&s, 8u);
    if (!BusServo_RequestAngle(&s)) {
      continue;
    }
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 15u) {
      BusServo_Poll(&s);
      led_tick();
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

static uint8_t joint_slot(uint8_t id) {
  switch (id) {
  case 4: return 3; /* pitch -> J3 */
  case 5: return 5; /* roll  -> J5 */
  case 6: return 4; /* yaw   -> J4 */
  default: return (id >= 1u && id <= 3u) ? (uint8_t)(id - 1u) : 0xFFu;
  }
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
  led_pwm_init();
  led_tick();
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
  led_tick();
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

  for (uint8_t i = 0; i < s_group_a.n && i < 3u; i++) {
    uint16_t pwm = 0;
    uint8_t id = s_ids_a[i];
    uint8_t slot = joint_slot(id);
    if (slot < 6u && BusServoGroup_GetPos(&s_group_a, i, &pwm)) {
      s_up.joints[slot] = pwm;
    }
  }
  for (uint8_t i = 0; i < s_group_b.n && i < 3u; i++) {
    uint16_t pwm = 0;
    uint8_t id = s_ids_b[i];
    uint8_t slot = joint_slot(id);
    if (slot < 6u && BusServoGroup_GetPos(&s_group_b, i, &pwm)) {
      s_up.joints[slot] = pwm;
    }
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
