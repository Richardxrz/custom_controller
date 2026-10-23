#include "arm_app.h"

#include <string.h>

#include "app_payload.h"
#include "board_io.h"
#include "dm_can.h"
#include "rm_frame.h"
#include "tx_uart_it.h"
#include "uart_rx.h"

#define ARM_JOINTS 6u
#define ARM_CTRL_PERIOD_MS 5u   /* 200 Hz 下发 */
#define ARM_TELEM_PERIOD_MS 20u /* 50 Hz 回传 */
#define ARM_LINK_TIMEOUT_MS 300u
#define ARM_ESTOP_BIT 0x04u
#define ARM_SLEW_RAD_PER_S 4.0f /* 27 Hz 输入下靠它把台阶磨平 */

/* 量程/限位先按「P ±12.5、V ±30」的通用值；T 量程按 8009/4340 暂定，上电机前对达妙手册核对。
 * 关节分配：J0..J2 = 8009（底座/肩/肘），J3..J5 = 4340（腕）。 */
static DmMotorCfg s_cfg[ARM_JOINTS] = {
    {1, -12.5f, 12.5f, -30.f, 30.f, -40.f, 40.f},
    {2, -12.5f, 12.5f, -30.f, 30.f, -40.f, 40.f},
    {3, -12.5f, 12.5f, -30.f, 30.f, -40.f, 40.f},
    {4, -12.5f, 12.5f, -30.f, 30.f, -28.f, 28.f},
    {5, -12.5f, 12.5f, -30.f, 30.f, -28.f, 28.f},
    {6, -12.5f, 12.5f, -30.f, 30.f, -28.f, 28.f},
};

/* 主端 500..2500 → 关节 -scale..+scale（rad）；dir 先全部 +1，装反了改这里 */
static const float s_scale[ARM_JOINTS] = {1.5f, 1.5f, 1.5f, 1.5f, 1.5f, 1.5f};
static const int8_t s_dir[ARM_JOINTS] = {1, 1, 1, 1, 1, 1};

static UartRx *s_rx;
static RmParser s_parser;
static CcUplink s_up;
static CcDownlink s_down;
static DmFeedback s_fb[ARM_JOINTS];
static bool s_fb_valid[ARM_JOINTS];
static float s_target[ARM_JOINTS];
static float s_smooth[ARM_JOINTS];
static uint32_t s_last_rx;
static uint32_t s_last_ctrl;
static uint32_t s_last_telem;
static uint8_t s_seq;
static bool s_have_cmd;
static bool s_estop;

static float map_pwm(uint8_t i, uint16_t pwm) {
  float x = ((float)pwm - 1500.0f) / 1000.0f;
  float v = x * s_scale[i] * (float)s_dir[i];
  if (v < s_cfg[i].p_min) {
    v = s_cfg[i].p_min;
  }
  if (v > s_cfg[i].p_max) {
    v = s_cfg[i].p_max;
  }
  return v;
}

static void parse_rx(uint32_t now) {
  uint8_t buf[64];
  uint16_t n = UartRx_Read(s_rx, buf, sizeof(buf));
  for (uint16_t i = 0; i < n; i++) {
    RmFrame f;
    if (!RmParser_Feed(&s_parser, buf[i], &f)) {
      continue;
    }
    if (f.cmd_id != 0x0311u || f.len != CC_UPLINK_LEN) {
      continue;
    }
    CcUplink up;
    if (!CcUplink_Decode(&up, f.data)) {
      continue;
    }
    s_up = up;
    s_have_cmd = true;
    s_last_rx = now;
    s_estop = (up.buttons & ARM_ESTOP_BIT) != 0u;
    for (uint8_t j = 0; j < ARM_JOINTS; j++) {
      s_target[j] = map_pwm(j, up.joints[j]);
    }
  }
}

static void poll_can(void) {
  uint16_t id = 0;
  uint8_t data[8];
  while (Board_Can1_Recv(&id, data)) {
    (void)id;
    for (uint8_t i = 0; i < ARM_JOINTS; i++) {
      DmFeedback fb;
      if (Dm_FeedbackDecode(&fb, &s_cfg[i], data)) {
        s_fb[i] = fb;
        s_fb_valid[i] = true;
      }
    }
  }
}

static void control(uint32_t now) {
  bool timeout = (uint32_t)(now - s_last_rx) > ARM_LINK_TIMEOUT_MS;
  const float step = ARM_SLEW_RAD_PER_S * (ARM_CTRL_PERIOD_MS / 1000.0f);
  for (uint8_t i = 0; i < ARM_JOINTS; i++) {
    float delta = s_target[i] - s_smooth[i];
    if (delta > step) delta = step;
    if (delta < -step) delta = -step;
    s_smooth[i] += delta;
    float pos = s_smooth[i];
    float kp = 8.0f;
    float kd = 0.5f;
    if (!s_have_cmd || timeout || s_estop) {
      /* 断链/急停：保持当前位置（用反馈值），加大阻尼，不直接掉力 */
      pos = s_fb_valid[i] ? s_fb[i].pos : s_target[i];
      kp = 12.0f;
      kd = 1.0f;
    }
    uint8_t d[8];
    Dm_MitPack(d, &s_cfg[i], pos, 0.0f, kp, kd, 0.0f);
    Board_Can1_Send(s_cfg[i].id, d);
  }
}

static void send_telemetry(void) {
  memset(&s_down, 0, sizeof(s_down));
  s_down.ver = 1;
  s_down.flags = s_estop ? 0x02u : 0x01u;
  for (uint8_t i = 0; i < ARM_JOINTS; i++) {
    if (s_fb_valid[i]) {
      s_down.joint[i].pos_urad = (int32_t)(s_fb[i].pos * 1000000.0f);
      s_down.joint[i].vel_crad = (int16_t)(s_fb[i].vel * 100.0f);
      s_down.joint[i].torque_cnm = (int16_t)(s_fb[i].torque * 100.0f);
      s_down.joint[i].err = s_fb[i].state;
    }
    s_down.joint[i].temp_c = 0; /* 温度位域待手册核对，先不猜 */
  }
  s_down.vbus_mv = 24000;
  s_down.lost_frames = 0;

  uint8_t payload[CC_DOWNLINK_LEN];
  CcDownlink_Encode(payload, &s_down);
  uint8_t frame[CC_DOWNLINK_LEN + 16u];
  uint16_t n = RmFrame_Encode(frame, sizeof(frame), 0x0310, s_seq++, payload, CC_DOWNLINK_LEN);
  if (n > 0u) {
    TxIt_Send(&huart1, frame, n);
  }
}

void Arm_Setup(void) {
  Board_Uart1_Init(921600); /* 图传发送端 */
  TxIt_Init(&huart1);
  s_rx = UartRx_Open(&huart1);
  RmParser_Reset(&s_parser);

  Board_Can1_Init();
  uint8_t en[8];
  Dm_SpecialFrame(en, 'e');
  for (uint8_t i = 0; i < ARM_JOINTS; i++) {
    Board_Can1_Send(s_cfg[i].id, en);
  }
  HAL_Delay(20);

  uint32_t now = HAL_GetTick();
  s_last_rx = now;
  s_last_ctrl = now;
  s_last_telem = now;
}

void Arm_Loop(void) {
  uint32_t now = HAL_GetTick();
  parse_rx(now);
  poll_can();

  if ((uint32_t)(now - s_last_ctrl) >= ARM_CTRL_PERIOD_MS) {
    s_last_ctrl = now;
    control(now);
  }
  if ((uint32_t)(now - s_last_telem) >= ARM_TELEM_PERIOD_MS) {
    s_last_telem = now;
    send_telemetry();
  }
}
