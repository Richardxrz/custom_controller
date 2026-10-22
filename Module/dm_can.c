#include "dm_can.h"

#include <string.h>

static float clampf(float x, float lo, float hi) {
  if (x < lo) {
    return lo;
  }
  if (x > hi) {
    return hi;
  }
  return x;
}

uint16_t Dm_FloatToUint(float x, float min, float max, uint8_t bits) {
  if (bits == 0u || bits > 16u || max <= min) {
    return 0u;
  }
  float span = max - min;
  float v = (clampf(x, min, max) - min) / span;
  uint32_t max_int = (1u << bits) - 1u;
  return (uint16_t)(v * (float)max_int);
}

float Dm_UintToFloat(uint16_t u, float min, float max, uint8_t bits) {
  if (bits == 0u || bits > 16u || max <= min) {
    return min;
  }
  uint32_t max_int = (1u << bits) - 1u;
  return (float)u / (float)max_int * (max - min) + min;
}

void Dm_MitPack(uint8_t out[8], const DmMotorCfg *cfg, float pos, float vel, float kp,
                float kd, float torque) {
  if (out == NULL || cfg == NULL) {
    return;
  }
  uint16_t p = Dm_FloatToUint(pos, cfg->p_min, cfg->p_max, 16u);
  uint16_t v = Dm_FloatToUint(vel, cfg->v_min, cfg->v_max, 12u);
  uint16_t kpi = Dm_FloatToUint(kp, 0.0f, 500.0f, 12u);
  uint16_t kdi = Dm_FloatToUint(kd, 0.0f, 5.0f, 12u);
  uint16_t t = Dm_FloatToUint(torque, cfg->t_min, cfg->t_max, 12u);

  out[0] = (uint8_t)(p >> 8);
  out[1] = (uint8_t)(p & 0xFFu);
  out[2] = (uint8_t)(v >> 4);
  out[3] = (uint8_t)(((v & 0x0Fu) << 4) | (kpi >> 8));
  out[4] = (uint8_t)(kpi & 0xFFu);
  out[5] = (uint8_t)(kdi >> 4);
  out[6] = (uint8_t)(((kdi & 0x0Fu) << 4) | (t >> 8));
  out[7] = (uint8_t)(t & 0xFFu);
}

bool Dm_FeedbackDecode(DmFeedback *fb, const DmMotorCfg *cfg, const uint8_t data[8]) {
  if (fb == NULL || cfg == NULL || data == NULL) {
    return false;
  }
  if (data[0] != cfg->id) {
    return false;
  }
  uint16_t p = (uint16_t)(((uint16_t)data[2] << 8) | data[3]);
  uint16_t v = (uint16_t)(((uint16_t)data[4] << 8) | data[5]);
  uint16_t t = (uint16_t)(((uint16_t)data[6] << 8) | data[7]);

  fb->id = data[0];
  fb->state = data[1];
  fb->pos = Dm_UintToFloat(p, cfg->p_min, cfg->p_max, 16u);
  fb->vel = Dm_UintToFloat(v, cfg->v_min, cfg->v_max, 12u);
  fb->torque = Dm_UintToFloat(t, cfg->t_min, cfg->t_max, 12u);
  return true;
}

void Dm_SpecialFrame(uint8_t out[8], char kind) {
  if (out == NULL) {
    return;
  }
  memset(out, 0xFF, 8);
  switch (kind) {
  case 'e': /* 使能 */
    out[7] = 0xFC;
    break;
  case 'd': /* 失能 */
    out[7] = 0xFD;
    break;
  case 'z': /* 设零 */
    out[7] = 0xFE;
    break;
  case 'c': /* 清错 */
    out[7] = 0xFB;
    break;
  default:
    out[7] = 0xFD;
    break;
  }
}
