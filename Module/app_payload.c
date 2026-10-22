#include "app_payload.h"

#include <string.h>

#include "rm_frame.h"

static void put_u16(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)(v >> 8);
}

static uint16_t get_u16(const uint8_t *p) {
  return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static void put_i32(uint8_t *p, int32_t v) {
  uint32_t u = (uint32_t)v;
  p[0] = (uint8_t)(u & 0xFFu);
  p[1] = (uint8_t)((u >> 8) & 0xFFu);
  p[2] = (uint8_t)((u >> 16) & 0xFFu);
  p[3] = (uint8_t)((u >> 24) & 0xFFu);
}

static int32_t get_i32(const uint8_t *p) {
  uint32_t u = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
               ((uint32_t)p[3] << 24);
  return (int32_t)u;
}

static uint16_t clamp_pwm(uint16_t v) {
  if (v < 500u) {
    return 500u;
  }
  if (v > 2500u) {
    return 2500u;
  }
  return v;
}

void CcUplink_Encode(uint8_t out[CC_UPLINK_LEN], const CcUplink *p) {
  if (out == NULL || p == NULL) {
    return;
  }
  memset(out, 0, CC_UPLINK_LEN);
  out[0] = p->ver_mode;
  out[1] = p->seq;
  for (uint8_t i = 0; i < CC_JOINTS; i++) {
    put_u16(&out[2u + 2u * i], clamp_pwm(p->joints[i]));
  }
  put_u16(&out[14], p->grip);
  out[16] = p->buttons;
  out[17] = p->flags;
  uint16_t crc = RmCrc16(out, CC_UPLINK_LEN - 2u);
  put_u16(&out[28], crc);
}

bool CcUplink_Decode(CcUplink *p, const uint8_t in[CC_UPLINK_LEN]) {
  if (p == NULL || in == NULL) {
    return false;
  }
  uint16_t crc = get_u16(&in[28]);
  if (RmCrc16(in, CC_UPLINK_LEN - 2u) != crc) {
    return false;
  }
  p->ver_mode = in[0];
  p->seq = in[1];
  for (uint8_t i = 0; i < CC_JOINTS; i++) {
    p->joints[i] = clamp_pwm(get_u16(&in[2u + 2u * i]));
  }
  p->grip = get_u16(&in[14]);
  p->buttons = in[16];
  p->flags = in[17];
  return true;
}

void CcDownlink_Encode(uint8_t out[CC_DOWNLINK_LEN], const CcDownlink *p) {
  if (out == NULL || p == NULL) {
    return;
  }
  memset(out, 0, CC_DOWNLINK_LEN);
  out[0] = p->ver;
  out[1] = p->flags;
  for (uint8_t i = 0; i < CC_JOINTS; i++) {
    put_i32(&out[2u + 4u * i], p->joint[i].pos_urad);
  }
  for (uint8_t i = 0; i < CC_JOINTS; i++) {
    put_u16(&out[26u + 2u * i], (uint16_t)p->joint[i].vel_crad);
  }
  for (uint8_t i = 0; i < CC_JOINTS; i++) {
    put_u16(&out[38u + 2u * i], (uint16_t)p->joint[i].torque_cnm);
  }
  for (uint8_t i = 0; i < CC_JOINTS; i++) {
    out[50u + i] = p->joint[i].temp_c;
  }
  for (uint8_t i = 0; i < CC_JOINTS; i++) {
    out[56u + i] = p->joint[i].err;
  }
  put_u16(&out[62], p->vbus_mv);
  put_u16(&out[64], p->lost_frames);
  put_u16(&out[66], RmCrc16(out, 66u));
}

bool CcDownlink_Decode(CcDownlink *p, const uint8_t in[CC_DOWNLINK_LEN]) {
  if (p == NULL || in == NULL) {
    return false;
  }
  uint16_t crc = get_u16(&in[66]);
  if (RmCrc16(in, 66u) != crc) {
    return false;
  }
  p->ver = in[0];
  p->flags = in[1];
  for (uint8_t i = 0; i < CC_JOINTS; i++) {
    p->joint[i].pos_urad = get_i32(&in[2u + 4u * i]);
  }
  for (uint8_t i = 0; i < CC_JOINTS; i++) {
    p->joint[i].vel_crad = (int16_t)get_u16(&in[26u + 2u * i]);
  }
  for (uint8_t i = 0; i < CC_JOINTS; i++) {
    p->joint[i].torque_cnm = (int16_t)get_u16(&in[38u + 2u * i]);
  }
  for (uint8_t i = 0; i < CC_JOINTS; i++) {
    p->joint[i].temp_c = in[50u + i];
  }
  for (uint8_t i = 0; i < CC_JOINTS; i++) {
    p->joint[i].err = in[56u + i];
  }
  p->vbus_mv = get_u16(&in[62]);
  p->lost_frames = get_u16(&in[64]);
  return true;
}
