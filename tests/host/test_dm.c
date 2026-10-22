/*
 * 达妙 MIT 帧编解码测试。
 * 注意：这里验证的是「位域打包/解包自洽 + 量程换算」，不等于验证了达妙协议本身；
 * 上电机前必须对照手册核对 FAQ 里列的三处不确定性。
 */
#include "dm_can.h"
#include "test_util.h"

static DmMotorCfg cfg4340(void) {
  DmMotorCfg c = {.id = 3, .p_min = -12.5f, .p_max = 12.5f, .v_min = -30.f, .v_max = 30.f,
                  .t_min = -10.f, .t_max = 10.f};
  return c;
}

static void case_pack_zero(void) {
  printf("[MIT 打包：零点]\n");
  DmMotorCfg c = cfg4340();
  uint8_t d[8];
  Dm_MitPack(d, &c, 0.f, 0.f, 0.f, 0.f, 0.f);
  uint8_t want[8] = {0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x07, 0xFF};
  t_check(memcmp(d, want, 8) == 0, "零位字节序列");
}

static void case_pack_max(void) {
  printf("[MIT 打包：满量程]\n");
  DmMotorCfg c = cfg4340();
  uint8_t d[8];
  Dm_MitPack(d, &c, 12.5f, 30.f, 0.f, 0.f, 10.f);
  uint8_t want[8] = {0xFF, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x0F, 0xFF};
  t_check(memcmp(d, want, 8) == 0, "最大位置/速度/力矩");

  Dm_MitPack(d, &c, 100.f, 100.f, 0.f, 0.f, 100.f);
  t_check(memcmp(d, want, 8) == 0, "超量程被夹到边界");
}

static void case_feedback(void) {
  printf("[反馈解包]\n");
  DmMotorCfg c = cfg4340();
  uint8_t d[8] = {0x03, 0x21, 0x7F, 0xFF, 0x07, 0xFF, 0x07, 0xFF};
  DmFeedback fb;
  t_check(Dm_FeedbackDecode(&fb, &c, d), "ID 匹配解码成功");
  t_check(fb.id == 3 && fb.state == 0x21, "ID / 状态字节");
  t_check(fb.pos > -0.01f && fb.pos < 0.01f, "位置约 0");
  t_check(fb.vel > -0.05f && fb.vel < 0.05f, "速度约 0");
  t_check(fb.torque > -0.02f && fb.torque < 0.02f, "力矩约 0");

  d[0] = 4;
  t_check(!Dm_FeedbackDecode(&fb, &c, d), "ID 不匹配返回 false");
}

static void case_scale(void) {
  printf("[量程换算]\n");
  t_check(Dm_FloatToUint(-12.5f, -12.5f, 12.5f, 16) == 0, "最小值 → 0");
  t_check(Dm_FloatToUint(12.5f, -12.5f, 12.5f, 16) == 65535, "最大值 → 65535");
  t_check(Dm_FloatToUint(0.f, -30.f, 30.f, 12) == 2047, "12 bit 中点");
  float x = Dm_UintToFloat(65535, -12.5f, 12.5f, 16);
  t_check(x > 12.49f && x <= 12.5f, "反算最大值");
}

static void case_special(void) {
  printf("[特殊帧]\n");
  uint8_t d[8];
  Dm_SpecialFrame(d, 'e');
  t_check(d[0] == 0xFF && d[7] == 0xFC, "使能帧 …FF FC");
  Dm_SpecialFrame(d, 'd');
  t_check(d[7] == 0xFD, "失能帧 …FF FD");
  Dm_SpecialFrame(d, 'z');
  t_check(d[7] == 0xFE, "设零帧 …FF FE");
}

int main(void) {
  case_pack_zero();
  case_pack_max();
  case_feedback();
  case_scale();
  case_special();
  return t_summary();
}
