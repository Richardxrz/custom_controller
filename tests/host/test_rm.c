/*
 * RM 帧 + 30 B / 300 B 载荷的主机侧测试。
 * 判据来源：2026 通信协议附录一（CRC）、载荷布局见 Module/app_payload.h。
 */
#include "app_payload.h"
#include "rm_frame.h"
#include "test_util.h"

static void case_crc(void) {
  printf("[crc]\n");
  /* 这几个值是用官方 CRC8_TAB 反推的反射算法算出来的（见 rm_frame.c 注释） */
  uint8_t a[] = {0x00};
  uint8_t b[] = {0x01};
  uint8_t h[] = {0xA5, 0x00, 0x00, 0x00};
  t_check(RmCrc8(a, 1) == 0x35, "CRC8([00]) = 0x35");
  t_check(RmCrc8(b, 1) == 0x6B, "CRC8([01]) = 0x6B");
  t_check(RmCrc8(h, 4) == 0xC3, "CRC8(A5 00 00 00) = 0xC3");
  t_check(RmCrc16(h, 4) == 0xD2F3, "CRC16(A5 00 00 00) = 0xD2F3");
  t_check(RmCrc16((const uint8_t *)"123456789", 9) == 0x29B1, "CRC16(\"123456789\") = 0x29B1");
}

static void case_frame_roundtrip(void) {
  printf("[rm_frame 往返]\n");
  uint8_t payload[30];
  for (uint8_t i = 0; i < 30; i++) {
    payload[i] = (uint8_t)(i * 7u + 3u);
  }
  uint8_t buf[64];
  uint16_t n = RmFrame_Encode(buf, sizeof(buf), 0x0311, 42, payload, 30);
  t_check(n == 39, "帧长 = 9 + 30 = 39");

  RmParser p;
  RmParser_Reset(&p);
  RmFrame f;
  bool got = false;
  for (uint16_t i = 0; i < n; i++) {
    got = RmParser_Feed(&p, buf[i], &f) || got;
  }
  t_check(got, "逐字节喂完能出帧");
  t_check(f.cmd_id == 0x0311, "cmd_id = 0x0311");
  t_check(f.seq == 42, "seq = 42");
  t_check(f.len == 30, "len = 30");
  t_check(memcmp(f.data, payload, 30) == 0, "data 一致");
}

static void case_frame_reject(void) {
  printf("[rm_frame 容错]\n");
  uint8_t payload[30] = {0};
  uint8_t buf[64];
  uint16_t n = RmFrame_Encode(buf, sizeof(buf), 0x0311, 1, payload, 30);
  buf[10] ^= 0xFF; /* 破坏 data，CRC16 必失败 */

  RmParser p;
  RmParser_Reset(&p);
  RmFrame f;
  bool got = false;
  for (uint16_t i = 0; i < n; i++) {
    got = RmParser_Feed(&p, buf[i], &f) || got;
  }
  t_check(!got, "CRC 错的帧被丢弃");

  /* 坏帧后面紧跟好帧：解析器要能重新对齐 */
  uint8_t good[64];
  uint16_t m = RmFrame_Encode(good, sizeof(good), 0x0311, 2, payload, 30);
  got = false;
  for (uint16_t i = 0; i < m; i++) {
    got = RmParser_Feed(&p, good[i], &f) || got;
  }
  t_check(got && f.seq == 2, "坏帧后能收到好帧");
}

static void case_frame_noise(void) {
  printf("[rm_frame 噪声]\n");
  uint8_t payload[4] = {1, 2, 3, 4};
  uint8_t buf[32];
  uint16_t n = RmFrame_Encode(buf, sizeof(buf), 0x0302, 7, payload, 4);

  RmParser p;
  RmParser_Reset(&p);
  RmFrame f;
  bool got = false;
  const char *noise = "hello\xA5\x00";
  for (const char *q = noise; *q; q++) {
    got = RmParser_Feed(&p, (uint8_t)*q, &f) || got;
  }
  for (uint16_t i = 0; i < n; i++) {
    got = RmParser_Feed(&p, buf[i], &f) || got;
  }
  t_check(got && f.cmd_id == 0x0302 && f.len == 4 && f.data[2] == 3, "噪声后仍能对齐");
}

static void case_uplink(void) {
  printf("[30B 上行]\n");
  CcUplink in;
  memset(&in, 0, sizeof(in));
  in.ver_mode = 0x11;
  in.seq = 9;
  for (uint8_t i = 0; i < CC_JOINTS; i++) {
    in.joints[i] = (uint16_t)(1000 + i * 100);
  }
  in.grip = 1234;
  in.buttons = 0x05;
  in.flags = 0x01;

  uint8_t buf[CC_UPLINK_LEN];
  CcUplink_Encode(buf, &in);
  t_check(RmCrc16(buf, CC_UPLINK_LEN - 2u) ==
              (uint16_t)(buf[28] | ((uint16_t)buf[29] << 8)),
          "CRC16 写入正确");

  CcUplink out;
  t_check(CcUplink_Decode(&out, buf), "解码成功");
  t_check(out.ver_mode == 0x11 && out.seq == 9 && out.grip == 1234, "标量字段一致");
  t_check(memcmp(out.joints, in.joints, sizeof(in.joints)) == 0, "关节一致");

  buf[5] ^= 0x01;
  t_check(!CcUplink_Decode(&out, buf), "改一个字节后解码失败");
}

static void case_downlink(void) {
  printf("[300B 下行]\n");
  CcDownlink in;
  memset(&in, 0, sizeof(in));
  in.ver = 1;
  in.flags = 0xA5;
  for (uint8_t i = 0; i < CC_JOINTS; i++) {
    in.joint[i].pos_urad = (int32_t)(-1000000 + (int32_t)i * 250000);
    in.joint[i].vel_crad = (int16_t)(-300 + i);
    in.joint[i].torque_cnm = (int16_t)(100 * i);
    in.joint[i].temp_c = (uint8_t)(30 + i);
    in.joint[i].err = (uint8_t)i;
  }
  in.vbus_mv = 24000;
  in.lost_frames = 3;

  uint8_t buf[CC_DOWNLINK_LEN];
  CcDownlink_Encode(buf, &in);
  CcDownlink out;
  t_check(CcDownlink_Decode(&out, buf), "解码成功");
  t_check(out.vbus_mv == 24000 && out.lost_frames == 3, "标量字段一致");
  t_check(out.joint[5].pos_urad == in.joint[5].pos_urad, "负位置一致");
  t_check(out.joint[3].vel_crad == in.joint[3].vel_crad, "速度一致");
  t_check(out.joint[2].temp_c == 32, "温度一致");

  buf[10] ^= 0x01;
  t_check(!CcDownlink_Decode(&out, buf), "改一个字节后解码失败");
}

int main(void) {
  case_crc();
  case_frame_roundtrip();
  case_frame_reject();
  case_frame_noise();
  case_uplink();
  case_downlink();
  return t_summary();
}
