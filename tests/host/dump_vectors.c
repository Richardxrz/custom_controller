/* 打印一组固定输入的十六进制输出，给 Python 侧做交叉验证（C ↔ Python 必须逐字节一致）。 */
#include <stdio.h>

#include "app_payload.h"
#include "rm_frame.h"

static void dump(const char *tag, const uint8_t *p, int n) {
  printf("%s=", tag);
  for (int i = 0; i < n; i++) {
    printf("%02x", p[i]);
  }
  printf("\n");
}

int main(void) {
  uint8_t z[1] = {0x00};
  uint8_t one[1] = {0x01};
  uint8_t h[4] = {0xA5, 0x00, 0x00, 0x00};
  printf("crc8_00=%02x\n", RmCrc8(z, 1));
  printf("crc8_01=%02x\n", RmCrc8(one, 1));
  printf("crc8_a5=%02x\n", RmCrc8(h, 4));
  printf("crc16_a5=%04x\n", RmCrc16(h, 4));

  CcUplink up;
  up.ver_mode = 0x11;
  up.seq = 9;
  for (int i = 0; i < 6; i++) {
    up.joints[i] = (uint16_t)(1000 + i * 100);
  }
  up.grip = 1234;
  up.buttons = 0x05;
  up.flags = 0x01;
  uint8_t u[CC_UPLINK_LEN];
  CcUplink_Encode(u, &up);
  dump("uplink", u, CC_UPLINK_LEN);

  CcDownlink down;
  for (int i = 0; i < 6; i++) {
    down.joint[i].pos_urad = -1000000 + i * 250000;
    down.joint[i].vel_crad = (int16_t)(-300 + i);
    down.joint[i].torque_cnm = (int16_t)(100 * i);
    down.joint[i].temp_c = (uint8_t)(30 + i);
    down.joint[i].err = (uint8_t)i;
  }
  down.ver = 1;
  down.flags = 0xA5;
  down.vbus_mv = 24000;
  down.lost_frames = 3;
  uint8_t d[CC_DOWNLINK_LEN];
  CcDownlink_Encode(d, &down);
  dump("downlink", d, CC_DOWNLINK_LEN);

  uint8_t payload[30];
  for (int i = 0; i < 30; i++) {
    payload[i] = (uint8_t)(i * 7 + 3);
  }
  uint8_t frame[64];
  uint16_t n = RmFrame_Encode(frame, sizeof(frame), 0x0311, 42, payload, 30);
  dump("frame0311", frame, n);
  return 0;
}
