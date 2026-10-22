#include "rm_frame.h"

#include <string.h>

// CRC8 用反射形式（0x8C = 0x31 的位反转），初值 0xFF。
// 这一条是从官方附录一的 CRC8_TAB 反推出来的：表里 table[1] = 0x5E，
// 只有「反射 + poly 0x8C + init 0」能对上；非反射 0x31 算出来是 0x31，对不上。
uint8_t RmCrc8(const uint8_t *data, uint32_t len) {
  uint8_t c = 0xFFu;
  for (uint32_t i = 0; i < len; i++) {
    c ^= data[i];
    for (uint8_t b = 0; b < 8u; b++) {
      c = (c & 0x01u) ? (uint8_t)((c >> 1) ^ 0x8Cu) : (uint8_t)(c >> 1);
    }
  }
  return c;
}

uint16_t RmCrc16(const uint8_t *data, uint32_t len) {
  uint16_t c = 0xFFFFu;
  for (uint32_t i = 0; i < len; i++) {
    c ^= (uint16_t)((uint16_t)data[i] << 8);
    for (uint8_t b = 0; b < 8u; b++) {
      c = (c & 0x8000u) ? (uint16_t)((c << 1) ^ 0x1021u) : (uint16_t)(c << 1);
    }
  }
  return c;
}

uint16_t RmFrame_Encode(uint8_t *out, uint16_t cap, uint16_t cmd_id, uint8_t seq,
                        const uint8_t *data, uint16_t len) {
  if (out == NULL || cap < (uint16_t)(9u + len) || len > RM_DATA_MAX ||
      (len > 0u && data == NULL)) {
    return 0u;
  }
  out[0] = RM_SOF;
  out[1] = (uint8_t)(len & 0xFFu);
  out[2] = (uint8_t)(len >> 8);
  out[3] = seq;
  out[4] = RmCrc8(out, 4u);
  out[5] = (uint8_t)(cmd_id & 0xFFu);
  out[6] = (uint8_t)(cmd_id >> 8);
  if (len > 0u) {
    memcpy(&out[7], data, len);
  }
  uint16_t crc = RmCrc16(out, (uint32_t)(7u + len));
  out[7u + len] = (uint8_t)(crc & 0xFFu);
  out[8u + len] = (uint8_t)(crc >> 8);
  return (uint16_t)(9u + len);
}

void RmParser_Reset(RmParser *p) {
  if (p != NULL) {
    p->n = 0u;
  }
}

// 丢第一个字节再重试：帧头可能出现在任何位置，长度/CRC 错时只能滑一个字节重新对齐。
static void resync(RmParser *p) {
  if (p->n > 1u) {
    memmove(p->raw, &p->raw[1], (size_t)(p->n - 1u));
    p->n--;
  } else {
    p->n = 0u;
  }
}

bool RmParser_Feed(RmParser *p, uint8_t byte, RmFrame *out) {
  if (p == NULL || out == NULL) {
    return false;
  }
  if (p->n == 0u && byte != RM_SOF) {
    return false;
  }
  if (p->n >= sizeof(p->raw)) {
    RmParser_Reset(p);
    return false;
  }
  p->raw[p->n++] = byte;

  for (;;) {
    if (p->n < 5u) {
      return false;
    }
    if (p->raw[0] != RM_SOF) {
      resync(p);
      if (p->n == 0u) {
        return false;
      }
      continue;
    }
    if (RmCrc8(p->raw, 4u) != p->raw[4]) {
      resync(p);
      continue;
    }
    uint16_t len = (uint16_t)(p->raw[1] | ((uint16_t)p->raw[2] << 8));
    if (len > RM_DATA_MAX) {
      resync(p);
      continue;
    }
    uint16_t total = (uint16_t)(9u + len);
    if (p->n < total) {
      return false; /* 还没收全 */
    }
    uint16_t crc = (uint16_t)(p->raw[7u + len] | ((uint16_t)p->raw[8u + len] << 8));
    if (RmCrc16(p->raw, (uint32_t)(7u + len)) != crc) {
      resync(p);
      continue;
    }
    out->cmd_id = (uint16_t)(p->raw[5] | ((uint16_t)p->raw[6] << 8));
    out->seq = p->raw[3];
    out->len = len;
    if (len > 0u) {
      memcpy(out->data, &p->raw[7], len);
    }
    RmParser_Reset(p);
    return true;
  }
}
