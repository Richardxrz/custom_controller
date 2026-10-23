#include "servo_codec.h"

#include <string.h>

enum {
  PS_HASH = 0, /* 等 '#' */
  PS_ID,       /* 等 3 位 ID 或 'O'(#OK!) */
  PS_BODY,     /* 等 body，直到 '!' */
  PS_OK_K,     /* 已收到 'O'，等 'K' */
  PS_OK_BANG,  /* 已收到 'OK'，等 '!' */
};

static const char *const kQuery[SERVO_Q_COUNT] = {
    "PRAD", "PID", "PMOD", "PVER", "PULK", "PULR", "PDPT", "PDCT", "PDST", "PRTV",
    "PCSM", "PCSR",
};

/* 手工写数字而不是 snprintf：裸机上 snprintf 会拖进几 KB 的格式化库，
 * 而这里只有 3/4 位定宽零填充，八行就够。 */
static uint16_t put_digits(char *out, uint16_t pos, uint16_t value, uint8_t width) {
  for (uint8_t i = width; i > 0; i--) {
    out[pos + i - 1u] = (char)('0' + (value % 10u));
    value /= 10u;
  }
  return (uint16_t)(pos + width);
}

uint16_t ServoCodec_BuildMove(char *out, uint16_t cap, uint8_t id, uint16_t pwm,
                              uint16_t t_ms) {
  if (out == NULL || cap < 15u) {
    return 0u;
  }
  out[0] = '#';
  uint16_t pos = put_digits(out, 1u, id, 3);
  out[pos++] = 'P';
  pos = put_digits(out, pos, pwm, 4);
  out[pos++] = 'T';
  pos = put_digits(out, pos, t_ms, 4);
  out[pos++] = '!';
  return pos;
}

uint16_t ServoCodec_BuildQuery(char *out, uint16_t cap, uint8_t id, ServoCodecQuery q) {
  if (out == NULL || q >= SERVO_Q_COUNT) {
    return 0u;
  }
  uint8_t qlen = (uint8_t)strlen(kQuery[q]);
  uint16_t need = (uint16_t)(1u + 3u + qlen + 1u);
  if (cap < need) {
    return 0u;
  }
  out[0] = '#';
  uint16_t pos = put_digits(out, 1u, id, 3);
  memcpy(&out[pos], kQuery[q], qlen);
  pos = (uint16_t)(pos + qlen);
  out[pos++] = '!';
  return pos;
}

uint16_t ServoCodec_BuildSetId(char *out, uint16_t cap, uint8_t old_id, uint8_t new_id) {
  if (out == NULL || cap < 11u) {
    return 0u;
  }
  out[0] = '#';
  uint16_t pos = put_digits(out, 1u, old_id, 3);
  out[pos++] = 'P';
  out[pos++] = 'I';
  out[pos++] = 'D';
  pos = put_digits(out, pos, new_id, 3);
  out[pos++] = '!';
  return pos;
}

void ServoCodec_Reset(ServoCodecParser *p) {
  if (p == NULL) {
    return;
  }
  p->state = PS_HASH;
  p->id = 0u;
  p->digits = 0u;
  p->len = 0u;
  p->overflow = false;
  p->body[0] = '\0';
}

static bool body_is_digits(const ServoCodecParser *p, uint8_t from) {
  if (p->len <= from) {
    return false;
  }
  for (uint8_t i = from; i < p->len; i++) {
    if (p->body[i] < '0' || p->body[i] > '9') {
      return false;
    }
  }
  return true;
}

/* 只按「形状」分类：
 *   "P"          -> ID 应答
 *   "P1234"      -> 数值
 *   "PMOD1"      -> 模式
 *   "PV0.97"     -> 版本
 *   "PRAD"/"PID" -> 命令回显（我们自己发出去的）
 *   "T25V07"     -> 温度/电压
 * 这里故意不判断「这是不是我在等的那条」——那是指令层的事。 */
static ServoFrameKind classify(const ServoCodecParser *p) {
  if (p->len == 1u && p->body[0] == 'P') {
    return SERVO_FRAME_ID;
  }
  if (p->len >= 1u && p->body[0] == 'P') {
    if (body_is_digits(p, 1u)) {
      return SERVO_FRAME_VALUE;
    }
    if (p->len >= 4u && memcmp(p->body, "PMOD", 4u) == 0) {
      return SERVO_FRAME_MOD;
    }
    if (p->len >= 2u && p->body[1] == 'V') {
      return SERVO_FRAME_VER;
    }
    return SERVO_FRAME_CMD;
  }
  if (p->len >= 1u && p->body[0] == 'T') {
    return SERVO_FRAME_TV;
  }
  return SERVO_FRAME_CMD;
}

static bool emit(ServoCodecParser *p, ServoCodecFrame *out, ServoFrameKind kind, uint8_t id) {
  out->id = id;
  out->kind = kind;
  out->len = p->len;
  memcpy(out->body, p->body, p->len);
  out->body[p->len] = '\0';
  ServoCodec_Reset(p);
  return true;
}

bool ServoCodec_Feed(ServoCodecParser *p, uint8_t byte, ServoCodecFrame *out) {
  if (p == NULL || out == NULL) {
    return false;
  }

  switch (p->state) {
  case PS_HASH:
    if (byte == '#') {
      ServoCodec_Reset(p);
      p->state = PS_ID;
    }
    return false;

  case PS_ID:
    if (byte >= '0' && byte <= '9') {
      if (p->digits < 3u) {
        p->id = (uint8_t)(p->id * 10u + (uint8_t)(byte - '0'));
        p->digits++;
        if (p->digits == 3u) {
          p->state = PS_BODY;
        }
      }
      return false;
    }
    if (p->digits == 0u && byte == 'O') { /* #OK! 的 'O' */
      p->state = PS_OK_K;
      return false;
    }
    if (byte == '#') { /* 帧头中途又来一个帧头：以新的为准 */
      ServoCodec_Reset(p);
      p->state = PS_ID;
      return false;
    }
    ServoCodec_Reset(p);
    return false;

  case PS_BODY:
    if (byte == '#') { /* 上一帧没等到 '!'，来了新帧头：丢弃残帧 */
      ServoCodec_Reset(p);
      p->state = PS_ID;
      return false;
    }
    if (byte == '!') {
      if (p->overflow) {
        ServoCodec_Reset(p);
        return false;
      }
      return emit(p, out, classify(p), p->id);
    }
    if (p->len < SERVO_CODEC_BODY_MAX) {
      p->body[p->len++] = (char)byte;
      p->body[p->len] = '\0';
    } else {
      p->overflow = true; /* 继续吞到 '!' 再复位，避免把 '!' 前的残渣当新帧 */
    }
    return false;

  case PS_OK_K:
    if (byte == 'K') {
      p->state = PS_OK_BANG;
    } else if (byte == '#') {
      ServoCodec_Reset(p);
      p->state = PS_ID;
    } else {
      ServoCodec_Reset(p);
    }
    return false;

  case PS_OK_BANG:
    if (byte == '!') {
      return emit(p, out, SERVO_FRAME_OK, 0u);
    }
    if (byte == '#') {
      ServoCodec_Reset(p);
      p->state = PS_ID;
    } else {
      ServoCodec_Reset(p);
    }
    return false;

  default:
    ServoCodec_Reset(p);
    return false;
  }
}

uint16_t ServoCodec_FrameValue(const ServoCodecFrame *f) {
  if (f == NULL || f->kind != SERVO_FRAME_VALUE || f->len < 2u) {
    return 0xFFFFu;
  }
  uint16_t v = 0u;
  for (uint8_t i = 1u; i < f->len; i++) {
    char c = f->body[i];
    if (c < '0' || c > '9') {
      return 0xFFFFu;
    }
    v = (uint16_t)(v * 10u + (uint16_t)(c - '0'));
  }
  return v;
}
