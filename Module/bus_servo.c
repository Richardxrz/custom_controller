#include "bus_servo.h"

#include <string.h>

#define BUS_SERVO_DEFAULT_TIMEOUT_MS 20u
#define BUS_SERVO_DRAIN_MAX 32u

/* 发请求前把接收侧残留清掉：
 * 上一次超时的迟到应答、回显、上电噪声，都会让本次应答对不上号。
 * 这里不依赖 Port 提供 flush —— 只要求 read 能取空。 */
static void drain_rx(BusServo *s) {
  uint8_t tmp[BUS_SERVO_DRAIN_MAX];
  while (s->port.read != NULL && s->port.read(s->port.ctx, tmp, sizeof(tmp)) > 0u) {
    /* 丢弃 */
  }
}

static bool start_transaction(BusServo *s, BusServoOp op, const char *cmd, uint16_t len) {
  if (s == NULL || s->port.write == NULL || cmd == NULL || len == 0u) {
    return false;
  }
  if (s->state == BUS_SERVO_WAIT) {
    return false; /* 单总线：上一笔还没结束，不能插队 */
  }
  drain_rx(s);
  ServoCodec_Reset(&s->parser);
  s->op = (uint8_t)op;
  s->sent_ms = (s->port.now_ms != NULL) ? s->port.now_ms(s->port.ctx) : 0u;
  s->state = BUS_SERVO_WAIT;
  s->port.write(s->port.ctx, (const uint8_t *)cmd, len);
  return true;
}

void BusServo_Init(BusServo *s, BusServoPort port, uint8_t id) {
  if (s == NULL) {
    return;
  }
  memset(s, 0, sizeof(*s));
  s->port = port;
  s->id = id;
  s->timeout_ms = BUS_SERVO_DEFAULT_TIMEOUT_MS;
  s->state = BUS_SERVO_IDLE;
  s->op = BUS_SERVO_OP_NONE;
  ServoCodec_Reset(&s->parser);
}

void BusServo_SetTimeout(BusServo *s, uint16_t ms) {
  if (s != NULL && ms > 0u) {
    s->timeout_ms = ms;
  }
}

bool BusServo_RequestAngle(BusServo *s) {
  if (s == NULL) {
    return false;
  }
  char cmd[16];
  uint16_t n = ServoCodec_BuildQuery(cmd, sizeof(cmd), s->id, SERVO_Q_RAD);
  return start_transaction(s, BUS_SERVO_OP_READ, cmd, n);
}

bool BusServo_SetAngle(BusServo *s, uint16_t pwm, uint16_t t_ms) {
  if (s == NULL || pwm < 500u || pwm > 2500u || t_ms > 9999u) {
    return false;
  }
  if (s->state == BUS_SERVO_WAIT) {
    return false;
  }
  char cmd[16];
  uint16_t n = ServoCodec_BuildMove(cmd, sizeof(cmd), s->id, pwm, t_ms);
  if (n == 0u || s->port.write == NULL) {
    return false;
  }
  /* 控制命令舵机不回话：发完就当这一笔结束了，不让状态机白等 20 ms */
  s->port.write(s->port.ctx, (const uint8_t *)cmd, n);
  s->op = BUS_SERVO_OP_NONE;
  s->state = BUS_SERVO_IDLE;
  return true;
}

bool BusServo_Release(BusServo *s) {
  if (s == NULL) {
    return false;
  }
  char cmd[16];
  uint16_t n = ServoCodec_BuildQuery(cmd, sizeof(cmd), s->id, SERVO_Q_ULK);
  return start_transaction(s, BUS_SERVO_OP_RELEASE, cmd, n);
}

void BusServo_Poll(BusServo *s) {
  if (s == NULL || s->port.read == NULL) {
    return;
  }

  uint8_t buf[BUS_SERVO_DRAIN_MAX];
  uint16_t n = s->port.read(s->port.ctx, buf, sizeof(buf));
  for (uint16_t i = 0u; i < n; i++) {
    ServoCodecFrame f;
    if (!ServoCodec_Feed(&s->parser, buf[i], &f)) {
      continue;
    }
    /* 回显：把「自己刚发出去的命令」环回收到了。舵机的真应答不会是命令字。 */
    if (f.kind == SERVO_FRAME_CMD) {
      continue;
    }
    if (s->state != BUS_SERVO_WAIT) {
      continue; /* 没在等应答，迟到的帧只当噪声 */
    }
    if (s->op == BUS_SERVO_OP_READ && f.kind == SERVO_FRAME_VALUE && f.id == s->id) {
      s->value = ServoCodec_FrameValue(&f);
      if (s->value != 0xFFFFu) {
        s->state = BUS_SERVO_DONE;
      }
    } else if (s->op == BUS_SERVO_OP_RELEASE && f.kind == SERVO_FRAME_OK) {
      s->state = BUS_SERVO_DONE;
    }
    /* 其它帧（别的 ID、别的类型）一律忽略：总线上可能还有别的舵机在说话 */
  }

  if (s->state == BUS_SERVO_WAIT && s->port.now_ms != NULL) {
    uint32_t now = s->port.now_ms(s->port.ctx);
    /* 用无符号差值算超时：计数器回绕时仍然正确（now - sent 是模 2^32 的差） */
    if ((uint32_t)(now - s->sent_ms) >= s->timeout_ms) {
      s->state = BUS_SERVO_TIMEOUT;
      s->err_cnt++;
    }
  }
}

bool BusServo_GetAngle(const BusServo *s, uint16_t *pwm) {
  if (s == NULL || pwm == NULL) {
    return false;
  }
  if (s->state != BUS_SERVO_DONE || s->op != BUS_SERVO_OP_READ) {
    return false;
  }
  *pwm = s->value;
  return true;
}

BusServoState BusServo_GetState(const BusServo *s) {
  return (s == NULL) ? BUS_SERVO_IDLE : s->state;
}

uint16_t BusServo_GetErrCnt(const BusServo *s) { return (s == NULL) ? 0u : s->err_cnt; }
