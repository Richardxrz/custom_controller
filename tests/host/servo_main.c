/*
 * 总线舵机模块（ringbuf + servo_codec + bus_servo）主机侧测试台。
 *
 * 不需要板子：驱动层用一个假 Port（内存环形缓冲 + 假时钟）替身，
 * 所以能测「回显混在应答前面」「超时不靠 sleep」「半包逐字节喂」这些真机很难复现的场景。
 *
 *   ./tests/host/run_servo.sh
 *
 * 输出约定：✅ 判据满足；❌ 判据不满足（同行给出期望/实际）。
 * 本文件只表达判据，不含"哪里写错了""该怎么改"。
 */
#include <stdio.h>
#include <string.h>

#include "bus_servo.h"
#include "ringbuf.h"
#include "servo_codec.h"

static int g_fail = 0;

static void check(bool cond, const char *what) {
  if (cond) {
    printf("  ✅ %s\n", what);
  } else {
    printf("  ❌ %s\n", what);
    g_fail++;
  }
}

static void check_str(const char *got, const char *want, const char *what) {
  if (got != NULL && strcmp(got, want) == 0) {
    printf("  ✅ %s（%s）\n", what, got);
  } else {
    printf("  ❌ %s：期望 \"%s\"，实际 \"%s\"\n", what, want, got == NULL ? "(null)" : got);
    g_fail++;
  }
}

/* ---------------------------------------------------------------- 假驱动 */

typedef struct {
  uint8_t tx[128];
  uint16_t tx_len;
  RingBuf rx;
  uint8_t rx_store[128];
  uint32_t now;
} Fake;

static void fake_write(void *ctx, const uint8_t *data, uint16_t len) {
  Fake *f = (Fake *)ctx;
  if ((uint16_t)(f->tx_len + len) > (uint16_t)sizeof(f->tx)) {
    len = (uint16_t)(sizeof(f->tx) - f->tx_len);
  }
  memcpy(&f->tx[f->tx_len], data, len);
  f->tx_len = (uint16_t)(f->tx_len + len);
}

static uint16_t fake_read(void *ctx, uint8_t *buf, uint16_t cap) {
  Fake *f = (Fake *)ctx;
  return RingBuf_GetBlock(&f->rx, buf, cap);
}

static uint32_t fake_now(void *ctx) { return ((Fake *)ctx)->now; }

static void fake_init(Fake *f) {
  memset(f, 0, sizeof(*f));
  RingBuf_Init(&f->rx, f->rx_store, (uint16_t)sizeof(f->rx_store));
}

static void fake_inject(Fake *f, const char *s) {
  RingBuf_PutBlock(&f->rx, (const uint8_t *)s, (uint16_t)strlen(s));
}

static BusServoPort fake_port(Fake *f) {
  BusServoPort p = {fake_write, fake_read, fake_now, f};
  return p;
}

/* 逐字节喂，返回是否至少切出一帧；整段里的帧数写到 frames */
static bool parse_str(const char *s, ServoCodecFrame *f, int *frames) {
  ServoCodecParser p;
  ServoCodec_Reset(&p);
  bool got = false;
  int n = 0;
  for (const char *q = s; *q != '\0'; q++) {
    if (ServoCodec_Feed(&p, (uint8_t)*q, f)) {
      got = true;
      n++;
    }
  }
  if (frames != NULL) {
    *frames = n;
  }
  return got;
}

/* ---------------------------------------------------------------- 用例 */

static void case_ringbuf(void) {
  printf("[ringbuf]\n");
  uint8_t store[5];
  RingBuf rb;
  RingBuf_Init(&rb, store, sizeof(store));
  check(RingBuf_IsEmpty(&rb), "初始为空");

  check(RingBuf_Put(&rb, 1) && RingBuf_Put(&rb, 2) && RingBuf_Put(&rb, 3) &&
            RingBuf_Put(&rb, 4),
        "容量 5 的环能放 4 个字节");
  check(!RingBuf_Put(&rb, 5), "满了之后 Put 返回 false（不覆盖旧数据）");
  check(RingBuf_Count(&rb) == 4, "Count == 4");

  uint8_t b = 0;
  check(RingBuf_Get(&rb, &b) && b == 1, "先进先出：第一个是 1");
  RingBuf_Flush(&rb);
  check(RingBuf_IsEmpty(&rb), "Flush 后为空");
}

static void case_build(void) {
  printf("[servo_codec 生成]\n");
  char cmd[32];

  uint16_t n = ServoCodec_BuildMove(cmd, sizeof(cmd), 3, 1500, 1000);
  cmd[n] = '\0';
  check(n == 15, "BuildMove 长度 15");
  check_str(cmd, "#003P1500T1000!", "BuildMove 内容");

  n = ServoCodec_BuildQuery(cmd, sizeof(cmd), 3, SERVO_Q_RAD);
  cmd[n] = '\0';
  check_str(cmd, "#003PRAD!", "BuildQuery(RAD)");

  n = ServoCodec_BuildQuery(cmd, sizeof(cmd), 12, SERVO_Q_ULK);
  cmd[n] = '\0';
  check_str(cmd, "#012PULK!", "BuildQuery(ULK) 两位数 ID 补零");

  n = ServoCodec_BuildSetId(cmd, sizeof(cmd), 0, 1);
  cmd[n] = '\0';
  check_str(cmd, "#000PID001!", "BuildSetId");

  check(ServoCodec_BuildMove(cmd, 8, 1, 1500, 1000) == 0, "缓冲不够时返回 0");
}

static void case_parse(void) {
  printf("[servo_codec 解析]\n");
  ServoCodecFrame f;

  {
    int frames = 0;
    bool got = parse_str("#003P1500!", &f, &frames);
    check(got && frames == 1 && f.kind == SERVO_FRAME_VALUE && f.id == 3, "数值应答 kind/id");
    check_str(f.body, "P1500", "数值应答 body");
    check(ServoCodec_FrameValue(&f) == 1500, "FrameValue == 1500");
  }
  {
    int frames = 0;
    bool got = parse_str("#003P!", &f, &frames);
    check(got && frames == 1 && f.kind == SERVO_FRAME_ID && f.id == 3, "ID 应答");
  }
  {
    int frames = 0;
    bool got = parse_str("#OK!", &f, &frames);
    check(got && frames == 1 && f.kind == SERVO_FRAME_OK, "#OK! 应答");
  }
  {
    int frames = 0;
    bool got = parse_str("#003PMOD1!", &f, &frames);
    check(got && frames == 1 && f.kind == SERVO_FRAME_MOD, "模式应答");
    check_str(f.body, "PMOD1", "模式应答 body");
  }
  {
    int frames = 0;
    bool got = parse_str("#003PV0.97!", &f, &frames);
    check(got && frames == 1 && f.kind == SERVO_FRAME_VER, "版本应答");
  }
  {
    int frames = 0;
    bool got = parse_str("#000T25V07!", &f, &frames);
    check(got && frames == 1 && f.kind == SERVO_FRAME_TV, "温度/电压应答（没有 P）");
  }
  {
    int frames = 0;
    bool got = parse_str("#003PRAD!", &f, &frames);
    check(got && frames == 1 && f.kind == SERVO_FRAME_CMD, "命令回显被分类为 CMD");
  }
  {
    int frames = 0;
    bool got = parse_str("noise#003P1500!", &f, &frames);
    check(got && frames == 1 && f.kind == SERVO_FRAME_VALUE, "前面的噪声字节被忽略");
  }
  {
    /* 半包：先喂一半，不出帧；补完后出帧 */
    ServoCodecParser p;
    ServoCodec_Reset(&p);
    bool got = false;
    const char *a = "#003P";
    for (const char *q = a; *q; q++) {
      got = ServoCodec_Feed(&p, (uint8_t)*q, &f) || got;
    }
    check(!got, "半包不出帧");
    const char *b = "1500!";
    for (const char *q = b; *q; q++) {
      got = ServoCodec_Feed(&p, (uint8_t)*q, &f) || got;
    }
    check(got && f.kind == SERVO_FRAME_VALUE && ServoCodec_FrameValue(&f) == 1500,
          "半包接上后出帧");
  }
  {
    /* 超长 body：整帧丢弃，且不会把后面的正常帧吃掉 */
    char junk[64];
    int pos = 0;
    pos += snprintf(junk + pos, sizeof(junk) - (size_t)pos, "#003P");
    for (int i = 0; i < 30; i++) {
      junk[pos++] = 'A';
    }
    pos += snprintf(junk + pos, sizeof(junk) - (size_t)pos, "!");
    strcat(junk, "#003P1500!");
    int frames = 0;
    bool got = parse_str(junk, &f, &frames);
    check(got && frames == 1 && f.kind == SERVO_FRAME_VALUE, "超长帧被丢弃，后续正常帧仍能解析");
  }
}

static void case_bus(void) {
  printf("[bus_servo]\n");
  Fake f;
  fake_init(&f);
  BusServo s;
  BusServo_Init(&s, fake_port(&f), 3);

  check(BusServo_RequestAngle(&s), "发起读角度成功");
  check(f.tx_len == 9 && memcmp(f.tx, "#003PRAD!", 9) == 0, "总线上发出 #003PRAD!");
  check(BusServo_GetState(&s) == BUS_SERVO_WAIT, "状态 = WAIT");
  check(!BusServo_RequestAngle(&s), "WAIT 期间再发请求被拒");

  uint16_t pwm = 0;
  fake_inject(&f, "#003P1500!");
  BusServo_Poll(&s);
  check(BusServo_GetState(&s) == BUS_SERVO_DONE, "收到应答 → DONE");
  check(BusServo_GetAngle(&s, &pwm) && pwm == 1500, "角度 = 1500");

  /* 回显 + 真应答混在一起 */
  Fake f2;
  fake_init(&f2);
  BusServo s2;
  BusServo_Init(&s2, fake_port(&f2), 3);
  BusServo_RequestAngle(&s2);
  fake_inject(&f2, "#003PRAD!#003P1750!");
  BusServo_Poll(&s2);
  check(BusServo_GetState(&s2) == BUS_SERVO_DONE && BusServo_GetAngle(&s2, &pwm) &&
            pwm == 1750,
        "回显在前、真应答在后：取到 1750");

  /* 超时：不 sleep，直接把假时钟往前拨 */
  Fake f3;
  fake_init(&f3);
  BusServo s3;
  BusServo_Init(&s3, fake_port(&f3), 5);
  BusServo_RequestAngle(&s3);
  f3.now = 25;
  BusServo_Poll(&s3);
  check(BusServo_GetState(&s3) == BUS_SERVO_TIMEOUT, "25 ms 没应答 → TIMEOUT");
  check(BusServo_GetErrCnt(&s3) == 1, "超时计数 = 1");

  /* 释力：等 #OK! */
  Fake f4;
  fake_init(&f4);
  BusServo s4;
  BusServo_Init(&s4, fake_port(&f4), 2);
  check(BusServo_Release(&s4), "发起释力成功");
  check(f4.tx_len == 9 && memcmp(f4.tx, "#002PULK!", 9) == 0, "总线上发出 #002PULK!");
  fake_inject(&f4, "#OK!");
  BusServo_Poll(&s4);
  check(BusServo_GetState(&s4) == BUS_SERVO_DONE, "收到 #OK! → DONE");

  /* 控制命令：不应答，发完即回 IDLE */
  Fake f5;
  fake_init(&f5);
  BusServo s5;
  BusServo_Init(&s5, fake_port(&f5), 1);
  check(BusServo_SetAngle(&s5, 1500, 1000), "SetAngle 成功");
  check(f5.tx_len == 15 && memcmp(f5.tx, "#001P1500T1000!", 15) == 0,
        "总线上发出 #001P1500T1000!");
  check(BusServo_GetState(&s5) == BUS_SERVO_IDLE, "控制命令不等应答，回 IDLE");
}

int main(void) {
  case_ringbuf();
  case_build();
  case_parse();
  case_bus();

  printf("\n=== %s（失败 %d 条）===\n", g_fail == 0 ? "全过" : "有失败", g_fail);
  return g_fail == 0 ? 0 : 1;
}
