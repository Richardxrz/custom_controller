/*
 * 6 轴调度（bus_servo_group）主机侧测试。
 * 用一个会「自动应答」的假总线：看到 PRAD 就回位置，看到 ULK 就回 #OK!，
 * 所以能测轮询顺序、周期、单轴超时跳过，而不需要真舵机。
 */
#include "bus_servo_group.h"
#include "ringbuf.h"
#include "test_util.h"

typedef struct {
  uint8_t tx[512];
  uint16_t tx_len;
  RingBuf rx;
  uint8_t store[256];
  uint32_t now;
} Fake;

static void fake_write(void *ctx, const uint8_t *d, uint16_t n) {
  Fake *f = (Fake *)ctx;
  if ((uint16_t)(f->tx_len + n) > (uint16_t)sizeof(f->tx)) {
    n = (uint16_t)(sizeof(f->tx) - f->tx_len);
  }
  memcpy(&f->tx[f->tx_len], d, n);
  f->tx_len = (uint16_t)(f->tx_len + n);
}

static uint16_t fake_read(void *ctx, uint8_t *out, uint16_t cap) {
  Fake *f = (Fake *)ctx;
  return RingBuf_GetBlock(&f->rx, out, cap);
}

static uint32_t fake_now(void *ctx) { return ((Fake *)ctx)->now; }

static void fake_init(Fake *f) {
  memset(f, 0, sizeof(*f));
  RingBuf_Init(&f->rx, f->store, (uint16_t)sizeof(f->store));
}

static BusServoPort fake_port(Fake *f) {
  BusServoPort p = {fake_write, fake_read, fake_now, f};
  return p;
}

/* 扫描新发出的命令并自动应答；silent_id 不回（用来制造超时） */
static void auto_respond(Fake *f, uint16_t *seen, const uint16_t *pos, int silent_id) {
  while (*seen < f->tx_len) {
    uint16_t i = *seen;
    while (i < f->tx_len && f->tx[i] != '#') {
      i++;
    }
    if (i >= f->tx_len) {
      *seen = f->tx_len;
      break;
    }
    uint16_t end = i;
    while (end < f->tx_len && f->tx[end] != '!') {
      end++;
    }
    if (end >= f->tx_len) {
      *seen = i; /* 命令还没写全 */
      break;
    }
    int id = (f->tx[i + 1] - '0') * 100 + (f->tx[i + 2] - '0') * 10 + (f->tx[i + 3] - '0');
    const char *cmd = (const char *)&f->tx[i + 5];
    char resp[16];
    if (id != silent_id) {
      if (strncmp(cmd, "RAD", 3) == 0) {
        snprintf(resp, sizeof(resp), "#%03dP%04d!", id, pos[id]);
        RingBuf_PutBlock(&f->rx, (const uint8_t *)resp, (uint16_t)strlen(resp));
      } else if (strncmp(cmd, "ULK", 3) == 0) {
        RingBuf_PutBlock(&f->rx, (const uint8_t *)"#OK!", 4);
      }
    }
    *seen = (uint16_t)(end + 1);
  }
  if (*seen >= f->tx_len) { /* 清掉已消费的命令，避免缓冲越写越满 */
    f->tx_len = 0;
    *seen = 0;
  }
}

static void spin(BusServoGroup *g, Fake *f, uint16_t *seen, const uint16_t *pos, int silent,
                 int steps) {
  for (int i = 0; i < steps; i++) {
    BusServoGroup_Poll(g, f->now);
    auto_respond(f, seen, pos, silent);
    f->now += 1;
  }
}

static void case_sweep(void) {
  printf("[6 轴轮询]\n");
  Fake f;
  fake_init(&f);
  uint8_t ids[6] = {1, 2, 3, 4, 5, 6};
  uint16_t pos[256] = {0};
  for (int i = 1; i <= 6; i++) {
    pos[i] = (uint16_t)(1400 + i * 10);
  }

  BusServoGroup g;
  BusServoGroup_Init(&g, fake_port(&f), ids, 6, 5);
  BusServoGroup_StartSweep(&g, 0);
  uint16_t seen = 0;
  spin(&g, &f, &seen, pos, -1, 200);

  t_check(BusServoGroup_GetRounds(&g) >= 1, "至少完成一轮");
  bool all = true;
  for (uint8_t i = 0; i < 6; i++) {
    uint16_t v = 0;
    if (!BusServoGroup_GetPos(&g, i, &v) || v != pos[i + 1]) {
      all = false;
    }
  }
  t_check(all, "6 轴位置都正确");
}

static void case_timeout(void) {
  printf("[单轴超时跳过]\n");
  Fake f;
  fake_init(&f);
  uint8_t ids[3] = {1, 2, 3};
  uint16_t pos[256] = {0};
  pos[1] = 1500;
  pos[2] = 1600;
  pos[3] = 1700;

  BusServoGroup g;
  BusServoGroup_Init(&g, fake_port(&f), ids, 3, 5);
  BusServoGroup_StartSweep(&g, 0);
  uint16_t seen = 0;
  spin(&g, &f, &seen, pos, 2, 300); /* 2 号不应答 */

  t_check(BusServoGroup_GetTimeouts(&g) >= 1, "2 号超时被计数");
  t_check(BusServoGroup_GetRounds(&g) >= 1, "超时不影响整轮完成");
  uint16_t v = 0;
  t_check(BusServoGroup_GetPos(&g, 0, &v) && v == 1500, "1 号仍然读到了");
  t_check(!BusServoGroup_GetPos(&g, 1, &v), "2 号没有有效值");
  t_check(BusServoGroup_GetPos(&g, 2, &v) && v == 1700, "3 号仍然读到了");
}

static void case_period(void) {
  printf("[周期模式]\n");
  Fake f;
  fake_init(&f);
  uint8_t ids[3] = {1, 2, 3};
  uint16_t pos[256] = {0};
  pos[1] = 1500;
  pos[2] = 1500;
  pos[3] = 1500;

  BusServoGroup g;
  BusServoGroup_Init(&g, fake_port(&f), ids, 3, 5);
  BusServoGroup_SetPeriod(&g, 20);
  BusServoGroup_StartSweep(&g, 0);
  uint16_t seen = 0;
  spin(&g, &f, &seen, pos, -1, 200); /* 200 ms */

  uint32_t rounds = BusServoGroup_GetRounds(&g);
  t_check(rounds >= 8 && rounds <= 11, "200 ms / 20 ms 周期约为 10 轮");
}

static void case_release(void) {
  printf("[逐个释力]\n");
  Fake f;
  fake_init(&f);
  uint8_t ids[3] = {1, 2, 3};
  uint16_t pos[256] = {0};

  BusServoGroup g;
  BusServoGroup_Init(&g, fake_port(&f), ids, 3, 5);
  BusServoGroup_ReleaseAll(&g, 0);
  uint16_t seen = 0;
  spin(&g, &f, &seen, pos, -1, 100);
  t_check(BusServoGroup_GetMode(&g) == GROUP_DONE, "全部释力后 mode = DONE");
  t_check(BusServoGroup_GetTimeouts(&g) == 0, "释力过程没有超时");
}

int main(void) {
  case_sweep();
  case_timeout();
  case_period();
  case_release();
  return t_summary();
}
