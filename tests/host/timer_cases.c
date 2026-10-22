/*
 * Timer（DWT 微秒计时器）主机侧行为判据。
 *
 *   ./tests/host/run_timer.sh            跑全部场景
 *   ./tests/host/run_timer.sh t_wrap     只跑一个（每个场景全新进程）
 *
 * 关键点：DWT 被替身接到了普通变量上，所以测试能直接"把时间拨到任意位置"，
 * 于是换算、回绕、统计这些都能**定量**判，而不是靠肉眼估。
 *
 * 本文件只表达**判据**（应该成立什么），不含"哪里写错了""该怎么改"。
 */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "stm32f4xx.h"
#include "timer.h"

/* ---- 被测代码依赖的量 ---- */
DWT_Type stub_dwt;
CoreDebug_Type stub_coredebug;
uint32_t SystemCoreClock = 168000000; /* 板上主频就是 168 MHz */

/* ---- 断言：只报判据是否满足 ---- */
static int g_fail;

static void ck_int(const char *rule, long got, long want) {
  int ok = (got == want);
  if (!ok) g_fail++;
  printf("  %s %s → 期望 %ld，实际 %ld\n", ok ? "✅" : "❌", rule, want, got);
}

static void ck_near(const char *rule, float got, float want, float tol) {
  float d = got - want;
  if (d < 0) d = -d;
  int ok = (d <= tol);
  if (!ok) g_fail++;
  printf("  %s %s → 期望 %.6f（允差 %.6f），实际 %.6f\n", ok ? "✅" : "❌", rule, want, tol, got);
}

static void ck_true(const char *rule, int cond) {
  if (!cond) g_fail++;
  printf("  %s %s\n", cond ? "✅" : "❌", rule);
}

static void begin(const char *name, const char *what) {
  printf("\n[%s] %s\n", name, what);
  /* 每个场景从干净状态开始：清替身寄存器 + 复位统计 */
  stub_dwt.CTRL = 0;
  stub_dwt.CYCCNT = 0;
  stub_coredebug.DEMCR = 0;
  Timer_Init();
  Timer_Reset();
}

/* 1 个 tick = 1/168 µs（168 MHz）；168 tick = 1 µs */
#define US(ticks) ((float)(ticks) / 168.0f)

/* ============================ 场景 ============================ */

/* 初始化：幂等 + 使能位真的被置上 */
static void t_init(void) {
  printf("\n[t_init] 初始化：两次调用都要成功，且 CYCCNTENA / TRCENA 被置上\n");
  stub_dwt.CTRL = 0;
  stub_coredebug.DEMCR = 0;
  int ok1 = Timer_Init();
  ck_int("第一次 Timer_Init 返回 true", ok1 ? 1 : 0, 1);
  ck_int("DWT->CTRL 的 CYCCNTENA 位被置上", (stub_dwt.CTRL & DWT_CTRL_CYCCNTENA_Msk) ? 1 : 0, 1);
  ck_int("CoreDebug->DEMCR 的 TRCENA 位被置上",
         (stub_coredebug.DEMCR & CoreDebug_DEMCR_TRCENA_Msk) ? 1 : 0, 1);
  int ok2 = Timer_Init();
  ck_int("第二次 Timer_Init 仍然返回 true（幂等）", ok2 ? 1 : 0, 1);
}

/* 换算：给定 tick 差，四个单位都要对得上 */
static void t_units(void) {
  begin("t_units", "换算：1680 tick（=10 µs @168 MHz）");
  stub_dwt.CYCCNT = 0;
  Timer_Start();
  stub_dwt.CYCCNT = 1680;
  Timer_Stop();
  ck_int("Timer_Delta_tick = 1680", (long)Timer_Delta_tick(), 1680);
  ck_near("Timer_Delta_us = 10 µs", Timer_Delta_us(), 10.0f, 0.001f);
  ck_near("Timer_Delta_ms = 0.01 ms", Timer_Delta_ms(), 0.01f, 0.0001f);
  ck_near("Timer_Delta_s = 0.00001 s", Timer_Delta_s(), 0.00001f, 1e-7f);
}

/* 回绕：CYCCNT 是 32 位，从接近满值走到小值，差值必须还是"走过的 tick 数" */
static void t_wrap(void) {
  begin("t_wrap", "回绕：0xFFFFFFF0 → 0x00000010（实际只走了 32 个 tick）");
  stub_dwt.CYCCNT = 0xFFFFFFF0u;
  Timer_Start();
  stub_dwt.CYCCNT = 0x00000010u;
  Timer_Stop();
  ck_int("跨回绕的差值 = 32（不是 40 多亿）", (long)Timer_Delta_tick(), 32);
  ck_near("换算成时间 ≈ 0.190476 µs", Timer_Delta_us(), US(32), 0.0001f);
}

/* 当前时刻：CYCCNT 走满 1 秒的 tick 数 → now_us 应为 1e6 */
static void t_now(void) {
  begin("t_now", "当前时刻：CYCCNT = 168000000（= 1 秒 @168 MHz）");
  stub_dwt.CYCCNT = 168000000u;
  ck_near("Timer_Now_us = 1000000", Timer_Now_us(), 1000000.0f, 1.0f);
}

/* 统计：次数递增、最大值跟着最大的那次走 */
static void t_count_max(void) {
  begin("t_count_max", "统计：测三次（100 / 500 / 200 tick）");
  uint32_t vals[3] = {100, 500, 200};
  for (int i = 0; i < 3; i++) {
    Timer_Start();
    stub_dwt.CYCCNT += vals[i];
    Timer_Stop();
  }
  ck_int("Timer_GetCount = 3", (long)Timer_GetCount(), 3);
  ck_near("最大值 = 500 tick ≈ 2.976 µs", Timer_GetMax_us(), US(500), 0.001f);
}

/* 最小值：复位（= 未测量哨兵态）之后，第一次测到的值必须成为最小值
   —— 这条才是真正验证"哨兵没有挡住真实测量"的判据 */
static void t_min(void) {
  begin("t_min", "最小值：复位后先测 100 tick，再测 300 tick");
  Timer_Start();
  stub_dwt.CYCCNT += 100;
  Timer_Stop();
  Timer_Start();
  stub_dwt.CYCCNT += 300;
  Timer_Stop();
  ck_near("最小值 = 100 tick ≈ 0.595 µs", Timer_GetMin_us(), US(100), 0.001f);
}

/* 复位：三个统计量都归零 */
static void t_reset(void) {
  begin("t_reset", "复位：测两次后调用 Timer_Reset");
  Timer_Start();
  stub_dwt.CYCCNT += 400;
  Timer_Stop();
  Timer_Reset();
  ck_int("复位后 Timer_GetCount = 0", (long)Timer_GetCount(), 0);
  ck_near("复位后最大值 = 0", Timer_GetMax_us(), 0.0f, 0.0001f);
  /* 契约（用户 2026-10-14 定）：复位后是"**还没测过**"状态，
     最小值应当是哨兵值，而不是 0 —— 因为 0 本身可能是一次真实测量值。 */
  printf("  · 复位后最小值 = %.6f（哨兵 = 未测量）\n", Timer_GetMin_us());
  ck_true("复位后最小值处于未测量哨兵态（≥ 1e8 µs）", Timer_GetMin_us() >= 1e8f);
}

/* 换算精度：不同量级的采样都不能出现"整数量化"的错 */
static void t_precision(void) {
  begin("t_precision", "换算精度：1 tick / 1 ms / 1 s 三个量级");
  stub_dwt.CYCCNT = 0;
  Timer_Start();
  stub_dwt.CYCCNT = 1;
  Timer_Stop();
  ck_near("1 tick ≈ 0.005952 µs", Timer_Delta_us(), US(1), 1e-6f);

  stub_dwt.CYCCNT = 0;
  Timer_Start();
  stub_dwt.CYCCNT = 168000; /* 1 ms */
  Timer_Stop();
  ck_near("168000 tick = 1000 µs", Timer_Delta_us(), 1000.0f, 0.01f);

  stub_dwt.CYCCNT = 0;
  Timer_Start();
  stub_dwt.CYCCNT = 168000000u; /* 1 s */
  Timer_Stop();
  ck_near("168000000 tick = 1 s", Timer_Delta_s(), 1.0f, 0.00001f);
}

/* ============================ 入口 ============================ */

typedef struct {
  const char *name;
  void (*fn)(void);
} case_t;

static const case_t k_cases[] = {
  {"t_init", t_init},           {"t_units", t_units},   {"t_wrap", t_wrap},
  {"t_now", t_now},             {"t_count_max", t_count_max},
  {"t_min", t_min},             {"t_reset", t_reset},   {"t_precision", t_precision},
};

int main(int argc, char **argv) {
  const char *which = (argc > 1) ? argv[1] : "all";
  printf("=== Timer 主机侧行为判据（%s）===\n", which);
  printf("图例：✅ 判据满足 / ❌ 判据不满足；本文件用 DWT 替身把时间拨到已知值，"
         "所以判据是定量的。\n");
  int ran = 0;
  for (unsigned i = 0; i < sizeof(k_cases) / sizeof(k_cases[0]); i++) {
    if (strcmp(which, "all") != 0 && strcmp(which, k_cases[i].name) != 0) continue;
    int before = g_fail;
    k_cases[i].fn();
    ran++;
    printf("  → 本场景不满足的判据数：%d\n", g_fail - before);
  }
  if (ran == 0) {
    printf("没有叫 %s 的场景\n", which);
    return 2;
  }
  printf("\n=== 不满足的判据总数：%d ===\n", g_fail);
  return g_fail ? 1 : 0;
}
