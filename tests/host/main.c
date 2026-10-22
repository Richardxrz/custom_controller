/*
 * tx_uart_it 主机侧测试台：把 Module/tx_uart_it.c 拿到 Mac 上跑，不需要板子。
 *
 *   ./tests/host/run.sh              跑全部场景
 *   ./tests/host/run.sh s_normal     只跑一个场景（每个场景都是全新进程）
 *
 * 输出约定：
 *   ✅ 判据满足   ❌ 判据不满足（同行给出 期望/实际）
 *   若出现 AddressSanitizer 报告 → 发生内存越界 / 野指针 / 未定义行为，
 *   进程会被当场中止，这本身就是该场景失败。
 * 本文件只表达**判据**（应该成立什么），不含"哪里写错了""该怎么改"。
 */
#include <stdio.h>
#include <string.h>

#include "tx_uart_it.h"
#include "usart.h"

/* ---- 被测代码依赖的全局量 ----
 * 两个句柄都是**测试替身**，不是 CubeMX 生成的：本工程里只有 huart6 是真句柄，
 * 硬件和 CubeMX 配置里都不存在第二个串口。
 * 之所以还要造一个：模块对外承诺"最多 2 个实例"（TX_INSTANCE_MAX），
 * "数据有没有写到自己那个实例上"这类问题只有注册第二个句柄才会暴露出来。
 * 模块只是保存指针、再把它交回给 HAL；替身从不碰 Instance 指向的寄存器，
 * 所以这个句柄不需要任何硬件或配置，只需是一个**不同的地址**。
 */
UART_HandleTypeDef huart6 = { .Instance = (USART_TypeDef *)0x40011400,
                              .gState = HAL_UART_STATE_READY,
                              .RxState = HAL_UART_STATE_READY };
UART_HandleTypeDef huart_b = { .Instance = (USART_TypeDef *)0x40004400, /* 假句柄 B */
                               .gState = HAL_UART_STATE_READY,
                               .RxState = HAL_UART_STATE_READY };
stub_hal_t g_stub;

/* 被测模块里定义的回调（这里只做声明，好让测试能按需触发它） */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

void stub_reset(void) {
  memset(&g_stub, 0, sizeof(g_stub));
  g_stub.tx_ret = HAL_OK;
}

/* 替身语义与真 HAL 对齐：
   Transmit_IT 成功 → huart->gState = BUSY_TX      （stm32f4xx_hal_uart.c:1323） */
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, const uint8_t *pData,
                                       uint16_t Size) {
  g_stub.tx_calls++;
  g_stub.tx_huart = huart;
  g_stub.tx_data = pData;
  g_stub.tx_size = Size;
  /* 真 HAL 只保存指针（huart->pTxBuffPtr = pData），不会复制数据；
     所以替身也保存指针，另外抄一小份"调用当时的内容"，供之后对比。 */
  g_stub.snap_n = (Size < sizeof(g_stub.snap)) ? (int)Size : (int)sizeof(g_stub.snap);
  if (pData != NULL && g_stub.snap_n > 0) {
    memcpy(g_stub.snap, pData, (size_t)g_stub.snap_n);
  }
  if (huart != NULL) {
    huart->gState = HAL_UART_STATE_BUSY_TX;
  }
  return g_stub.tx_ret;
}

/* 打一条替身记录：句柄 / 数据指针 / 长度 / **调用当时**的内容 */
static void dump_call(const char *label) {
  printf("  · %s：句柄=%p 数据指针=%p 长度=%u 调用当时内容=", label, (void *)g_stub.tx_huart,
         (const void *)g_stub.tx_data, g_stub.tx_size);
  for (int i = 0; i < g_stub.snap_n; i++) printf(" %02X", g_stub.snap[i]);
  printf("\n");
}

/* 打"这根指针**现在**指向的内容"：和调用当时比，就知道这块内存有没有被改 */
static void dump_now(const char *label) {
  printf("  · %s：数据指针=%p 现在内容=", label, (const void *)g_stub.tx_data);
  if (g_stub.tx_data == NULL || g_stub.snap_n == 0) {
    printf("(没有可读内容)\n");
    return;
  }
  for (int i = 0; i < g_stub.snap_n; i++) printf(" %02X", g_stub.tx_data[i]);
  printf("\n");
}

/* GetState → gState | RxState                          （:2955-2961） */
HAL_UART_StateTypeDef HAL_UART_GetState(const UART_HandleTypeDef *huart) {
  if (huart == NULL) return HAL_UART_STATE_RESET;
  return (HAL_UART_StateTypeDef)(huart->gState | huart->RxState);
}

/* 模拟 HAL 的"发送完成"中断路径。真 HAL 的顺序是：
 *   UART_EndTransmit_IT: huart->gState = READY;            ← :3609
 *                        HAL_UART_TxCpltCallback(huart);   ← :3616
 * 即回调执行时，HAL 那半边状态已经复位。 */
static void simulate_tc(UART_HandleTypeDef *huart) {
  huart->gState = HAL_UART_STATE_READY;
  HAL_UART_TxCpltCallback(huart);
}

/* ---- 断言：只报判据是否满足 ---- */
static int g_fail;

static void ck_int(const char *rule, long got, long want) {
  int ok = (got == want);
  if (!ok) g_fail++;
  printf("  %s %s → 期望 %ld，实际 %ld\n", ok ? "✅" : "❌", rule, want, got);
}

static void ck_ptr(const char *rule, const void *got, const void *want) {
  int ok = (got == want);
  if (!ok) g_fail++;
  printf("  %s %s → 期望 %p，实际 %p\n", ok ? "✅" : "❌", rule, want, got);
}

static void ck_mem(const char *rule, const uint8_t *got, const uint8_t *want, int n) {
  int ok = (got != NULL) && (memcmp(got, want, (size_t)n) == 0);
  if (!ok) g_fail++;
  if (ok) {
    printf("  ✅ %s\n", rule);
    return;
  }
  printf("  ❌ %s\n    期望：", rule);
  for (int i = 0; i < n; i++) printf(" %02X", want[i]);
  printf("\n    实际：");
  if (got == NULL) {
    printf(" (空指针)\n");
  } else {
    for (int i = 0; i < n; i++) printf(" %02X", got[i]);
    printf("\n");
  }
}

static void begin(const char *name, const char *what) {
  printf("\n[%s] %s\n", name, what);
  stub_reset();
}

/* ============================ 场景 ============================ */

/* 用户忘了 Init 就直接发 */
static void s_uninit(void) {
  begin("s_uninit", "没注册串口就调用发送（模拟忘记 Init）");
  uint8_t buf[4] = {0x00, 0x01, 0x02, 0x03};
  TxIt_State ret = TxIt_Send(&huart6, buf, sizeof(buf));
  ck_int("没注册的串口，Send 必须拒绝（TX_ERROR=2）", (long)ret, (long)TX_ERROR);
  ck_int("没注册的串口，不许调到 HAL", g_stub.tx_calls, 0);
  ck_int("没注册的串口查状态，返回 TX_ERROR=2", (long)TxIt_GetState(&huart6), (long)TX_ERROR);
}

/* 最普通的一次发送 */
static void s_normal(void) {
  begin("s_normal", "正常流：注册 huart6，发 4 字节 00 01 02 03");
  uint8_t buf[4] = {0x00, 0x01, 0x02, 0x03};
  TxIt_Init(&huart6);
  ck_int("Send 返回 TX_OK=0", (long)TxIt_Send(&huart6, buf, sizeof(buf)), (long)TX_OK);
  ck_int("HAL 被调用了 1 次", g_stub.tx_calls, 1);
  ck_ptr("HAL 拿到的句柄就是我注册的 huart6", g_stub.tx_huart, &huart6);
  ck_int("HAL 拿到的长度 = 我传的 4", g_stub.tx_size, 4);
  ck_mem("HAL 拿到的数据 = 我传的 00 01 02 03", g_stub.tx_data, buf, 4);
  ck_int("发送中状态应可观测为 TX_BUSY=3", (long)TxIt_GetState(&huart6), (long)TX_BUSY);
  printf("  -- 模拟 HAL 发完（TC 中断）--\n");
  simulate_tc(&huart6);
  ck_int("发完后状态回到 TX_READY=1", (long)TxIt_GetState(&huart6), (long)TX_READY);
}

/* 长度 0 */
static void s_len0(void) {
  begin("s_len0", "长度传 0");
  uint8_t buf[4] = {0x00, 0x01, 0x02, 0x03};
  TxIt_Init(&huart6);
  ck_int("长度 0 必须被拒（TX_ERROR=2）", (long)TxIt_Send(&huart6, buf, 0), (long)TX_ERROR);
  ck_int("被拒时不许调到 HAL", g_stub.tx_calls, 0);
}

/* 数据指针为空 */
static void s_null(void) {
  begin("s_null", "数据指针传 NULL");
  TxIt_Init(&huart6);
  ck_int("NULL 数据必须被拒（TX_ERROR=2）", (long)TxIt_Send(&huart6, NULL, 4), (long)TX_ERROR);
  ck_int("被拒时不许调到 HAL", g_stub.tx_calls, 0);
}

/* 超过实例缓冲区容量：.h 的契约写着"length 太大会导致数据截断" → 截断后照发 */
static void s_over(void) {
  begin("s_over", "长度 300，超过实例缓冲区容量 128（.h 契约：截断后照发）");
  uint8_t big[300];
  for (int i = 0; i < 300; i++) big[i] = (uint8_t)i;
  TxIt_Init(&huart6);
  ck_int("超长请求被截断后照发，返回 TX_OK=0", (long)TxIt_Send(&huart6, big, sizeof(big)),
         (long)TX_OK);
  printf("  -- 进程能走到这里，说明没有越界写内存 --\n");
  ck_int("HAL 被调用了 1 次", g_stub.tx_calls, 1);
  ck_int("HAL 拿到的长度 = 容量 128（不是 300）", g_stub.tx_size, 128);
  ck_mem("HAL 拿到的是输入的前 128 字节", g_stub.tx_data, big, 128);
}

/* 边界：正好塞满 */
static void s_clamp128(void) {
  begin("s_clamp128", "长度正好等于容量 128（边界，不该被截断、不该越界）");
  uint8_t buf[128];
  for (int i = 0; i < 128; i++) buf[i] = (uint8_t)(0xA0 + i);
  TxIt_Init(&huart6);
  ck_int("Send 返回 TX_OK=0", (long)TxIt_Send(&huart6, buf, sizeof(buf)), (long)TX_OK);
  ck_int("HAL 拿到的长度 = 128", g_stub.tx_size, 128);
  ck_mem("HAL 拿到的 128 字节 = 我传进去的", g_stub.tx_data, buf, 128);
}

/* 两个实例：数据是否各归各 */
static void s_two(void) {
  begin("s_two", "注册两个句柄（真 huart6 + 假句柄 B），各自发一次（B 未发完）");
  uint8_t a[4] = {0x00, 0x01, 0x02, 0x03};
  uint8_t b[4] = {0xAA, 0xBB, 0xCC, 0xDD};
  TxIt_Init(&huart6);
  TxIt_Init(&huart_b);
  TxIt_Send(&huart6, a, sizeof(a));
  ck_ptr("发给 huart6 时，HAL 拿到的句柄是 huart6", g_stub.tx_huart, &huart6);
  ck_int("发给 huart6 时，HAL 拿到的长度是 4", g_stub.tx_size, 4);
  TxIt_Send(&huart_b, b, sizeof(b));
  dump_call("第二笔（发往假句柄 B）HAL 记下的");
  ck_ptr("发给假句柄 B 时，HAL 拿到的句柄是 B", g_stub.tx_huart, &huart_b);
  ck_int("发给假句柄 B 时，HAL 拿到的长度是 4", g_stub.tx_size, 4);
  ck_mem("发给假句柄 B 时，HAL 拿到的是 B 的数据 AA BB CC DD", g_stub.tx_data, b, 4);
  printf("  -- 只模拟 huart6 发完 --\n");
  simulate_tc(&huart6);
  ck_int("huart6 发完 → TX_READY=1", (long)TxIt_GetState(&huart6), (long)TX_READY);
  ck_int("假句柄 B 还在发 → TX_BUSY=3", (long)TxIt_GetState(&huart_b), (long)TX_BUSY);
}

/* 上一次没发完就再发一次 */
static void s_reentry(void) {
  begin("s_reentry", "上一笔还没发完就再调 Send（重入）");
  uint8_t a[4] = {0x00, 0x01, 0x02, 0x03};
  uint8_t b[4] = {0xAA, 0xBB, 0xCC, 0xDD};
  TxIt_Init(&huart6);
  ck_int("第一次 Send 返回 TX_OK=0", (long)TxIt_Send(&huart6, a, sizeof(a)), (long)TX_OK);
  dump_call("第一次 Send 之后，HAL 记下的");
  ck_int("第二次 Send 必须被拒（TX_BUSY=3）", (long)TxIt_Send(&huart6, b, sizeof(b)),
         (long)TX_BUSY);
  ck_int("被拒时不许再调 HAL（总次数仍是 1）", g_stub.tx_calls, 1);
  dump_now("第二次被拒之后，同一根指针");
  ck_mem("HAL 正在发的那笔数据不能被第二次覆盖（覆盖 = 发出去的字节被改坏）", g_stub.tx_data, a,
         4);
}

/* 发完一笔再发下一笔 */
static void s_resend(void) {
  begin("s_resend", "第一笔发完之后，再发一笔新数据");
  uint8_t a[4] = {0x00, 0x01, 0x02, 0x03};
  uint8_t b[4] = {0xAA, 0xBB, 0xCC, 0xDD};
  TxIt_Init(&huart6);
  TxIt_Send(&huart6, a, sizeof(a));
  simulate_tc(&huart6);
  ck_int("第二笔 Send 返回 TX_OK=0", (long)TxIt_Send(&huart6, b, sizeof(b)), (long)TX_OK);
  ck_ptr("HAL 拿到的句柄是 huart6", g_stub.tx_huart, &huart6);
  ck_int("HAL 拿到的长度是 4", g_stub.tx_size, 4);
  ck_mem("HAL 拿到的是第二笔的新数据 AA BB CC DD", g_stub.tx_data, b, 4);
}

/* ============================ 入口 ============================ */

typedef struct {
  const char *name;
  void (*fn)(void);
} scenario_t;

static const scenario_t k_scenes[] = {
  {"s_uninit", s_uninit},     {"s_normal", s_normal},     {"s_len0", s_len0},
  {"s_null", s_null},         {"s_over", s_over},         {"s_clamp128", s_clamp128},
  {"s_two", s_two},           {"s_reentry", s_reentry},   {"s_resend", s_resend},
};

int main(int argc, char **argv) {
  const char *which = (argc > 1) ? argv[1] : "all";
  printf("=== tx_uart_it 主机侧测试台（%s）===\n", which);
  printf("图例：✅ 判据满足 / ❌ 判据不满足；出现 AddressSanitizer 报告 = 内存越界或野指针，"
         "进程当场中止。\n");
  int ran = 0;
  for (unsigned i = 0; i < sizeof(k_scenes) / sizeof(k_scenes[0]); i++) {
    if (strcmp(which, "all") != 0 && strcmp(which, k_scenes[i].name) != 0) continue;
    int before = g_fail;
    k_scenes[i].fn();
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
