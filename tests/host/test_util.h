#ifndef TEST_UTIL_H
#define TEST_UTIL_H

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* 每个测试都是一个独立进程（模块内部有 static 状态），所以这里用 static 不会互相干扰。 */
static int g_fail = 0;

static void t_check(bool cond, const char *what) {
  if (cond) {
    printf("  \xE2\x9C\x85 %s\n", what);
  } else {
    printf("  \xE2\x9D\x8C %s\n", what);
    g_fail++;
  }
}

static int t_summary(void) {
  printf("\n=== %s（失败 %d 条）===\n", g_fail == 0 ? "全过" : "有失败", g_fail);
  return g_fail == 0 ? 0 : 1;
}

#endif
