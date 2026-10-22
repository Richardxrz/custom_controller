/*
 * 主机侧测试用的 stm32f4xx.h 替身。
 * 只覆盖 Module/timer.c 用到的那几个内核外设：DWT（周期计数器）和 CoreDebug（跟踪使能）。
 *
 * 真板子上 DWT/CoreDebug 是固定地址的寄存器（CMSIS 里写成宏）；
 * 主机上没有那些地址，所以这里把它们指到两个普通全局变量上，
 * 测试就能像调寄存器一样直接给 CYCCNT 赋一个已知值 —— 时间可控，判据才能定量。
 */
#ifndef STUB_STM32F4XX_H
#define STUB_STM32F4XX_H

#include <stdint.h>

/* 真 CMSIS 的 DWT：CTRL 在 +0x00，CYCCNT 在 +0x04 */
typedef struct {
  volatile uint32_t CTRL;   /* Control Register      */
  volatile uint32_t CYCCNT; /* Cycle Count Register  */
} DWT_Type;

/* 真 CMSIS 的 CoreDebug：DEMCR 在 +0x0C */
typedef struct {
  volatile uint32_t DHCSR;
  volatile uint32_t DCRSR;
  volatile uint32_t DCRDR;
  volatile uint32_t DEMCR; /* Debug Exception and Monitor Control Register */
} CoreDebug_Type;

#define CoreDebug_DEMCR_TRCENA_Msk (1UL << 24) /* 使能跟踪（TRCENA） */
#define DWT_CTRL_CYCCNTENA_Msk (1UL << 0)      /* 使能周期计数器（CYCCNTENA） */

extern DWT_Type stub_dwt;
extern CoreDebug_Type stub_coredebug;

#define DWT (&stub_dwt)
#define CoreDebug (&stub_coredebug)

extern uint32_t SystemCoreClock;

#endif /* STUB_STM32F4XX_H */
