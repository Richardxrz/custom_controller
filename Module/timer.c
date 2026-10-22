#include "timer.h"
#include "stm32f4xx.h"

bool initialized_ = false;
uint32_t start_ = 0;
uint32_t stop_ = 0;
float max_dt_us_ = TIMER_NO_MEASUREMENT_MAX_US;
float min_dt_us_ = TIMER_NO_MEASUREMENT_MIN_US;
uint32_t count_ = 0;

bool Timer_Init(void) {
  if (initialized_) {
    return true;
  }

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  if (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) {
    initialized_ = true;
    return true;
  }

  return false;
}

void Timer_Start(void) { start_ = DWT->CYCCNT; }

void Timer_Stop(void) {
  stop_ = DWT->CYCCNT;
  float dticks = Timer_Delta_us();
  if (dticks > max_dt_us_) {
    max_dt_us_ = dticks;
  }
  if (dticks < min_dt_us_) {
    min_dt_us_ = dticks;
  }
  count_++;
}

float Timer_Delta_us(void) {
  uint32_t dticks = stop_ - start_;
  return (float)dticks * 1000000.0f / (float)SystemCoreClock;
}

float Timer_Delta_ms(void) { return Timer_Delta_us() / 1000.0f; }

float Timer_Delta_s(void) { return Timer_Delta_us() / 1000000.0f; }

uint32_t Timer_Delta_tick(void) { return stop_ - start_; }

float Timer_Now_us(void) { return (float)DWT->CYCCNT * 1000000 / (float)SystemCoreClock; }

float Timer_GetMax_us(void) { return max_dt_us_; }

float Timer_GetMin_us(void) { return min_dt_us_; }

uint32_t Timer_GetCount(void) { return count_; }

void Timer_Reset(void) {
  max_dt_us_ = TIMER_NO_MEASUREMENT_MAX_US;
  min_dt_us_ = TIMER_NO_MEASUREMENT_MIN_US;
  count_ = 0;
}
