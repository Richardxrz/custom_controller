#ifndef TIMER_H
#define TIMER_H

#include <stdbool.h>
#include <stdint.h>

#define TIMER_NO_MEASUREMENT_MIN_US 1e9f
#define TIMER_NO_MEASUREMENT_MAX_US 0.0f

/**
 * @brief 初始化
 */
bool Timer_Init(void);
/**
 * @brief 开始计时
 */
void Timer_Start(void);
/**
 * @brief 停止计时
 */
void Timer_Stop(void);
/**
 * @brief 获取计时间隔
 */
float Timer_Delta_us(void);
float Timer_Delta_ms(void);
float Timer_Delta_s(void);
uint32_t Timer_Delta_tick(void);
/**
 * @brief 获取当前时刻（单位：微秒）
 */
float Timer_Now_us(void);
/**
 * @brief 重置计时器统计变量
 */
void Timer_Reset(void);
/**
 * @brief 获取最大时间间隔
 */
float Timer_GetMax_us(void);
/**
 * @brief 获取最小时间间隔
 * @nore  未测量时初值是1e9
 */
float Timer_GetMin_us(void);
/**
 * @brief 获取计时次数
 */
uint32_t Timer_GetCount(void);

#endif
