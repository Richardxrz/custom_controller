/**
 * @file    bus_servo_group.h
 * @brief   多轴总线舵机调度：轮流读角度 / 一次全部释力
 *
 * 为什么需要这一层：
 *     单总线是半双工，同一时刻只能有一个事务；6 个舵机不能并发。
 *     这一层负责「轮到谁、什么时候轮下一轮、某轴超时了怎么办」。
 *
 * 用法：
 *     BusServoGroup g;
 *     uint8_t ids[6] = {1,2,3,4,5,6};
 *     BusServoGroup_Init(&g, port, ids, 6, 5);
 *     BusServoGroup_SetPeriod(&g, 0);      // 0 = 背靠背，一轮约 10.4 ms
 *     BusServoGroup_StartSweep(&g, now);
 *
 *     while (1) {
 *       BusServoGroup_Poll(&g, now_ms);
 *       uint16_t pwm;
 *       if (BusServoGroup_GetPos(&g, 0, &pwm)) { ... }
 *     }
 *
 * 调度策略：
 *     背靠背：上一条应答一到就发下一条，一轮 6 轴约 10.4 ms（115200 时）；
 *     周期：若 SetPeriod(ms) > 0，每轮开始前等到周期到点，保证采样间隔均匀；
 *     超时：单轴超过 timeout_ms 没应答 → 记一次超时、跳过该轴，继续下一轴。
 */
#ifndef BUS_SERVO_GROUP_H
#define BUS_SERVO_GROUP_H

#include <stdbool.h>
#include <stdint.h>

#include "bus_servo.h"

#define BUS_SERVO_GROUP_MAX 8u

typedef enum {
  GROUP_IDLE = 0,
  GROUP_SWEEP,
  GROUP_RELEASE,
  GROUP_DONE,
} BusServoGroupMode;

typedef struct {
  BusServo servo[BUS_SERVO_GROUP_MAX];
  BusServoPort port;
  BusServoGroupMode mode;
  uint8_t n;
  uint8_t idx;
  uint8_t phase;
  uint16_t period_ms;
  uint16_t timeout_ms;
  uint32_t next_start_ms;
  uint32_t rounds;
  uint16_t timeouts;
  uint16_t pos[BUS_SERVO_GROUP_MAX];
  bool pos_valid[BUS_SERVO_GROUP_MAX];
} BusServoGroup;

/**
 * @brief  初始化
 * @param  ids      舵机 ID 数组（长度 n）
 * @param  timeout  单轴事务超时，建议 5 ms（一次往返约 1.7 ms）
 */
extern void BusServoGroup_Init(BusServoGroup *g, BusServoPort port, const uint8_t *ids, uint8_t n,
                               uint16_t timeout);

/**
 * @brief  设置每轮周期（ms）；0 = 背靠背
 */
extern void BusServoGroup_SetPeriod(BusServoGroup *g, uint16_t ms);

/**
 * @brief  开始连续采样
 */
extern void BusServoGroup_StartSweep(BusServoGroup *g, uint32_t now_ms);

/**
 * @brief  开始逐个释力（完成后 mode = GROUP_DONE）
 */
extern void BusServoGroup_ReleaseAll(BusServoGroup *g, uint32_t now_ms);

/**
 * @brief  推进状态机；主循环里每 1 ms 调一次
 */
extern void BusServoGroup_Poll(BusServoGroup *g, uint32_t now_ms);

/**
 * @brief  取某轴最近一次有效角度
 * @retval false = 还没有有效值
 */
extern bool BusServoGroup_GetPos(const BusServoGroup *g, uint8_t idx, uint16_t *pwm);

extern BusServoGroupMode BusServoGroup_GetMode(const BusServoGroup *g);
extern uint32_t BusServoGroup_GetRounds(const BusServoGroup *g);
extern uint16_t BusServoGroup_GetTimeouts(const BusServoGroup *g);

#endif
