/**
 * @file    bus_servo.h
 * @brief   总线舵机指令层：一次「发命令 → 等应答 → 超时放弃」的事务
 *
 * 这一层只依赖两个抽象：
 *     1. 字节怎么发/怎么收（BusServoPort，由驱动层实现）
 *     2. 字节长什么样（servo_codec，纯函数）
 * 所以它能在电脑上用一个假 Port 完整跑起来，不需要板子、不需要 sleep。
 *
 * 最小用法（主循环里）：
 *     BusServo s;
 *     BusServo_Init(&s, port, 3);           // 3 号舵机
 *
 *     BusServo_RequestAngle(&s);            // 发起一次读角度
 *     while (1) {
 *       BusServo_Poll(&s);                  // 每 1ms 调一次
 *       uint16_t pwm;
 *       if (BusServo_GetAngle(&s, &pwm)) { 用 pwm }
 *     }
 *
 * 状态：
 *     IDLE ──请求──> WAIT ──有效应答──> DONE
 *                      └────超时────> TIMEOUT
 *     DONE/TIMEOUT 都允许发起下一次请求；WAIT 期间发起新请求会被拒。
 *
 * 约束：
 *     同一时刻只允许一个未完成的事务（单总线半双工，本来也只能这样）；
 *     回显（自己发出去的命令被环回）会被识别成 SERVO_FRAME_CMD 并丢弃；
 *     超时时间在 Init 里默认 20 ms，可用 BusServo_SetTimeout 改。
 */
#ifndef BUS_SERVO_H
#define BUS_SERVO_H

#include <stdbool.h>
#include <stdint.h>

#include "servo_codec.h"

/** 驱动层需要提供的最小能力（函数指针 + 上下文，方便替换成假实现） */
typedef struct {
  void (*write)(void *ctx, const uint8_t *data, uint16_t len);
  uint16_t (*read)(void *ctx, uint8_t *buf, uint16_t cap);
  uint32_t (*now_ms)(void *ctx);
  void *ctx;
} BusServoPort;

typedef enum {
  BUS_SERVO_IDLE = 0,
  BUS_SERVO_WAIT,    /* 已发出，等应答 */
  BUS_SERVO_DONE,    /* 收到有效应答 */
  BUS_SERVO_TIMEOUT, /* 超时 */
} BusServoState;

typedef enum {
  BUS_SERVO_OP_NONE = 0,
  BUS_SERVO_OP_READ,    /* 期望 VALUE 应答 */
  BUS_SERVO_OP_RELEASE, /* 期望 OK 应答 */
} BusServoOp;

typedef struct {
  BusServoPort port;
  uint8_t id;
  uint16_t timeout_ms;
  BusServoState state;
  uint8_t op;
  uint32_t sent_ms;
  uint16_t value;    /* 最近一次 VALUE 应答的数值 */
  uint16_t err_cnt;  /* 超时次数 */
  ServoCodecParser parser; /* 内部使用，调用方不要改 */
} BusServo;

/**
 * @brief  初始化一个舵机对象（不碰硬件，只存参数）
 * @param  port 驱动层实现
 * @param  id   舵机 ID（0..254）
 */
extern void BusServo_Init(BusServo *s, BusServoPort port, uint8_t id);

/**
 * @brief  设置事务超时
 * @param  ms 默认 20 ms；建议 ≥ 一次往返时间（6 轴 115200 时约 2 ms）的 3 倍
 */
extern void BusServo_SetTimeout(BusServo *s, uint16_t ms);

/**
 * @brief  发起读角度
 * @retval false = 上一个事务还没结束（WAIT），本次被拒
 */
extern bool BusServo_RequestAngle(BusServo *s);

/**
 * @brief  发一条控制命令（舵机不应答，发完即回 IDLE）
 * @retval false = 忙 / 参数非法
 */
extern bool BusServo_SetAngle(BusServo *s, uint16_t pwm, uint16_t t_ms);

/**
 * @brief  发起释力
 * @retval false = 忙
 */
extern bool BusServo_Release(BusServo *s);

/**
 * @brief  推进状态机：读字节、喂解析器、判超时
 * @note   主循环里每 1 ms 左右调一次；不阻塞
 */
extern void BusServo_Poll(BusServo *s);

/**
 * @brief  取最近一次读角度的结果
 * @retval true = 结果有效（自上次请求后收到过 VALUE 应答）
 */
extern bool BusServo_GetAngle(const BusServo *s, uint16_t *pwm);

/**
 * @brief  取状态 / 超时计数
 */
extern BusServoState BusServo_GetState(const BusServo *s);
extern uint16_t BusServo_GetErrCnt(const BusServo *s);

#endif
