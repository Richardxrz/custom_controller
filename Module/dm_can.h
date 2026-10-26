/**
 * @file    dm_can.h
 * @brief   达妙（DM）电机 CAN 帧编解码：MIT 模式 + 使能/失能/设零
 *
 * 帧格式（MIT 模式，8 字节，大端位域）：
 *     data[0..1]  pos  16 bit
 *     data[2..3]  vel  12 bit（data[2] 高 8 位，data[3] 高 4 位）
 *     data[3..4]  kp   12 bit（data[3] 低 4 位 + data[4]）
 *     data[5..6]  kd   12 bit（data[5] 高 8 位，data[6] 高 4 位）
 *     data[6..7]  t    12 bit（data[6] 低 4 位 + data[7]）
 *
 * 反馈帧（电机 → 主机，8 字节）：
 *     data[0]    电机 ID
 *     data[1]    状态字节（错误码/温度，具体位域以达妙手册为准）
 *     data[2..3] pos 原始值
 *     data[4..5] vel 原始值
 *     data[6..7] torque 原始值
 *
 * ⚠️ 不确定性（必须对达妙手册核对后再上电机）：
 *     1. 反馈帧的 CAN ID：常见实现是「主控 ID」（默认 0），data[0] 才是电机 ID；
 *     2. data[1] 里错误码/温度的位域；
 *     3. 每种电机的 P/V/T 量程（本模块用 cfg 显式给，不写死）。
 *
 * 量程换算（达妙 SDK 同款）：
 *     uint = (x - min) / (max - min) * ((1 << bits) - 1)
 */
#ifndef DM_CAN_H
#define DM_CAN_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint8_t id;
  float p_min, p_max; /* rad */
  float v_min, v_max; /* rad/s */
  float t_min, t_max; /* N·m */
} DmMotorCfg;

typedef struct {
  uint8_t id;
  uint8_t state;      /* 反馈首字节：0x01 失能 / 0x11 使能 */
  uint8_t temp_mos;   /* MOS 温度 ℃ */
  uint8_t temp_rotor; /* 线圈温度 ℃ */
  float pos;          /* rad */
  float vel;          /* rad/s */
  float torque;       /* N·m */
} DmFeedback;

/**
 * @brief  打一帧 MIT 控制
 * @param  out 8 字节
 * @note   超出量程的值会被夹到边界
 */
extern void Dm_MitPack(uint8_t out[8], const DmMotorCfg *cfg, float pos, float vel, float kp,
                       float kd, float torque);

/**
 * @brief  解一帧反馈
 * @retval false = 数据里的 ID 与 cfg 不符（不是这台电机的帧）
 */
extern bool Dm_FeedbackDecode(DmFeedback *fb, const DmMotorCfg *cfg, const uint8_t data[8]);

/**
 * @brief  特殊命令帧（8 字节）
 * @param  kind 'e'=使能 'd'=失能 'z'=设零 'c'=清错
 */
extern void Dm_SpecialFrame(uint8_t out[8], char kind);

/**
 * @brief  浮点 ↔ 定点（暴露出来给测试和调试用）
 */
extern uint16_t Dm_FloatToUint(float x, float min, float max, uint8_t bits);
extern float Dm_UintToFloat(uint16_t u, float min, float max, uint8_t bits);

#endif
