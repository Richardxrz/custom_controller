/**
 * @file    servo_codec.h
 * @brief   众灵总线舵机的 ASCII 协议编解码（纯函数，不碰 HAL、不碰时间）
 *
 * 协议（手册）：'#' + 3 位 ID + 'P' + 4 位 PWM + 'T' + 4 位时间 + '!'
 *   控制：  #003P1500T1000!
 *   查询：  #003PRAD!   ->  #003P1500!
 *           #003PID!   ->  #003P!        （ID 存在）
 *           #003PMOD!  ->  #003PMOD1!
 *           #003PVER!  ->  #003PV0.97!
 *           #003PULK!  ->  #OK!
 *           #003PRTV!  ->  #000T25V07!   （注意这条没有 P）
 *   改 ID： #000PID001! -> #001P!
 *
 * 为什么单独一层：
 *   同一根总线上「字节长什么样」会随设备变（舵机 ASCII / RM 帧 / 达妙 CAN），
 *   而「字节怎么搬」不随协议变。这层只管字节 ↔ 语义，因此能脱离硬件在电脑上直接测。
 *
 * 最小用法：
 *     char cmd[16];
 *     uint16_t n = ServoCodec_BuildMove(cmd, sizeof(cmd), 3, 1500, 1000);
 *     uart_write(cmd, n);
 *
 *     ServoCodecParser p; ServoCodec_Reset(&p);
 *     ServoCodecFrame f;
 *     for (每个收到的字节 b)
 *       if (ServoCodec_Feed(&p, b, &f)) { 用 f.kind / f.id / f.body }
 *
 * 约束：
 *     编解码器不判超时、不判「这条应答是不是我要的」——那是指令层的事；
 *     它只负责「从字节流里切出一帧，并说明它像什么」。
 */
#ifndef SERVO_CODEC_H
#define SERVO_CODEC_H

#include <stdbool.h>
#include <stdint.h>

#define SERVO_CODEC_BODY_MAX 23 /* 不含结尾 '\0' */

/** 查询类命令（对应手册里的 Pxxx） */
typedef enum {
  SERVO_Q_RAD = 0, /* 读位置 */
  SERVO_Q_ID,      /* 读/检 ID */
  SERVO_Q_MOD,     /* 读模式 */
  SERVO_Q_VER,     /* 读版本 */
  SERVO_Q_ULK,     /* 释力 */
  SERVO_Q_ULR,     /* 恢复力矩 */
  SERVO_Q_DPT,     /* 暂停 */
  SERVO_Q_DCT,     /* 继续 */
  SERVO_Q_DST,     /* 停止 */
  SERVO_Q_PRTV,    /* 温度/电压 */
  SERVO_Q_PCSM,    /* 开机释力（持久配置） */
  SERVO_Q_PCSR,    /* 开机恢复力矩（持久配置） */
  SERVO_Q_COUNT
} ServoCodecQuery;

/** 一帧应答「像什么」——只做结构分类，不做业务判断 */
typedef enum {
  SERVO_FRAME_NONE = 0,
  SERVO_FRAME_ID,    /* #003P!       ID 应答 */
  SERVO_FRAME_VALUE, /* #003P1500!   数值应答 */
  SERVO_FRAME_MOD,   /* #003PMOD1!   模式 */
  SERVO_FRAME_VER,   /* #003PV0.97!  版本 */
  SERVO_FRAME_OK,    /* #OK!         通用成功 */
  SERVO_FRAME_TV,    /* #000T25V07!  温度/电压（这条没有 P） */
  SERVO_FRAME_CMD,   /* 像命令回显：body 是字母，如 #003PRAD! */
} ServoFrameKind;

typedef struct {
  uint8_t id;
  ServoFrameKind kind;
  char body[SERVO_CODEC_BODY_MAX + 1]; /* 含结尾 '\0' */
  uint8_t len;
} ServoCodecFrame;

typedef struct {
  uint8_t state;
  uint8_t id;
  uint8_t digits;
  char body[SERVO_CODEC_BODY_MAX + 1];
  uint8_t len;
  bool overflow;
} ServoCodecParser;

/**
 * @brief  生成控制命令
 * @param  out  输出缓冲
 * @param  cap  out 容量；不够返回 0（不写任何字节）
 * @param  pwm  500..2500
 * @param  t_ms 0..9999
 * @retval 写入的字节数（15）
 */
extern uint16_t ServoCodec_BuildMove(char *out, uint16_t cap, uint8_t id, uint16_t pwm,
                                     uint16_t t_ms);

/**
 * @brief  生成查询命令
 * @retval 写入的字节数；容量不够返回 0
 */
extern uint16_t ServoCodec_BuildQuery(char *out, uint16_t cap, uint8_t id, ServoCodecQuery q);

/**
 * @brief  生成改 ID 命令
 * @retval 写入的字节数（11）；容量不够返回 0
 */
extern uint16_t ServoCodec_BuildSetId(char *out, uint16_t cap, uint8_t old_id, uint8_t new_id);

/**
 * @brief  复位解析器（开始等新帧）
 */
extern void ServoCodec_Reset(ServoCodecParser *p);

/**
 * @brief  喂一个字节
 * @param  out 切出完整帧时写入
 * @retval true = 这一字节让一帧完整了
 * @note   一帧最长 3+23 字节；body 超长时整帧丢弃并复位，不会卡死
 */
extern bool ServoCodec_Feed(ServoCodecParser *p, uint8_t byte, ServoCodecFrame *out);

/**
 * @brief  把 VALUE 帧的正文解析成十进制数（body 形如 "P1500"）
 * @retval 数值；非数字返回 0xFFFF
 */
extern uint16_t ServoCodec_FrameValue(const ServoCodecFrame *f);

#endif
