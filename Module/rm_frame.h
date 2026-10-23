/**
 * @file    rm_frame.h
 * @brief   RoboMaster 裁判系统串口帧（A5 帧）编解码
 *
 * 帧格式（2026 通信协议 表 1-2）：
 *     A5 | data_length(2,LE) | seq | CRC8 | cmd_id(2,LE) | data | CRC16(2,LE)
 *
 * 校验（官方附录一）：
 *     CRC8  ：多项式 0x31 的反射形式（0x8C），初值 0xFF，覆盖帧头前 4 字节
 *     CRC16 ：CCITT，poly 0x1021，初值 0xFFFF，覆盖整帧（不含末尾 CRC16 本身）
 *
 * 用法：
 *     uint8_t buf[64];
 *     uint16_t n = RmFrame_Encode(buf, sizeof(buf), 0x0311, seq, payload, 30);
 *
 *     RmParser p; RmParser_Reset(&p);
 *     RmFrame f;
 *     for (每个字节 b) if (RmParser_Feed(&p, b, &f)) { f.cmd_id / f.data / f.len }
 *
 * 约束：
 *     数据段最长 64 字节（够 0x0311 的 30 B 和以后扩展）；
 *     CRC 错、长度非法时自动丢弃并从下一个字节重新找帧头，不会卡死。
 */
#ifndef RM_FRAME_H
#define RM_FRAME_H

#include <stdbool.h>
#include <stdint.h>

#define RM_SOF 0xA5u
#define RM_DATA_MAX 300u

typedef struct {
  uint16_t cmd_id;
  uint8_t seq;
  uint16_t len;
  uint8_t data[RM_DATA_MAX];
} RmFrame;

typedef struct {
  uint8_t raw[RM_DATA_MAX + 16u];
  uint16_t n;
} RmParser;

extern uint8_t RmCrc8(const uint8_t *data, uint32_t len);
extern uint16_t RmCrc16(const uint8_t *data, uint32_t len);

/**
 * @brief  组一帧
 * @param  cap 输出缓冲容量；不够返回 0
 * @retval 整帧字节数（9 + len）
 */
extern uint16_t RmFrame_Encode(uint8_t *out, uint16_t cap, uint16_t cmd_id, uint8_t seq,
                               const uint8_t *data, uint16_t len);

/**
 * @brief  复位解析器
 */
extern void RmParser_Reset(RmParser *p);

/**
 * @brief  喂一个字节
 * @retval true = 收到一帧完整且校验通过的帧
 */
extern bool RmParser_Feed(RmParser *p, uint8_t byte, RmFrame *out);

#endif
