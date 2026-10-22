/**
 * @file    app_payload.h
 * @brief   自定义控制器与机器人之间的 30 B / 300 B 载荷
 *
 * 上行（终端 → 机器人，图传 0x0311，30 B）：
 *     0      ver_mode   低 4 位版本=1，高 4 位模式
 *     1      seq
 *     2..13  joints[6]  uint16 LE，主端舵机原始值 500..2500
 *     14..15 grip        uint16 LE
 *     16     buttons    bit0 夹紧 bit1 松开 bit2 急停 bit3 离合
 *     17     flags      bit0 主端有效 bit1 标定 bit2 电池低
 *     18..27 reserved
 *     28..29 crc16（对 0..27）
 *
 * 下行（机器人 → 终端，图传 0x0310，300 B）：
 *     0      ver
 *     1      flags
 *     2..25  6 × int32 pos_urad
 *     26..37 6 × int16 vel_crad（0.01 rad/s）
 *     38..49 6 × int16 torque_cnm（0.01 N·m）
 *     50..55 6 × uint8 temp_c
 *     56..61 6 × uint8 err
 *     62..63 uint16 vbus_mv
 *     64..65 uint16 lost_frames
 *     66..67 crc16（对 0..65）
 *     其余补 0
 */
#ifndef APP_PAYLOAD_H
#define APP_PAYLOAD_H

#include <stdbool.h>
#include <stdint.h>

#define CC_UPLINK_LEN 30u
#define CC_DOWNLINK_LEN 300u
#define CC_JOINTS 6u
#define CC_UPLINK_VER 1u

typedef struct {
  uint8_t ver_mode;
  uint8_t seq;
  uint16_t joints[CC_JOINTS];
  uint16_t grip;
  uint8_t buttons;
  uint8_t flags;
} CcUplink;

typedef struct {
  int32_t pos_urad;
  int16_t vel_crad;
  int16_t torque_cnm;
  uint8_t temp_c;
  uint8_t err;
} CcJointFb;

typedef struct {
  uint8_t ver;
  uint8_t flags;
  CcJointFb joint[CC_JOINTS];
  uint16_t vbus_mv;
  uint16_t lost_frames;
} CcDownlink;

/**
 * @brief  上行打包
 * @param  out 至少 CC_UPLINK_LEN 字节；越界关节值会被夹到 500..2500
 * @note   会写入 CRC16
 */
extern void CcUplink_Encode(uint8_t out[CC_UPLINK_LEN], const CcUplink *p);

/**
 * @brief  上行解包
 * @retval false = 长度/CRC 不对
 */
extern bool CcUplink_Decode(CcUplink *p, const uint8_t in[CC_UPLINK_LEN]);

/**
 * @brief  下行打包（300 B，未用区域补 0）
 */
extern void CcDownlink_Encode(uint8_t out[CC_DOWNLINK_LEN], const CcDownlink *p);

/**
 * @brief  下行解包
 * @retval false = CRC 不对
 */
extern bool CcDownlink_Decode(CcDownlink *p, const uint8_t in[CC_DOWNLINK_LEN]);

#endif
