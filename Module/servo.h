#ifndef SERVO_H
#define SERVO_H

#include "tim.h"
#include <stdint.h>
// 原版接口以及头文件内容：
//
// ```CPP
// class Servo {
//   Servo(TIM_HandleTypeDef *htim, uint16_t channel, float clock_hz, float max_angle);
//   void start();          // HAL_TIM_PWM_Start
//   void set(float angle); // angle ∈ [0, max_angle]
//   TIM_HandleTypeDef *htim_; uint16_t channel_; const float clock_hz_, max_angle_;
// };
//
// void Servo::set(float angle) {
//   __HAL_TIM_SET_PRESCALER(htim_, clock_hz_ / 1e6f - 1);                        // 预分频到 1 MHz
//   __HAL_TIM_SET_AUTORELOAD(htim_, 20000 - 1);                                  // 50 Hz
//   __HAL_TIM_SET_COMPARE(htim_, channel_, (angle / max_angle_) * 2000 + 500);   // [500, 2500]
// }
// ```
//
// 复刻点：
// 1. 初始化函数 —— 制定定时器和通道以及时钟频率和最大角度限幅
// 2. 开始函数 —— 启动定时器
// 3. 制定角度旋转函数 —— 控制舵机旋转指定角度
// 4. 私有变量记录指定的定时器和通道以及时钟频率和最大角度限幅
//
// 设计点：
// 1. 开启定时器可以直接在初始化函数中实现 —— 模块与用户职责划分点1
// 2. 控制舵机旋转指定角度的接口名改成：Servo_RotateDgree（）—— 接口命名点1
//
// TODO:
// 下面第三条设计点需要重新考虑，因为模块提供了实例接口，需要用户提供舵机的自身最大角度属性，
// 同时保留用户修改硬件配置的权限，因此实例是属于用户的，同时由用户负责管理
//
// 3. 为了支持多实例，模块负责管理实例并拥有实例 —— 模块与用户职责划分点2
// 4. 为了方便用户使用该模块，提供全新的功能实现：模块提供舵机实例接口，
//    用户创建实例后传递给初始化模块，
//    在初始化的时候不需要用户选择定时器和通道，
//    只需要通过实例接口提供最大角度限幅参数和舵机转速以及舵机旋转方向等参数即可。—— 模块功能新增点1
// 5. 为了不让用户关注硬件配置，硬件配置内容直接由模块管理，但是保留用户操作的权限，
//    也就是提供用户修改硬件配置的接口，但是初始化使用模块的固定参数。—— 模块与用户职责划分点3
// TODO:
// 6. 提供连续控制舵机旋转的接口，比方说遥控器控制舵机连续旋转，也就是提供增量控制接口 ——
//    模块功能新增点2
//
// 考虑到舵机在实际使用的时候不需要更换硬件接口，所以在软件上只做了注册并没有对应的注销函数等

// 由于舵机精度比较低，所以单位就一律采用uint8_t
// 默认TIM1和TIM8的PSC是167，CNT的计数值是1999
// 对于舵机来说周期是50hz，产生周期为20ms的pwm波形
struct Tim {
  TIM_HandleTypeDef *handle;
  uint16_t channel;
};

typedef struct Servo {
  float max_angle; // 舵机自身的最大可活动范围（两个边界之间的角度）
  struct Tim tim;
} Servo;

int8_t Servo_Init(Servo *servo);
void Servo_RotateDeg(Servo *servo, float angle);
void Servo_RotateRad(Servo *servo, float rad);
void Servo_Step(Servo *servo, float angle);
void Servo_TIM_Set_PSC(Servo *servo, uint32_t psc);
void Servo_TIM_Set_COMPARE(Servo *servo, uint32_t ccr);
void Servo_TIM_Set_AUTORELOAD(Servo *servo, uint32_t arr);

#endif
