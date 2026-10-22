#include "servo.h"
#include <math.h>

static float angle2pwm(const Servo *servo, float angle);
static float angle2rad(const float angle);
static float rad2angle(const float rad);
static float rad2pwm(const Servo *servo, float rad);

#define SERVO_MAX_TIMS 7

union float2bits {
  float f;
  uint32_t b;
};

struct Tim tims[SERVO_MAX_TIMS] = {
    {&htim1, TIM_CHANNEL_1}, {&htim1, TIM_CHANNEL_2}, {&htim1, TIM_CHANNEL_3},
    {&htim1, TIM_CHANNEL_4}, {&htim8, TIM_CHANNEL_1}, {&htim8, TIM_CHANNEL_2},
    {&htim8, TIM_CHANNEL_3},
};
static uint8_t used_tims_cnt = 0;

static float psc = 0;
static float arr = 0;
static float ccr = 0;

int8_t Servo_Init(Servo *servo) {
  if (servo == NULL) {
    return 0;
  }

  // 防止重复注册
  for (int i = 0; i < used_tims_cnt; i++) {
    if (servo->tim.handle == tims[i].handle) {
      return 0;
    }
  }

  // 顺序注册并分配硬件
  used_tims_cnt++;
  uint8_t idx = used_tims_cnt - 1;
  servo->tim = tims[idx];

  HAL_TIM_PWM_Start(servo->tim.handle, servo->tim.channel);

  return 1;
}

// TODO: 目前旋转函数都是静默失败的，用户看不到程序也检测不到，看看有没有对应的需求
void Servo_RotateDeg(Servo *servo, float angle) {
  if (servo == NULL) {
    return;
  }
  if (angle < 0 || angle > servo->max_angle) {
    return;
  }
  float pwm = angle2pwm(servo, angle);
  if (pwm == 0) {
    return;
  }
  __HAL_TIM_SET_COMPARE(servo->tim.handle, servo->tim.channel, pwm);
}

void Servo_RotateRad(Servo *servo, float rad) {
  if (servo == NULL) {
    return;
  }
  float max_rad = angle2rad(servo->max_angle);
  if (rad < 0 || rad > max_rad) {
    return;
  }
  float pwm = rad2pwm(servo, rad);
  if (pwm == 0) {
    return;
  }
  __HAL_TIM_SET_COMPARE(servo->tim.handle, servo->tim.channel, pwm);
}

// void Servo_Step(Servo *servo, float angle, float step) {
//   if (servo == NULL) {
//     return;
//   }
//   if (angle < 0 || angle > servo->max_angle) {
//     return;
//   }
//
// }

// TODO:
// 在SERVO模块层设置psc的语义就不再是他本身了，那是HAL层也就是硬件抽象层的说法，
// 包括下面的设置COMPARE，设置AUTORELOAD ———— 语义重定义
//
// ccr的数值决定了pwm的占空比，arr的数值决定了pwm的脉宽和周期，脉宽对应舵机的旋转角度。
// 关于周期：舵机对pwm周期识别是10～30ms
// psc的数值间接的影响pwm周期，或者说决定了单片机识别的分辨率，最大可活动范围映射成完整的最大计数值，
// 也就是说一个计数单位就对应了 360 / (168e10 / psc) 的角度值

// 设置分辨率
void Servo_Set_Resolution(Servo *servo, float resolution) {
  psc = 168e6f / resolution - 1;
  __HAL_TIM_SET_PRESCALER(servo->tim.handle, psc);
}

void Servo_TIM_Set_PSC(Servo *servo, uint32_t psc) {
  __HAL_TIM_SET_PRESCALER(servo->tim.handle, psc);
}

// 设置周期
void Servo_Set_Cycle(Servo *servo, float cycle) {
  arr = 168e6f / (psc + 1) * cycle - 1;
  __HAL_TIM_SET_AUTORELOAD(servo->tim.handle, arr);
}

void Servo_TIM_Set_COMPARE(Servo *servo, uint32_t ccr) {
  __HAL_TIM_SET_COMPARE(servo->tim.handle, servo->tim.channel, ccr);
}

// 设置占空比
void Servo_Set_Duty(Servo *servo, float duty) {
  ccr = duty * (arr + 1) - 1;
  __HAL_TIM_SET_COMPARE(servo->tim.handle, servo->tim.channel, ccr);
}

void Servo_TIM_Set_AUTORELOAD(Servo *servo, uint32_t arr) {
  __HAL_TIM_SET_AUTORELOAD(servo->tim.handle, arr);
}

// 返回值为0表示发生错误
static float angle2pwm(const Servo *servo, float angle) {
  if (servo == NULL) {
    return 0;
  }
  if (angle < 0 || angle > servo->max_angle) {
    return 0;
  }
  return (angle / servo->max_angle) * 2000.0f + 500.0f;
}

static float rad2pwm(const Servo *servo, float rad) {
  if (servo == NULL) {
    return 0;
  }
  float max_rad = angle2rad(servo->max_angle);
  if (rad < 0 || rad > max_rad) {
    return 0;
  }
  return (rad / max_rad) * 2000.0f + 500.0f;
}

static float angle2rad(const float angle) { return (angle / 180.0f) * M_PI; }
static float rad2angle(const float rad) { return (rad / M_PI) * 180.0f; }
