/**
 * @file    master_app.h
 * @brief   主端（自定义控制器）应用：6 轴舵机采样 → 0x0302 帧 → PC
 */
#ifndef MASTER_APP_H
#define MASTER_APP_H

void Master_Setup(void);
void Master_Loop(void);

#endif
