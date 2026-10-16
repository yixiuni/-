#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/* 初始化电机控制 */
void Motor_Init(void);

/* 设置左右轮输出，范围 -100 到 100
   正数：前进；负数：后退；0：停止
   这是输出百分比，实际速度后续由编码器测量 */
void Motor_SetOutput(int16_t left, int16_t right);

/* 停止两个电机 */
void Motor_Stop(void);
/* AT8236 IN1=IN2=high: both outputs low, electrical braking.
 * Use after speed control has been stopped. Motor_Stop/start releases it. */
void Motor_Brake(void);

#endif