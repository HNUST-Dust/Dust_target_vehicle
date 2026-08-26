#ifndef MOTOR_H
#define MOTOR_H

/* 总初始化：底盘 + 云台 + ZDT 步进电机 */
void Motor_Init(void);

/* 1kHz 定时器控制回调 */
void Motor_Control(void);

#endif
