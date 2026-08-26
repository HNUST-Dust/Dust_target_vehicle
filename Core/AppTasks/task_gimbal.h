#ifndef TASK_GIMBAL_H
#define TASK_GIMBAL_H

void task_gimbal_loop(void *argument);

/* 随机云台速度目标：方向跟随 dir 符号，幅度 [10,30] */
float generate_random_velocity(float dir);

#endif
