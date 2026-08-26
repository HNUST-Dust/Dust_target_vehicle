#ifndef TASK_CAN_RECV_H
#define TASK_CAN_RECV_H

#include "cmsis_os2.h"
#include "motor_3508.h"

/* ── 全局反馈数据（各控制 task 直接读）── */
extern float           gimbal_feedback_pos;     /* 6220 位置反馈 */
extern float           gimbal_feedback_vel;     /* 6220 速度反馈 */
extern int16_t         chassis_rpm_lf;          /* 3508 左轮 RPM */
extern int16_t         chassis_rpm_rf;          /* 3508 右轮 RPM */
extern M3508_Angle_t   chassis_angle_lf;        /* 3508 左轮角度跟踪 */
extern M3508_Angle_t   chassis_angle_rf;

void task_can_recv(void *argument);

#endif
