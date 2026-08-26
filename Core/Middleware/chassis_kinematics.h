#ifndef CHASSIS_KINEMATICS_H
#define CHASSIS_KINEMATICS_H

#include <stdint.h>
#include "pid.h"

#define CHASSIS_MAX_RPM    480.0f
#define CHASSIS_GEARBOX    19.0f
#define CHASSIS_MAX_CURRENT 16384

/* 底盘电机控制上下文 */
typedef struct {
    float lf_target_rpm;       /* 左轮目标 RPM（输出轴） */
    float rf_target_rpm;
    PID_t pid_lf;              /* 左轮速度环 PID */
    PID_t pid_rf;              /* 右轮速度环 PID */
} Chassis_t;

/* 初始化底盘 PID */
void Chassis_Init(Chassis_t *cs);

/* 差速运动学：vx/vR → RPM */
void Chassis_Movement(float vx, float vR, float *out_lf, float *out_rf);

#endif
