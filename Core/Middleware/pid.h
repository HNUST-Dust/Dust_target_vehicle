#ifndef PID_H
#define PID_H

#include <stdint.h>

/* ── 基本 PID 控制器 ── */
typedef struct {
    float kp, ki, kd;
    float integral;
    float err_last;
    float out_max, out_min;
} PID_t;

float PID_Calculate(PID_t *pid, float target, float measure, float dt);
void  PID_Reset(PID_t *pid);
void  PID_SetParam(PID_t *pid, float kp, float ki, float kd);
void  PID_SetLimit(PID_t *pid, float max, float min);

/* ── 一阶低通滤波器 ── */
typedef struct {
    float alpha;    /* 滤波系数 0~1, 越小越平滑 */
    float output;
} LPF_t;

float LPF_Update(LPF_t *lpf, float input);

/* ── 6220 级联位置→速度控制器 ── */
typedef struct {
    /* 位置环参数 */
    float p_kp, p_kd;
    /* 速度环参数 */
    float v_kp, v_ki, v_kd;
    /* 速度前馈 */
    float ff;

    /* 内部状态 */
    float pos_target;       /* 位置目标 */
    float vel_target;       /* 速度目标（位置环输出） */
    PID_t pos_pid;          /* 位置环 PID */
    PID_t vel_pid;          /* 速度环 PID */
    LPF_t vel_lpf;          /* 速度测量值滤波 */
    float pos_last;         /* 上次位置（微分用） */
    float vel_integral;     /* 速度环积分 */
    uint8_t pos_div_cnt;    /* 位置环分频计数 */
} CascadedPID_t;

/* 配置级联控制器参数 */
void  CascadedPID_Config(CascadedPID_t *cp,
                         float p_kp, float p_kd,
                         float v_kp, float v_ki, float v_kd,
                         float v_lpf_alpha);

/* 速度模式：只跑速度环 */
float CascadedPID_UpdateVel(CascadedPID_t *cp, float target_vel,
                            float measure_vel, float dt);

/* 位置模式：位置环→速度环级联 */
float CascadedPID_UpdatePos(CascadedPID_t *cp, float target_pos,
                            float measure_pos, float measure_vel, float dt);

/* 清除积分 */
void  CascadedPID_Clear(CascadedPID_t *cp);

/* 摩擦力前馈 (±0.05 Nm) */
float FF_Friction(float target_vel);

/* 随机速度生成 (测试用, 范围 [-30,-10] ∪ [10,30] rad/s) */
float GenerateRandomVel(unsigned int *seed);

#endif