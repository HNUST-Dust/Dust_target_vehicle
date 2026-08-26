#ifndef MOTOR_3508_H
#define MOTOR_3508_H

#include <stdint.h>

#define M3508_ENCODER_PER_ROUND  8192
#define M3508_GEARBOX_RATE       19.0f
#define M3508_MAX_CURRENT        16384

typedef struct {
    int16_t  rpm;
    uint16_t encoder;
} M3508_Feedback_t;

typedef struct {
    int32_t  total_encoder;      /* 多圈累计编码器值 */
    float    angle_rad;          /* 输出轴角度 (rad) */
    float    omega_rads;         /* 输出轴角速度 (rad/s) */
    uint16_t pre_encoder;        /* 上次原始编码器值 */
    int16_t  total_round;        /* 多圈圈数 */
} M3508_Angle_t;

void motor_3508_send_current    (int16_t lf, int16_t rf);
void motor_3508_parse_feedback  (const uint8_t *data, M3508_Feedback_t *fb);
void motor_3508_update_angle    (M3508_Angle_t *angle, const M3508_Feedback_t *fb);

#endif
