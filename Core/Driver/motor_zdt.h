#ifndef MOTOR_ZDT_H
#define MOTOR_ZDT_H

#include <stdint.h>

/* ── 编码器刻度（65536/圈）作为内部唯一坐标系 ──
 *  0x36 位置反馈是编码器原始计数，1 圈 = 65536
 *  位置命令是脉冲域，1 圈 = 3200（16 细分）
 *  换算：编码器刻度 × 3200 / 65536 = 脉冲，即 × 25 >> 9 */
#define ZDT_ENCODER_PER_REV   65536
#define ZDT_PULSE_PER_REV     3200

/* 软限位（编码器刻度，= 56000 脉冲 × 65536/3200）*/
#define ZDT_POS_MAX   1146880
#define ZDT_POS_MIN   0

/* ── 统一反馈解析结果 ── */
typedef struct {
    uint8_t cmd;            /* 功能码：0x31 / 0x36 / 0x3A / 0x3B */
    int32_t position;       /* 0x36 电机内部多圈位置（编码器刻度）*/
    uint16_t encoder;       /* 0x31 物理绝对编码器（0~65535）*/
    uint8_t dir;            /* 0x36 方向 */
    uint8_t is_stalled;     /* 0x3A 堵转标志 */
    uint8_t homing_status;  /* 0x3B 回零状态 */
} ZDT_Feedback_t;

/* 位置命令：target 为编码器刻度，内部换算成脉冲发送 */
void motor_zdt_send_position       (uint8_t id, int32_t target);
void motor_zdt_send_velocity       (uint8_t id, int32_t speed, uint16_t acc);
void motor_zdt_send_enable         (uint8_t id, uint8_t state);
void motor_zdt_send_request_pos    (uint8_t id);              /* 0x36 */
void motor_zdt_send_zero_position  (uint8_t id);              /* 0x0A 6D 清零位置 */
void motor_zdt_send_request_encoder_raw (uint8_t id);         /* 0x31 */
void motor_zdt_send_homing_trigger (uint8_t id);
void motor_zdt_send_homing_status  (uint8_t id);
void motor_zdt_send_force_stop     (uint8_t id);
void motor_zdt_send_release_stall  (uint8_t id);
void motor_zdt_send_request_status (uint8_t id);              /* 0x3A */

/* 统一反馈解析：自动识别 cmd 并填充对应字段 */
void motor_zdt_parse_feedback(const uint8_t *data, ZDT_Feedback_t *fb);

#endif