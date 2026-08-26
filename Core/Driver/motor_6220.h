#ifndef MOTOR_6220_H
#define MOTOR_6220_H

#include <stdint.h>

#define M6220_P_MIN  -12.5f
#define M6220_P_MAX   12.5f
#define M6220_V_MIN  -30.0f
#define M6220_V_MAX   30.0f
#define M6220_T_MIN  -18.0f
#define M6220_T_MAX   18.0f

/* 扭矩映射模式 */
#define M6220_TORQUE_MODE_VELOCITY  0   /* ±5 Nm → 0~4095 */
#define M6220_TORQUE_MODE_POSITION  1   /* ±18 Nm → 0~4095 */

void motor_6220_send_torque (uint8_t id, float torque, uint8_t mode);
void motor_6220_send_enable  (uint8_t id);
void motor_6220_send_disable (uint8_t id);

uint8_t motor_6220_parse_feedback(const uint8_t *data, float *pos, float *vel, float *torque);

#endif