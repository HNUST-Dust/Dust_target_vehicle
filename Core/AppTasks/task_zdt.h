#ifndef TASK_ZDT_H
#define TASK_ZDT_H

#include <stdint.h>

/* ══════════════════════════════════════════════════════════
 *  坐标系：所有位置量统一为「编码器刻度」= 65536/圈
 *  每次上电自动回零（速度模式撞底），位置从 0 开始
 *  仅在发送位置命令时换算成脉冲（3200/圈），见 motor_zdt.c
 * ══════════════════════════════════════════════════════════ */

/* 遥控器映射（由 task_ibus 写入）*/
extern volatile int32_t  zdt_step;        /* 步进增量（编码器刻度）*/
extern volatile uint8_t  zdt_enabled;     /* 1=允许 ZDT 控制 */
extern volatile uint8_t  zdt_homing_req;  /* task_ibus: s1=2 边沿置1, task_zdt: 回零后清0 */

/* 外部可读的 ZDT 状态 */
extern volatile int32_t  zdt_target[2];       /* 目标位置（编码器刻度，绝对坐标）*/
extern volatile int32_t  zdt_real_pos[2];     /* 真实位置（编码器刻度，0x36 反馈+offset）*/
extern volatile int32_t  zdt_abs_offset[2];   /* 驱动器零点对应的物理绝对位置（编码器刻度）*/
extern volatile uint8_t  zdt_motor_pos_valid[2]; /* 0x36 新反馈到达标志 */
extern volatile uint8_t  zdt_is_stalled[2];   /* 堵转标志（CAN 接收任务更新）*/
extern volatile uint8_t  zdt_calibrating[2];  /* 1=正在回零 */

void task_zdt_loop(void *argument);

#endif