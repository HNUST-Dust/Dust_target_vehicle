#ifndef TASK_IBUS_H
#define TASK_IBUS_H

#include "ibus_decoder.h"
#include <stdint.h>

/* 遥控器通道值（全局，各 task 直接读）*/
extern volatile int16_t  remote_ch0, remote_ch1, remote_ch2, remote_ch3;
extern volatile int16_t  remote_s1, remote_s2;
extern volatile uint8_t  gimbal_enabled;    /* 1=允许云台控制 */
extern volatile uint8_t  chassis_enabled;   /* 1=允许底盘控制 */
extern volatile uint8_t chassis_auto_mode;  /* 1=自动往返模式（s2=1） */
extern volatile float    chassis_vx;        /* 前进 -1~1 */
extern volatile float    chassis_vR;        /* 转向 -1~1 */

void task_ibus_recv(void *argument);

#endif
