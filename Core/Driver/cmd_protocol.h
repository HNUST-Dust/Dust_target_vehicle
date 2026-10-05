#ifndef CMD_PROTOCOL_H
#define CMD_PROTOCOL_H

#include <stdint.h>

/* ══════════════════════════════════════════════════════════════
 *  上位机 ↔ STM32 串口命令协议 (UART7, 115200 8N1)
 *
 *  帧格式 (固定 8 字节):
 *    [0xA5] [CMD] [D0] [D1] [D2] [D3] [D4] [XOR]
 *     ↑       ↑     └──── 5字节数据 ────┘    ↑
 *    帧头    命令    含义取决于 CMD         校验 = 前7字节 XOR
 *
 *  怎么新增命令？
 *    1. 在下面加 #define CMD_xxx 0x40
 *    2. 在 task_cmd_uart.c 的 switch 里加 case
 *    3. 在 HTML 的 CONFIG 里加对应按钮
 * ══════════════════════════════════════════════════════════════ */

 #define CMD_FRAME_HEADER   0xA5  /*帧头*/
 #define CMD_FRAME_SIZE    8     /*帧长*/

/* ── 命令字 ── */
 #define CMD_CHASSIS_CFG   0x10    /*底盘参数配置  D0:距离（0.1m） D1:速度（rpm） D2:加速度 */
 #define CMD_CHASSIS_GO    0x11    /*底盘运动控制  D0=0停/1动 */ 
 #define CMD_GIMBAL_CFG    0x20    /*云台参数配置  D0:模式(1固定/2区间) D1:速度1 D2:速度2(区间最大) D3:加速度  */
 #define CMD_GIMBAL_GO     0x21    /* 云台执行: D0=0停/1启动 */
 #define CMD_ARMOR_MOVE    0x30
 #define CMD_ALL_STOP      0xFF    /* 紧急停止 */
 #define CMD_HEARTBEAT     0x01    /* 心跳 */




/*—— 全局控制变量（HTML 下发，电机任务读取）── */
extern volatile uint8_t  cmd_chassis_mode;    /* 0=停, 1=往返 */
extern volatile float    cmd_chassis_dist_m;   /* 单程距离(米) */
extern volatile float    cmd_chassis_speed;    /* 速度(输出轴RPM) */
extern volatile uint8_t  cmd_chassis_accel;    /* 加速度 */
extern volatile uint8_t  cmd_gimbal_mode;      /* 0=停 1=固定速度 2=区间变速 */
extern volatile float    cmd_gimbal_speed;     /* 固定速度值 / 区间最低速度 */
extern volatile float    cmd_gimbal_speed_max; /* 区间最高速度 */
extern volatile uint8_t  cmd_gimbal_accel;     /* 加速度(0=无限制, 越大加速越快) */
extern volatile uint8_t  cmd_gimbal_run;       /* 执行标志 */
extern volatile uint8_t  cmd_armor_motor;      /* 电机号 */
extern volatile uint8_t  cmd_armor_accel;      /* 加速度 */
extern volatile uint32_t  cmd_last_rx_tick;    /* 上次接收的有效帧 */

void CMD_ParseFrame(const uint8_t *frame);

#endif