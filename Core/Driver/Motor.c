#include "Motor.h"
#include "motor_6220.h"
#include "motor_3508.h"
#include "motor_zdt.h"

/* ══════════════════════════════════════════════════════════════
 *  Motor_Init — 总初始化入口
 *  启动前使能所有电机，发送初始安全指令
 *  ⚠ 在 MX_FDCAN1_Init() + bsp_fdcan_init() 之后调用
 * ══════════════════════════════════════════════════════════════*/
void Motor_Init(void)
{
    /* ── M3508 底盘电机（初始 0 电流）── */
    motor_3508_send_current(0, 0);

    /* ── ZDT 步进电机 ── */
    motor_zdt_send_enable(1, 1);
    motor_zdt_send_enable(2, 1);
}
