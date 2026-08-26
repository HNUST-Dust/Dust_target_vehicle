#include "task_safety.h"
#include "task_ibus.h"
#include "cmsis_os2.h"
#include "cmd_protocol.h"
#include "task_zdt.h"

/*蓝牙模块断联时间值*/
#define BT_TIMEOUT_MS  2000

/* 摇杆中位死区：|ch| 小于它算回中 */
#define STICK_DEADBAND   10

void task_safety_loop(void *argument)
{
    (void)argument;

    while (1)
    {
        uint32_t now = osKernelGetTickCount();

        /* 条件1：蓝牙断开 */
        uint8_t bt_disconnected =
            ((now - cmd_last_rx_tick) > BT_TIMEOUT_MS) ? 1 : 0;

        /* 条件2：遥控器无操作（四摇杆回中 + s2 不在自动往返档）*/
        uint8_t stick_centered =
            (remote_ch0 > -STICK_DEADBAND && remote_ch0 < STICK_DEADBAND) &&
            (remote_ch1 > -STICK_DEADBAND && remote_ch1 < STICK_DEADBAND) &&
            (remote_ch2 > -STICK_DEADBAND && remote_ch2 < STICK_DEADBAND) &&
            (remote_ch3 > -STICK_DEADBAND && remote_ch3 < STICK_DEADBAND);

        uint8_t remote_no_op = stick_centered && (remote_s2 != 1);

        if (bt_disconnected && remote_no_op)
        {
            /* 触发保护：所有电机速度归零 */
            cmd_chassis_mode     = 0;
            cmd_gimbal_mode      = 0;
            cmd_gimbal_run       = 0;
            cmd_gimbal_speed     = 0;
            cmd_gimbal_speed_max = 0;
            cmd_armor_motor      = 0;

            chassis_enabled      = 0;
            chassis_auto_mode    = 0;
            chassis_vx           = 0.0f;
            chassis_vR           = 0.0f;

            gimbal_enabled       = 0;

            zdt_enabled          = 0;
            zdt_step             = 0;
        }
        /* 条件不满足 → 什么都不做，自动恢复 */

        osDelay(50);   /* 50ms 检查一次 */
    }
}