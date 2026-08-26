#include <stdio.h>

#include "task_chassis.h"
#include "task_ibus.h"
#include "task_can_recv.h"
#include "chassis_kinematics.h"
#include "motor_3508.h"
#include "cmsis_os2.h"
#include "cmd_protocol.h"
#include "usart.h"


/* ══════════════════════════════════════════════════════════════
 *  自动往返参数（s2=1 触发，修改数值即可调整行为）
 * ══════════════════════════════════════════════════════════════ */
//#define AUTO_DISTANCE_M         3.0f      /* 单程距离（米）          */         */
//#define AUTO_OUTPUT_RPM         100.0f    /* 行驶速度（轮子 RPM）     */
#define WHEEL_RADIUS_M     0.125f    /* 轮子半径（米）*/
#define GEARBOX_RATIO            19.0f     /* 减速比 */
#define ENCODER_PER_REV          8192.0f   /* 编码器每圈脉冲数 */
#define SWOOTH_TAU  0.15f    /* 速度滤波时间常数（秒） */

#define CHASSIS_VX_MAX_SLEW  2.0f      /* 正常行驶限幅斜率：满幅换向约 1s */
#define CHASSIS_VX_DEADBAND  0.05f     /* 方向判断死区（与摇杆死区一致） */
#define CHASSIS_VX_BRAKE_SLEW 5.0f     /* 换向时快速减速到 0 的斜率 */
#define CHASSIS_BRAKE_HOLD_S 0.3f      /* 换向驻车时间（秒） */
#define CHASSIS_VX_ENTRY_SLEW 1.0f     /* 换向后慢速爬升斜率（越小越缓） */
#define AUTO_BRAKE_TIME_S  0.15f       /* 自动往返换向：快速减速到 0 所需时间(秒) */
#define AUTO_HOLD_S        0.3f        /* 自动往返换向：驻车时间(秒) */
#define AUTO_ENTRY_TIME_S  0.8f        /* 自动往返换向：慢速爬升进入新方向的时间(秒) */
#define AUTO_RPM_STOP_EPS  40.0f       /* 判定"已到 0 速"的 RPM 阈值 */


/* ── 自动往返状态（前进 → 后退 → 前进 → ...，无停顿）── */
typedef enum {
    AUTO_IDLE = 0,      /* 未激活 */
    AUTO_FORWARD,       /* 前进中 */
    AUTO_BACKWARD,      /* 后退中 */
} auto_state_t;

/* ── 前后方向切换状态机 ── */
typedef enum {
    VX_DRIVE = 0,   /* 正常行驶：vx_slewed 跟随 chassis_vx */
    VX_BRAKE,       /* 检测到换向：把 vx_slewed 快速减到 0 */
    VX_HOLD,        /* 已到 0：驻车一小会儿，让从动轮卸载 */
    VX_ENTRY,       /* 换向后：慢速爬升进入新方向，减小从动轮翻转的偏航 */
} vx_dir_state_t;


static Chassis_t chassis;

/* ══════════════════════════════════════════════════════════════
 *  UART7(蓝牙) 上报：左/右轮 目标 RPM vs 实际 RPM
 *  1Hz 低频打印，便于阅读；工程未开启浮点格式化，转速按整数显示。
 * ══════════════════════════════════════════════════════════════ */
static void chassis_report_uart7(float lf_tgt, float rf_tgt,
                                 int16_t lf_act, int16_t rf_act)
{
    char buf[64];
    int len = snprintf(buf, sizeof(buf),
                       "L tgt:%d act:%d   R tgt:%d act:%d\r\n",
                       (int)lf_tgt, lf_act,
                       (int)rf_tgt, rf_act);
    if (len > 0)
        HAL_UART_Transmit(&huart7, (uint8_t *)buf, (uint16_t)len, 100);
}

void task_chassis_loop(void *argument)
{
    (void)argument;
    Chassis_Init(&chassis);

    auto_state_t auto_state     = AUTO_IDLE;
    int32_t      auto_start_enc = 0;

    uint32_t last_tick = osKernelGetTickCount();

    float smooth1_lf = 0.0f,  smooth2_lf = 0.0f;
    float smooth1_rf = 0.0f,  smooth2_rf = 0.0f;
    float vx_slewed = 0.0f;  /*限斜率后的前进命令*/
    vx_dir_state_t vx_dir_state = VX_DRIVE;
    float vx_last_cmd = 0.0f;        /* 上一次的原始指令方向 */
    uint32_t vx_hold_start = 0;      /* 进入驻车的时刻 */
    float auto_target_cur = 0.0f;      /* 自动往返：方向整形后的目标 RPM */
    vx_dir_state_t auto_dir_state = VX_DRIVE;
    float auto_last_dir = 0.0f;        /* 上一次自动指令方向 */
    uint32_t auto_hold_start = 0;      /* 进入自动驻车的时刻 */

    static uint32_t last_report_tick = 0;    /* UART7 上报节拍 */


    while (1)
    {
        uint32_t now = osKernelGetTickCount();
        float dt = (now - last_tick) * 0.001f;
        if (dt <= 0.0f || dt > 0.05f) dt = 0.001f;
        last_tick = now;

        float alpha = dt / (SWOOTH_TAU + dt);

        /* ── 1Hz 经 UART7(蓝牙) 上报 左/右轮 目标 vs 实际 RPM ── */
        if ((now - last_report_tick) >= 1000) {
            last_report_tick = now;
            chassis_report_uart7(smooth2_lf, smooth2_rf,
                                 chassis_rpm_lf, chassis_rpm_rf);
        }

        /* ── 底盘未使能 → 关电机 + 重置自动状态 ── */
        int run_auto = chassis_enabled ? chassis_auto_mode : cmd_chassis_mode;

        if (!chassis_enabled && !cmd_chassis_mode) {
            motor_3508_send_current(0, 0);
            auto_state = AUTO_IDLE;
            smooth1_lf = smooth2_lf = 0.0f;
            smooth1_rf = smooth2_rf = 0.0f;
            vx_slewed = 0.0f;
            vx_dir_state = VX_DRIVE;
            vx_last_cmd  = 0.0f;
            auto_target_cur = 0.0f;
            auto_dir_state  = VX_DRIVE;
            auto_last_dir   = 0.0f;

            osDelay(1);
            continue;
        }

        /* ══════════════════════════════════════════════════════
         *  自动往返模式（s2=1）
         * ══════════════════════════════════════════════════════ */
        if (run_auto) {

            /* 刚进入 → 记录起点编码器 */
            if (auto_state == AUTO_IDLE) {
                auto_start_enc = chassis_angle_lf.total_encoder;
                auto_state = AUTO_FORWARD;
            }

             /* 运行时计算（每次循环都算，保证上位机改参数立刻生效）*/
            float motor_rpm     = cmd_chassis_speed * GEARBOX_RATIO;
            float wheel_circ_m  = 2.0f * 3.14159265f * WHEEL_RADIUS_M;
            float encoder_target = (cmd_chassis_dist_m / wheel_circ_m)
                                   * GEARBOX_RATIO * ENCODER_PER_REV;
            /* 根据状态计算目标 RPM */
            float target_rpm;
            switch (auto_state) {

            case AUTO_FORWARD:
                target_rpm = motor_rpm;
                if ((chassis_angle_lf.total_encoder - auto_start_enc)
                        >= (int32_t)encoder_target) {
                    auto_start_enc = chassis_angle_lf.total_encoder;
                    auto_state = AUTO_BACKWARD;
                }
                break;

            case AUTO_BACKWARD:
                target_rpm = -motor_rpm;
                if ((auto_start_enc - chassis_angle_lf.total_encoder)
                        >= (int32_t)encoder_target) {
                    auto_start_enc = chassis_angle_lf.total_encoder;
                    auto_state = AUTO_FORWARD;
                }
                break;

            default:
                target_rpm = 0.0f;
                break;
            }

            /* 二阶低通平滑目标速度（S 曲线软启动） */
            //smooth1_lf += alpha * (target_rpm - smooth1_lf);
            //smooth2_lf += alpha * (smooth1_lf - smooth2_lf);
            //smooth1_rf += alpha * (-target_rpm - smooth1_rf);
            //smooth2_rf += alpha * (smooth1_rf - smooth2_rf);
                        /* ── 自动往返：换向时也做 刹车→驻车→慢速爬升 ── */
            {
                float a_dir = (target_rpm > 0.0f) ? 1.0f :
                              ((target_rpm < 0.0f) ? -1.0f : 0.0f);

                /* 检测 FORWARD↔BACKWARD 换向 → 进入刹车 */
                if (a_dir != 0.0f && a_dir != auto_last_dir) {
                    auto_dir_state = VX_BRAKE;
                    auto_last_dir  = a_dir;
                }

                float a_step, a_delta;
                switch (auto_dir_state) {

                case VX_DRIVE:
                    auto_target_cur = target_rpm;   /* 正常行驶：直接跟随 */
                    break;

                case VX_BRAKE:
                    a_step  = (motor_rpm / AUTO_BRAKE_TIME_S) * dt;
                    a_delta = 0.0f - auto_target_cur;
                    if (a_delta >  a_step) a_delta =  a_step;
                    if (a_delta < -a_step) a_delta = -a_step;
                    auto_target_cur += a_delta;
                    if (auto_target_cur > -AUTO_RPM_STOP_EPS &&
                        auto_target_cur <  AUTO_RPM_STOP_EPS) {
                        auto_target_cur = 0.0f;
                        auto_dir_state  = VX_HOLD;
                        auto_hold_start = now;
                    }
                    break;

                case VX_HOLD:
                    auto_target_cur = 0.0f;
                    if ((float)(now - auto_hold_start) * 0.001f >= AUTO_HOLD_S) {
                        auto_dir_state = VX_ENTRY;
                    }
                    break;

                case VX_ENTRY:
                    a_step  = (motor_rpm / AUTO_ENTRY_TIME_S) * dt;
                    a_delta = target_rpm - auto_target_cur;
                    if (a_delta >  a_step) a_delta =  a_step;
                    if (a_delta < -a_step) a_delta = -a_step;
                    auto_target_cur += a_delta;
                    if (target_rpm - auto_target_cur > -AUTO_RPM_STOP_EPS &&
                        target_rpm - auto_target_cur <  AUTO_RPM_STOP_EPS) {
                        auto_dir_state = VX_DRIVE;
                    }
                    break;
                }
            }

            /* 二阶低通平滑目标速度（S 曲线软启动） */
            smooth1_lf += alpha * (auto_target_cur - smooth1_lf);
            smooth2_lf += alpha * (smooth1_lf - smooth2_lf);
            smooth1_rf += alpha * (-auto_target_cur - smooth1_rf);
            smooth2_rf += alpha * (smooth1_rf - smooth2_rf);


            /* PID 闭环 + 发电流 */
            float lf_i = PID_Calculate(&chassis.pid_lf, smooth2_lf,
                                       (float)chassis_rpm_lf, dt);
            float rf_i = PID_Calculate(&chassis.pid_rf, smooth2_rf,
                                       (float)chassis_rpm_rf, dt);

            if (lf_i >  CHASSIS_MAX_CURRENT) lf_i =  CHASSIS_MAX_CURRENT;
            if (lf_i < -CHASSIS_MAX_CURRENT) lf_i = -CHASSIS_MAX_CURRENT;
            if (rf_i >  CHASSIS_MAX_CURRENT) rf_i =  CHASSIS_MAX_CURRENT;
            if (rf_i < -CHASSIS_MAX_CURRENT) rf_i = -CHASSIS_MAX_CURRENT;

            motor_3508_send_current((int16_t)lf_i, (int16_t)rf_i);

            osDelay(1);
            continue;    
        }

        /* ══════════════════════════════════════════════════════
         *  摇杆手动模式（s2=2）
         * ══════════════════════════════════════════════════════ */


         /* ── 方向切换检测：换向时快速减速到 0 + 驻车 ── */
        {
            float cmd_dir = 0.0f;
            if (chassis_vx >  CHASSIS_VX_DEADBAND) cmd_dir =  1.0f;
            else if (chassis_vx < -CHASSIS_VX_DEADBAND) cmd_dir = -1.0f;

            /* 检测到方向反转（换向）→ 进入刹车 */
            if (cmd_dir != 0.0f && cmd_dir != vx_last_cmd) {
                vx_dir_state = VX_BRAKE;
                vx_last_cmd  = cmd_dir;
            }

            float vx_step, delta;

            switch (vx_dir_state) {

            case VX_DRIVE:
                /* 正常行驶：vx_slewed 以 CHASSIS_VX_MAX_SLEW 跟随 chassis_vx */
                vx_step = CHASSIS_VX_MAX_SLEW * dt;
                delta   = chassis_vx - vx_slewed;
                if (delta >  vx_step) delta =  vx_step;
                if (delta < -vx_step) delta = -vx_step;
                vx_slewed += delta;
                break;

            case VX_BRAKE:
                /* 快速减速回 0 */
                vx_step = CHASSIS_VX_BRAKE_SLEW * dt;
                delta   = 0.0f - vx_slewed;
                if (delta >  vx_step) delta =  vx_step;
                if (delta < -vx_step) delta = -vx_step;
                vx_slewed += delta;
                if (vx_slewed > -CHASSIS_VX_DEADBAND && vx_slewed < CHASSIS_VX_DEADBAND) {
                    vx_slewed = 0.0f;
                    vx_dir_state  = VX_HOLD;
                    vx_hold_start = now;
                }
                break;

            case VX_HOLD:
                /* 驻车一小会儿：让从动轮卸载、车身稳定，再进入新方向 */
                vx_slewed = 0.0f;
                if ((float)(now - vx_hold_start) * 0.001f >= CHASSIS_BRAKE_HOLD_S) {
                    vx_dir_state = VX_ENTRY;
                }
                break;

            case VX_ENTRY:
                /* 换向后慢速爬升进入新方向，减小从动轮翻转产生的偏航 */
                vx_step = CHASSIS_VX_ENTRY_SLEW * dt;
                delta   = chassis_vx - vx_slewed;
                if (delta >  vx_step) delta =  vx_step;
                if (delta < -vx_step) delta = -vx_step;
                vx_slewed += delta;
                if (chassis_vx - vx_slewed > -CHASSIS_VX_DEADBAND &&
                    chassis_vx - vx_slewed <  CHASSIS_VX_DEADBAND) {
                    vx_dir_state = VX_DRIVE;   /* 已追上新方向，恢复正常行驶 */
                }
                break;
            }
        }

        /* 运动学：vx/vR → RPM */
        Chassis_Movement(vx_slewed, chassis_vR,
                         &chassis.lf_target_rpm, &chassis.rf_target_rpm);

        float lf_motor = chassis.lf_target_rpm * CHASSIS_GEARBOX;
        float rf_motor = chassis.rf_target_rpm * CHASSIS_GEARBOX;

        /* 二阶低通平滑目标速度（S 曲线软启动） */
            smooth1_lf += alpha * (lf_motor - smooth1_lf);
            smooth2_lf += alpha * (smooth1_lf - smooth2_lf);
            smooth1_rf += alpha * (rf_motor - smooth1_rf);
            smooth2_rf += alpha * (smooth1_rf - smooth2_rf);

        float lf_i = PID_Calculate(&chassis.pid_lf, smooth2_lf,
                                   (float)chassis_rpm_lf, dt);
        float rf_i = PID_Calculate(&chassis.pid_rf, smooth2_rf,
                                   (float)chassis_rpm_rf, dt);

        if (lf_i >  CHASSIS_MAX_CURRENT) lf_i =  CHASSIS_MAX_CURRENT;
        if (lf_i < -CHASSIS_MAX_CURRENT) lf_i = -CHASSIS_MAX_CURRENT;
        if (rf_i >  CHASSIS_MAX_CURRENT) rf_i =  CHASSIS_MAX_CURRENT;
        if (rf_i < -CHASSIS_MAX_CURRENT) rf_i = -CHASSIS_MAX_CURRENT;

        motor_3508_send_current((int16_t)lf_i, (int16_t)rf_i);

        osDelay(1);
    }
}
