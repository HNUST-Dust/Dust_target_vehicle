#include "task_gimbal.h"
#include "task_ibus.h"
#include "task_can_recv.h"
#include "cmd_protocol.h"
#include "motor_6220.h"
#include "pid.h"
#include "cmsis_os2.h"

#define CH3_VEL_SCALE  (30.0f / 660.0f)
/* 速度归零后保持 500ms 再切位置模式 */
#define VEL_ZERO_THRESHOLD  500

/* HTML 云台命令：uint8_t(0~255) → 6220 速度 rad/s（最大值对齐电机上限 ±45） */
#define CMD_VEL_SCALE   (30.0f / 255.0f)

static CascadedPID_t gimbal;

/* ══════════════════════════════════════════════════════════════
 *  generate_random_velocity — 生成指定方向的随机云台速度
 *
 *  方向跟随输入 dir 的符号，只随机幅度（10~30）
 *    dir >= 0 → 返回 +[10, 30]
 *    dir <  0 → 返回 -[30, 10]
 *  seed 为函数内部 static，调用只需传方向：
 *    float vel = generate_random_velocity(+1.0f);
 * ══════════════════════════════════════════════════════════════ */

void task_gimbal_loop(void *argument)
{
    (void)argument;

    CascadedPID_Config(&gimbal,
        8.0f, 0.2f,          /* p_kp, p_kd(位置阻尼) */
        1.5f, 0.08f, 0.08f,  /* v_kp, v_ki, v_kd(速度阻尼) */
        0.35f);               /* 速度滤波系数(0.6→0.4，反馈更快减滞后) */

    uint32_t last_tick = osKernelGetTickCount();
    uint16_t vel_zero_cnt = 0;

    uint16_t stop_cnt = 0;
    float stop_hold_pos = 0.0f;

    /* ── HTML 模式状态 ── */
    float    cmd_ramped_vel  = 0.0f;   /* 经加速度斜坡后的当前目标速度 */
    float    cmd_interval_t  = 0.0f;   /* 区间变速的时间相位 */

    /* 6220 上电需 1.5s 后才响应使能命令，不能立即发 */
    osDelay(1500);
    motor_6220_send_enable(0x01);

    /* 6220 是"发一帧命令回一帧反馈"型，不会自动持续反馈。
     * 初始化不等待反馈，直接进控制循环 — 每 1ms 发一次命令，
     * 反馈自然随之而来，速度观测器在循环中收敛。 */
    motor_6220_send_torque(0x01, 0.0f, M6220_TORQUE_MODE_VELOCITY);

    while(1)
    {
        /* dt */
        uint32_t now = osKernelGetTickCount();
        float dt = (now - last_tick) * 0.001f;
        if (dt <= 0.0f || dt > 0.1f) dt = 0.001f;
        last_tick = now;

        /* ══════════════════════════════════════════════════════
         *  优先级：遥控器 > HTML 上位机 > 停止
         *
         *  gimbal_enabled   : 遥控器 s1=3 置 1
         *  cmd_gimbal_run   : HTML 上位机「启动」按钮置 1
         *
         *  两者同时有效时，遥控器优先，不会冲突。
         * ══════════════════════════════════════════════════════ */
        if (!gimbal_enabled && !cmd_gimbal_run) {
            /* ── 两者都关 → 停止 ── */
            cmd_ramped_vel = 0.0f;
            cmd_interval_t = 0.0f;

            if(stop_cnt < VEL_ZERO_THRESHOLD) 
            {
              /* 阶段1：500ms 内速度自然归零（只跑速度环，无位置突变）*/
                stop_cnt++;
                float torque = CascadedPID_UpdateVel(&gimbal, 0.0f,
                                                     gimbal_feedback_vel, dt);
                motor_6220_send_torque(0x01, torque, M6220_TORQUE_MODE_VELOCITY);
            } else {
                /* 阶段2：锁定当前位置 */
                if (stop_cnt == VEL_ZERO_THRESHOLD) {
                    stop_hold_pos = gimbal_feedback_pos;  /* 只捕获一次 */
                    stop_cnt++;
                }
                float torque = CascadedPID_UpdatePos(&gimbal, stop_hold_pos,
                                                     gimbal_feedback_pos,
                                                     gimbal_feedback_vel, dt);
                motor_6220_send_torque(0x01, torque, M6220_TORQUE_MODE_POSITION);
            }
        }
           


            /*
            CascadedPID_Clear(&gimbal);
            vel_zero_cnt   = 0;
            cmd_ramped_vel = 0.0f;
            cmd_interval_t = 0.0f;
            motor_6220_send_torque(0x01, 0.0f, M6220_TORQUE_MODE_VELOCITY);
            */
        
        else if (gimbal_enabled) {
            /* ── 遥控器模式（原逻辑不变）── */
            cmd_gimbal_run = 0 ;
            cmd_ramped_vel = 0.0f;   /* 退出 HTML 模式时清斜坡 */
            cmd_interval_t = 0.0f;
            stop_cnt = 0;

            float vel = (float)remote_ch3 * CH3_VEL_SCALE;
            if (vel > -1.0f && vel < 1.0f) vel = 0.0f;

            if (vel == 0.0f) {
                /* 速度归零：计数超过阈值才切位置保持 */
                if (vel_zero_cnt < VEL_ZERO_THRESHOLD) {
                    vel_zero_cnt++;
                    float torque = CascadedPID_UpdateVel(&gimbal, 0.0f,
                                                         gimbal_feedback_vel, dt);
                    motor_6220_send_torque(0x01, torque, M6220_TORQUE_MODE_VELOCITY);
                } else {
                    /* 进入位置保持（锁住目标位置不漂移）*/
                    static float hold_pos = 0;
                    if (vel_zero_cnt == VEL_ZERO_THRESHOLD)
                        hold_pos = gimbal_feedback_pos;
                    vel_zero_cnt++;
                    float torque = CascadedPID_UpdatePos(&gimbal, hold_pos,
                                                         gimbal_feedback_pos,
                                                         gimbal_feedback_vel, dt);
                    motor_6220_send_torque(0x01, torque, M6220_TORQUE_MODE_POSITION);
                }
            } else {
                vel_zero_cnt = 0;
                float torque = CascadedPID_UpdateVel(&gimbal, vel,
                                                     gimbal_feedback_vel, dt);
                motor_6220_send_torque(0x01, torque, M6220_TORQUE_MODE_VELOCITY);
            }
        }
        else {
            /* ── HTML 上位机模式（cmd_gimbal_run=1 且 gimbal_enabled=0）── */
            vel_zero_cnt = 0;   /* HTML 模式不做位置保持 */
            stop_cnt = 0;

            /* 1. 根据模式计算目标速度 */
            float desired_vel = 0.0f;

            if (cmd_gimbal_mode == 2) {
                /* 区间变速：三角波在 [low, high] 之间往返 */
                float low  = (float)cmd_gimbal_speed     * CMD_VEL_SCALE;
                float high = (float)cmd_gimbal_speed_max * CMD_VEL_SCALE;
                float range = high - low;
                if (range < 0.5f) range = 0.5f;   /* 防止卡死 */

                float period = 4.0f;   /* 一个完整往返 4 秒 */
                float half   = period * 0.5f;

                cmd_interval_t += dt;
                if (cmd_interval_t > period) cmd_interval_t -= period;

                if (cmd_interval_t < half)
                    desired_vel = low + range * (cmd_interval_t / half);
                else
                    desired_vel = high - range * ((cmd_interval_t - half) / half);
            } else {
                /* 模式 1：固定速度 */
                desired_vel = (float)cmd_gimbal_speed * CMD_VEL_SCALE;
            }

            /* 2. 加速度斜坡限制 */
            float accel_rate;
            if (cmd_gimbal_accel == 0) {
                accel_rate = 100.0f;   /* accel=0 表示无限制，瞬变 */
            } else {
                /* accel 越大加速越快：accel=255 → 约 50 单位/s² */
                accel_rate = (float)cmd_gimbal_accel * (50.0f / 255.0f);
            }

            float max_step = accel_rate * dt;
            float diff = desired_vel - cmd_ramped_vel;
            if (diff >  max_step) diff =  max_step;
            if (diff < -max_step) diff = -max_step;
            cmd_ramped_vel += diff;

            /* 3. 级联 PID 速度控制 */
            float torque = CascadedPID_UpdateVel(&gimbal, cmd_ramped_vel,
                                                 gimbal_feedback_vel, dt);
            motor_6220_send_torque(0x01, torque, M6220_TORQUE_MODE_VELOCITY);
        }

        osDelay(1);   /* 1ms 控制周期 */
    }
}
