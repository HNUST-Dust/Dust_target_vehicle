#include "task_zdt.h"
#include "motor_zdt.h"
#include "cmsis_os2.h"
#include <stdio.h>

/* ── ZDT 全局状态（编码器刻度域，65536/圈） ── */
volatile int32_t  zdt_step          = 0;
volatile uint8_t  zdt_enabled       = 0;
volatile int32_t  zdt_target[2]     = {0, 0};
volatile int32_t  zdt_real_pos[2]   = {0, 0};
volatile uint8_t  zdt_motor_pos_valid[2] = {0, 0};
volatile uint8_t  zdt_is_stalled[2] = {0, 0};
volatile uint8_t  zdt_calibrating[2]= {0, 0};
volatile uint8_t  zdt_homing_req    = 0;

/* ── 软限位（编码器刻度，= 56000 脉冲 × 65536/3200 = 17.5 圈）── */
#define ZDT_SOFT_LIMIT_TOP      1146880
#define ZDT_SOFT_LIMIT_BOTTOM   0

/* 同步模式：使用 ID=0x00 广播一发控制双电机 */
#define ZDT_SYNC_ID  0x00

/* ── 回零参数 ── */
#define ZDT_HOMING_SPEED   200      /* RPM，向底部回零（太快易误触发堵转保护）*/
#define ZDT_HOMING_TIMEOUT 600      /* 6s 超时保护（600 × 10ms）*/

/* ══════════════════════════════════════════════════════════════
 *  双电机同时回零（并行）：两个电机同时向底部速度模式撞底，
 *  各自独立检测堵转，撞底的先停，都完成后归零
 * ══════════════════════════════════════════════════════════════ */
static void zdt_homing_both(void)
{
    /* 两个电机同时回零（正方向 = 撞底部限位）*/
    motor_zdt_send_velocity(1, (int32_t)ZDT_HOMING_SPEED, 0);
    motor_zdt_send_velocity(2, (int32_t)ZDT_HOMING_SPEED, 0);

    uint32_t tick = 0;
    uint32_t timeout = 0;
    uint8_t done[2] = {0, 0};

    /* 等待两个电机都撞底堵转（或超时）*/
    while ((!done[0] || !done[1]) && timeout < ZDT_HOMING_TIMEOUT) {
        if ((tick++ % 5) == 0) {
            if (!done[0]) motor_zdt_send_request_status(1);
            if (!done[1]) motor_zdt_send_request_status(2);
        }
        osDelay(10);
        timeout++;

        for (uint8_t i = 0; i < 2; i++) {
            if (!done[i] && zdt_is_stalled[i]) {
                /* 该电机撞底：停止 + 释放 */
                motor_zdt_send_velocity(i + 1, 0, 0);
                motor_zdt_send_release_stall(i + 1);
                done[i] = 1;
            }
        }
    }

    /* 超时未完成的电机，强制停止 */
    for (uint8_t i = 0; i < 2; i++) {
        if (!done[i]) {
            motor_zdt_send_velocity(i + 1, 0, 0);
            motor_zdt_send_release_stall(i + 1);
        }
    }

    /* 等待堵转清除 + 统一收尾（不管撞底成功与否）*/
    for (uint8_t i = 0; i < 2; i++) {
        uint32_t wait = 0;
        while (zdt_is_stalled[i] && wait < 50) {
            osDelay(10);
            wait++;
            if ((wait % 2) == 0)
                motor_zdt_send_request_status(i + 1);
        }

        /* 统一：解除堵转 + 当前位置清零为原点 + 目标归零 */
        motor_zdt_send_release_stall(i + 1);
        osDelay(20);
        motor_zdt_send_zero_position(i + 1);
        osDelay(50);

        zdt_is_stalled[i]   = 0;
        zdt_calibrating[i]  = 0;
        zdt_target[i]       = 0;
        zdt_real_pos[i]     = 0;
        printf("ZDT %d homing done (zeroed)\r\n", i + 1);
    }
}

/* ══════════════════════════════════════════════════════════════
 *  步进电机控制 + 监控任务（10ms 周期）
 *  职责 1：上电自动回零（速度模式撞底）
 *  职责 2：摇杆积分 → 软限位 → 发绝对位置指令
 *  职责 3：轮询 0x36/0x3A → 堵转自动解除
 * ══════════════════════════════════════════════════════════════ */
void task_zdt_loop(void *argument)
{
    (void)argument;

    /* 上电使能双电机，等 1.5s 驱动器就绪后同时回零 */
    motor_zdt_send_enable(1, 1);
    motor_zdt_send_enable(2, 1);
    osDelay(1500);

    zdt_homing_both();

    uint8_t stall_cnt = 0;
    uint8_t poll_phase = 0;     /* 轮流查询：0=电机1, 1=电机2 */
    uint32_t stall_release_t[2] = {0, 0};   /* 堵转释放冷却时间戳 */

    while (1)
    {
        /* ── 每 10 次循环（~100ms）轮流查询一个电机的位置+状态 ── */
        if (++stall_cnt >= 10) {
            stall_cnt = 0;
            uint8_t id = poll_phase ? 2 : 1;
            motor_zdt_send_request_status(id);
            motor_zdt_send_request_pos(id);
            poll_phase ^= 1;
        }

        /* ── 堵转检测与急救（回零期间不自动解除）──
         *  加 500ms 冷却：释放后不立即重复，防止"释放-旋转-再堵转"死循环 */
        for (uint8_t i = 0; i < 2; i++) {
            if (zdt_is_stalled[i] && !zdt_calibrating[i]) {
                uint32_t now = osKernelGetTickCount();
                if ((now - stall_release_t[i]) > 500) {
                    stall_release_t[i] = now;
                    uint8_t id = i + 1;
                    printf("ZDT %d stalled, releasing...\r\n", id);
                    motor_zdt_send_release_stall(id);
                    zdt_target[i] = zdt_real_pos[i];   /* 追平实际位置 */
                    zdt_is_stalled[i] = 0;
                }
            }
        }

        /* ── 手动回零请求（task_ibus: s1=2 边沿触发）── */
        if (zdt_homing_req) {
            zdt_homing_req = 0;
            printf("ZDT manual homing...\r\n");
            zdt_homing_both();
        }

        /* ── ZDT 位置控制（回零完成后执行）── */
        if (zdt_enabled && !zdt_calibrating[0] && !zdt_calibrating[1]) {
            if (zdt_step != 0) {
                zdt_target[0] += zdt_step;
                zdt_target[1]  = zdt_target[0];   /* 默认双电机联动 */

                if (zdt_target[0] > ZDT_SOFT_LIMIT_TOP) {
                    zdt_target[0] = ZDT_SOFT_LIMIT_TOP;
                    zdt_target[1] = ZDT_SOFT_LIMIT_TOP;
                }
                if (zdt_target[0] < ZDT_SOFT_LIMIT_BOTTOM) {
                    zdt_target[0] = ZDT_SOFT_LIMIT_BOTTOM;
                    zdt_target[1] = ZDT_SOFT_LIMIT_BOTTOM;
                }

                /* 单独地址发两个电机（广播时 2 号可能不执行）*/
                motor_zdt_send_position(1, zdt_target[0]);
                motor_zdt_send_position(2, zdt_target[1]);
            }
        }

        osDelay(10);
    }
}