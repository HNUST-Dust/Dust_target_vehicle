#include "task_can_recv.h"
#include "task_zdt.h"
#include "bsp_fdcan.h"
#include "motor_6220.h"
#include "motor_zdt.h"
#include <string.h>

/* ── 裁判系统击打计数（LCD 任务显示用）── */
extern uint16_t BIG_BALL;
extern uint16_t SMALL_BALL;

/* ── 全局反馈数据 ── */
float           gimbal_feedback_pos = 0.0f;
float           gimbal_feedback_vel = 0.0f;
int16_t         chassis_rpm_lf      = 0;
int16_t         chassis_rpm_rf      = 0;
M3508_Angle_t   chassis_angle_lf    = {0};
M3508_Angle_t   chassis_angle_rf    = {0};


static uint8_t motor_id_from_msg(const CAN_RxMsg_t *msg)
{
    if (msg->frame_type == 0)
        return (uint8_t)(msg->can_id & 0xFF);
    return (uint8_t)((msg->can_id >> 8) & 0xFF);
}


void task_can_recv(void *argument)
{
    (void)argument;
    CAN_RxMsg_t msg;

    while (1)
    {
        if (osMessageQueueGet(can_rx_queueHandleHandle, &msg, NULL, osWaitForever) != osOK)
            continue;

        if (msg.frame_type == 0)
        {
            /* ── 标准帧 ── */
            switch (msg.can_id)
            {
            case 0x00: {         /* 6220 云台电机反馈 */
                float pos, vel, tor;
                uint8_t err = motor_6220_parse_feedback(msg.data, &pos, &vel, &tor);
                if (!err) {
                    gimbal_feedback_pos = pos;
                    gimbal_feedback_vel = vel;
                }
                else {
                    uint8_t data = 0xCC;
                      bsp_fdcan_send(0x777, &data, 1, 0);
                    }
                
                break;
            }
            case 0x201: {        /* 3508 左前轮反馈 */
                M3508_Feedback_t fb;
                motor_3508_parse_feedback(msg.data, &fb);
                chassis_rpm_lf = fb.rpm;
                motor_3508_update_angle(&chassis_angle_lf, &fb);
                break;
            }
            case 0x202: {        /* 3508 右前轮反馈 */
                M3508_Feedback_t fb;
                motor_3508_parse_feedback(msg.data, &fb);
                chassis_rpm_rf = fb.rpm;
                motor_3508_update_angle(&chassis_angle_rf, &fb);
                break;
            }
            case 0x777: {         /* 装甲板击打数 */
                /* 上电前 500ms 内的 0x777 帧视为总线抖动/初始化帧，不计数，
                 * 避免开机就显示非零击打数（如上电后大弹丸直接显示 2） */
                if (osKernelGetTickCount() < 500U) break;
                if (msg.data[0] == 2) BIG_BALL++;
                if (msg.data[0] == 3) SMALL_BALL++;
                break;
            }
            }
        }
        else
        {
            /* ── 扩展帧（ZDT 步进电机）── */
            uint8_t id = motor_id_from_msg(&msg);
            uint8_t idx = (id >= 1 && id <= 2) ? id - 1 : 0xFF;

            if (idx > 1) continue;

            ZDT_Feedback_t fb;
            motor_zdt_parse_feedback(msg.data, &fb);

            if (fb.cmd == 0x36) {
                /* 直接存编码器刻度位置，标记新反馈到达（回零锚定用）*/
                zdt_real_pos[idx] = fb.position;
                zdt_motor_pos_valid[idx] = 1;
            }
            else if (fb.cmd == 0x3A) {
                zdt_is_stalled[idx] = fb.is_stalled;
            }
        }
    }
}
