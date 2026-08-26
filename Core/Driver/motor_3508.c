#include "motor_3508.h"
#include "bsp_fdcan.h"

/* ──────────────────────────────────────────────
 *  发送电流指令（ID=0x200, 标准帧, 8 bytes）
 *  data[0:1] = 左轮电流（定点数, 大端序）
 *  data[2:3] = 右轮电流（定点数, 大端序）
 *  data[4:7] = 0
 * ────────────────────────────────────────────── */
void motor_3508_send_current(int16_t lf, int16_t rf)
{
    uint8_t data[8] = {0};

    data[0] = (uint8_t)(lf >> 8);
    data[1] = (uint8_t)(lf & 0xFF);
    data[2] = (uint8_t)(rf >> 8);
    data[3] = (uint8_t)(rf & 0xFF);

    bsp_fdcan_send(0x200, data, 8, 0);
}

/* ──────────────────────────────────────────────
 *  解析单电机反馈帧（ID=0x201/0x202）
 *  data[0:1] = 编码器值 (0~8191)
 *  data[2:3] = RPM（有符号）
 * ────────────────────────────────────────────── */
void motor_3508_parse_feedback(const uint8_t *data, M3508_Feedback_t *fb)
{
    if (!fb) return;
    fb->encoder = ((uint16_t)data[0] << 8) | data[1];
    fb->rpm     =  (int16_t)((uint16_t)data[2] << 8) | data[3];
}

/* ──────────────────────────────────────────────
 *  多圈编码器跟踪 + 角度/角速度换算
 *  每次收到反馈帧后调用
 * ────────────────────────────────────────────── */
void motor_3508_update_angle(M3508_Angle_t *a, const M3508_Feedback_t *fb)
{
    if (!a || !fb) return;

    int16_t delta = (int16_t)(fb->encoder - a->pre_encoder);

    if (delta < -M3508_ENCODER_PER_ROUND / 2)
        a->total_round++;
    else if (delta > M3508_ENCODER_PER_ROUND / 2)
        a->total_round--;

    a->total_encoder = a->total_round * M3508_ENCODER_PER_ROUND + fb->encoder;
    a->angle_rad     = (float)a->total_encoder / M3508_ENCODER_PER_ROUND
                       * 6.2831855f / M3508_GEARBOX_RATE;
    a->omega_rads    = (float)fb->rpm * 0.104719755f / M3508_GEARBOX_RATE;
    a->pre_encoder   = fb->encoder;
}