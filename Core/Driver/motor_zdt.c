#include "motor_zdt.h"
#include "bsp_fdcan.h"
#include <string.h>

/* 脉冲域软限位（= 编码器 1146880 × 25>>9 = 56000 脉冲 = 17.5 圈）*/
#define ZDT_PULSE_LIMIT  56000

/* ──────────────────────────────────────────────
 *  位置模式：双帧协议 (ID<<8 + 0) + (ID<<8 + 1)
 *  target 为编码器刻度（65536/圈），内部换算成脉冲
 *  换算：× 3200/65536 = × 25 >> 9
 * ────────────────────────────────────────────── */
void motor_zdt_send_position(uint8_t id, int32_t target)
{
    uint8_t data[8] = {0};
    uint32_t base = (uint32_t)id << 8;

    /* 编码器刻度 → 脉冲（int64 防溢出）*/
    int32_t pulses = (int32_t)(((int64_t)target * 25) >> 9);

    if (pulses < -ZDT_PULSE_LIMIT) pulses = -ZDT_PULSE_LIMIT;
    if (pulses >  ZDT_PULSE_LIMIT) pulses =  ZDT_PULSE_LIMIT;

    uint32_t abs_val = (pulses < 0) ? (uint32_t)(-pulses) : (uint32_t)(pulses);
    uint8_t  dir     = (pulses < 0) ? 0x00 : 0x01;   /* 正目标 → 上升 */

    /* Frame 0: ID=(id<<8)+0, 8 bytes */
    data[0] = 0xFD;
    data[1] = dir;
    data[2] = (300 * 10) >> 8;      // max_speed H = 3000 RPM
    data[3] = (300 * 10) & 0xFF;    // max_speed L
    data[4] = 0;                    // acc=0 直接按设定速度运行，无曲线加减速
    data[5] = (uint8_t)(abs_val >> 24);
    data[6] = (uint8_t)(abs_val >> 16);
    data[7] = (uint8_t)(abs_val >> 8);
    bsp_fdcan_send(base + 0, data, 8, 1);

    /* Frame 1: ID=(id<<8)+1, 5 bytes */
    memset(data, 0, 8);
    data[0] = 0xFD;
    data[1] = (uint8_t)(abs_val & 0xFF);
    data[2] = 0x01;                 // 绝对位置模式
    data[3] = 0x00;                 // 多机同步
    data[4] = 0x6B;                 // 校验
    bsp_fdcan_send(base + 1, data, 5, 1);
}

/* ──────────────────────────────────────────────
 *  速度模式：单帧 (ID<<8 + 0), 7 bytes
 * ────────────────────────────────────────────── */
void motor_zdt_send_velocity(uint8_t id, int32_t speed, uint16_t acc)
{
    uint8_t data[8] = {0};
    uint32_t abs_speed = (speed < 0) ? (uint32_t)(-speed) : (uint32_t)(speed);
    if (abs_speed > 65535) abs_speed = 65535;
    if (acc > 255) acc = 255;

    data[0] = 0xF6;
    data[1] = (speed < 0) ? 0x01 : 0x00;
    data[2] = (uint8_t)(abs_speed >> 8);
    data[3] = (uint8_t)(abs_speed & 0xFF);
    data[4] = (uint8_t)acc;
    data[5] = 0x00;
    data[6] = 0x6B;

    bsp_fdcan_send((uint32_t)id << 8, data, 7, 1);
}

/* ──────────────────────────────────────────────
 *  使能/失能 (ID<<8 + 0), 5 bytes
 * ────────────────────────────────────────────── */
void motor_zdt_send_enable(uint8_t id, uint8_t state)
{
    uint8_t data[8] = {0};
    data[0] = 0xF3;
    data[1] = 0xAB;
    data[2] = state;
    data[3] = 0x00;
    data[4] = 0x6B;
    bsp_fdcan_send((uint32_t)id << 8, data, 5, 1);
}

/* ──────────────────────────────────────────────
 *  查询实时位置 0x36 (ID<<8 + 0), 2 bytes
 * ────────────────────────────────────────────── */
void motor_zdt_send_request_pos(uint8_t id)
{
    uint8_t data[8] = {0};
    data[0] = 0x36;
    data[1] = 0x6B;
    bsp_fdcan_send((uint32_t)id << 8, data, 2, 1);
}

/* ──────────────────────────────────────────────
 *  当前位置清零 0x0A 6D (ID<<8 + 0), 3 bytes
 *  将当前位置角度、位置误差、脉冲数等全部清零（设为原点）
 * ────────────────────────────────────────────── */
void motor_zdt_send_zero_position(uint8_t id)
{
    uint8_t data[8] = {0};
    data[0] = 0x0A;
    data[1] = 0x6D;
    data[2] = 0x6B;
    bsp_fdcan_send((uint32_t)id << 8, data, 3, 1);
}

/* ──────────────────────────────────────────────
 *  查询物理绝对编码器 0x31 (ID<<8 + 0), 2 bytes
 *  返回校准后的编码器值（0~65535，固定物理刻度）
 * ────────────────────────────────────────────── */
void motor_zdt_send_request_encoder_raw(uint8_t id)
{
    uint8_t data[8] = {0};
    data[0] = 0x31;
    data[1] = 0x6B;
    bsp_fdcan_send((uint32_t)id << 8, data, 2, 1);
}

/* ──────────────────────────────────────────────
 *  触发回零 (ID<<8 + 0), 4 bytes
 * ────────────────────────────────────────────── */
void motor_zdt_send_homing_trigger(uint8_t id)
{
    uint8_t data[8] = {0};
    data[0] = 0x9A;
    data[1] = 0x02;         // 多圈无限位碰撞回零
    data[2] = 0x00;        // 多机位标志位
    data[3] = 0x6B;
    bsp_fdcan_send((uint32_t)id << 8, data, 4, 1);
}

/* ──────────────────────────────────────────────
 *  查询回零状态 (ID<<8 + 0), 2 bytes
 * ────────────────────────────────────────────── */
void motor_zdt_send_homing_status(uint8_t id)
{
    uint8_t data[8] = {0};
    data[0] = 0x3B;
    data[1] = 0x6B;
    bsp_fdcan_send((uint32_t)id << 8, data, 2, 1);
}

/* ──────────────────────────────────────────────
 *  强制停止回零 (ID<<8 + 0), 3 bytes
 * ────────────────────────────────────────────── */
void motor_zdt_send_force_stop(uint8_t id)
{
    uint8_t data[8] = {0};
    data[0] = 0x9C;
    data[1] = 0x48;
    data[2] = 0x6B;
    bsp_fdcan_send((uint32_t)id << 8, data, 3, 1);
}

/* ──────────────────────────────────────────────
 *  解除堵转保护 (ID<<8 + 0), 3 bytes
 * ────────────────────────────────────────────── */
void motor_zdt_send_release_stall(uint8_t id)
{
    uint8_t data[8] = {0};
    data[0] = 0x0E;
    data[1] = 0x52;
    data[2] = 0x6B;
    bsp_fdcan_send((uint32_t)id << 8, data, 3, 1);
}

/* ── 0x3A 查询电机状态（有无堵转等）── */
void motor_zdt_send_request_status(uint8_t id)
{
    uint8_t data[8] = {0};
    data[0] = 0x3A;
    data[1] = 0x6B;
    bsp_fdcan_send((uint32_t)id << 8, data, 2, 1);
}

/* ──────────────────────────────────────────────
 *  统一反馈解析：根据 data[0] 自动识别帧类型
 *  0x31 → 物理绝对编码器 (0~65535)
 *  0x36 → 电机内部多圈位置（编码器刻度 65536/圈）
 *  0x3A → is_stalled
 *  0x3B → homing_status
 * ⚠ 0x31/0x36 的字节位置需按说明书核对
 * ────────────────────────────────────────────── */
void motor_zdt_parse_feedback(const uint8_t *data, ZDT_Feedback_t *fb)
{
    if (!fb) return;
    fb->cmd = data[0];

    switch (data[0]) {
    case 0x31: {
        /* CAN 剥离地址后：data[1]=编码器高8位, data[2]=低8位, data[3]=校验 */
        fb->encoder = ((uint16_t)data[1] << 8) | data[2];
        break;
    }
    case 0x36: {
        /* CAN 剥离地址后：data[1]=符号位, data[2:5]=32位绝对值(大端), data[6]=校验 */
        uint32_t raw = ((uint32_t)data[2] << 24) |
                       ((uint32_t)data[3] << 16) |
                       ((uint32_t)data[4] << 8)  |
                       ((uint32_t)data[5]);
        /* 直接存编码器刻度（65536/圈），不做脉冲换算 */
        fb->position = (data[1] == 0x01) ? -(int32_t)raw : (int32_t)raw;
        break;
    }
    case 0x3A:
        fb->is_stalled = (data[1] & 0x04) ? 1 : 0;
        break;
    case 0x3B: {
        uint8_t s = data[1];
        if (s & 0x04)      fb->homing_status = 0;   // 回零中
        else if (s & 0x08) fb->homing_status = 2;   // 失败
        else               fb->homing_status = 1;   // 成功/空闲
        break;
    }
    default:
        break;
    }
}