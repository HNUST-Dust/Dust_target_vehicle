#include "motor_6220.h"
#include "bsp_fdcan.h"

/* ──────────────────────────────────────────────
 *  发送扭矩控制帧
 *  id   : 电机 ID（默认 0x01）
 *  torque : 扭矩（Nm）
 *  mode : M6220_TORQUE_MODE_VELOCITY / POSITION
 * ────────────────────────────────────────────── */
void motor_6220_send_torque(uint8_t id, float torque, uint8_t mode)
{
    uint8_t data[8] = {0};
    uint16_t electric;
    const float span = 4095.0f;

    if (mode == M6220_TORQUE_MODE_POSITION) {
        /* ±18 Nm → 0~4095 */
        if (torque <= M6220_T_MIN) electric = 0;
        else if (torque >= M6220_T_MAX) electric = (uint16_t)span;
        else electric = (uint16_t)((torque - M6220_T_MIN) / (M6220_T_MAX - M6220_T_MIN) * span);
    } else {
        /* ±5 Nm → 0~4095 */
        if (torque <= -5.0f) electric = 0;
        else if (torque >= 5.0f) electric = (uint16_t)span;
        else electric = (uint16_t)((torque + 5.0f) / 10.0f * span);
    }

    data[6] = (uint8_t)((electric >> 8) & 0x0F);
    data[7] = (uint8_t)(electric & 0xFF);

    bsp_fdcan_send(id, data, 8, 0);
}

/* ──────────────────────────────────────────────
 *  使能电机
 * ────────────────────────────────────────────── */
void motor_6220_send_enable(uint8_t id)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    bsp_fdcan_send(id, data, 8, 0);
}

/* ──────────────────────────────────────────────
 *  失能电机
 * ────────────────────────────────────────────── */
void motor_6220_send_disable(uint8_t id)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    bsp_fdcan_send(id, data, 8, 0);
}

/* ──────────────────────────────────────────────
 *  解析反馈帧，返回故障码检测结果
 *  6220 反馈: ID=0x00, 8 bytes
 *    data[0] 低4位=电机ID, 高4位=错误码
 *      0=失能 1=使能(状态,非故障)
 *      8=超压 9=欠压 10=过流 11=MOS过温 12=线圈过温
 *      13=通讯丢失 14=过载(故障)
 *    data[1:2] = 16位位置编码器 (0~65535)
 *    data[3:4] 高4位 = 12位速度
 *    data[4] 低4位 + data[5] = 12位扭矩
 *  返回：1=有故障（数据不可信，不更新 pos/vel/torque），0=正常
 * ────────────────────────────────────────────── */
uint8_t motor_6220_parse_feedback(const uint8_t *data, float *pos, float *vel, float *torque)
{
    uint8_t err = (data[0] & 0xF0) >> 4;

    switch (err) {
        case 0:   /* 失能（状态，非故障）*/
        case 1:   /* 使能（状态，非故障）*/
            break;
        default:  /* 8超压 9欠压 10过流 11MOS过温 12线圈过温 13通讯丢失 14过载 及未知 */
            return 1;
    }

    uint16_t raw_pos    = ((uint16_t)data[1] << 8) | data[2];
    uint16_t raw_vel    = ((uint16_t)data[3] << 4) | (data[4] >> 4);
    uint16_t raw_torque = ((uint16_t)(data[4] & 0x0F) << 8) | data[5];

    if (pos)   *pos   = (float)raw_pos / 65535.0f * (M6220_P_MAX - M6220_P_MIN) + M6220_P_MIN;
    if (vel)   *vel   = (float)raw_vel / 4095.0f   * (M6220_V_MAX - M6220_V_MIN) + M6220_V_MIN;
    if (torque)*torque= (float)raw_torque / 4095.0f * (M6220_T_MAX - M6220_T_MIN) + M6220_T_MIN;

    return 0;   /* 正常 */
}
