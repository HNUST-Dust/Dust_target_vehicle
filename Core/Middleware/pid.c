#include "pid.h"

/* ══════════════════════════════════════════════
 *              基本 PID 控制器
 * ══════════════════════════════════════════════ */
float PID_Calculate(PID_t *pid, float target, float measure, float dt)
{
    float err = target - measure;
    float p_out = pid->kp * err;

    pid->integral += err * dt;
    float i_out = pid->ki * pid->integral;

    float d_out = pid->kd * (err - pid->err_last) / dt;
    pid->err_last = err;

    float output = p_out + i_out + d_out;

    if (output > pid->out_max) {
        pid->integral -= err * dt;
        output = pid->out_max;
    } else if (output < pid->out_min) {
        pid->integral -= err * dt;
        output = pid->out_min;
    }
    return output;
}

void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->err_last = 0.0f;
}

void PID_SetParam(PID_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp; pid->ki = ki; pid->kd = kd;
}

void PID_SetLimit(PID_t *pid, float max, float min)
{
    pid->out_max = max; pid->out_min = min;
}

/* ══════════════════════════════════════════════
 *              一阶低通滤波器
 * ══════════════════════════════════════════════ */
float LPF_Update(LPF_t *lpf, float input)
{
    lpf->output = lpf->alpha * lpf->output + (1.0f - lpf->alpha) * input;
    return lpf->output;
}

/* ══════════════════════════════════════════════
 *          级联位置→速度控制器 (6220)
 * ══════════════════════════════════════════════ */
#define POS_LOOP_DIV  5
#define V_POS_MAX     10.0f
#define V_POS_MIN    -10.0f
#define V_MAX         30.0f
#define V_MIN        -30.0f
#define T_MAX         18.0f
#define T_MIN        -18.0f
#define P_RANGE      25.0f

void CascadedPID_Config(CascadedPID_t *cp,
                        float p_kp, float p_kd,
                        float v_kp, float v_ki, float v_kd,
                        float v_lpf_alpha)
{
    cp->p_kp = p_kp;  cp->p_kd = p_kd;
    cp->v_kp = v_kp;  cp->v_ki = v_ki;  cp->v_kd = v_kd;
    cp->vel_lpf.alpha = v_lpf_alpha;
    cp->ff = 0.0f;
    CascadedPID_Clear(cp);
}

void CascadedPID_Clear(CascadedPID_t *cp)
{
    cp->vel_integral = 0.0f;
    cp->vel_lpf.output = 0.0f;
    cp->pos_pid.integral = 0.0f;
    cp->pos_pid.err_last = 0.0f;
    cp->vel_pid.integral = 0.0f;
    cp->vel_pid.err_last = 0.0f;
    cp->pos_last = 0.0f;
    cp->pos_div_cnt = 0;
}

/* ── 速度环核心 ──
 * hold=1（位置保持模式）：不清速度积分、积分门槛放低到 0.15，
 *   否则位置保持时积分被 0.5 死区 / 0.1 清零卡住，攒不出保持力矩 → 抖动 */
static float vel_loop(CascadedPID_t *cp, float target_vel,
                      float measure_vel, float dt, uint8_t hold)
{
    cp->vel_target = target_vel;
    float filtered = LPF_Update(&cp->vel_lpf, measure_vel);
    float err = cp->vel_target - filtered;
    float abs_tv = (cp->vel_target > 0) ? cp->vel_target : -cp->vel_target;
    float abs_err = (err > 0) ? err : -err;

    /* 目标速度≈0 时清积分（纯速度模式防停不住）；位置保持时不清（保留保持力矩）*/
    if (abs_tv < 0.1f && !hold)
        cp->vel_integral = 0.0f;

    float igate = hold ? 0.15f : 0.5f;   /* 保持模式积分门槛放宽 */
    if (abs_err >= igate)
        cp->vel_integral += cp->v_ki * err * dt;

    if (cp->vel_integral > T_MAX) cp->vel_integral = T_MAX;
    if (cp->vel_integral < T_MIN) cp->vel_integral = T_MIN;

    /* v_kd 项 = -v_kd×实测速度：纯速度阻尼，两种模式都压抖动 */
    float torque = cp->v_kp * err + cp->vel_integral - cp->v_kd * filtered;

    if (abs_tv > 2.0f)
        torque += FF_Friction(cp->vel_target);

    if (torque > T_MAX) torque = T_MAX;
    if (torque < T_MIN) torque = T_MIN;
    return torque;
}

/* ── 速度模式（纯速度环）── */
float CascadedPID_UpdateVel(CascadedPID_t *cp, float target_vel,
                            float measure_vel, float dt)
{
    return vel_loop(cp, target_vel, measure_vel, dt, 0);
}

/* ── 位置模式（级联）── */
float CascadedPID_UpdatePos(CascadedPID_t *cp, float target_pos,
                            float measure_pos, float measure_vel, float dt)
{
    cp->pos_target = target_pos;
    cp->pos_div_cnt++;

    if (cp->pos_div_cnt >= POS_LOOP_DIV)
    {
        cp->pos_div_cnt = 0;
        float d_pos = target_pos - measure_pos;
        float half = P_RANGE * 0.5f;
        if (d_pos > half)       d_pos -= P_RANGE;
        else if (d_pos < -half) d_pos += P_RANGE;

        cp->vel_target = cp->p_kp * d_pos;
        if (cp->vel_target > V_MAX) cp->vel_target = V_MAX;
        if (cp->vel_target < V_MIN) cp->vel_target = V_MIN;

        float deriv = (d_pos - cp->pos_last) / (dt * POS_LOOP_DIV);
        cp->pos_last = d_pos;
        float damping = cp->p_kd * deriv;

        /* 速度环（保持模式：低积分门槛，能建立保持力矩）*/
        float filtered = LPF_Update(&cp->vel_lpf, measure_vel);
        float err = cp->vel_target - filtered;
        float abs_err = (err > 0) ? err : -err;

        if (abs_err >= 0.15f)
            cp->vel_integral += cp->v_ki * err * dt;

        if (cp->vel_integral > T_MAX) cp->vel_integral = T_MAX;
        if (cp->vel_integral < T_MIN) cp->vel_integral = T_MIN;

        float torque = cp->v_kp * err + cp->vel_integral - cp->v_kd * filtered;
        torque += cp->ff * d_pos + damping;

        if (torque > T_MAX) torque = T_MAX;
        if (torque < T_MIN) torque = T_MIN;
        return torque;
    }

    /* 中间节拍：沿用位置环输出的 vel_target 跑速度环（hold=1 不清积分）*/
    return vel_loop(cp, cp->vel_target, measure_vel, dt, 1);
}

/* ══════════════════════════════════════════════
 *              辅助函数
 * ══════════════════════════════════════════════ */
float FF_Friction(float target_vel)
{
    if (target_vel > 0.01f)       return  0.05f;
    else if (target_vel < -0.01f) return -0.05f;
    else                          return  0.0f;
}

float GenerateRandomVel(unsigned int *seed)
{
    *seed = *seed * 1103515245 + 12345;
    int r = (int)(*seed & 0x7FFF) % 42;
    if (r < 21) return -30.0f + (float)r;
    else        return  10.0f + (float)(r - 21);
}
