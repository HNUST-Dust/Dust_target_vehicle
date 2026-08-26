#include "chassis_kinematics.h"

void Chassis_Init(Chassis_t *cs)
{
    PID_SetParam(&cs->pid_lf, 10.0f, 0.0f, 0.0015f);
    PID_SetLimit(&cs->pid_lf,  CHASSIS_MAX_CURRENT, -CHASSIS_MAX_CURRENT);
    PID_Reset(&cs->pid_lf);

    PID_SetParam(&cs->pid_rf, 10.0f, 0.0f, 0.0015f);
    PID_SetLimit(&cs->pid_rf,  CHASSIS_MAX_CURRENT, -CHASSIS_MAX_CURRENT);
    PID_Reset(&cs->pid_rf);
}

void Chassis_Movement(float vx, float vR, float *out_lf, float *out_rf)
{
    float forward = vx * CHASSIS_MAX_RPM;
    float rotate  = vR * CHASSIS_MAX_RPM;

    *out_lf =  forward + rotate;
    *out_rf = -(forward - rotate);
}
