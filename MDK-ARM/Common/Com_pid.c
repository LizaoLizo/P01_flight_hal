#include "Com_pid.h"

void Com_PID_Calc(PID_Struct *pid)
{
    pid->err = pid->measure - pid->desire;

    pid->integral += pid->err;

    if (pid -> last_err == 0)
    {
        pid -> last_err = pid->err;
    }
    float der = pid -> err - pid-> last_err;

    pid->output  = pid->kp * pid->err + (pid->ki * pid-> integral * PID_PERIOD) + (pid->kd * der / PID_PERIOD);

    pid->last_err = pid -> err;





}

void Com_PID_Calc_Chain(PID_Struct * out_pid, PID_Struct * in_pid)
{
    Com_PID_Calc(out_pid);

    in_pid->desire = out_pid->output;

    Com_PID_Calc(in_pid);

    
}

int16_t Com_limit(int16_t speed, int16_t max_speed, int16_t min_speed)
{
    if (speed > max_speed)
    {
        return max_speed;
    }
    else if (speed < min_speed)
    {
        return min_speed;
    }
    return speed;
}
