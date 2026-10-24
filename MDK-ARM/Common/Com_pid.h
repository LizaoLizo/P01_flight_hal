#ifndef __COM_PID__
#define __COM_PID__

#include "main.h"

#define PID_PERIOD 0.006


typedef struct
{
    float kp;
    float ki;
    float kd;
    float err;
    float desire;
    float measure;
    float last_err;
    float integral;
    float output;
}PID_Struct;

void Com_PID_Calc(PID_Struct * pid);

void Com_PID_Calc_Chain(PID_Struct *  out_pid, PID_Struct * in_pid);

int16_t Com_limit(int16_t speed, int16_t max_speed, int16_t min_speed);

#endif // __COM_PID__