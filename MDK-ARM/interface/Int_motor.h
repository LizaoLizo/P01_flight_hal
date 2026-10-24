#ifndef __INT_MOTOR__
#define __INT_MOTOR__

#include "tim.h"
#include "Com_debug.h"

typedef struct 
{
    TIM_HandleTypeDef *tim;
    uint16_t channel;
    int16_t speed;
}Motor_Struct;



void Int_motor_set_speed(Motor_Struct *motor);

void Int_motor_start(Motor_Struct *motor);

#endif // __INT_MOTOR__