#ifndef __APP_FLIGHT__
#define __APP_FLIGHT__

#include "Int_mpu6050.h"
#include "Com_debug.h"
#include "Com_filter.h"
#include "math.h"
#include "Com_imu.h"
#include "Com_pid.h"
#include "Int_motor.h"
#include "Int_VL53L1X.h"

extern Motor_Struct left_top_motor;
extern Motor_Struct left_bottom_motor;
extern Motor_Struct right_top_motor;
extern Motor_Struct right_bottom_motor;

void App_flight_init(void);

void App_flight_get_euler_angle(void);

void App_flight_pid_process(void);

void App_flight_control_motor(void);

void App_flight_fix_height_pid_process(void);


#endif // __APP_Flight__
