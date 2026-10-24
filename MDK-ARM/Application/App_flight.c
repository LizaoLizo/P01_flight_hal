#include "App_flight.h"

Gyro_Accel_Struct gyro_accel_data = {0};
Euler_struct euler_angle = {0};
Gyro_struct last_gyro = {0};
float gyro_z_sum = 0;
extern Remote_Data remote_data;
extern Flight_State flight_state;
extern TaskHandle_t flight_task_handle;
extern TaskHandle_t com_task_handle;

Motor_Struct left_top_motor = {.tim = &htim3, .channel = TIM_CHANNEL_1, .speed = 0};
Motor_Struct left_bottom_motor = {.tim = &htim4, .channel = TIM_CHANNEL_4, .speed = 0};
Motor_Struct right_top_motor = {.tim = &htim2, .channel = TIM_CHANNEL_2, .speed = 0};
Motor_Struct right_bottom_motor = {.tim = &htim1, .channel = TIM_CHANNEL_3, .speed = 0};

PID_Struct pitch_pid  = {.kp = -7.0f, .ki = 0.0f, .kd = 0.0f};
PID_Struct gyro_y_pid = {.kp = 3.0f, .ki = 0.0f, .kd = 0.50f};

PID_Struct roll_pid   = {.kp = -7.0, .ki = 0.0, .kd = 0.0};
PID_Struct gyro_x_pid = {.kp =  3.0, .ki = 0.0, .kd = 0.50};

/* 首次测试关闭偏航控制 */
PID_Struct yaw_pid    = {.kp = -3.0, .ki = 0.0, .kd = 0.0};
PID_Struct gyro_z_pid = {.kp = -5.0, .ki = 0.0, .kd = 0.0};

PID_Struct height_pid   = {.kp = -0.5, .ki = 0.0, .kd = -0.20 };

extern uint16_t fix_height;
void App_flight_init(void)
{
    Int_MPU6050_Init();

    Int_motor_start(&left_top_motor);
    Int_motor_start(&left_bottom_motor);
    Int_motor_start(&right_top_motor);
    Int_motor_start(&right_bottom_motor);

    Int_VL53L1X_Init();
}


void App_flight_get_euler_angle(void)
{
     Int_MPU6050_Get_Data(&gyro_accel_data);

    /* 陀螺仪：低通滤波 */
    gyro_accel_data.gyro.gyro_x =
        Common_Filter_LowPass(gyro_accel_data.gyro.gyro_x,
                              last_gyro.gyro_x);
    gyro_accel_data.gyro.gyro_y =
        Common_Filter_LowPass(gyro_accel_data.gyro.gyro_y,
                              last_gyro.gyro_y);
    gyro_accel_data.gyro.gyro_z =
        Common_Filter_LowPass(gyro_accel_data.gyro.gyro_z,
                              last_gyro.gyro_z);
        last_gyro = gyro_accel_data.gyro;

    //printf("%d,%d,%d\r\n",
           //(int)gyro_accel_data.gyro.gyro_x,
           //(int)gyro_accel_data.gyro.gyro_y,
           //(int)gyro_accel_data.gyro.gyro_z);

    gyro_accel_data.accel.accel_x =
        (int16_t)Common_Filter_KalmanFilter(
            &kfs[0], gyro_accel_data.accel.accel_x);

    gyro_accel_data.accel.accel_y =
        (int16_t)Common_Filter_KalmanFilter(
            &kfs[1], gyro_accel_data.accel.accel_y);

    gyro_accel_data.accel.accel_z =
        (int16_t)Common_Filter_KalmanFilter(
            &kfs[2], gyro_accel_data.accel.accel_z);

    //printf("%d,%d,%d\r\n",
           //(int)gyro_accel_data.accel.accel_x,
           //(int)gyro_accel_data.accel.accel_y,
           //(int)gyro_accel_data.accel.accel_z);

    //euler_angle.pitch = atan2(gyro_accel_data.accel.accel_x * 1.0, gyro_accel_data.accel.accel_z) / 3.1415926 * 180;

    //euler_angle.roll = atan2(gyro_accel_data.accel.accel_y * 1.0, gyro_accel_data.accel.accel_z) / 3.1415926 * 180;

    //gyro_z_sum += (gyro_accel_data.gyro.gyro_z * 2000.0 / 32768.0) * 0.006;
    //euler_angle.yaw = gyro_z_sum;

    Common_IMU_GetEulerAngle(&gyro_accel_data, &euler_angle, 0.006);

    //debug_printf(":%.2f, %.2f, %.2f\n", euler_angle.pitch, euler_angle.roll, euler_angle.yaw);

    

}

void App_flight_pid_process(void)
{
    // pitch angle
    pitch_pid.desire = (remote_data.pit - 500) / 50.0;
    pitch_pid.measure = euler_angle.pitch;

    gyro_y_pid.measure =
    gyro_accel_data.gyro.gyro_y *
    2000.0f / 32768.0f;

    Com_PID_Calc_Chain(&pitch_pid, &gyro_y_pid);

    static uint8_t print_count = 0;

if (++print_count >= 20)
{
    print_count = 0;

    //debug_printf(
        //"pitch=%.2f, desire=%.2f, gyroY=%.2f, yOut=%.2f, pit=%d\r\n",
        //euler_angle.pitch,
        //pitch_pid.desire,
        //gyro_y_pid.measure,
        //gyro_y_pid.output,
        //remote_data.pit
    //);
}

    //debug_printf(":%.2f, %.2f\n", gyro_y_pid.err, gyro_y_pid.output);

    //printf("%.2f,%.2f\r\n",
       //euler_angle.pitch,
       //gyro_y_pid.output);
    roll_pid.desire = (remote_data.rol - 500) / 50.0;
    roll_pid.measure = euler_angle.roll;
    gyro_x_pid.measure = (gyro_accel_data.gyro.gyro_x * 2000.0f / 32768.0);

    Com_PID_Calc_Chain(&roll_pid, &gyro_x_pid);

    yaw_pid.desire = (remote_data.yaw - 500) / 50.0;

    yaw_pid.measure = euler_angle.yaw;

    gyro_z_pid.measure = (gyro_accel_data.gyro.gyro_z * 2000.0f / 32768.0);

    Com_PID_Calc_Chain(&yaw_pid, &gyro_z_pid);

}

void App_flight_control_motor(void)
{
    switch (flight_state)
    {
        case IDLE:
        left_top_motor.speed = 0;
        left_bottom_motor.speed = 0;
        right_top_motor.speed = 0;
        right_bottom_motor.speed = 0;
        break;

        case NORMAL:
{
    left_top_motor.speed = 400;
    left_bottom_motor.speed = 400;
    right_top_motor.speed = 400;
    right_bottom_motor.speed = 400;
    break;
}

/* {
    int16_t base_speed =
    Com_limit((int16_t)remote_data.thr, 1000, 0);

    int16_t pitch_out = (int16_t)gyro_y_pid.output;
    int16_t roll_out  = (int16_t)gyro_x_pid.output;

    left_top_motor.speed =
        base_speed + pitch_out - roll_out;

    right_top_motor.speed =
        base_speed + pitch_out + roll_out;

    left_bottom_motor.speed =
        base_speed - pitch_out - roll_out;

    right_bottom_motor.speed =
        base_speed - pitch_out + roll_out;

    break;
}*/



        case FIX_HEIGHT:
        {
    int16_t base_speed =
    Com_limit((int16_t)remote_data.thr, 1000, 0);

    int16_t pitch_out = (int16_t)gyro_y_pid.output;
    int16_t roll_out  = (int16_t)gyro_x_pid.output;

    left_top_motor.speed =
        base_speed + pitch_out - roll_out + height_pid.output;

    right_top_motor.speed =
        base_speed + pitch_out + roll_out + height_pid.output;

    left_bottom_motor.speed =
        base_speed - pitch_out - roll_out + height_pid.output;

    right_bottom_motor.speed =
        base_speed - pitch_out + roll_out + height_pid.output;

}

        break;

        case FAIL:
        left_top_motor.speed -= 2;
        left_bottom_motor.speed -= 2;
        right_top_motor.speed -= 2;
        right_bottom_motor.speed -= 2;
        if (left_top_motor.speed <= 0 && left_bottom_motor.speed <= 0 && right_top_motor.speed <= 0 && right_bottom_motor.speed <= 0)
        {
            //flight_state = IDLE;
            xTaskNotifyGive(com_task_handle);
        }
        break;

        default:
        break;
    }

left_top_motor.speed =
    Com_limit(left_top_motor.speed, 1000, 0);

left_bottom_motor.speed =
    Com_limit(left_bottom_motor.speed, 1000, 0);

right_top_motor.speed =
    Com_limit(right_top_motor.speed, 1000, 0);

right_bottom_motor.speed =
    Com_limit(right_bottom_motor.speed, 1000, 0);

    if (remote_data.thr < 50)
    {
        left_top_motor.speed = 0;
        left_bottom_motor.speed = 0;
        right_top_motor.speed = 0;
        right_bottom_motor.speed = 0;

    }



    Int_motor_set_speed(&left_top_motor);
    Int_motor_set_speed(&left_bottom_motor);
    Int_motor_set_speed(&right_top_motor);
    Int_motor_set_speed(&right_bottom_motor);

    static uint8_t motor_print_count = 0;

if (++motor_print_count >= 20)
{
    motor_print_count = 0;

    //debug_printf(
        ///"LT=%d, LB=%d, RT=%d, RB=%d, yOut=%.2f\r\n",
        //left_top_motor.speed,
        //left_bottom_motor.speed,
        //right_top_motor.speed,
        //right_bottom_motor.speed,
        //gyro_y_pid.output
    //);
}
}

void App_flight_fix_height_pid_process(void)
{
    height_pid.desire = fix_height;
    height_pid.measure = Int_VL53L1X_GetDistance();

    Com_PID_Calc(&height_pid);
}
