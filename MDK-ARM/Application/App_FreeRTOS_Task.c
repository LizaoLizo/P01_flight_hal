#include "App_FreeRTOS_Task.h" 

LED_Struct left_top_led = {.port = LED1_GPIO_Port, .pin = LED1_Pin};
LED_Struct right_top_led = {.port = LED2_GPIO_Port, .pin = LED2_Pin};
LED_Struct right_bottom_led = {.port = LED3_GPIO_Port, .pin = LED3_Pin};
LED_Struct left_bottom_led = {.port = LED4_GPIO_Port, .pin = LED4_Pin};

Remote_State remote_state = REMOTE_DISCONNECTED;

Flight_State flight_state = IDLE;

Remote_Data remote_data = {.thr = 0, .yaw = 500, .pit = 500, .rol = 500, .fix_height = 0, .shutdown = 0};

uint16_t fix_height = 0;

uint8_t back_buff[TX_PLOAD_WIDTH] = {0};

void power_task(void * args);

#define POWER_TASK_STACK_SIZE 128

#define POWER_TASK_PRIORITY 4

TaskHandle_t power_task_handle;


void flight_task(void *args);
#define FLIGHT_TASK_STACK_SIZE 128
#define FLIGHT_TASK_PRIORITY 3
TaskHandle_t flight_task_handle;
#define FLIGHT_TASK_PERIOD 6

void led_task(void * args);
#define LED_TASK_STACK_SIZE 128
#define LED_TASK_PRIORITY 1
TaskHandle_t led_task_handle;
#define LED_TASK_PERIOD 100

void com_task(void *args);
#define COM_TASK_STACK_SIZE 128
#define COM_TASK_PRIORITY 4
TaskHandle_t com_task_handle;
// ��������
#define COM_TASK_PERIOD 10


void App_FreeRTOS_start(void)
{
    xTaskCreate(power_task, "power_task", POWER_TASK_STACK_SIZE, NULL, POWER_TASK_PRIORITY, &power_task_handle);

    xTaskCreate(flight_task, "flight_task", FLIGHT_TASK_STACK_SIZE, NULL, FLIGHT_TASK_PRIORITY, &flight_task_handle);

    xTaskCreate(led_task, "led_task", LED_TASK_STACK_SIZE, NULL, LED_TASK_PRIORITY, &led_task_handle);

    xTaskCreate(com_task, "com_task", COM_TASK_STACK_SIZE, NULL, COM_TASK_PRIORITY, &com_task_handle);

    vTaskStartScheduler();
}

void power_task( void * args)
{

    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1)
    {

        //vTaskDelayUntil(&xLastWakeTime, POWER_TASK_PERIOD);

        //Int_IP5305T_start();

        uint32_t res = ulTaskNotifyTake(pdTRUE, POWER_TASK_PERIOD);
        if (res != 0)
        {
            Int_IP5305T_shutdown();
        }
        else
        {
            Int_IP5305T_start();
        }
    }
}

void flight_task(void *args)
{
    /* 初始全部关闭 */
    left_top_motor.speed = 0;
    left_bottom_motor.speed = 0;
    right_top_motor.speed = 0;
    right_bottom_motor.speed = 0;

    Int_motor_set_speed(&left_top_motor);
    Int_motor_set_speed(&left_bottom_motor);
    Int_motor_set_speed(&right_top_motor);
    Int_motor_set_speed(&right_bottom_motor);

    // 内部完成校准

    Int_motor_start(&left_top_motor);
    Int_motor_start(&left_bottom_motor);
    Int_motor_start(&right_top_motor);
    Int_motor_start(&right_bottom_motor);


    TickType_t xLastWakeTime = xTaskGetTickCount();

    uint8_t count = 0;

    App_flight_init();


while (1)
{
    App_flight_get_euler_angle();

    App_flight_pid_process();

    if (flight_state == FIX_HEIGHT)
    {
        count++;
        if (count >= 4)
        {
            App_flight_fix_height_pid_process();
            count = 0;
        }
    }

    App_flight_control_motor();

    uint16_t distance = Int_VL53L1X_GetDistance();

    //printf("%u\r\n", (unsigned int)distance);

    vTaskDelayUntil(&xLastWakeTime, FLIGHT_TASK_PERIOD);
}
}

void led_task(void *args)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint8_t count = 0;

    while (1)
    {
        count++;
        // first two light show connected state
        if (remote_state == REMOTE_CONNECTED)
        {
            //determine the current connected state
            Int_led_turn_on(&left_top_led);
            Int_led_turn_on(&right_top_led);
        }
        else if (remote_state == REMOTE_DISCONNECTED)
        {
            Int_led_turn_off(&left_top_led);
            Int_led_turn_off(&right_top_led);
        }
        
        // last two light show flight state
        if (flight_state == IDLE)
        {
            // 500ms on 500ms off
            if (count % 5 == 0)
            {
                Int_led_toggle(&left_bottom_led);

                Int_led_toggle(&right_bottom_led);
            }
        }
        else if (flight_state == NORMAL)
        {
            // 200ms on 200ms off
            if (count % 2 == 0)
            {
                Int_led_toggle(&left_bottom_led);

                Int_led_toggle(&right_bottom_led);
            }
        }
        else if (flight_state == FIX_HEIGHT)
        {
            Int_led_turn_on(&left_bottom_led);

            Int_led_turn_on(&right_bottom_led);
        }
        else if (flight_state == FAIL)
        {
            Int_led_turn_off(&left_bottom_led);
            Int_led_turn_off(&right_bottom_led);
        }

        // reset the count
        if (count == 10)
        {
            count = 0;
        }




        vTaskDelayUntil(
            &xLastWakeTime,
            pdMS_TO_TICKS(500)
        );
    }
}


void com_task(void *args)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    //Int_bat_ADC_Init();

    while (1)
    {
        // Receive Data
        uint8_t res = App_receive_data();
        
        // According to the return value and solve connected state
       App_receive_process_connect_state(res);

        if (res == 0 && remote_data.shutdown == 1)
{
    /* 防止飞行任务重新启动电机 */
    //vTaskSuspend(flight_task_handle);

    left_top_motor.speed = 0;
    left_bottom_motor.speed = 0;
    right_top_motor.speed = 0;
    right_bottom_motor.speed = 0;

    Int_motor_set_speed(&left_top_motor);
    Int_motor_set_speed(&left_bottom_motor);
    Int_motor_set_speed(&right_top_motor);
    Int_motor_set_speed(&right_bottom_motor);

    flight_state = FAIL;
    remote_state = REMOTE_DISCONNECTED;

    xTaskNotifyGive(power_task_handle);
}


    App_process_flight_state();

/*
float voltage = Int_bat_ADC_Read();

uint16_t voltage_100 =
    (uint16_t)(voltage * 100.0f + 0.5f);

sprintf((char *)back_buff,
        "%u.%02u",
        voltage_100 / 100U,
        voltage_100 % 100U);

debug_printf("voltage:%u.%02u\r\n",
             voltage_100 / 100U,
             voltage_100 % 100U);
*/
    vTaskDelay(COM_TASK_PERIOD);
    }

    // Flight State

}
