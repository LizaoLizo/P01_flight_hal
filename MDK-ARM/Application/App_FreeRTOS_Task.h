#ifndef __APP_FREERTOS_TASK__
#define __APP_FREERTOS_TASK__
#define POWER_TASK_PERIOD pdMS_TO_TICKS(10000)

#include "FreeRTOS.h"
#include "task.h"
#include "Com_debug.h"
#include "Com_config.h"
#include "Int_IP5305T.h"
#include "Int_motor.h"
#include "Int_led.h"
#include "Int_SI24R1.h"
#include <string.h>
#include "App_receive_data.h"
#include "App_flight.h"
#include "i2c.h"
#include "Int_bat_ADC.h"

void App_FreeRTOS_start(void);

#endif // __APP_FREERTOS_TASK__
