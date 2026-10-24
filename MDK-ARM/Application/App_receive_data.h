#ifndef __APP_RECEIVE_DATA__
#define __APP_RECEIVE_DATA__

#include "Int_SI24R1.h"
#include "Com_config.h"
#include "Int_VL53L1X.h"

#define FRAME_HEAD_CHECK_1 's'
#define FRAME_HEAD_CHECK_2 'g'
#define FRAME_HEAD_CHECK_3 'g'

#define MAX_RETRY_TIMES 10


uint8_t App_receive_data(void);

void App_receive_process_connect_state(uint8_t res);

void App_process_flight_state(void);

#endif     // __APP_RECEIVE_DATA__
