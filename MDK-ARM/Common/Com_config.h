#ifndef _COM_CONFIG_H
#define _COM_CONFIG_H

#include "main.h"


// connected state
typedef enum
{
    REMOTE_CONNECTED = 0,
    REMOTE_DISCONNECTED,
}Remote_State;

// flight state
typedef enum
{
    IDLE = 0,
    NORMAL,
    FIX_HEIGHT,
    FAIL,
}Flight_State;

typedef enum
{
    FREE = 0,
    MAX,
    LEAVE_MAX,
    MIN,
    UNLOCK,
} Thr_state;

typedef struct 
{
    int16_t thr;
    int16_t yaw;
    int16_t pit;
    int16_t rol;
    uint8_t shutdown;     // 0 no shutdown 1 shutdown
    uint8_t fix_height;
}Remote_Data;

typedef struct 
{
    int16_t gyro_x; // Right Positive
    int16_t gyro_y; // Forward Positive
    int16_t gyro_z; // Anticlock-wise Positive
}Gyro_struct;

typedef struct
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
} Accel_struct;

typedef struct
{
    Gyro_struct gyro;
    Accel_struct accel;
}Gyro_Accel_Struct;

typedef struct
{
    float yaw;
    float pitch;
    float roll;
} Euler_struct;





#endif 
