#include "Int_mpu6050.h"
#include <stdio.h>

static int32_t acc_x_offset = 0;
static int32_t acc_y_offset = 0;
static int32_t acc_z_offset = 0;

int32_t gyro_x_offset = 0;
int32_t gyro_y_offset = 0;
int32_t gyro_z_offset = 0;

void Int_MPU6050_Write_Reg(uint8_t reg, uint8_t data)
{
    // HAL have fixed Read Write Function
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR_WRITE, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

}

void Int_MPU6050_Read_Reg(uint8_t reg, uint8_t * data)
{

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR_READ, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 1000);
}

void Int_MPU6050_calculate_offset(void)
{
    // 1. Waiting for Drone be stable
    Accel_struct current_accel = {0};
    Accel_struct last_accel = {0};
    uint8_t count = 0;
    Int_MPU6050_Get_Acc(&last_accel);

    while (count < 100)
    {
        Int_MPU6050_Get_Acc(&current_accel);
        if (abs(current_accel.accel_x - last_accel.accel_x) < 400 &&
    abs(current_accel.accel_y - last_accel.accel_y) < 400 &&
    abs(current_accel.accel_z - last_accel.accel_z) < 400)
    {
        count++;
    }
    else
    {
        count = 0;
    }
    last_accel = current_accel;
    vTaskDelay(6);
    }

    Gyro_Accel_Struct gyro_accel_data = {0};
    int32_t acc_x_sum = 0;
    int32_t acc_y_sum = 0;
    int32_t acc_z_sum = 0;

    int32_t gyro_x_sum = 0;
    int32_t gyro_y_sum = 0;
    int32_t gyro_z_sum = 0;

    for (uint8_t i = 0; i < 100; i++)
    {
        Int_MPU6050_Get_Data(&gyro_accel_data);
        acc_x_sum += (gyro_accel_data.accel.accel_x - 0);
        acc_y_sum += (gyro_accel_data.accel.accel_y - 0);
        acc_z_sum += (gyro_accel_data.accel.accel_z - 16384);

        gyro_x_sum += (gyro_accel_data.gyro.gyro_x - 0);
        gyro_y_sum += (gyro_accel_data.gyro.gyro_y - 0);
        gyro_z_sum += (gyro_accel_data.gyro.gyro_z - 0);

        // EVERY MEASUREMENT DELAY
        vTaskDelay(6);
    }

    acc_x_offset = acc_x_sum / 100;
    acc_y_offset = acc_y_sum / 100;
    acc_z_offset = acc_z_sum / 100;

    gyro_x_offset = gyro_x_sum / 100;
    gyro_y_offset = gyro_y_sum / 100;
    gyro_z_offset = gyro_z_sum / 100;


}

void Int_MPU6050_Init(void)
{
    printf("MPU INIT START\r\n");

    Int_MPU6050_Write_Reg(0x6B, 0x80);

    uint8_t data = 0;

    while (data != 0x40)
    {
        Int_MPU6050_Read_Reg(0x6B, &data);
    }

    printf("MPU RESET OK\r\n");
    
        Int_MPU6050_Write_Reg(0x6B, 0x00);

    Int_MPU6050_Write_Reg(0x1B, 3 << 3);
    
    Int_MPU6050_Write_Reg(0x1C, 0x00);

    Int_MPU6050_Write_Reg(0x38, 0x00);

    Int_MPU6050_Write_Reg(0x6A, 0x00);

    Int_MPU6050_Write_Reg(0x19, 0x01);

    Int_MPU6050_Write_Reg(0x1A, 1);

    Int_MPU6050_Write_Reg(0x6B, 0x01);

    Int_MPU6050_Write_Reg(0x6C, 0x00);

    printf("CAL START\r\n");
    Int_MPU6050_calculate_offset();
    printf("CAL DONE\r\n");


}

void Int_MPU6050_Get_Gyro(Gyro_struct *gyro)
{
    uint8_t height = 0;
    uint8_t low = 0;
    int16_t raw;

    Int_MPU6050_Read_Reg(MPU_GYRO_XOUTH_REG, &height);
    Int_MPU6050_Read_Reg(MPU_GYRO_XOUTL_REG, &low);
    raw = (int16_t)(((uint16_t)height << 8) | low);
gyro->gyro_x = (int16_t)(raw - gyro_x_offset);

    Int_MPU6050_Read_Reg(MPU_GYRO_YOUTH_REG, &height);
    Int_MPU6050_Read_Reg(MPU_GYRO_YOUTL_REG, &low);
   raw = (int16_t)(((uint16_t)height << 8) | low);
gyro->gyro_y = (int16_t)(raw - gyro_y_offset);

    Int_MPU6050_Read_Reg(MPU_GYRO_ZOUTH_REG, &height);
    Int_MPU6050_Read_Reg(MPU_GYRO_ZOUTL_REG, &low);
   raw = (int16_t)(((uint16_t)height << 8) | low);
gyro->gyro_z = (int16_t)(raw - gyro_z_offset);
}

void Int_MPU6050_Get_Acc(Accel_struct *acc)
{
    uint8_t height = 0;
    uint8_t low = 0;
    int16_t raw;

    Int_MPU6050_Read_Reg(MPU_ACCEL_XOUTH_REG, &height);
    Int_MPU6050_Read_Reg(MPU_ACCEL_XOUTL_REG, &low);
    raw = (int16_t)(((uint16_t)height << 8) | low);
    acc->accel_x = (int16_t)(raw - acc_x_offset);

    Int_MPU6050_Read_Reg(MPU_ACCEL_YOUTH_REG, &height);
    Int_MPU6050_Read_Reg(MPU_ACCEL_YOUTL_REG, &low);
    raw = (int16_t)(((uint16_t)height << 8) | low);
    acc->accel_y = (int16_t)(raw - acc_y_offset);

    Int_MPU6050_Read_Reg(MPU_ACCEL_ZOUTH_REG, &height);
    Int_MPU6050_Read_Reg(MPU_ACCEL_ZOUTL_REG, &low);
    raw = (int16_t)(((uint16_t)height << 8) | low);
    acc->accel_z = (int16_t)(raw - acc_z_offset);
}

void Int_MPU6050_Get_Data(Gyro_Accel_Struct * data)
{
    Int_MPU6050_Get_Gyro(&data->gyro);
    Int_MPU6050_Get_Acc(&data->accel);
}