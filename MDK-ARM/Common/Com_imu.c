#include "Com_imu.h"

/* ============================ 欧拉角计算开始 ============================ */

/* 计算欧拉角用到的参数 */
float RtA = 57.2957795f;                 /* 弧度转角度 */
float Gyro_G = 4000.0f / 65536.0f;      /* 度/秒 */
float Gyro_Gr =
    4000.0f / 65536.0f / 180.0f * 3.1415926f; /* 弧度/秒 */

#define squa(Sq) (((float)(Sq)) * ((float)(Sq)))

/**
 * @brief 快速计算 1 / sqrt(number)
 * @param number 输入值
 * @return 平方根倒数
 */
static float Q_rsqrt(float number)
{
    long i;
    float x2;
    float y;
    const float threehalfs = 1.5f;

    x2 = number * 0.5f;
    y = number;

    i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;

    /* 第一次牛顿迭代 */
    y = y * (threehalfs - (x2 * y * y));

    return y;
}

/* Z 轴方向上的加速度 */
static double normAccz;

/**
 * @brief 根据 MPU6050 六轴数据计算欧拉角
 * @param gyroAccel MPU6050 六轴数据
 * @param eulerAngle 输出的欧拉角
 * @param dt 采样周期，单位为秒
 */
void Common_IMU_GetEulerAngle(
    Gyro_Accel_Struct *gyroAccel,
    Euler_struct *eulerAngle,
    float dt)
{
    volatile struct V
    {
        float x;
        float y;
        float z;
    } Gravity, Acc, Gyro, AccGravity;

    static struct V GyroIntegError = {0};
    static float KpDef = 0.8f;
    static float KiDef = 0.0003f;
    static Quaternion_Struct NumQ = {1, 0, 0, 0};

    float q0_t;
    float q1_t;
    float q2_t;
    float q3_t;
    float NormQuat;
    float HalfTime = dt * 0.5f;

    /* 提取等效旋转矩阵中的重力分量 */
    Gravity.x =
        2.0f * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);

    Gravity.y =
        2.0f * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);

    Gravity.z =
        1.0f - 2.0f *
        (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);

    /* 加速度归一化 */
    NormQuat = Q_rsqrt(
        squa(gyroAccel->accel.accel_x) +
        squa(gyroAccel->accel.accel_y) +
        squa(gyroAccel->accel.accel_z));

    Acc.x = gyroAccel->accel.accel_x * NormQuat;
    Acc.y = gyroAccel->accel.accel_y * NormQuat;
    Acc.z = gyroAccel->accel.accel_z * NormQuat;

    /* 加速度向量与估计重力向量作叉乘 */
    AccGravity.x =
        Acc.y * Gravity.z - Acc.z * Gravity.y;

    AccGravity.y =
        Acc.z * Gravity.x - Acc.x * Gravity.z;

    AccGravity.z =
        Acc.x * Gravity.y - Acc.y * Gravity.x;

    /* 对误差进行积分 */
    GyroIntegError.x += AccGravity.x * KiDef;
    GyroIntegError.y += AccGravity.y * KiDef;
    GyroIntegError.z += AccGravity.z * KiDef;

    /* 使用加速度误差补偿陀螺仪角速度 */
    Gyro.x =
        gyroAccel->gyro.gyro_x * Gyro_Gr +
        KpDef * AccGravity.x +
        GyroIntegError.x;

    Gyro.y =
        gyroAccel->gyro.gyro_y * Gyro_Gr +
        KpDef * AccGravity.y +
        GyroIntegError.y;

    Gyro.z =
        gyroAccel->gyro.gyro_z * Gyro_Gr +
        KpDef * AccGravity.z +
        GyroIntegError.z;

    /* 一阶龙格-库塔法更新四元数 */
    q0_t =
        (-NumQ.q1 * Gyro.x -
         NumQ.q2 * Gyro.y -
         NumQ.q3 * Gyro.z) *
        HalfTime;

    q1_t =
        (NumQ.q0 * Gyro.x -
         NumQ.q3 * Gyro.y +
         NumQ.q2 * Gyro.z) *
        HalfTime;

    q2_t =
        (NumQ.q3 * Gyro.x +
         NumQ.q0 * Gyro.y -
         NumQ.q1 * Gyro.z) *
        HalfTime;

    q3_t =
        (-NumQ.q2 * Gyro.x +
         NumQ.q1 * Gyro.y +
         NumQ.q0 * Gyro.z) *
        HalfTime;

    NumQ.q0 += q0_t;
    NumQ.q1 += q1_t;
    NumQ.q2 += q2_t;
    NumQ.q3 += q3_t;

    /* 四元数归一化 */
    NormQuat = Q_rsqrt(
        squa(NumQ.q0) +
        squa(NumQ.q1) +
        squa(NumQ.q2) +
        squa(NumQ.q3));

    NumQ.q0 *= NormQuat;
    NumQ.q1 *= NormQuat;
    NumQ.q2 *= NormQuat;
    NumQ.q3 *= NormQuat;

    /* 机体坐标系下的 Z 方向向量 */
    float vecxZ =
        2.0f * NumQ.q0 * NumQ.q2 -
        2.0f * NumQ.q1 * NumQ.q3;

    float vecyZ =
        2.0f * NumQ.q2 * NumQ.q3 +
        2.0f * NumQ.q0 * NumQ.q1;

    float veczZ =
        1.0f -
        2.0f * NumQ.q1 * NumQ.q1 -
        2.0f * NumQ.q2 * NumQ.q2;

    /* 将 Z 轴陀螺仪原始值转换为角速度 */
    float yaw_G =
        gyroAccel->gyro.gyro_z * Gyro_G;

    /*
     * 数据过小时认为是零偏或噪声，
     * 不认为发生了偏航动作。
     */
    if ((yaw_G > 0.5f) || (yaw_G < -0.5f))
    {
        /* 角速度积分得到偏航角 */
        eulerAngle->yaw += yaw_G * dt;
    }

    /* 计算俯仰角 */
    eulerAngle->pitch = asin(vecxZ) * RtA;

    /* 计算横滚角 */
    eulerAngle->roll = atan2f(vecyZ, veczZ) * RtA;

    /*
     * 计算机体 Z 轴方向的合加速度。
     * 倾斜时会考虑三个加速度轴在 Z 方向上的投影。
     */
    normAccz =
        gyroAccel->accel.accel_x * vecxZ +
        gyroAccel->accel.accel_y * vecyZ +
        gyroAccel->accel.accel_z * veczZ;
}

/**
 * @brief 获取机体 Z 轴方向上的合加速度
 * @return Z 轴方向上的加速度
 */
float Common_IMU_GetNormAccZ(void)
{
    return (float)normAccz;
}

/* ============================ 欧拉角计算结束 ============================ */
