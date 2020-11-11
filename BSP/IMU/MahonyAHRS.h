#ifndef MahonyAHRS_H
#define MahonyAHRS_H
#include "stm32f4xx.h"
#include "math.h"

#define PI 3.1415927f
//角度单位相关转换
#define RAD_TO_ANGLE 57.2957795f //弧度转角度
#define RAW_TO_ANGLE 0.0610351f //原始数据转角度，对应±2000°/s
#define RAW_TO_RAD 0.0010653f //原始数据转弧度，对应±2000°/s

#define FILTER_NUM 2

typedef enum {
    ACC_GYRO_MAG = 0,
    TEMP_PRESS
} Type_t;

/* MPU6050--加速度计结构体 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} Acc_t;

/* MPU6050--陀螺仪结构体 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} Gyro_t;

/* HMC5883L--磁力计结构体 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} Mag_t;

/* float结构体 */
typedef struct
{
    float x;
    float y;
    float z;
} Float_t;

/* 姿态解�?--角度�? */
typedef struct
{
    float yaw;
    float roll;
    float pitch;
} Angle_t;

// 函数声明
float invSqrt(float x);
void Open_Calib(void);//打开零偏校准
u8 Calib_Status(void);//读取零偏校准状态
void MPU6050_Offset(void);//进行零偏校准
void GY86_Read(void);//读取GY-86数据
void Quat_Init(void);//四元数初始化
void Attitude_Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);//更新姿态角
void AHRS_Time_Init(void);//姿态解算时钟初始化
float Get_AHRS_Time(void);//获取姿态解算时间

#endif
