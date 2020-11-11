#include "mpu6050.h"
#include "bsp_bluetooth.h"
#include "bsp_i2c.h"
/**
 * @brief MPU6050芯片初始化配置函数
 * 
 */
void MPU6050_Init(void){
    uint8_t data;

    //唤醒MPU6050
    data=0x00;
    Sensor_Write(MPU6050_ADDRESS,MPU6050_RA_PWR_MGMT_1,1,&data);
    //MPU配置
    data=0x00;
    Sensor_Write(MPU6050_ADDRESS,MPU6050_RA_CONFIG,1,&data);
    //写入采样率 1K
    data=0x07;
    Sensor_Write(MPU6050_ADDRESS,MPU6050_RA_SMPLRT_DIV,1,&data);    
    //加速度计配置   4G
    data=0x08;
    Sensor_Write(MPU6050_ADDRESS,MPU6050_RA_ACCEL_CONFIG,1,&data);
    //陀螺仪配置   2000
    data=0x18;
    Sensor_Write(MPU6050_ADDRESS,MPU6050_RA_GYRO_CONFIG,1,&data);
    //I2C_MST关
    data=0xC0;
    Sensor_Write(MPU6050_ADDRESS,MPU6050_RA_USER_CTRL,1,&data);  
    //打开旁通模式
    data=0x82;
    Sensor_Write(MPU6050_ADDRESS,MPU6050_RA_INT_PIN_CFG,1,&data);
}

/**
 * @brief 读取MPU6050器件ID并且显示
 * 
 * @return uint8_t 成功返回0，失败返回1
 */
uint8_t MPU6050ReadID(void)
{
	unsigned char Re = 0;
    Sensor_Read(MPU6050_ADDRESS,MPU6050_RA_WHO_AM_I,1,&Re);    //读器件地址
    
	if(Re != 0x68)
	{
		MPU_ERROR("MPU6050 dectected error!\r\n检测不到MPU6050模块，请检查模块与开发板的接线");
		return 0;
	}
	else
	{
		MPU_INFO("MPU6050 ID = %d\r\n",Re);
		return 1;
	}		
}

/**
 * @brief 读取加速度计数据
 * 
 * @param accData 存放读取的加速度计数据
 * @return int 成功返回1,失败返回0
 */
int MPU6050ReadAcc(uint8_t *accData)
{
    Sensor_Read(MPU6050_ADDRESS,MPU6050_ACC_OUT,6, accData);//读取加速度六字节数据
	return 1;
}

/**
 * @brief 读取角速度计数据
 * 
 * @param gyroData 存放读取的角速度计数据
 * @return int 成功返回1,失败返回0
 */
int MPU6050ReadGyro(uint8_t *gyroData)
{
    Sensor_Read(MPU6050_ADDRESS,MPU6050_GYRO_OUT,6,gyroData); //读取陀螺仪六字节数据
	return 1;
}
