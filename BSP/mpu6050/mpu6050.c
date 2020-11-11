#include "mpu6050.h"
#include "bsp_bluetooth.h"
#include "bsp_i2c.h"
/**
 * @brief MPU6050оƬ��ʼ�����ú���
 * 
 */
void MPU6050_Init(void){
    uint8_t data;

    //����MPU6050
    data=0x00;
    Sensor_Write(MPU6050_ADDRESS,MPU6050_RA_PWR_MGMT_1,1,&data);
    //MPU����
    data=0x00;
    Sensor_Write(MPU6050_ADDRESS,MPU6050_RA_CONFIG,1,&data);
    //д������� 1K
    data=0x07;
    Sensor_Write(MPU6050_ADDRESS,MPU6050_RA_SMPLRT_DIV,1,&data);    
    //���ٶȼ�����   4G
    data=0x08;
    Sensor_Write(MPU6050_ADDRESS,MPU6050_RA_ACCEL_CONFIG,1,&data);
    //����������   2000
    data=0x18;
    Sensor_Write(MPU6050_ADDRESS,MPU6050_RA_GYRO_CONFIG,1,&data);
    //I2C_MST��
    data=0xC0;
    Sensor_Write(MPU6050_ADDRESS,MPU6050_RA_USER_CTRL,1,&data);  
    //����ͨģʽ
    data=0x82;
    Sensor_Write(MPU6050_ADDRESS,MPU6050_RA_INT_PIN_CFG,1,&data);
}

/**
 * @brief ��ȡMPU6050����ID������ʾ
 * 
 * @return uint8_t �ɹ�����0��ʧ�ܷ���1
 */
uint8_t MPU6050ReadID(void)
{
	unsigned char Re = 0;
    Sensor_Read(MPU6050_ADDRESS,MPU6050_RA_WHO_AM_I,1,&Re);    //��������ַ
    
	if(Re != 0x68)
	{
		MPU_ERROR("MPU6050 dectected error!\r\n��ⲻ��MPU6050ģ�飬����ģ���뿪����Ľ���");
		return 0;
	}
	else
	{
		MPU_INFO("MPU6050 ID = %d\r\n",Re);
		return 1;
	}		
}

/**
 * @brief ��ȡ���ٶȼ�����
 * 
 * @param accData ��Ŷ�ȡ�ļ��ٶȼ�����
 * @return int �ɹ�����1,ʧ�ܷ���0
 */
int MPU6050ReadAcc(uint8_t *accData)
{
    Sensor_Read(MPU6050_ADDRESS,MPU6050_ACC_OUT,6, accData);//��ȡ���ٶ����ֽ�����
	return 1;
}

/**
 * @brief ��ȡ���ٶȼ�����
 * 
 * @param gyroData ��Ŷ�ȡ�Ľ��ٶȼ�����
 * @return int �ɹ�����1,ʧ�ܷ���0
 */
int MPU6050ReadGyro(uint8_t *gyroData)
{
    Sensor_Read(MPU6050_ADDRESS,MPU6050_GYRO_OUT,6,gyroData); //��ȡ���������ֽ�����
	return 1;
}
