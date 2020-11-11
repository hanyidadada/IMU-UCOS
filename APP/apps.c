#include "apps.h"
//变量声明
extern uint8_t gyroOffset, accOffset; //用于零偏校准
extern Acc_t acc, offsetAcc; //原始数据、零偏数据
extern Gyro_t gyro, offsetGyro; //原始数据、零偏数据
extern Mag_t mag; //原始数据
extern Float_t fGyro; //角速度数据（rad）
extern Angle_t angle; //姿态解算-角度值


/**
 * @brief 串口任务函数
 * 
 * @param p_arg 
 */
void Task_COM(void *p_arg){
	OS_CPU_SysTickInit();
	while (1) {
        Send_Senser(acc.x, acc.y, acc.z, gyro.x * RAW_TO_ANGLE, gyro.y * RAW_TO_ANGLE, gyro.z * RAW_TO_ANGLE, mag.x, mag.y, mag.z); //发送传感器原始数据帧
        
				if (!Calib_Status()) { //零偏校准结束
            Send_Attitude(angle.roll, angle.pitch, angle.yaw); //发送姿态数据帧
        }
        OSTimeDly(10);
    }	
}

/**
 * @brief 姿态解算任务
 * 
 * @param p_arg 
 */
void Task_AHRS(void *p_arg){
	OS_CPU_SysTickInit();
	while (1) {
        GY86_Read(); //读取加速度、角速度、磁场强度
				if (!Calib_Status()) { //零偏校准结束
            Attitude_Update(fGyro.x, fGyro.y, fGyro.z, acc.x, acc.y, acc.z, mag.x, mag.y, mag.z); //姿态解算
        }
        OSTimeDly(1);
    }
}

/**
 * @brief 硬件初始化函数
 * 
 */
void BSP_Init(void){
	Debug_USART_Config();
	IIC_Config();
	MPU6050_Init();
	if(MPU6050ReadID()){
		printf("MPU Init SUCCESS.\n");
	}
	HMC_Init();
	
}
