#include "stm32f4xx.h"
#include "apps.h"
/**********************************姿态、高度解算相关******************************************************/
uint8_t gyroOffset, accOffset; //用于零偏校准
Acc_t acc, offsetAcc; //原始数据、零偏数据
Gyro_t gyro, offsetGyro; //原始数据、零偏数据
Mag_t mag; //原始数据
Float_t fGyro; //角速度数据（rad）
Angle_t angle; //姿态解算-角度值
u8 sendBuf[50];//发送缓存

OS_STK		Task_ComStack[TASK_COM_STK_SIZE];
OS_STK		Task_AHRSStack[TASK_COM_STK_SIZE];

/**
 * @brief 主函数main
 * 
 * @return int 
 */
int main(void)
{ 
	OSInit();   
	BSP_Init();
	Quat_Init();
	AHRS_Time_Init();
	Open_Calib();
	OSTaskCreate(Task_COM,(void *)0,(OS_STK *)&Task_ComStack[TASK_COM_STK_SIZE-1],TASK_COM_PRIO );
	OSTaskCreate(Task_AHRS,(void *)0,(OS_STK *)&Task_AHRSStack[TASK_COM_STK_SIZE-1],TASK_AHRS_PRIO );
	OSStart();	
}



