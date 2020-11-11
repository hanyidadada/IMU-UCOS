#ifndef __APP_H__
#define __APP_H__

#include "ucos_ii.h"
#include "bsp_bluetooth.h"
#include "bsp_i2c.h"
#include "mpu6050.h"
#include "hmc5883.h"
#include "MahonyAHRS.h"
//任务优先级定义
#define TASK_COM_STK_SIZE               512
#define TASK_COM_PRIO                   7
#define TASK_AHRS_STK_SIZE              512
#define TASK_AHRS_PRIO                  6

extern OS_STK		Task_ComStack[TASK_COM_STK_SIZE];
extern OS_STK		Task_AHRSStack[TASK_COM_STK_SIZE];
//任务函数定义
void Task_COM(void *p_arg);
void Task_AHRS(void *p_arg);
void BSP_Init(void);

#endif


