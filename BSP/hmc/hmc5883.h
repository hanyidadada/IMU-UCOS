#ifndef __HMC5883_H
#define __HMC5883_H
#include "stm32f4xx.h"
//磁力计相关寄存器地址宏定义
#define HMC5883L_CONFIG_A         0x00
#define HMC5883L_CONFIG_B         0x01
#define HMC5883L_MODE             0x02
#define HMC5883L_DATA             0x03
#define HMC5883L_STATUS           0x09
#define HMC5883L_IDENTIFICATION_A 0x0A
#define HMC5883L_IDENTIFICATION_B 0x0B
#define HMC5883L_IDENTIFICATION_C 0x0C

#define HMC5883L_ADDRESS          0x1E            

int HMC_Read(uint8_t* hmc);
void HMC_Init(void);
#endif

