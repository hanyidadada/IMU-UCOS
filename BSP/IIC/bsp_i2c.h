#ifndef __BSP_I2C_H
#define __BSP_I2C_H
#include "stm32f4xx.h"

#define I2Cx_FLAG_TIMEOUT             ((uint32_t) 900) 
#define I2Cx_LONG_TIMEOUT             ((uint32_t) (300 * I2Cx_FLAG_TIMEOUT)) 
#define I2C_INFO(fmt,arg...)           printf("<<-I2C-INFO->> "fmt"\n",##arg)
#define I2C_ERROR(fmt,arg...)          printf("<<-I2C-ERROR->> "fmt"\n",##arg)
//GY-86使用的IIC定义
#define GY_Speed            400000
#define STM32_OWN_ADRESS7   0X00  
#define GY_I2C              I2C1
#define GY_I2C_CLK          RCC_APB1Periph_I2C1
#define GY_I2C_ClockCmd     RCC_APB1PeriphClockCmd  
//SCL相关引脚定义
#define GY_SCL_GPIO_PORT    GPIOB
#define GY_SCL_GPIO_PIN     GPIO_Pin_6
#define GY_SCL_GPIO_CLK     RCC_AHB1Periph_GPIOB
#define GY_SCL_AF           GPIO_AF_I2C1
#define GY_SCL_PINSOURCE    GPIO_PinSource6
#define GY_SCL_CLOCKCMD     RCC_AHB1PeriphClockCmd
//SDA相关引脚定义
#define GY_SDA_GPIO_PORT    GPIOB
#define GY_SDA_GPIO_PIN     GPIO_Pin_7
#define GY_SDA_GPIO_CLK     RCC_AHB1Periph_GPIOB
#define GY_SDA_AF           GPIO_AF_I2C1
#define GY_SDA_PINSOURCE    GPIO_PinSource7
#define GY_SDA_CLOCKCMD     RCC_AHB1PeriphClockCmd

void Soft_Dely(uint32_t num);
void IIC_Config(void);
int IIC_Byte_Write(unsigned char slave_addr,unsigned char reg_addr,unsigned short lens,const unsigned char* data);
int IIC_Byte_Read(unsigned char slave_addr,unsigned char reg_addr,unsigned short lens,unsigned char* data);
int Sensor_Write(uint8_t slave_addr,uint8_t reg_addr,unsigned short lens,uint8_t* data);
int Sensor_Read(uint8_t slave_addr,uint8_t reg_addr,unsigned short lens,uint8_t* data);
static uint32_t I2Cx_TIMEOUT_UserCallback(char value);
#endif
