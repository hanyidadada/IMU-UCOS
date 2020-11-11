#ifndef __BSP_USART_H
#define	__BSP_USART_H

#include "stm32f4xx.h"
#include "stdio.h"
//瀹氫箟涓插彛閰嶇疆
#define DEBUG_USART                             USART3
#define DEBUG_USART_CLK                         RCC_APB1Periph_USART3
#define DEBUG_USART_BAUDRATE                    9600 //涓插彛娉㈢壒鐜�
#define DEBUG_USART_IRQHandler                  USART3_IRQHandler
#define DEBUG_USART_IRQ                 		USART3_IRQn
//瀹氫箟RX寮曡剼鐩稿叧閰嶇疆
#define DEBUG_USART_RX_GPIO_PORT                GPIOD
#define DEBUG_USART_RX_GPIO_CLK                 RCC_AHB1Periph_GPIOD
#define DEBUG_USART_RX_PIN                      GPIO_Pin_9
#define DEBUG_USART_RX_AF                       GPIO_AF_USART3
#define DEBUG_USART_RX_SOURCE                   GPIO_PinSource9
//瀹氫箟TX寮曡剼鐩稿叧閰嶇疆
#define DEBUG_USART_TX_GPIO_PORT                GPIOD
#define DEBUG_USART_TX_GPIO_CLK                 RCC_AHB1Periph_GPIOD
#define DEBUG_USART_TX_PIN                      GPIO_Pin_8
#define DEBUG_USART_TX_AF                       GPIO_AF_USART3
#define DEBUG_USART_TX_SOURCE                   GPIO_PinSource8

void Debug_USART_Config(void);
void Send_Senser(int16_t ACCEL_X, int16_t ACCEL_Y, int16_t ACCEL_Z, int16_t GYRO_X, int16_t GYRO_Y, int16_t GYRO_Z, int16_t MAG_X, int16_t MAG_Y, int16_t MAG_Z);
void Send_Attitude(float roll, float pitch, float yaw);
void SendStr(const char* str);
void SendByte(u8 frame, u8* p);
void SendHalfWord(u8 frame, u16* p);
void SendWord(u8 frame, u32* p);
void Send_5_float(u8 frame,float f1,float f2,float f3,float f4,float f5);
#endif 

