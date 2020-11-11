#include "hmc5883.h"
#include "bsp_bluetooth.h"
#include "bsp_i2c.h"
/**
 * @brief 磁力计初始化
 * 
 */
void HMC_Init(void){
    uint8_t data=0X00;
	
    data = 0x70;
	Sensor_Write(HMC5883L_ADDRESS,HMC5883L_CONFIG_A,1,&data);
	data = 0xE0;
	Sensor_Write(HMC5883L_ADDRESS,HMC5883L_CONFIG_B,1,&data);
	Soft_Dely(0xFFFFF);
	data = 0x00;
    Sensor_Write(HMC5883L_ADDRESS,HMC5883L_MODE,1,&data);
}

/**
 * @brief 读取磁力计数据
 * 
 * @param hmc 数据存储指针
 * @return int 成功返回1
 */
int HMC_Read(uint8_t* hmc){
    Sensor_Read(HMC5883L_ADDRESS,HMC5883L_DATA,6,hmc);//璇诲彇纾佸姏璁℃暟鎹�
	return 1;
}

