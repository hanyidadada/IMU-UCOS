#include "hmc5883.h"
#include "bsp_bluetooth.h"
#include "bsp_i2c.h"
/**
 * @brief 纾佸姏璁¤姱鐗囧垵濮嬪寲
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
 * @brief 璇诲彇纾佸姏鏁版嵁
 * 
 * @param hmc 鏁版嵁瀛樺偍鎸囬拡锛岃鍙栧悗鐨勬暟鎹瓨鏀捐嚦璇ユ寚閽堝
 * @return int 鎴愬姛鍒欒繑鍥�1
 */
int HMC_Read(uint8_t* hmc){
    Sensor_Read(HMC5883L_ADDRESS,HMC5883L_DATA,6,hmc);//璇诲彇纾佸姏璁℃暟鎹�
	return 1;
}

