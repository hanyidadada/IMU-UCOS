#include "bsp_i2c.h"
#include "bsp_bluetooth.h"

/**
 * @brief IIC鍒濆鍖栭厤缃紝IIC1浣跨敤PB6,PB7
 * 
 */
void IIC_Config(void){
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;
    
    GY_I2C_ClockCmd(GY_I2C_CLK,ENABLE);
    
    GY_SCL_CLOCKCMD(GY_SCL_GPIO_CLK,ENABLE);
    GY_SDA_CLOCKCMD(GY_SDA_GPIO_CLK,ENABLE);
    GPIO_PinAFConfig(GY_SCL_GPIO_PORT,GY_SCL_PINSOURCE,GY_SCL_AF);
    GPIO_PinAFConfig(GY_SDA_GPIO_PORT,GY_SDA_PINSOURCE,GY_SDA_AF);
    
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType=GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    
    GPIO_InitStructure.GPIO_Pin=GY_SCL_GPIO_PIN;
    GPIO_Init(GY_SCL_GPIO_PORT,&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin=GY_SDA_GPIO_PIN;
    GPIO_Init(GY_SDA_GPIO_PORT,&GPIO_InitStructure);
    
    I2C_InitStructure.I2C_Ack=I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress=I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed=GY_Speed;
    I2C_InitStructure.I2C_DutyCycle=I2C_DutyCycle_2;
    I2C_InitStructure.I2C_Mode=I2C_Mode_I2C;
    I2C_InitStructure.I2C_OwnAddress1=STM32_OWN_ADRESS7;
       
    I2C_Cmd(GY_I2C,ENABLE);
	I2C_Init(GY_I2C,&I2C_InitStructure);
    I2C_AcknowledgeConfig(GY_I2C, ENABLE);  
}

/**
 * @brief iic鍐欏嚱鏁帮紝鍚戞寚瀹氬湴鍧€鍙戦€佺壒瀹氬瓧鑺傞暱搴︽暟鎹�
 * 
 * @param slave_addr 浠庢満鍦板潃
 * @param reg_addr 瀵勫瓨鍣ㄥ湴鍧€
 * @param lens 鏁版嵁闀垮害
 * @param data 鏁版嵁鎸囬拡
 * @return int 鎴愬姛杩斿洖0锛屽嚭閿欒繑鍥�1
 */
int IIC_Byte_Write(unsigned char slave_addr,unsigned char reg_addr,unsigned short lens,const unsigned char* data){
    uint32_t count_wait=I2Cx_LONG_TIMEOUT;
    uint32_t i=0;
    int value=0;
   //绛夊緟I2C閫€鍑哄繖
    while(I2C_GetFlagStatus(GY_I2C,I2C_FLAG_BUSY)!= RESET){
        if(count_wait--==0) return I2Cx_TIMEOUT_UserCallback(1);
    } 
    
    //浜х敓璧峰淇″彿
    I2C_GenerateSTART(GY_I2C,ENABLE);
    //绛夊緟EV5
    count_wait=I2Cx_LONG_TIMEOUT;
     while(I2C_GetFlagStatus(GY_I2C,I2C_FLAG_SB)!=SET){
            if(count_wait--==0){ return I2Cx_TIMEOUT_UserCallback(2);}
    }
    
    //鍙戦€佽澶囧湴鍧€
    I2C_Send7bitAddress(GY_I2C,(slave_addr<<1),I2C_Direction_Transmitter);
    //妫€娴婨V6浜嬩欢
    count_wait=I2Cx_LONG_TIMEOUT;
    while(I2C_CheckEvent(GY_I2C,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)!=SUCCESS){
        if(count_wait--==0) return  I2Cx_TIMEOUT_UserCallback(3);
    }
    
    //娓呴櫎ADDR鏍囧織浣�
    I2C_ReadRegister(GY_I2C,I2C_Register_SR1);
    I2C_ReadRegister(GY_I2C,I2C_Register_SR2);
    //绛夊緟浼犺緭
    count_wait=I2Cx_LONG_TIMEOUT;
    while(I2C_GetFlagStatus(GY_I2C,I2C_FLAG_TXE)!=SUCCESS){
        if(count_wait--==0){ return I2Cx_TIMEOUT_UserCallback(4);}
    }
    //鍙戦€佸啓鍏ョ殑瀛樺偍鍗曞厓鍦板潃
    I2C_SendData(GY_I2C,reg_addr);

    
    for(i=0;i<lens;i++){
          //妫€娴婨V8_2
        count_wait=I2Cx_LONG_TIMEOUT;
        while(I2C_CheckEvent(GY_I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTED)!=SUCCESS){
            if(count_wait--==0) 
            {    return  I2Cx_TIMEOUT_UserCallback(5);
            }
        }
        //鍙戦€佸啓鍏ョ殑鏁版嵁
        I2C_SendData(GY_I2C,data[i]);
    }
    //妫€娴婨V8_2
    count_wait=I2Cx_LONG_TIMEOUT;
    while(I2C_CheckEvent(GY_I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTED)!=SUCCESS){
        if(count_wait--==0) return  I2Cx_TIMEOUT_UserCallback(6);
    }
    //浜х敓缁撴潫淇″彿
    I2C_GenerateSTOP(GY_I2C,ENABLE);
    
    return value;
}

/**
 * @brief IIC璇诲嚱鏁帮紝浠庢寚瀹氬湴鍧€璇诲彇瀹氶暱鐨勬暟鎹�
 * 
 * @param slave_addr 浠庢満鍦板潃
 * @param reg_addr 瀵勫瓨鍣ㄥ湴鍧€
 * @param lens 璇诲彇鏁版嵁闀垮害
 * @param data 瀛樺偍鏁版嵁鎸囬拡
 * @return int 鎴愬姛杩斿洖0锛屽け璐ヨ繑鍥�1
 */
int IIC_Byte_Read(unsigned char slave_addr,unsigned char reg_addr,unsigned short lens,unsigned char* data){
    uint32_t count_wait=I2Cx_LONG_TIMEOUT;
    uint32_t i=0;
    int value=0;
    //绛夊緟I2C閫€鍑哄繖
    while(I2C_GetFlagStatus(GY_I2C,I2C_FLAG_BUSY)!=RESET){
        if(count_wait--==0) return I2Cx_TIMEOUT_UserCallback(7);
    } 
    
    //绗竴娆′骇鐢熻捣濮嬩俊鍙�
    I2C_GenerateSTART(GY_I2C,ENABLE);
    
    //绛夊緟EV5
    count_wait=I2Cx_LONG_TIMEOUT;
    while(I2C_GetFlagStatus(GY_I2C,I2C_FLAG_SB)!=SET){
            if(count_wait--==0) return I2Cx_TIMEOUT_UserCallback(8);
    }
    
    //鍙戦€佽澶囧湴鍧€
    I2C_Send7bitAddress(GY_I2C,(slave_addr<<1),I2C_Direction_Transmitter);
    //妫€娴婨V6浜嬩欢
    count_wait=I2Cx_LONG_TIMEOUT;
    while(I2C_CheckEvent(GY_I2C,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)!=SUCCESS){
        if(count_wait--==0) return  I2Cx_TIMEOUT_UserCallback(9);
    }
    
    //娓呴櫎ADDR鏍囧織浣�
    I2C_ReadRegister(GY_I2C,I2C_Register_SR1);
    I2C_ReadRegister(GY_I2C,I2C_Register_SR2);
    //绛夊緟浼犺緭
    count_wait=I2Cx_LONG_TIMEOUT;
    while(I2C_GetFlagStatus(GY_I2C,I2C_FLAG_TXE)!=SUCCESS){
        if(count_wait--==0) return I2Cx_TIMEOUT_UserCallback(10);
    }
    
    //鍙戦€佸啓鍏ョ殑瀛樺偍鍗曞厓鍦板潃
    I2C_SendData(GY_I2C,reg_addr);
    //妫€娴婨V8_2
    count_wait=I2Cx_LONG_TIMEOUT;
    while(I2C_CheckEvent(GY_I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTED)!=SUCCESS){
        if(count_wait--==0) return  I2Cx_TIMEOUT_UserCallback(11);
    }
    
     //绗簩娆′骇鐢熻捣濮嬩俊鍙�
    I2C_GenerateSTART(GY_I2C,ENABLE);
    
    //绛夊緟EV5
    count_wait=I2Cx_LONG_TIMEOUT;
    while(I2C_CheckEvent(GY_I2C,I2C_EVENT_MASTER_MODE_SELECT)!=SUCCESS){
            if(count_wait--==0) return I2Cx_TIMEOUT_UserCallback(12);
    }
    
     //鍙戦€佽澶囧湴鍧€
    I2C_Send7bitAddress(GY_I2C,(slave_addr<<1),I2C_Direction_Receiver);
    //妫€娴婨V6浜嬩欢
    count_wait=I2Cx_LONG_TIMEOUT;
    while(I2C_CheckEvent(GY_I2C,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)!=SUCCESS){
        if(count_wait--==0) return  I2Cx_TIMEOUT_UserCallback(13);
    }
    
    if(lens == 1){
        //鍦ㄦ渶鍚庝竴涓暟鎹箣鍓嶏紝鍏抽棴搴旂瓟
        I2C_AcknowledgeConfig(GY_I2C, DISABLE);
        
        //娓呴櫎ADDR鏍囧織浣�
        I2C_ReadRegister(GY_I2C,I2C_Register_SR1);
        I2C_ReadRegister(GY_I2C,I2C_Register_SR2);
        
        //浜х敓鍋滄浣�
        I2C_GenerateSTOP(GY_I2C, ENABLE);
        
        //RXNE浣嶆娴�
        count_wait=I2Cx_LONG_TIMEOUT;
        while(I2C_GetFlagStatus(GY_I2C,I2C_FLAG_RXNE)!=SET){
                if(count_wait--==0) return I2Cx_TIMEOUT_UserCallback(14);
        }
        
        data[0]=I2C_ReceiveData(GY_I2C);
    }
    else if(lens == 2){
        //鍏抽棴搴旂瓟
        I2C_AcknowledgeConfig(GY_I2C, DISABLE);
        //缃甈OS浣�
        GY_I2C->CR1 |= I2C_CR1_POS;
        //娓呴櫎ADDR鏍囧織浣�
        I2C_ReadRegister(GY_I2C,I2C_Register_SR1);
        I2C_ReadRegister(GY_I2C,I2C_Register_SR2);
        //妫€娴婤TF鏍囧織浣�
        count_wait=I2Cx_LONG_TIMEOUT;
        while(I2C_GetFlagStatus(GY_I2C,I2C_FLAG_BTF)!=SET){
                if(count_wait--==0) return I2Cx_TIMEOUT_UserCallback(15);
        }
        //浜х敓鍋滄浣�
        I2C_GenerateSTOP(GY_I2C, ENABLE);
        
        data[0]=I2C_ReceiveData(GY_I2C);
        data[1]=I2C_ReceiveData(GY_I2C);
    }
    else if(lens == 3){
        //娓呴櫎ADDR鏍囧織浣�
        I2C_ReadRegister(GY_I2C,I2C_Register_SR1);
        I2C_ReadRegister(GY_I2C,I2C_Register_SR2);
        //妫€娴婤TF鏍囧織浣�
        count_wait=I2Cx_LONG_TIMEOUT;
        while(I2C_GetFlagStatus(GY_I2C,I2C_FLAG_BTF)!=SET){
                if(count_wait--==0) return I2Cx_TIMEOUT_UserCallback(16);
        }
        //鍏抽棴搴旂瓟
        I2C_AcknowledgeConfig(GY_I2C, DISABLE);
        data[0]=I2C_ReceiveData(GY_I2C);
        //浜х敓鍋滄浣�
        I2C_GenerateSTOP(GY_I2C, ENABLE);
        data[1]=I2C_ReceiveData(GY_I2C);
        //RXNE浣嶆娴�
        count_wait=I2Cx_LONG_TIMEOUT;
        while(I2C_GetFlagStatus(GY_I2C,I2C_FLAG_RXNE)!=SET){
                if(count_wait--==0) return I2Cx_TIMEOUT_UserCallback(17);
        }
        data[2]=I2C_ReceiveData(GY_I2C);
    }
    else{
        //娓呴櫎ADDR鏍囧織浣�
        I2C_ReadRegister(GY_I2C,I2C_Register_SR1);
        I2C_ReadRegister(GY_I2C,I2C_Register_SR2);
        
        for(i=0; i<(lens); i++)
        {
          if(i==(lens-3))
          {
            //妫€娴婤TF鏍囧織浣�
            count_wait=I2Cx_LONG_TIMEOUT;
            while(I2C_GetFlagStatus(GY_I2C,I2C_FLAG_BTF)!=SET){
                    if(count_wait--==0) return I2Cx_TIMEOUT_UserCallback(16);
            }
            //鍏抽棴搴旂瓟
            I2C_AcknowledgeConfig(GY_I2C, DISABLE);
            data[i++] = I2C_ReceiveData(GY_I2C);
            
            //鍙戦€佸仠姝綅
            I2C_GenerateSTOP(GY_I2C, ENABLE);        
            data[i++] = I2C_ReceiveData(GY_I2C);
            
            //RXNE浣嶆娴�
        count_wait=I2Cx_LONG_TIMEOUT;
        while(I2C_GetFlagStatus(GY_I2C,I2C_FLAG_RXNE)!=SET){
                if(count_wait--==0) return I2Cx_TIMEOUT_UserCallback(17);
        }
            /* Read 1 bytes */
            data[i++] = I2C_ReceiveData(GY_I2C);  
            goto end;
          }
                
          //RXNE浣嶆娴�
        count_wait=I2Cx_LONG_TIMEOUT;
        while(I2C_GetFlagStatus(GY_I2C,I2C_FLAG_RXNE)!=SET){
                if(count_wait--==0) return I2Cx_TIMEOUT_UserCallback(18);
        }
          data[i] = I2C_ReceiveData(GY_I2C); 
        }  
        
    }
end:
    //娓呴櫎BTF鏍囧織浣�
    I2C_ClearFlag(GY_I2C, I2C_FLAG_BTF);
    //绛夊緟I2C閫€鍑哄繖
    while(I2C_GetFlagStatus(GY_I2C,I2C_FLAG_BUSY)!=RESET){
        if(count_wait--==0) return I2Cx_TIMEOUT_UserCallback(19);
    } 

    //涓轰笅涓€娆″仛鍑嗗
    I2C_AcknowledgeConfig(GY_I2C, ENABLE);
      //鍏抽棴POS鏍囧織
    GY_I2C->CR1 &= ~I2C_CR1_POS;
    
    return value;
}

/**
 * @brief IIC瓒呮椂鍥炴樉鍑芥暟锛屽苟涓旇緭鍑烘寚瀹氱殑鏍囧織鍊�
 * 
 * @param value 鍥炴樉鐨勫嚱鏁版爣蹇楀€�
 * @return uint32_t 杩斿洖1
 */
static uint32_t I2Cx_TIMEOUT_UserCallback(char value){

  I2C_InitTypeDef I2C_InitStructure;
  
  I2C_GenerateSTOP(GY_I2C, ENABLE);
  I2C_SoftwareResetCmd(GY_I2C, ENABLE);
  I2C_SoftwareResetCmd(GY_I2C, DISABLE);
  
  I2C_DeInit(GY_I2C);
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = STM32_OWN_ADRESS7;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = GY_Speed;
    
  /* Enable the I2C peripheral */
  I2C_Cmd(GY_I2C, ENABLE);  
    
  /* Initialize the I2C peripheral */
  I2C_Init(GY_I2C, &I2C_InitStructure);
	
  I2C_ERROR("I2C callback error code = %d",value);
 
  return 1;
}

/**
 * @brief 浼犳劅鍣ㄥ啓鍑芥暟锛屽皝瑁匢IC鍐欏嚱鏁帮紝骞朵笖澶辫触鍚庡娆″皾璇�
 * 
 * @param slave_addr 浠庢満鍦板潃
 * @param reg_addr 瀵勫瓨鍣ㄥ湴鍧€
 * @param lens 鏁版嵁闀垮害
 * @param data 鏁版嵁鎸囬拡
 * @return int 鎴愬姛杩斿洖0锛屽け璐ヨ繑鍥�1
 */
int Sensor_Write(uint8_t slave_addr,uint8_t reg_addr,unsigned short lens,uint8_t* data){
    int ret=0,retries=0;
    static unsigned int NUM  = 55;
Again:  
    ret = 0;
    ret = IIC_Byte_Write( slave_addr, reg_addr, lens, data);

    if(ret && NUM)
    {
       if( retries++ > 4 )
          return ret;
        
       Soft_Dely(0XFFFFF);
       goto Again;
    } 
    return ret;
    
}

/**
 * @brief 浼犳劅鍣ㄨ鍑芥暟锛屽皝瑁匢IC璇诲嚱鏁板苟涓旀彁渚涘け璐ュ悗澶氭灏濊瘯鏈哄埗
 * 
 * @param slave_addr 浠庢満鍦板潃
 * @param reg_addr 瀵勫瓨鍣ㄥ湴鍧€
 * @param lens 鏁版嵁闀垮害
 * @param data 鏁版嵁瀛樺偍鎸囬拡
 * @return int 鎴愬姛杩斿洖0锛屽け璐ヨ繑鍥�1
 */
int Sensor_Read(uint8_t slave_addr,uint8_t reg_addr,unsigned short lens,uint8_t* data){
    int ret=0,retries=0;
    static unsigned int NUM  = 55;
Again:  
    ret = 0;
    ret = IIC_Byte_Read( slave_addr, reg_addr, lens, data);

    if(ret && NUM)
    {
        if( retries++ > 4 )
            return ret;
    
        Soft_Dely(NUM);
        goto Again;
    } 
    return ret;
}

/**
 * @brief 杞欢寤惰繜num娆�
 * 
 * @param num 寤惰繜娆℃暟num
 */
void Soft_Dely(uint32_t num){
    while(num)
        num--;
}
