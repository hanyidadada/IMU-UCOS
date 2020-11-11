#include "bsp_i2c.h"
#include "bsp_bluetooth.h"

/**
 * @brief IIC初始化函数，IIC1，PB6,PB7
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
 * @brief iic字节写入函数
 * 
 * @param slave_addr 从机地址
 * @param reg_addr 寄存器地址
 * @param lens 数据长度
 * @param data 数据指针
 * @return int 成功返回0，失败返回1
 */
int IIC_Byte_Write(unsigned char slave_addr,unsigned char reg_addr,unsigned short lens,const unsigned char* data){
    uint32_t count_wait=I2Cx_LONG_TIMEOUT;
    uint32_t i=0;
    int value=0;
   //等待I2C退出忙
    while(I2C_GetFlagStatus(GY_I2C,I2C_FLAG_BUSY)!= RESET){
        if(count_wait--==0) return I2Cx_TIMEOUT_UserCallback(1);
    } 
    
    //产生起始信号
    I2C_GenerateSTART(GY_I2C,ENABLE);
    //等待EV5
    count_wait=I2Cx_LONG_TIMEOUT;
     while(I2C_GetFlagStatus(GY_I2C,I2C_FLAG_SB)!=SET){
            if(count_wait--==0){ return I2Cx_TIMEOUT_UserCallback(2);}
    }
    
    //发送设备地址
    I2C_Send7bitAddress(GY_I2C,(slave_addr<<1),I2C_Direction_Transmitter);
    //检测EV6事件
    count_wait=I2Cx_LONG_TIMEOUT;
    while(I2C_CheckEvent(GY_I2C,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)!=SUCCESS){
        if(count_wait--==0) return  I2Cx_TIMEOUT_UserCallback(3);
    }
    
    //清除ADDR标志位
    I2C_ReadRegister(GY_I2C,I2C_Register_SR1);
    I2C_ReadRegister(GY_I2C,I2C_Register_SR2);
    //等待传输
    count_wait=I2Cx_LONG_TIMEOUT;
    while(I2C_GetFlagStatus(GY_I2C,I2C_FLAG_TXE)!=SUCCESS){
        if(count_wait--==0){ return I2Cx_TIMEOUT_UserCallback(4);}
    }
    //发送写入的存储单元地址
    I2C_SendData(GY_I2C,reg_addr);

    
    for(i=0;i<lens;i++){
          //检测EV8_2
        count_wait=I2Cx_LONG_TIMEOUT;
        while(I2C_CheckEvent(GY_I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTED)!=SUCCESS){
            if(count_wait--==0) 
            {    return  I2Cx_TIMEOUT_UserCallback(5);
            }
        }
        //发送写入的数据
        I2C_SendData(GY_I2C,data[i]);
    }
    //检测EV8_2
    count_wait=I2Cx_LONG_TIMEOUT;
    while(I2C_CheckEvent(GY_I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTED)!=SUCCESS){
        if(count_wait--==0) return  I2Cx_TIMEOUT_UserCallback(6);
    }
    //产生结束信号
    I2C_GenerateSTOP(GY_I2C,ENABLE);
    
    return value;
}

/**
 * @brief IIC字节读取函数
 * 
 * @param slave_addr 从机地址
 * @param reg_addr 寄存器地址
 * @param lens 数据长度
 * @param data 数据指针
 * @return int 成功返回0，失败返回1
 */
int IIC_Byte_Read(unsigned char slave_addr,unsigned char reg_addr,unsigned short lens,unsigned char* data){
    uint32_t count_wait=I2Cx_LONG_TIMEOUT;
    uint32_t i=0;
    int value=0;
    //等待I2C退出忙
    while(I2C_GetFlagStatus(GY_I2C,I2C_FLAG_BUSY)!=RESET){
        if(count_wait--==0) return I2Cx_TIMEOUT_UserCallback(7);
    } 
    
    //第一次产生起始信号
    I2C_GenerateSTART(GY_I2C,ENABLE);
    
    //等待EV5
    count_wait=I2Cx_LONG_TIMEOUT;
    while(I2C_GetFlagStatus(GY_I2C,I2C_FLAG_SB)!=SET){
            if(count_wait--==0) return I2Cx_TIMEOUT_UserCallback(8);
    }
    
    //发送设备地址
    I2C_Send7bitAddress(GY_I2C,(slave_addr<<1),I2C_Direction_Transmitter);
    //检测EV6事件
    count_wait=I2Cx_LONG_TIMEOUT;
    while(I2C_CheckEvent(GY_I2C,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)!=SUCCESS){
        if(count_wait--==0) return  I2Cx_TIMEOUT_UserCallback(9);
    }
    
    //清除ADDR标志位
    I2C_ReadRegister(GY_I2C,I2C_Register_SR1);
    I2C_ReadRegister(GY_I2C,I2C_Register_SR2);
    //等待传输
    count_wait=I2Cx_LONG_TIMEOUT;
    while(I2C_GetFlagStatus(GY_I2C,I2C_FLAG_TXE)!=SUCCESS){
        if(count_wait--==0) return I2Cx_TIMEOUT_UserCallback(10);
    }
    
    //发送写入的存储单元地址
    I2C_SendData(GY_I2C,reg_addr);
    //检测EV8_2
    count_wait=I2Cx_LONG_TIMEOUT;
    while(I2C_CheckEvent(GY_I2C,I2C_EVENT_MASTER_BYTE_TRANSMITTED)!=SUCCESS){
        if(count_wait--==0) return  I2Cx_TIMEOUT_UserCallback(11);
    }
    
     //第二次产生起始信号
    I2C_GenerateSTART(GY_I2C,ENABLE);
    
    //等待EV5
    count_wait=I2Cx_LONG_TIMEOUT;
    while(I2C_CheckEvent(GY_I2C,I2C_EVENT_MASTER_MODE_SELECT)!=SUCCESS){
            if(count_wait--==0) return I2Cx_TIMEOUT_UserCallback(12);
    }
    
     //发送设备地址
    I2C_Send7bitAddress(GY_I2C,(slave_addr<<1),I2C_Direction_Receiver);
    //检测EV6事件
    count_wait=I2Cx_LONG_TIMEOUT;
    while(I2C_CheckEvent(GY_I2C,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)!=SUCCESS){
        if(count_wait--==0) return  I2Cx_TIMEOUT_UserCallback(13);
    }
    
    if(lens == 1){
        //在最后一个数据之前，关闭应答
        I2C_AcknowledgeConfig(GY_I2C, DISABLE);
        
        //清除ADDR标志位
        I2C_ReadRegister(GY_I2C,I2C_Register_SR1);
        I2C_ReadRegister(GY_I2C,I2C_Register_SR2);
        
        //产生停止位
        I2C_GenerateSTOP(GY_I2C, ENABLE);
        
        //RXNE位检测
        count_wait=I2Cx_LONG_TIMEOUT;
        while(I2C_GetFlagStatus(GY_I2C,I2C_FLAG_RXNE)!=SET){
                if(count_wait--==0) return I2Cx_TIMEOUT_UserCallback(14);
        }
        
        data[0]=I2C_ReceiveData(GY_I2C);
    }
    else if(lens == 2){
        //关闭应答
        I2C_AcknowledgeConfig(GY_I2C, DISABLE);
        //置POS位
        GY_I2C->CR1 |= I2C_CR1_POS;
        //清除ADDR标志位
        I2C_ReadRegister(GY_I2C,I2C_Register_SR1);
        I2C_ReadRegister(GY_I2C,I2C_Register_SR2);
        //检测BTF标志位
        count_wait=I2Cx_LONG_TIMEOUT;
        while(I2C_GetFlagStatus(GY_I2C,I2C_FLAG_BTF)!=SET){
                if(count_wait--==0) return I2Cx_TIMEOUT_UserCallback(15);
        }
        //产生停止位
        I2C_GenerateSTOP(GY_I2C, ENABLE);
        
        data[0]=I2C_ReceiveData(GY_I2C);
        data[1]=I2C_ReceiveData(GY_I2C);
    }
    else if(lens == 3){
        //清除ADDR标志位
        I2C_ReadRegister(GY_I2C,I2C_Register_SR1);
        I2C_ReadRegister(GY_I2C,I2C_Register_SR2);
        //检测BTF标志位
        count_wait=I2Cx_LONG_TIMEOUT;
        while(I2C_GetFlagStatus(GY_I2C,I2C_FLAG_BTF)!=SET){
                if(count_wait--==0) return I2Cx_TIMEOUT_UserCallback(16);
        }
        //关闭应答
        I2C_AcknowledgeConfig(GY_I2C, DISABLE);
        data[0]=I2C_ReceiveData(GY_I2C);
        //产生停止位
        I2C_GenerateSTOP(GY_I2C, ENABLE);
        data[1]=I2C_ReceiveData(GY_I2C);
        //RXNE位检测
        count_wait=I2Cx_LONG_TIMEOUT;
        while(I2C_GetFlagStatus(GY_I2C,I2C_FLAG_RXNE)!=SET){
                if(count_wait--==0) return I2Cx_TIMEOUT_UserCallback(17);
        }
        data[2]=I2C_ReceiveData(GY_I2C);
    }
    else{
        //清除ADDR标志位
        I2C_ReadRegister(GY_I2C,I2C_Register_SR1);
        I2C_ReadRegister(GY_I2C,I2C_Register_SR2);
        
        for(i=0; i<(lens); i++)
        {
          if(i==(lens-3))
          {
            //检测BTF标志位
            count_wait=I2Cx_LONG_TIMEOUT;
            while(I2C_GetFlagStatus(GY_I2C,I2C_FLAG_BTF)!=SET){
                    if(count_wait--==0) return I2Cx_TIMEOUT_UserCallback(16);
            }
            //关闭应答
            I2C_AcknowledgeConfig(GY_I2C, DISABLE);
            data[i++] = I2C_ReceiveData(GY_I2C);
            
            //发送停止位
            I2C_GenerateSTOP(GY_I2C, ENABLE);        
            data[i++] = I2C_ReceiveData(GY_I2C);
            
            //RXNE位检测
        count_wait=I2Cx_LONG_TIMEOUT;
        while(I2C_GetFlagStatus(GY_I2C,I2C_FLAG_RXNE)!=SET){
                if(count_wait--==0) return I2Cx_TIMEOUT_UserCallback(17);
        }
            /* Read 1 bytes */
            data[i++] = I2C_ReceiveData(GY_I2C);  
            goto end;
          }
                
          //RXNE位检测
        count_wait=I2Cx_LONG_TIMEOUT;
        while(I2C_GetFlagStatus(GY_I2C,I2C_FLAG_RXNE)!=SET){
                if(count_wait--==0) return I2Cx_TIMEOUT_UserCallback(18);
        }
          data[i] = I2C_ReceiveData(GY_I2C); 
        }  
        
    }
end:
    //清除BTF标志位
    I2C_ClearFlag(GY_I2C, I2C_FLAG_BTF);
    //等待I2C退出忙
    while(I2C_GetFlagStatus(GY_I2C,I2C_FLAG_BUSY)!=RESET){
        if(count_wait--==0) return I2Cx_TIMEOUT_UserCallback(19);
    } 

    //为下一次做准备
    I2C_AcknowledgeConfig(GY_I2C, ENABLE);
      //关闭POS标志
    GY_I2C->CR1 &= ~I2C_CR1_POS;
    
    return value;
}

/**
 * @brief IIC超时返回函数
 * 
 * @param value 出错标志
 * @return uint32_t 成功1
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
 * @brief 感应器写函数，并且提供出错尝试机制
 * 
 * @param slave_addr 从机地址
 * @param reg_addr 寄存器地址
 * @param lens 数据长度
 * @param data 数据指针
 * @return int 成功返回0，失败返回1
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
 * @brief 感应器读函数，并且提供出错尝试机制
 * 
 * @param slave_addr 从机地址
 * @param reg_addr 寄存器地址
 * @param lens 数据长度
 * @param data 数据指针
 * @return int 成功返回0，失败返回1
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
 * @brief 软件延时函数
 * 
 * @param num 延时计数num
 */
void Soft_Dely(uint32_t num){
    while(num)
        num--;
}
