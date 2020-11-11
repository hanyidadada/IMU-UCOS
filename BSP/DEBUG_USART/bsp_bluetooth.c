#include "bsp_bluetooth.h"

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
extern u8 sendBuf[50];
/**
 * @brief 串口初始化配置函数
 * 
 */
void Debug_USART_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	/* 打开串口GPIO时钟 */
	 RCC_AHB1PeriphClockCmd(DEBUG_USART_RX_GPIO_CLK|DEBUG_USART_TX_GPIO_CLK,ENABLE);
	
	  /* GPIO初始化 */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  /* 配置Tx引脚为复用功能  */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_PIN  ;  
  GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);
	
  /* 配置Rx引脚为复用功能 */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_PIN;
  GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);
	
	/* GPIO具体复用成什么 */
	 /* 连接 PXx 到 USARTx_Tx*/
  GPIO_PinAFConfig(DEBUG_USART_RX_GPIO_PORT,DEBUG_USART_RX_SOURCE,DEBUG_USART_RX_AF);
  /*  连接 PXx 到 USARTx__Rx*/
  GPIO_PinAFConfig(DEBUG_USART_TX_GPIO_PORT,DEBUG_USART_TX_SOURCE,DEBUG_USART_TX_AF);
	
	/* 第二步：配置串口初始化结构体 */
	  /* 使能 USART 时钟 */
  RCC_APB1PeriphClockCmd(DEBUG_USART_CLK, ENABLE);
	 /* 配置串DEBUG_USART 模式 */
  /* 波特率设置：DEBUG_USART_BAUDRATE */
  USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;
  /* 字长(数据位+校验位)：8 */
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  /* 停止位：1个停止位 */
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  /* 校验位选择：不使用校验 */
  USART_InitStructure.USART_Parity = USART_Parity_No;
  /* 硬件流控制：不使用硬件流 */
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  /* USART模式控制：同时使能接收和发送 */
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  /* 完成USART初始化配置 */
  USART_Init(DEBUG_USART, &USART_InitStructure);
	
	
	/* 使能串口 */
  USART_Cmd(DEBUG_USART, ENABLE);
}

/**
 * @brief 非中断方式发送
 * 
 * @param DataToSend 数据指针
 * @param data_num  字符串长度
 */
void USART_NItSend(unsigned char* DataToSend, u8 data_num)
{
    int i;
    for (i = 0; i < data_num; i++) {
        while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TC) == RESET)
            ; 
        USART_SendData(DEBUG_USART, DataToSend[i]);
    }
}
/**
 * @brief 输出重定向，将printf定向到串口发送
 * 
 * @param ch 发送的字符
 * @param f 文件指针
 * @return int 成功返回字符ascii码
 */
int fputc(int ch, FILE *f)
{
		/* 发送一个字节数据到串口 */
		USART_SendData(DEBUG_USART, (uint8_t) ch);
		
		
		while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}
/**
 * @brief 输入重定向，从串口接收区获取输入
 * 
 * @param f 文件指针
 * @return int 
 */
int fgetc(FILE *f)
{
		
		while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(DEBUG_USART);
}

/**
 * @brief 感应器数据发送函数
 * 
 * @param ACCEL_X 加速度计x数据
 * @param ACCEL_Y 加速度计y数据
 * @param ACCEL_Z 加速度计z数据
 * @param GYRO_X 角速度计x数据
 * @param GYRO_Y 角速度计y数据
 * @param GYRO_Z 角速度计z数据
 * @param MAG_X 磁力计x数据
 * @param MAG_Y 磁力计y数据
 * @param MAG_Z 磁力计z数据
 */
void Send_Senser(int16_t ACCEL_X, int16_t ACCEL_Y, int16_t ACCEL_Z, int16_t GYRO_X, int16_t GYRO_Y, int16_t GYRO_Z, int16_t MAG_X, int16_t MAG_Y, int16_t MAG_Z) //鍙戦€佺敤鎴锋暟鎹紝杩欓噷鏈�6涓暟鎹�
{
   u8 _cnt = 0;
    u8 sum = 0; //以下为计算sum校验字节，从0xAA也就是首字节，一直到sum字节前一字节
    int i;

    sendBuf[_cnt++] = 0xAA; //0xAA为帧头
    sendBuf[_cnt++] = 0x05; //0x05为数据发送源，具体请参考匿名协议，本字节用户可以随意更改
    sendBuf[_cnt++] = 0xAF; //0xAF为数据目的地，AF表示上位机，具体请参考匿名协议
    sendBuf[_cnt++] = 0x02; //0x02，表示本帧为传感器原始数据帧
    sendBuf[_cnt++] = 0; //本字节表示数据长度，这里先=0，函数最后再赋值，这样就不用人工计算长度了

    sendBuf[_cnt++] = BYTE1(ACCEL_X); //将要发送的数据放至发送缓冲区
    sendBuf[_cnt++] = BYTE0(ACCEL_X);

    sendBuf[_cnt++] = BYTE1(ACCEL_Y);
    sendBuf[_cnt++] = BYTE0(ACCEL_Y);

    sendBuf[_cnt++] = BYTE1(ACCEL_Z);
    sendBuf[_cnt++] = BYTE0(ACCEL_Z);

    sendBuf[_cnt++] = BYTE1(GYRO_X);
    sendBuf[_cnt++] = BYTE0(GYRO_X);

    sendBuf[_cnt++] = BYTE1(GYRO_Y);
    sendBuf[_cnt++] = BYTE0(GYRO_Y);

    sendBuf[_cnt++] = BYTE1(GYRO_Z);
    sendBuf[_cnt++] = BYTE0(GYRO_Z);

    sendBuf[_cnt++] = BYTE1(MAG_X);
    sendBuf[_cnt++] = BYTE0(MAG_X);

    sendBuf[_cnt++] = BYTE1(MAG_Y);
    sendBuf[_cnt++] = BYTE0(MAG_Y);

    sendBuf[_cnt++] = BYTE1(MAG_Z);
    sendBuf[_cnt++] = BYTE0(MAG_Z);

    sendBuf[4] = _cnt - 5; //_cnt用来计算数据长度，减5为减去帧开头5个非数据字节

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //将sum校验数据放置最后一字节
    USART_NItSend(sendBuf, _cnt);

}

/**
 * @brief 发送姿态解算三个姿态角给上位机
 * 
 * @param roll 横滚角
 * @param pitch 俯仰角
 * @param yaw 偏航角
 */
void Send_Attitude(float roll, float pitch, float yaw)
{
    u8 _cnt = 0;
    u8 sum = 0; //以下为计算sum校验字节，从0xAA也就是首字节，一直到sum字节前一字节
    int i;
    int32_t ALT_USE = 0;
    u8 FLY_MODEL = 0, ARMED = 0;
    vs16 _temp;

    sendBuf[_cnt++] = 0xAA; //0xAA为帧头
    sendBuf[_cnt++] = 0x05; //0x05为数据发送源，具体请参考匿名协议，本字节用户可以随意更改
    sendBuf[_cnt++] = 0xAF; //0xAF为数据目的地，AF表示上位机，具体请参考匿名协议
    sendBuf[_cnt++] = 0x01; //0x01，表示本帧为姿态数据帧
    sendBuf[_cnt++] = 0; //本字节表示数据长度，这里先=0，函数最后再赋值，这样就不用人工计算长度了

    _temp = (int)(roll * 100);
    sendBuf[_cnt++] = BYTE1(_temp); //将要发送的数据放至发送缓冲区
    sendBuf[_cnt++] = BYTE0(_temp);

    _temp = (int)(pitch * 100);
    sendBuf[_cnt++] = BYTE1(_temp);
    sendBuf[_cnt++] = BYTE0(_temp);

    _temp = (int)(yaw * 100);
    sendBuf[_cnt++] = BYTE1(_temp);
    sendBuf[_cnt++] = BYTE0(_temp);

    sendBuf[_cnt++] = BYTE3(ALT_USE); //假数据，为了符合数据帧要求
    sendBuf[_cnt++] = BYTE2(ALT_USE);
    sendBuf[_cnt++] = BYTE1(ALT_USE);
    sendBuf[_cnt++] = BYTE0(ALT_USE);

    sendBuf[_cnt++] = FLY_MODEL;
    sendBuf[_cnt++] = ARMED;

    sendBuf[4] = _cnt - 5; //_cnt用来计算数据长度，减5为减去帧开头5个非数据字节

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //将sum校验数据放置最后一字节

    USART_NItSend(sendBuf, _cnt);

}

/**
 * @brief 发送遥控器以及电机PWM信号给上位机
 * 
 * @param THR 油门信号
 * @param YAW 偏航信号
 * @param ROLL 横滚信号
 * @param PITCH 俯仰信号
 * @param motor1 电机1PWM
 * @param motor2 电机2PWM
 * @param motor3 电机3PWM
 * @param motor4 电机4PWM
 */
void Send_RCData_Motor(int16_t THR, int16_t YAW, int16_t ROLL, int16_t PITCH, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    u8 _cnt = 0;
    u8 sum = 0; //以下为计算sum校验字节，从0xAA也就是首字节，一直到sum字节前一字节
    int i;
    int16_t AUX = 0;

    sendBuf[_cnt++] = 0xAA; //0xAA为帧头
    sendBuf[_cnt++] = 0x05; //0x05为数据发送源，具体请参考匿名协议，本字节用户可以随意更改
    sendBuf[_cnt++] = 0xAF; //0xAF为数据目的地，AF表示上位机，具体请参考匿名协议
    sendBuf[_cnt++] = 0x03; //0x03，表示本帧为接收机、电机速度数据帧
    sendBuf[_cnt++] = 0; //本字节表示数据长度，这里先=0，函数最后再赋值，这样就不用人工计算长度了

    sendBuf[_cnt++] = BYTE1(THR); //将要发送的数据放至发送缓冲区
    sendBuf[_cnt++] = BYTE0(THR);

    sendBuf[_cnt++] = BYTE1(YAW);
    sendBuf[_cnt++] = BYTE0(YAW);

    sendBuf[_cnt++] = BYTE1(ROLL);
    sendBuf[_cnt++] = BYTE0(ROLL);

    sendBuf[_cnt++] = BYTE1(PITCH);
    sendBuf[_cnt++] = BYTE0(PITCH);

    sendBuf[_cnt++] = BYTE1(motor1);
    sendBuf[_cnt++] = BYTE0(motor1);

    sendBuf[_cnt++] = BYTE1(motor2);
    sendBuf[_cnt++] = BYTE0(motor2);

    sendBuf[_cnt++] = BYTE1(motor3);
    sendBuf[_cnt++] = BYTE0(motor3);

    sendBuf[_cnt++] = BYTE1(motor4);
    sendBuf[_cnt++] = BYTE0(motor4);

    sendBuf[_cnt++] = BYTE1(AUX);
    sendBuf[_cnt++] = BYTE0(AUX);

    sendBuf[_cnt++] = BYTE1(AUX);
    sendBuf[_cnt++] = BYTE0(AUX);

    sendBuf[4] = _cnt - 5; //_cnt用来计算数据长度，减5为减去帧开头5个非数据字节

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //将sum校验数据放置最后一字节
		
    USART_NItSend(sendBuf, _cnt);
}

/**
 * @brief 发送字符串给上位机
 * 
 * @param str 字符串指针
 */
void SendStr(const char* str)
{
    u8 _cnt = 0;
    u8 i = 0;
    u8 sum = 0;

    sendBuf[_cnt++] = 0xAA;
    sendBuf[_cnt++] = 0x05;
    sendBuf[_cnt++] = 0xAF;
    sendBuf[_cnt++] = 0xA0;
    sendBuf[_cnt++] = 0;
    while (*(str + i) != '\0') {
        sendBuf[_cnt++] = *(str + i);
        i++;
        if (_cnt > 50)
            break;
    }

    sendBuf[4] = _cnt - 5;

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum;

    USART_NItSend(sendBuf, _cnt);
}

/**
 * @brief 发送字节数据给上位机
 * 
 * @param frame 用户自定义字节
 * @param p 数据指针
 */
void SendByte(u8 frame, u8* p)
{
    u8 _cnt = 0;
    u8 sum = 0; //以下为计算sum校验字节，从0xAA也就是首字节，一直到sum字节前一字节
    int i;

    sendBuf[_cnt++] = 0xAA; //0xAA为帧头
    sendBuf[_cnt++] = 0x05; //0x05为数据发送源，具体请参考匿名协议，本字节用户可以随意更改
    sendBuf[_cnt++] = 0xAF; //0xAF为数据目的地，AF表示上位机，具体请参考匿名协议
    sendBuf[_cnt++] = frame; //用户自定义数据帧
    sendBuf[_cnt++] = 0; //本字节表示数据长度，这里先=0，函数最后再赋值，这样就不用人工计算长度了

    sendBuf[_cnt++] = *p; //将要发送的数据放至发送缓冲区

    sendBuf[4] = _cnt - 5; //_cnt用来计算数据长度，减5为减去帧开头5个非数据字节

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //将sum校验数据放置最后一字节

    USART_NItSend(sendBuf, _cnt);
}

/**
 * @brief 发送半字给上位机
 * 
 * @param frame 用户自定义功能字节
 * @param p 数据指针
 */
void SendHalfWord(u8 frame, u16* p)
{
   u8 _cnt = 0;
    u8 sum = 0; //以下为计算sum校验字节，从0xAA也就是首字节，一直到sum字节前一字节
    int i;

    sendBuf[_cnt++] = 0xAA; //0xAA为帧头
    sendBuf[_cnt++] = 0x05; //0x05为数据发送源，具体请参考匿名协议，本字节用户可以随意更改
    sendBuf[_cnt++] = 0xAF; //0xAF为数据目的地，AF表示上位机，具体请参考匿名协议
    sendBuf[_cnt++] = frame; //用户自定义数据帧
    sendBuf[_cnt++] = 0; //本字节表示数据长度，这里先=0，函数最后再赋值，这样就不用人工计算长度了

    sendBuf[_cnt++] = BYTE1(*p); //将要发送的数据放至发送缓冲区
    sendBuf[_cnt++] = BYTE0(*p);

    sendBuf[4] = _cnt - 5; //_cnt用来计算数据长度，减5为减去帧开头5个非数据字节

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //将sum校验数据放置最后一字节

    USART_NItSend(sendBuf, _cnt);
}

/**
 * @brief 发送字数据给上位机
 * @param frame 用户自定义功能字节
 * @param p 数据指针
 */
void SendWord(u8 frame, u32* p)
{
   u8 _cnt = 0;
    u8 sum = 0; //以下为计算sum校验字节，从0xAA也就是首字节，一直到sum字节前一字节
    int i;

    sendBuf[_cnt++] = 0xAA; //0xAA为帧头
    sendBuf[_cnt++] = 0x05; //0x05为数据发送源，具体请参考匿名协议，本字节用户可以随意更改
    sendBuf[_cnt++] = 0xAF; //0xAF为数据目的地，AF表示上位机，具体请参考匿名协议
    sendBuf[_cnt++] = frame; //用户自定义数据帧
    sendBuf[_cnt++] = 0; //本字节表示数据长度，这里先=0，函数最后再赋值，这样就不用人工计算长度了

    sendBuf[_cnt++] = BYTE3(*p); //将要发送的数据放至发送缓冲区
    sendBuf[_cnt++] = BYTE2(*p);
    sendBuf[_cnt++] = BYTE1(*p);
    sendBuf[_cnt++] = BYTE0(*p);

    sendBuf[4] = _cnt - 5; //_cnt用来计算数据长度，减5为减去帧开头5个非数据字节

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //将sum校验数据放置最后一字节

    USART_NItSend(sendBuf, _cnt);
}

/**
 * @brief 发哦那个5浮点数给上位机
 * 
 * @param frame 用户自定义功能字
 * @param f1 浮点数1
 * @param f2 浮点数2
 * @param f3 浮点数3
 * @param f4 浮点数4
 * @param f5 浮点数5
 */
void Send_5_float(u8 frame,float f1,float f2,float f3,float f4,float f5){
    u8 _cnt = 0;
    u8 sum = 0; //以下为计算sum校验字节，从0xAA也就是首字节，一直到sum字节前一字节
    int i;
    vs16 _temp;

    sendBuf[_cnt++] = 0xAA; //0xAA为帧头
    sendBuf[_cnt++] = 0x05; //0x05为数据发送源，具体请参考匿名协议，本字节用户可以随意更改
    sendBuf[_cnt++] = 0xAF; //0xAF为数据目的地，AF表示上位机，具体请参考匿名协议
    sendBuf[_cnt++] = frame; //用户自定义帧
    sendBuf[_cnt++] = 0; //本字节表示数据长度，这里先=0，函数最后再赋值，这样就不用人工计算长度了

    _temp = f1;
    sendBuf[_cnt++] = BYTE1(_temp); //将要发送的数据放至发送缓冲区
    sendBuf[_cnt++] = BYTE0(_temp);

    _temp = f2;
    sendBuf[_cnt++] = BYTE1(_temp);
    sendBuf[_cnt++] = BYTE0(_temp);

    _temp = f3;
    sendBuf[_cnt++] = BYTE1(_temp);
    sendBuf[_cnt++] = BYTE0(_temp);

    _temp = f4;
    sendBuf[_cnt++] = BYTE1(_temp);
    sendBuf[_cnt++] = BYTE0(_temp);

    _temp = f5;
    sendBuf[_cnt++] = BYTE1(_temp);
    sendBuf[_cnt++] = BYTE0(_temp);

    sendBuf[4] = _cnt - 5; //_cnt用来计算数据长度，减5为减去帧开头5个非数据字节

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //将sum校验数据放置最后一字节

    USART_NItSend(sendBuf, _cnt);
}


