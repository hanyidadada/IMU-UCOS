#include "bsp_bluetooth.h"

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
extern u8 sendBuf[50];
/**
 * @brief 涓插彛閰嶇疆鍒濆鍖栧嚱鏁帮紝閰嶇疆涓插彛3锛屾尝鐗圭巼9600
 * 
 */
void Debug_USART_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	/* 绗竴姝ワ細鍒濆鍖朑PIO */
	 RCC_AHB1PeriphClockCmd(DEBUG_USART_RX_GPIO_CLK|DEBUG_USART_TX_GPIO_CLK,ENABLE);
	
	  /* GPIO鍒濆鍖� */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  /* 閰嶇疆Tx寮曡剼涓哄鐢ㄥ姛鑳�  */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_PIN  ;  
  GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);
	
	  /* 閰嶇疆Rx寮曡剼涓哄鐢ㄥ姛鑳� */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_PIN;
  GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);
	
	/* GPIO鍏蜂綋澶嶇敤鎴愪粈涔� */
	 /* 杩炴帴 PXx 鍒� USARTx_Tx*/
  GPIO_PinAFConfig(DEBUG_USART_RX_GPIO_PORT,DEBUG_USART_RX_SOURCE,DEBUG_USART_RX_AF);
  /*  杩炴帴 PXx 鍒� USARTx__Rx*/
  GPIO_PinAFConfig(DEBUG_USART_TX_GPIO_PORT,DEBUG_USART_TX_SOURCE,DEBUG_USART_TX_AF);
	
	/* 绗簩姝ワ細閰嶇疆涓插彛鍒濆鍖栫粨鏋勪綋 */
	  /* 浣胯兘 USART 鏃堕挓 */
  RCC_APB1PeriphClockCmd(DEBUG_USART_CLK, ENABLE);
	  /* 閰嶇疆涓睤EBUG_USART 妯″紡 */
  /* 娉㈢壒鐜囪缃細DEBUG_USART_BAUDRATE */
  USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;
  /* 瀛楅暱(鏁版嵁浣�+鏍￠獙浣�)锛�8 */
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  /* 鍋滄浣嶏細1涓仠姝綅 */
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  /* 鏍￠獙浣嶉€夋嫨锛氫笉浣跨敤鏍￠獙 */
  USART_InitStructure.USART_Parity = USART_Parity_No;
  /* 纭欢娴佹帶鍒讹細涓嶄娇鐢ㄧ‖浠舵祦 */
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  /* USART妯″紡鎺у埗锛氬悓鏃朵娇鑳芥帴鏀跺拰鍙戦€� */
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  /* 瀹屾垚USART鍒濆鍖栭厤缃� */
  USART_Init(DEBUG_USART, &USART_InitStructure);
	
	
	/* 浣胯兘涓插彛 */
  USART_Cmd(DEBUG_USART, ENABLE);
}

/**
 * @brief 涓插彛闈炰腑鏂彂閫�
 * 
 * @param DataToSend 闇€瑕佸彂閫佺殑瀛楃涓叉寚閽�
 * @param data_num  瀛楃涓查暱搴�
 */
void USART_NItSend(unsigned char* DataToSend, u8 data_num)
{
    int i;
    for (i = 0; i < data_num; i++) {
        while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TC) == RESET)
            ; //鍒濆鍖栫粓涓插彛鍙戦€佷娇鑳芥椂STM32浼氳嚜鍔ㄥ彂閫佷竴涓┖闂插抚锛屽鑷碩C浣嶇疆1
        USART_SendData(DEBUG_USART, DataToSend[i]);
    }
}
/**
 * @brief 杈撳嚭閲嶅畾鍚戝嚱鏁帮紝閫氳繃涓插彛鍙戦€佷竴涓瓧鑺傜殑鏁版嵁
 * 
 * @param ch asscii缂栫爜鐨勫瓧绗︽暟鎹�
 * @param f 鏂囦欢绗�
 * @return int 杩斿洖鍙戦€佺殑瀛楃
 */
int fputc(int ch, FILE *f)
{
		/* 鍙戦€佷竴涓瓧鑺傛暟鎹埌涓插彛 */
		USART_SendData(DEBUG_USART, (uint8_t) ch);
		
		
		while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}
/**
 * @brief 杈撳叆閲嶅畾鍚戯紝閫氳繃璇诲彇涓插彛
 * 
 * @param f 鏂囦欢鎸囬拡
 * @return int 
 */
int fgetc(FILE *f)
{
		
		while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(DEBUG_USART);
}

/**
 * @brief 鍩轰簬鍖垮悕绉戝垱閫氫俊鍗忚6.0锛屽彂閫丮PU6050鏁版嵁鍒颁笂浣嶆満鏄剧ず
 * 
 * @param ACCEL_X 鍔犻€熷害璁鏁版嵁
 * @param ACCEL_Y 鍔犻€熷害璁鏁版嵁
 * @param ACCEL_Z 鍔犻€熷害璁鏁版嵁
 * @param GYRO_X 瑙掗€熷害璁鏁版嵁
 * @param GYRO_Y 瑙掗€熷害璁鏁版嵁
 * @param GYRO_Z 瑙掗€熷害璁鏁版嵁
 * @param MAG_X 纾佸姏璁鏁版嵁
 * @param MAG_Y 纾佸姏璁鏁版嵁
 * @param MAG_Z 纾佸姏璁鏁版嵁
 */
void Send_Senser(int16_t ACCEL_X, int16_t ACCEL_Y, int16_t ACCEL_Z, int16_t GYRO_X, int16_t GYRO_Y, int16_t GYRO_Z, int16_t MAG_X, int16_t MAG_Y, int16_t MAG_Z) //鍙戦€佺敤鎴锋暟鎹紝杩欓噷鏈�6涓暟鎹�
{
    u8 _cnt = 0;
    u8 sum = 0; //浠ヤ笅涓鸿绠梥um鏍￠獙瀛楄妭锛屼粠0xAA涔熷氨鏄瀛楄妭锛屼竴鐩村埌sum瀛楄妭鍓嶄竴瀛楄妭
    int i;

    sendBuf[_cnt++] = 0xAA; //0xAA涓哄抚澶�
    sendBuf[_cnt++] = 0x05; //0x05涓烘暟鎹彂閫佹簮锛屽叿浣撹鍙傝€冨尶鍚嶅崗璁紝鏈瓧鑺傜敤鎴峰彲浠ラ殢鎰忔洿鏀�
    sendBuf[_cnt++] = 0xAF; //0xAF涓烘暟鎹洰鐨勫湴锛孉F琛ㄧず涓婁綅鏈猴紝鍏蜂綋璇峰弬鑰冨尶鍚嶅崗璁�
    sendBuf[_cnt++] = 0x02; //0x02锛岃〃绀烘湰甯т负浼犳劅鍣ㄥ師濮嬫暟鎹抚
    sendBuf[_cnt++] = 0; //鏈瓧鑺傝〃绀烘暟鎹暱搴︼紝杩欓噷鍏�=0锛屽嚱鏁版渶鍚庡啀璧嬪€硷紝杩欐牱灏变笉鐢ㄤ汉宸ヨ绠楅暱搴︿簡

    sendBuf[_cnt++] = BYTE1(ACCEL_X); //灏嗚鍙戦€佺殑鏁版嵁鏀捐嚦鍙戦€佺紦鍐插尯
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

    sendBuf[4] = _cnt - 5; //_cnt鐢ㄦ潵璁＄畻鏁版嵁闀垮害锛屽噺5涓哄噺鍘诲抚寮€澶�5涓潪鏁版嵁瀛楄妭

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //灏唖um鏍￠獙鏁版嵁鏀剧疆鏈€鍚庝竴瀛楄妭
    USART_NItSend(sendBuf, _cnt);

}

/**
 * @brief 鍩轰簬鍖垮悕绉戝垱6.0閫氫俊鍗忚锛屽彂閫佷笁涓Э鎬佽缁欎笂浣嶆満
 * 
 * @param roll 妯粴瑙�
 * @param pitch 淇话瑙�
 * @param yaw 鍋忚埅瑙�
 */
void Send_Attitude(float roll, float pitch, float yaw)
{
    u8 _cnt = 0;
    u8 sum = 0; //浠ヤ笅涓鸿绠梥um鏍￠獙瀛楄妭锛屼粠0xAA涔熷氨鏄瀛楄妭锛屼竴鐩村埌sum瀛楄妭鍓嶄竴瀛楄妭
    int i;
    int32_t ALT_USE = 0;
    u8 FLY_MODEL = 0, ARMED = 0;
    vs16 _temp;

    sendBuf[_cnt++] = 0xAA; //0xAA涓哄抚澶�
    sendBuf[_cnt++] = 0x05; //0x05涓烘暟鎹彂閫佹簮锛屽叿浣撹鍙傝€冨尶鍚嶅崗璁紝鏈瓧鑺傜敤鎴峰彲浠ラ殢鎰忔洿鏀�
    sendBuf[_cnt++] = 0xAF; //0xAF涓烘暟鎹洰鐨勫湴锛孉F琛ㄧず涓婁綅鏈猴紝鍏蜂綋璇峰弬鑰冨尶鍚嶅崗璁�
    sendBuf[_cnt++] = 0x01; //0x01锛岃〃绀烘湰甯т负濮挎€佹暟鎹抚
    sendBuf[_cnt++] = 0; //鏈瓧鑺傝〃绀烘暟鎹暱搴︼紝杩欓噷鍏�=0锛屽嚱鏁版渶鍚庡啀璧嬪€硷紝杩欐牱灏变笉鐢ㄤ汉宸ヨ绠楅暱搴︿簡

    _temp = (int)(roll * 100);
    sendBuf[_cnt++] = BYTE1(_temp); //灏嗚鍙戦€佺殑鏁版嵁鏀捐嚦鍙戦€佺紦鍐插尯
    sendBuf[_cnt++] = BYTE0(_temp);

    _temp = (int)(pitch * 100);
    sendBuf[_cnt++] = BYTE1(_temp);
    sendBuf[_cnt++] = BYTE0(_temp);

    _temp = (int)(yaw * 100);
    sendBuf[_cnt++] = BYTE1(_temp);
    sendBuf[_cnt++] = BYTE0(_temp);

    sendBuf[_cnt++] = BYTE3(ALT_USE); //鍋囨暟鎹紝涓轰簡绗﹀悎鏁版嵁甯ц姹�
    sendBuf[_cnt++] = BYTE2(ALT_USE);
    sendBuf[_cnt++] = BYTE1(ALT_USE);
    sendBuf[_cnt++] = BYTE0(ALT_USE);

    sendBuf[_cnt++] = FLY_MODEL;
    sendBuf[_cnt++] = ARMED;

    sendBuf[4] = _cnt - 5; //_cnt鐢ㄦ潵璁＄畻鏁版嵁闀垮害锛屽噺5涓哄噺鍘诲抚寮€澶�5涓潪鏁版嵁瀛楄妭

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //灏唖um鏍￠獙鏁版嵁鏀剧疆鏈€鍚庝竴瀛楄妭

    USART_NItSend(sendBuf, _cnt);

}

/**
 * @brief 鍩轰簬鍖垮悕绉戝垱6.0閫氫俊鍗忚锛屽彂閫侀仴鎺у櫒鏁版嵁浠ュ強鐢垫満鏁版嵁缁欎笂浣嶆満
 * 
 * @param THR 娌归棬淇″彿
 * @param YAW 鍋忚埅淇″彿
 * @param ROLL 妯粴淇″彿
 * @param PITCH 淇话淇″彿
 * @param motor1 鐢垫満1PWM鍊�
 * @param motor2 鐢垫満2PWM鍊�
 * @param motor3 鐢垫満3PWM鍊�
 * @param motor4 鐢垫満4PWM鍊�
 */
void Send_RCData_Motor(int16_t THR, int16_t YAW, int16_t ROLL, int16_t PITCH, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    u8 _cnt = 0;
    u8 sum = 0; //浠ヤ笅涓鸿绠梥um鏍￠獙瀛楄妭锛屼粠0xAA涔熷氨鏄瀛楄妭锛屼竴鐩村埌sum瀛楄妭鍓嶄竴瀛楄妭
    int i;
    int16_t AUX = 0;

    sendBuf[_cnt++] = 0xAA; //0xAA涓哄抚澶�
    sendBuf[_cnt++] = 0x05; //0x05涓烘暟鎹彂閫佹簮锛屽叿浣撹鍙傝€冨尶鍚嶅崗璁紝鏈瓧鑺傜敤鎴峰彲浠ラ殢鎰忔洿鏀�
    sendBuf[_cnt++] = 0xAF; //0xAF涓烘暟鎹洰鐨勫湴锛孉F琛ㄧず涓婁綅鏈猴紝鍏蜂綋璇峰弬鑰冨尶鍚嶅崗璁�
    sendBuf[_cnt++] = 0x03; //0x03锛岃〃绀烘湰甯т负鎺ユ敹鏈恒€佺數鏈洪€熷害鏁版嵁甯�
    sendBuf[_cnt++] = 0; //鏈瓧鑺傝〃绀烘暟鎹暱搴︼紝杩欓噷鍏�=0锛屽嚱鏁版渶鍚庡啀璧嬪€硷紝杩欐牱灏变笉鐢ㄤ汉宸ヨ绠楅暱搴︿簡

    sendBuf[_cnt++] = BYTE1(THR); //灏嗚鍙戦€佺殑鏁版嵁鏀捐嚦鍙戦€佺紦鍐插尯
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

    sendBuf[4] = _cnt - 5; //_cnt鐢ㄦ潵璁＄畻鏁版嵁闀垮害锛屽噺5涓哄噺鍘诲抚寮€澶�5涓潪鏁版嵁瀛楄妭

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //灏唖um鏍￠獙鏁版嵁鏀剧疆鏈€鍚庝竴瀛楄妭
		
    USART_NItSend(sendBuf, _cnt);
}

/**
 * @brief 鍩轰簬鍖垮悕绉戝垱6.0閫氫俊鍗忚锛屽彂閫佸瓧绗︿覆缁欎笂浣嶆満
 * 
 * @param str 瀛楃涓叉寚閽�
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
 * @brief 鍩轰簬鍖垮悕绉戝垱6.0閫氫俊鍗忚锛屽彂閫佸瓧鑺傛暟鎹埌涓婁綅鏈�
 * 
 * @param frame 鐢ㄦ埛瀹氫箟鍔熻兘瀛楄妭
 * @param p 鐢ㄦ埛鑷畾涔夋暟鎹寚閽�
 */
void SendByte(u8 frame, u8* p)
{
    u8 _cnt = 0;
    u8 sum = 0; //浠ヤ笅涓鸿绠梥um鏍￠獙瀛楄妭锛屼粠0xAA涔熷氨鏄瀛楄妭锛屼竴鐩村埌sum瀛楄妭鍓嶄竴瀛楄妭
    int i;

    sendBuf[_cnt++] = 0xAA; //0xAA涓哄抚澶�
    sendBuf[_cnt++] = 0x05; //0x05涓烘暟鎹彂閫佹簮锛屽叿浣撹鍙傝€冨尶鍚嶅崗璁紝鏈瓧鑺傜敤鎴峰彲浠ラ殢鎰忔洿鏀�
    sendBuf[_cnt++] = 0xAF; //0xAF涓烘暟鎹洰鐨勫湴锛孉F琛ㄧず涓婁綅鏈猴紝鍏蜂綋璇峰弬鑰冨尶鍚嶅崗璁�
    sendBuf[_cnt++] = frame; //鐢ㄦ埛鑷畾涔夋暟鎹抚
    sendBuf[_cnt++] = 0; //鏈瓧鑺傝〃绀烘暟鎹暱搴︼紝杩欓噷鍏�=0锛屽嚱鏁版渶鍚庡啀璧嬪€硷紝杩欐牱灏变笉鐢ㄤ汉宸ヨ绠楅暱搴︿簡

    sendBuf[_cnt++] = *p; //灏嗚鍙戦€佺殑鏁版嵁鏀捐嚦鍙戦€佺紦鍐插尯

    sendBuf[4] = _cnt - 5; //_cnt鐢ㄦ潵璁＄畻鏁版嵁闀垮害锛屽噺5涓哄噺鍘诲抚寮€澶�5涓潪鏁版嵁瀛楄妭

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //灏唖um鏍￠獙鏁版嵁鏀剧疆鏈€鍚庝竴瀛楄妭

    USART_NItSend(sendBuf, _cnt);
}

/**
 * @brief 鍩轰簬鍖垮悕绉戝垱6.0閫氫俊鍗忚,鍙戦€佸崐瀛楁暟鎹�
 * 
 * @param frame 鐢ㄦ埛瀹氫箟鍔熻兘瀛楄妭
 * @param p 鐢ㄦ埛鏁版嵁鎸囬拡
 */
void SendHalfWord(u8 frame, u16* p)
{
    u8 _cnt = 0;
    u8 sum = 0; //浠ヤ笅涓鸿绠梥um鏍￠獙瀛楄妭锛屼粠0xAA涔熷氨鏄瀛楄妭锛屼竴鐩村埌sum瀛楄妭鍓嶄竴瀛楄妭
    int i;

    sendBuf[_cnt++] = 0xAA; //0xAA涓哄抚澶�
    sendBuf[_cnt++] = 0x05; //0x05涓烘暟鎹彂閫佹簮锛屽叿浣撹鍙傝€冨尶鍚嶅崗璁紝鏈瓧鑺傜敤鎴峰彲浠ラ殢鎰忔洿鏀�
    sendBuf[_cnt++] = 0xAF; //0xAF涓烘暟鎹洰鐨勫湴锛孉F琛ㄧず涓婁綅鏈猴紝鍏蜂綋璇峰弬鑰冨尶鍚嶅崗璁�
    sendBuf[_cnt++] = frame; //鐢ㄦ埛鑷畾涔夋暟鎹抚
    sendBuf[_cnt++] = 0; //鏈瓧鑺傝〃绀烘暟鎹暱搴︼紝杩欓噷鍏�=0锛屽嚱鏁版渶鍚庡啀璧嬪€硷紝杩欐牱灏变笉鐢ㄤ汉宸ヨ绠楅暱搴︿簡

    sendBuf[_cnt++] = BYTE1(*p); //灏嗚鍙戦€佺殑鏁版嵁鏀捐嚦鍙戦€佺紦鍐插尯
    sendBuf[_cnt++] = BYTE0(*p);

    sendBuf[4] = _cnt - 5; //_cnt鐢ㄦ潵璁＄畻鏁版嵁闀垮害锛屽噺5涓哄噺鍘诲抚寮€澶�5涓潪鏁版嵁瀛楄妭

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //灏唖um鏍￠獙鏁版嵁鏀剧疆鏈€鍚庝竴瀛楄妭

    USART_NItSend(sendBuf, _cnt);
}

/**
 * @brief 鍩轰簬鍖垮悕绉戝垱6.0閫氫俊鍗忚锛屽彂閫佸瓧闀垮害鏁版嵁
 * 
 * @param frame 鐢ㄦ埛鑷畾涔夊姛鑳藉瓧鑺�
 * @param p 鐢ㄦ埛鏁版嵁鎸囬拡
 */
void SendWord(u8 frame, u32* p)
{
    u8 _cnt = 0;
    u8 sum = 0; //浠ヤ笅涓鸿绠梥um鏍￠獙瀛楄妭锛屼粠0xAA涔熷氨鏄瀛楄妭锛屼竴鐩村埌sum瀛楄妭鍓嶄竴瀛楄妭
    int i;

    sendBuf[_cnt++] = 0xAA; //0xAA涓哄抚澶�
    sendBuf[_cnt++] = 0x05; //0x05涓烘暟鎹彂閫佹簮锛屽叿浣撹鍙傝€冨尶鍚嶅崗璁紝鏈瓧鑺傜敤鎴峰彲浠ラ殢鎰忔洿鏀�
    sendBuf[_cnt++] = 0xAF; //0xAF涓烘暟鎹洰鐨勫湴锛孉F琛ㄧず涓婁綅鏈猴紝鍏蜂綋璇峰弬鑰冨尶鍚嶅崗璁�
    sendBuf[_cnt++] = frame; //鐢ㄦ埛鑷畾涔夋暟鎹抚
    sendBuf[_cnt++] = 0; //鏈瓧鑺傝〃绀烘暟鎹暱搴︼紝杩欓噷鍏�=0锛屽嚱鏁版渶鍚庡啀璧嬪€硷紝杩欐牱灏变笉鐢ㄤ汉宸ヨ绠楅暱搴︿簡

    sendBuf[_cnt++] = BYTE3(*p); //灏嗚鍙戦€佺殑鏁版嵁鏀捐嚦鍙戦€佺紦鍐插尯
    sendBuf[_cnt++] = BYTE2(*p);
    sendBuf[_cnt++] = BYTE1(*p);
    sendBuf[_cnt++] = BYTE0(*p);

    sendBuf[4] = _cnt - 5; //_cnt鐢ㄦ潵璁＄畻鏁版嵁闀垮害锛屽噺5涓哄噺鍘诲抚寮€澶�5涓潪鏁版嵁瀛楄妭

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //灏唖um鏍￠獙鏁版嵁鏀剧疆鏈€鍚庝竴瀛楄妭

    USART_NItSend(sendBuf, _cnt);
}

/**
 * @brief 鍩轰簬鍖垮悕绉戝垱6.0閫氫俊鍗忚锛屽彂閫佷簲涓诞鐐规暟
 * 
 * @param frame 鐢ㄦ埛瀹氫箟鍔熻兘瀛楄妭
 * @param f1 娴偣鏁�1
 * @param f2 娴偣鏁�2
 * @param f3 娴偣鏁�3
 * @param f4 娴偣鏁�4
 * @param f5 娴偣鏁�5
 */
void Send_5_float(u8 frame,float f1,float f2,float f3,float f4,float f5){
    u8 _cnt = 0;
    u8 sum = 0; //浠ヤ笅涓鸿绠梥um鏍￠獙瀛楄妭锛屼粠0xAA涔熷氨鏄瀛楄妭锛屼竴鐩村埌sum瀛楄妭鍓嶄竴瀛楄妭
    int i;
    vs16 _temp;

    sendBuf[_cnt++] = 0xAA; //0xAA涓哄抚澶�
    sendBuf[_cnt++] = 0x05; //0x05涓烘暟鎹彂閫佹簮锛屽叿浣撹鍙傝€冨尶鍚嶅崗璁紝鏈瓧鑺傜敤鎴峰彲浠ラ殢鎰忔洿鏀�
    sendBuf[_cnt++] = 0xAF; //0xAF涓烘暟鎹洰鐨勫湴锛孉F琛ㄧず涓婁綅鏈猴紝鍏蜂綋璇峰弬鑰冨尶鍚嶅崗璁�
    sendBuf[_cnt++] = frame; //鐢ㄦ埛鑷畾涔夊抚
    sendBuf[_cnt++] = 0; //鏈瓧鑺傝〃绀烘暟鎹暱搴︼紝杩欓噷鍏�=0锛屽嚱鏁版渶鍚庡啀璧嬪€硷紝杩欐牱灏变笉鐢ㄤ汉宸ヨ绠楅暱搴︿簡

    _temp = f1;
    sendBuf[_cnt++] = BYTE1(_temp); //灏嗚鍙戦€佺殑鏁版嵁鏀捐嚦鍙戦€佺紦鍐插尯
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

    sendBuf[4] = _cnt - 5; //_cnt鐢ㄦ潵璁＄畻鏁版嵁闀垮害锛屽噺5涓哄噺鍘诲抚寮€澶�5涓潪鏁版嵁瀛楄妭

    for (i = 0; i < _cnt; i++)
        sum += sendBuf[i];

    sendBuf[_cnt++] = sum; //灏唖um鏍￠獙鏁版嵁鏀剧疆鏈€鍚庝竴瀛楄妭

    USART_NItSend(sendBuf, _cnt);
}


