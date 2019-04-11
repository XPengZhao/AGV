#include "main.h"
#include "usart6.h"
#include "RefereeSystem.h"
#include "string.h"
//#define RS_DEBUG_INFO

RefereeSystem_t RefereeSystemData;
//CRC8, Ploynomial = X^8+X^5+X^4+1
const unsigned char CRC8_INIT = 0xff;
const unsigned char CRC8_TABLE[256] = {
	0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41, 
	0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
	0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
	0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
	0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
	0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
	0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
	0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9, 
	0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd, 
	0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50, 
	0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 
	0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73, 
	0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b, 
	0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16, 
	0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8, 
	0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35, 
};
//CRC16
uint16_t CRC16_INIT = 0xffff; 
const uint16_t wCRC_Table[256] =
{ 
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 
	0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 
	0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 
	0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 
	0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd, 
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5, 
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974, 
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb, 
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72, 
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738, 
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff, 
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5, 
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134, 
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c, 
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb, 
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232, 
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, 
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9, 
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78, 
};


u8 data_to_send[50];

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
	
//change the define to select which usart used to communicate with
//pc(mainfold or JesonTxone or JesonTxtwo)
//these info will be sent to the pc
//1.Euler Angle of Gimbal
//2.Chassis Motor Speed
//3.Rc Input
//4.Odometer Info(Unimplemented)
//5.
//these Info will Get from PC
//Chassis Control Data (mm/s)
//Gimbal Control Data (Delta angle)

#define USART3_PC_SEND

static void Data_Send(u8 * _val, u8 _len)
{
#ifdef USART3_PC_SEND
	Usart2_Send(_val, _len);
#else
	Usart3_Send(_val, _len);
#endif

}

static void PC_Send( u8 *_val, u8 _len)
{
#ifdef USART3_PC_SEND
	Usart3_Send(_val, _len);
#else
	Usart2_Send(_val, _len);
#endif
}

/**
  * @brief ????task
  * @param ??????????????
  * @retval None
  * @details ??????????????????????????????
  */
void DatatransferTask(u32 sys_time)
{
if (sys_time%10==0)
	{
		ANO_DT_Send_Status(Roll,Pitch,Yaw,SelfCheckErrorFlag,0,0);//???????????
	}
	else if((sys_time+1)%10==0)
	{
		ANO_DT_Send_Senser((vs16)MPU6500_Acc.x,(vs16)MPU6500_Acc.y,(vs16)MPU6500_Acc.z,
												(vs16)MPU6500_Gyro.x,(vs16)MPU6500_Gyro.y,(vs16)MPU6500_Gyro.z,
											(vs16)MagValue.x,(vs16)MagValue.y,(vs16)MagValue.z);
		
	}
	else if((sys_time+2)%10==0)
	{

	}
	else if ((sys_time+5)%10==0)
	{
		
	}
	else
	{

	}
	if((sys_time+3)%20==0)
	{
		RefereeSys_Send_Data(RefereeSystemData.userData);
	}
	else if((sys_time+4)%20==0)
	{
		PC_Send_IMU(Pitch,Roll,Yaw,GMPitchEncoder.ecd_angle,GMYawEncoder.ecd_angle,SelfCheckErrorFlag);
	}
	if ((sys_time+5)%50==0)
	{
		ANO_DT_Send_RCData(RC_CtrlData.rc.ch0,RC_CtrlData.rc.ch1,RC_CtrlData.rc.ch2,RC_CtrlData.rc.ch3,RC_CtrlData.rc.s1*300+1000,RC_CtrlData.rc.s2*300+1000,RC_CtrlData.mouse.x+1000,RC_CtrlData.mouse.y+1000,RC_CtrlData.mouse.z+1000,RC_CtrlData.key.v);
		PC_Send_RC();
	}
	else if((sys_time+6)%50==0)
	{
		ANO_DT_Send_MotoPWM(ABS(CM1Encoder.filter_rate),ABS(CM2Encoder.filter_rate),ABS(CM3Encoder.filter_rate),
												ABS(CM4Encoder.filter_rate),ABS(GMPitchEncoder.filter_rate),ABS(GMYawEncoder.filter_rate),0,0);
		PC_SendMotor(CM1Encoder.filter_rate,CM2Encoder.filter_rate,CM3Encoder.filter_rate,CM4Encoder.filter_rate,
								GMPitchEncoder.filter_rate,GMYawEncoder.filter_rate,0,0);
	}
	else if((sys_time+7)%50==0)
	{
		ANO_DT_Send_Power((u16)RefereeSystemData.extPowerHeatData.chassisPower, (u16)RefereeSystemData.extPowerHeatData.chassisPowerBuffer);
		//ANO_DT_Send_Power(123, 456);
	}
	else if((sys_time +8 )%50 ==0)
	{
		//ANO_DT_Send_Speed(123.0f, 456.0f, 789.0f);
	}
	if (send_check)
	{
		send_check = 0;
		ANO_DT_Send_Check(checkdata_to_send,checksum_to_send);
	}
	if (send_pid1)
	{
		send_pid1=0;
		ANO_DT_Send_PID(1,PID_arg[0].kp,PID_arg[0].ki,PID_arg[0].kd,
											PID_arg[1].kp,PID_arg[1].ki,PID_arg[1].kd,
											PID_arg[2].kp,PID_arg[2].ki,PID_arg[2].kd);
		
	}
	else if(send_pid2)
	{
		send_pid2=0;
		ANO_DT_Send_PID(2,PID_arg[3].kp,PID_arg[3].ki,PID_arg[3].kd,
											PID_arg[4].kp,PID_arg[4].ki,PID_arg[4].kd,
											PID_arg[5].kp,PID_arg[5].ki,PID_arg[5].kd);
	}
	else if (send_pid3)
	{
		send_pid3=0;
				ANO_DT_Send_PID(3,PID_arg[6].kp,PID_arg[6].ki,PID_arg[6].kd,
											PID_arg[7].kp,PID_arg[7].ki,PID_arg[7].kd,
											0,0,0);
	}

}

/**
  * @brief ??2?????
  * @param data	?DR??????????
  * @retval None
  * @details ?????????BasicProtocolAnalysis????
  */
void Usart2_DataPrepare(u8 data)
{
	static u8 RxBuffer[50];
	static u8 _data_len = 0,_data_cnt = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)						//????
	{
		state=1;
		RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)			//??
	{
		state=2;
		RxBuffer[1]=data;
	}
	else if(state==2&&data<0XF1)			//???
	{
		state=3;
		RxBuffer[2]=data;
	}
	else if(state==3&&data<50)				//??
	{
		state = 4;
		RxBuffer[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)		//???
	{
		_data_len--;
		RxBuffer[4+_data_cnt++]=data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)								//???
	{
		state = 0;
		RxBuffer[4+_data_cnt]=data;
		BasicProtocolAnalysis(RxBuffer,_data_cnt+5);
		
	}
	else
		state = 0;

}

/**
  * @brief ??3?????
  * @param data	?DR??????????
  * @retval None
  * @details ?????????BasicProtocolAnalysis????
  */
void Usart3_DataPrepare(u8 data)
{
	static u8 RxBuffer[50];
	static u8 _data_len = 0,_data_cnt = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)						//????
	{
		state=1;
		RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)			//??
	{
		state=2;
		RxBuffer[1]=data;
	}
	else if(state==2&&data<0XF1)			//???
	{
		state=3;
		RxBuffer[2]=data;
	}
	else if(state==3&&data<50)				//??
	{
		state = 4;
		RxBuffer[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)		//???
	{
		_data_len--;
		RxBuffer[4+_data_cnt++]=data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)								//???
	{
		state = 0;
		RxBuffer[4+_data_cnt]=data;
		BasicProtocolAnalysis(RxBuffer,_data_cnt+5);
		
	}
	else
		state = 0;


}



/**
  * @brief 串口6数据预解析
  * @param data	从DR寄存器中读取到的数据
  * @retval None
  * @details 若解析成功则跳转到BasicProtocolAnalysis进行处理
  */
u8 CRC8 = 0;	u16 FrameTail = 0;
void Usart6_DataPrepare(u8* pData)
{
	u8 index = 0;
	u16 _data_len = 0;
	u8 Seq = 0;
	u16 CmdID = 0;	
    
    u32 floatdata = 0;
    
	if(pData == NULL)
		return;

	while(index < BSP_USART6_DMA_RX_BUF_LEN && pData[index++] != 0xA5);	//is it the start of frame?

	if(index > BSP_USART6_DMA_RX_BUF_LEN)
		return;
	
	if((index + 1) < BSP_USART6_DMA_RX_BUF_LEN)							// get the length of data
	{
		_data_len = (u16)pData[index] | ((u16)pData[index + 1])<<8;
		index++;
	}
	if(_data_len > (BSP_USART6_DMA_RX_BUF_LEN - index - 6))				//Seq(1B)+CRC8(1B)+CmdID(2B)+FrameTail(2B) 
		return;
	else{
		Seq = pData[++index];
		CRC8 = pData[++index];
        if(!(Verify_CRC8_Check_Sum(pData,5,CRC8_INIT)))
            return;
		CmdID = pData[index + 1] | ((u16)pData[index + 2])<<8;
		index +=2;
		FrameTail = pData[7+_data_len] | (u16)(pData[8+_data_len]<<8);
		switch(CmdID)
		{
			case ROBOTSTATE:
			// get stageRemainTime
				RefereeSystemData.extGameRobotState.stageRemainTime = ((u16)pData[index + 1]) | ((u16)pData[index + 2])<<8;
				index +=2;
			// get game process
				RefereeSystemData.extGameRobotState.gameProcess = pData[++index];
			// get robot level
				RefereeSystemData.extGameRobotState.robotLevel = pData[++index];
			// get remain HP
				RefereeSystemData.extGameRobotState.remainHP = ((u16)pData[index + 1]) | ((u16)pData[index + 2])<<8;
				index +=2;
			// get maxHP
				RefereeSystemData.extGameRobotState.maxHP = ((u16)pData[index + 1]) | ((u16)pData[index + 2])<<8;
				index +=2;
			// get position_t
				RefereeSystemData.extGameRobotState.position.validFlag = pData[++index];
                floatdata = (u32)pData[index+1] | (u32)pData[index+2]<<8 | (u32)pData[index+3]<<16 | (u32)pData[index+4]<<24;
                RefereeSystemData.extGameRobotState.position.x = *((float*)&floatdata);
				index +=4;
                floatdata = (u32)pData[index+1] | (u32)pData[index+2]<<8 | (u32)pData[index+3]<<16 | (u32)pData[index+4]<<24;
				RefereeSystemData.extGameRobotState.position.y = *((float *)&floatdata);	
				index +=4;
                floatdata = (u32)pData[index+1] | (u32)pData[index+2]<<8 | (u32)pData[index+3]<<16 | (u32)pData[index+4]<<24;                
				RefereeSystemData.extGameRobotState.position.z = *((float*)&floatdata);				
				index +=4;
                floatdata = (u32)pData[index+1] | (u32)pData[index+2]<<8 | (u32)pData[index+3]<<16 | (u32)pData[index+4]<<24;                
				RefereeSystemData.extGameRobotState.position.yaw = *((float*)&floatdata);				
				index +=4;
			// output the received data
			#ifdef RS_DEBUG_INFO
   			 	printf("Seq:\t%d\r\n",Seq);
				printf("RobotState:\r\n");
				printf("stageRemainTime:%d\r\n",RefereeSystemData.extGameRobotState.stageRemainTime);
				printf("gameProcess:%d\r\n",RefereeSystemData.extGameRobotState.gameProcess);
				printf("robotLevel:%d\r\n",RefereeSystemData.extGameRobotState.robotLevel);
				printf("remainHP:%d\r\n",RefereeSystemData.extGameRobotState.remainHP);
				printf("maxHP:%d\r\n",RefereeSystemData.extGameRobotState.maxHP);
				printf("position X:\t%f\r\n",RefereeSystemData.extGameRobotState.position.x);
				printf("position Y:\t%f\r\n",RefereeSystemData.extGameRobotState.position.y);
				printf("position Z:\t%f\r\n",RefereeSystemData.extGameRobotState.position.z);
				printf("position Yaw:\t%f\r\n",RefereeSystemData.extGameRobotState.position.yaw);
    			printf("\r\n");
			#endif
				break;
			case ROBOTHURT:
			// get armorType
				RefereeSystemData.extRobotHurt.armorType = pData[index + 1] & 0x0f;
			// get hurtType
				RefereeSystemData.extRobotHurt.hurtType = (pData[index + 1] & 0xf0)>>4;	
				index +=1;
			#ifdef RS_DEBUG_INFO
				printf("Seq:\t%d\r\n",Seq);
				printf("RobotHurtData:\r\n");
				printf("armorType:\t%d\r\n",RefereeSystemData.extRobotHurt.armorType);
				printf("hurtType:\t%d\r\n",RefereeSystemData.extRobotHurt.hurtType);
				printf("\r\n");
			#endif	
				break;
			case SHOOTDATA:
			// get the bullet type
				RefereeSystemData.extShootData.bulletType = pData[++index];
			// get the frequency of shoot
				RefereeSystemData.extShootData.bulletFreq = pData[++index];
			// get the bullet speed
                floatdata = (u32)pData[index+1] | (u32)pData[index+2]<<8 | (u32)pData[index+3]<<16 | (u32)pData[index+4]<<24;                            
				RefereeSystemData.extShootData.bulletSpeed = *((float*)&(floatdata));
				index +=4;
			// get the reserved data
				/**/
			#ifdef RS_DEBUG_INFO
				printf("Seq:\t%d\r\n",Seq);
				printf("ShootData:\r\n");
				if(RefereeSystemData.extShootData.bulletType == 1)
					printf("bulletType:\t17 mm\r\n");
				else if(RefereeSystemData.extShootData.bulletType == 2)
					printf("bulletType:\t42 mm\r\n");
				printf("bulletFreq: %d\tHz\r\n",RefereeSystemData.extShootData.bulletFreq);
				printf("bulletSpeed: %f\tm/s\r\n",RefereeSystemData.extShootData.bulletSpeed);
				printf("\r\n");
			#endif
				break;
			case POWERHEAT:
			// get power data
                floatdata = (u32)pData[index+1] | (u32)pData[index+2]<<8 | (u32)pData[index+3]<<16 | (u32)pData[index+4]<<24;                
				RefereeSystemData.extPowerHeatData.chassisVolt = *((float*)&floatdata);
				index +=4;
                floatdata = (u32)pData[index+1] | (u32)pData[index+2]<<8 | (u32)pData[index+3]<<16 | (u32)pData[index+4]<<24;                            
				RefereeSystemData.extPowerHeatData.chassisCurrent = *((float*)&floatdata);
				index +=4;
                floatdata = (u32)pData[index+1] | (u32)pData[index+2]<<8 | (u32)pData[index+3]<<16 | (u32)pData[index+4]<<24;                
				RefereeSystemData.extPowerHeatData.chassisPower = *((float*)&floatdata);
				index +=4;
                floatdata = (u32)pData[index+1] | (u32)pData[index+2]<<8 | (u32)pData[index+3]<<16 | (u32)pData[index+4]<<24;                
				RefereeSystemData.extPowerHeatData.chassisPowerBuffer = *((float*)&floatdata);
				index +=4;
			// get heat data
				RefereeSystemData.extPowerHeatData.shooterHeat0 = ((u16)pData[index + 1]) | ((u16)pData[index + 2])<<8;
				index +=2;
				RefereeSystemData.extPowerHeatData.shooterHeat1 = ((u16)pData[index + 1]) | ((u16)pData[index + 2])<<8;
				index +=2;
			#ifdef RS_DEBUG_INFO
				printf("Seq:\t%d\r\n",Seq);
				printf("Power & Heat:\r\n");
				printf("chassisVolt: %f\tV\r\n",RefereeSystemData.extPowerHeatData.chassisVolt);
				printf("chassisCurrent: %f\tA\r\n",RefereeSystemData.extPowerHeatData.chassisCurrent);
				printf("chassisPower: %f\tW\r\n",RefereeSystemData.extPowerHeatData.chassisPower);				
				printf("chassisPowerBuffer: %f\tW\r\n",RefereeSystemData.extPowerHeatData.chassisPowerBuffer);
				printf("17 mm shooter's heat: %d\tJ\r\n",RefereeSystemData.extPowerHeatData.shooterHeat0);
				printf("42 mm shooter's heat: %d\tJ\r\n",RefereeSystemData.extPowerHeatData.shooterHeat1);
				printf("\r\n");
			#endif
				break;
			case RFIDDETECT:
				RefereeSystemData.extRfidDetect.cardType = pData[++index];
				RefereeSystemData.extRfidDetect.cardIndex = pData[++index];
			#ifdef RS_DEBUG_INFO
				printf("Seq:\t%d\r\n",Seq);
				printf("Rfid detect:\r\n");
				printf("cardType: %d\r\n",RefereeSystemData.extRfidDetect.cardType);
				printf("cardIndex: %d\r\n",RefereeSystemData.extRfidDetect.cardIndex);
				printf("\r\n");
			#endif
				break;
			case GAMERESULT:
				RefereeSystemData.extGameResult.winner = pData[++index];
			#ifdef RS_DEBUG_INFO
				printf("Seq:\t%d\r\n",Seq);
				printf("Game Result:\r\n");
				switch(RefereeSystemData.extGameResult.winner)
				{
					case 0:
						printf("match endded in a draw\r\n");break;
					case 1:
						printf("Red win!\r\n");break;
					case 2:
						printf("Blue win!\r\n");break;
					default: break;
				}
				printf("\r\n");
			#endif
				break;
			case BUFF:
				RefereeSystemData.extGetBuff.buffType = pData[++index];
				RefereeSystemData.extGetBuff.buffAddition = pData[++index];
			#ifdef RS_DEBUG_INFO
				printf("Seq:\t%d\r\n",Seq);
				printf("Buff Type:\r\n");
				printf("buffer type: %d\r\n",RefereeSystemData.extGetBuff.buffType);
				printf("buffer addition: %d\r\n",RefereeSystemData.extGetBuff.buffAddition);
				printf("\r\n");
			#endif
				break;
			case RESERVED:
			#ifdef RS_DEBUG_INFO
				printf("Seq\t%d\r\n",Seq);
				printf("UserData:\r\n");
				floatdata = (u32)pData[index+1] | (u32)pData[index+2]<<8 | (u32)pData[index+3]<<16 | (u32)pData[index+4]<<24;                
				printf("data1: %f\r\n",*((float*)&floatdata));
				index +=4;
				floatdata = (u32)pData[index+1] | (u32)pData[index+2]<<8 | (u32)pData[index+3]<<16 | (u32)pData[index+4]<<24;                
				printf("data2: %f\r\n",*((float*)&floatdata));
				index +=4;	
				floatdata = (u32)pData[index+1] | (u32)pData[index+2]<<8 | (u32)pData[index+3]<<16 | (u32)pData[index+4]<<24;                
				printf("data3: %f\r\n",*((float*)&floatdata));
				index +=4;			
				printf("mask: %d\r\n",pData[index+1]);
				printf("\r\n");
			#endif
				break;
			default:	//error cmd
				break;
		}
	}

	return;
}

/**
  * @brief 发送attack
  * @param head 帧类型
  * @param check_sum 校验位
  * @retval None
  */
void ANO_DT_Send_Check(u8 head, u8 check_sum)
{
	u8 sum = 0;
	u8 i;
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEF;
	data_to_send[3]=2;
	data_to_send[4]=head;
	data_to_send[5]=check_sum;
	
	

	for(i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

	Data_Send(data_to_send, 7);
}


/**
  * @brief 向上位机发送三个欧拉角
  * @param 三个欧拉角，
  * @retval None
  */
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, u32 alt, u8 fly_model, u8 armed)
{
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2 = alt;
	u8 sum = 0;
	u8 i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	Data_Send(data_to_send, _cnt);
}

void ANO_DT_Send_Power(u16 votage, u16 current)
{
	u8 _cnt=0;
	u16 temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0;
	
	temp = votage;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	temp = current;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	Data_Send(data_to_send, _cnt);
}

void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)
{
	u8 _cnt=0;
	vs16 _temp;
		u8 sum = 0;
	u8 i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
/////////////////////////////////////////
	_temp = 0;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	
	
	data_to_send[3] = _cnt-4;
	

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Data_Send(data_to_send, _cnt);
}


/**
  * @brief 发送遥控器数据
  * @param None
  * @retval None
  */
void ANO_DT_Send_RCData(u16 ch0,u16 ch1,u16 ch2,u16 ch3,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
	u8 _cnt=0;
	u8 i=0;
	u8 sum = 0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(ch0);
	data_to_send[_cnt++]=BYTE0(ch0);
	data_to_send[_cnt++]=BYTE1(ch1);
	data_to_send[_cnt++]=BYTE0(ch1);
	data_to_send[_cnt++]=BYTE1(ch2);
	data_to_send[_cnt++]=BYTE0(ch2);
	data_to_send[_cnt++]=BYTE1(ch3);
	data_to_send[_cnt++]=BYTE0(ch3);
	data_to_send[_cnt++]=BYTE1(aux1);
	data_to_send[_cnt++]=BYTE0(aux1);
	data_to_send[_cnt++]=BYTE1(aux2);
	data_to_send[_cnt++]=BYTE0(aux2);
	data_to_send[_cnt++]=BYTE1(aux3);
	data_to_send[_cnt++]=BYTE0(aux3);
	data_to_send[_cnt++]=BYTE1(aux4);
	data_to_send[_cnt++]=BYTE0(aux4);
	data_to_send[_cnt++]=BYTE1(aux5);
	data_to_send[_cnt++]=BYTE0(aux5);
	data_to_send[_cnt++]=BYTE1(aux6);
	data_to_send[_cnt++]=BYTE0(aux6);

	data_to_send[3] = _cnt-4;
	

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	Data_Send(data_to_send, _cnt);
}

/**
  * @brief 发送PID至上位机
  * @param 三组PID的值及其group序号
  * @retval None
  * @details None
  */
void ANO_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	u8 _cnt=0;
	vs16 _temp;
	u8 sum=0;
	u8 i=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10+group-1;
	data_to_send[_cnt++]=0;
	
	
	_temp = p1_p * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_i  * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_d  * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_p  * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_i  * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_d * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_p  * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_i  * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_d * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	Data_Send(data_to_send, _cnt);
}
/**
  * @brief 发送电机转速
  * @param None
  * @retval None
  */
void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8)
{
	u8 _cnt=0;
	u8 sum = 0;
	u8 i=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1(m_1);
	data_to_send[_cnt++]=BYTE0(m_1);
	data_to_send[_cnt++]=BYTE1(m_2);
	data_to_send[_cnt++]=BYTE0(m_2);
	data_to_send[_cnt++]=BYTE1(m_3);
	data_to_send[_cnt++]=BYTE0(m_3);
	data_to_send[_cnt++]=BYTE1(m_4);
	data_to_send[_cnt++]=BYTE0(m_4);
	data_to_send[_cnt++]=BYTE1(m_5);
	data_to_send[_cnt++]=BYTE0(m_5);
	data_to_send[_cnt++]=BYTE1(m_6);
	data_to_send[_cnt++]=BYTE0(m_6);
	data_to_send[_cnt++]=BYTE1(m_7);
	data_to_send[_cnt++]=BYTE0(m_7);
	data_to_send[_cnt++]=BYTE1(m_8);
	data_to_send[_cnt++]=BYTE0(m_8);
	
	data_to_send[3] = _cnt-4;
	
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	Data_Send(data_to_send, _cnt);
}


void ANO_DT_Send_Speed(float x_s,float y_s,float z_s)
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x0B;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(0.1f *x_s);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(0.1f *y_s);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(0.1f *z_s);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	Data_Send(data_to_send, _cnt);

}

void RefereeSys_Send_Data(extUserData_t userdata)
{
	static u8 pSeq = 0;
	u32 u32data;
	u16 wCRC = 0;
	u8 frame[22];
	//define the head of frame
	frame[0] = 0xA5;
	frame[1] = sizeof(extUserData_t) & 0x00ff;
	frame[2] = (sizeof(extUserData_t) & 0xff00)>>8;
	frame[3] = pSeq++;
	frame[4] = Get_CRC8_Check_Sum(frame,4,CRC8_INIT);
	//define the CmdID: 0x0100
	frame[5] = 0x00;
	frame[6] = 0x01;
	//define the user data
	u32data = *((u32*)&(userdata.data1));
	frame[7] = u32data & 0x000000ff;
	frame[8] = (u32data & 0x0000ff00)>>8;
	frame[9] = (u32data & 0x00ff0000)>>16;
	frame[10]= (u32data & 0xff000000)>>24;

	u32data = *((u32*)&(userdata.data2));
	frame[11] = u32data & 0x000000ff;
	frame[12] = (u32data & 0x0000ff00)>>8;
	frame[13] = (u32data & 0x00ff0000)>>16;
	frame[14]= (u32data & 0xff000000)>>24;

	u32data = *((u32*)&(userdata.data3));
	frame[15] = u32data & 0x000000ff;
	frame[16] = (u32data & 0x0000ff00)>>8;
	frame[17] = (u32data & 0x00ff0000)>>16;
	frame[18]= (u32data & 0xff000000)>>24;

	frame[19] = userdata.mask;
	//write frame tail -> CRC16
	wCRC = Get_CRC16_Check_Sum(frame,20,CRC16_INIT);
	frame[20] = wCRC & 0x00ff;
	frame[21] = (u8)((wCRC>>8) & 0x00ff);
	Usart6_Send(frame,22);
	return;
}

unsigned char Get_CRC8_Check_Sum(u8* pMessage, u32 dwLength, u8 ucCRC8)
{
	unsigned char index;
	while(dwLength--)
	{
		index = ucCRC8 ^ (*pMessage++);
		ucCRC8 = CRC8_TABLE[index];
	}
	return ucCRC8;
}

/*
**Description : CRC8 Verify function
**Input :Data to verify , Stream length = Data + checksum
**Output:True or False (CRC Verify Result)
*/
unsigned char Verify_CRC8_Check_Sum(u8* pMessage, u32 dwLength, u8 ucCRC8)
{
	unsigned char ucExpected = 0;
	if(pMessage == 0 || dwLength <= 2)	return 0;
	ucExpected = Get_CRC8_Check_Sum(pMessage, dwLength - 1, CRC8_INIT);
	return (ucExpected  == pMessage[dwLength - 1]);
}

/* 
** Descriptions: CRC16 checksum function 
** Input: Data to check,Stream length, initialized checksum 
** Output: CRC checksum 
*/ 
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC)
{ 
	uint8_t chData; 
	if (pchMessage == NULL) 
	{ 
		return 0xFFFF; 
	} 
	while(dwLength--)
	{ 
		chData = *pchMessage++;
		(wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
	} 
	return wCRC;
}

/* 
** Descriptions: CRC16 Verify function 
** Input: Data to Verify,Stream length = Data + checksum 
** Output: True or False (CRC Verify Result) 
*/ 
unsigned char Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{ 
	uint16_t wExpected = 0;
	if ((pchMessage == NULL) || (dwLength <= 2))
	{ 
		return 0; 
	} 
	wExpected = Get_CRC16_Check_Sum ( pchMessage, dwLength - 2, CRC16_INIT);
	return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

void PC_Send_IMU(float pit, float rol, float yaw,float Gim_Pitch, float Gim_Yaw, u32 alt)
{
	u8 i=0;
	u8 sum = 0;
	PC_Send_IMU_t PC_IMU_Message;
	PC_IMU_Message.Pitch=pit;
	PC_IMU_Message.Roll=rol;
	PC_IMU_Message.Yaw=yaw;
	PC_IMU_Message.state=alt;
	PC_IMU_Message.Gimbal_Pitch=Gim_Pitch;
	PC_IMU_Message.Gimbal_Yaw=Gim_Yaw;
	
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0x01;
	data_to_send[3]=sizeof(PC_IMU_Message);
	memcpy(data_to_send+4,(u8*)&PC_IMU_Message,sizeof(PC_IMU_Message));
	for(i=0;i<sizeof(PC_IMU_Message)+4;i++)
		sum += data_to_send[i];
	data_to_send[sizeof(PC_IMU_Message)+4]=sum;
	PC_Send(data_to_send, sizeof(PC_IMU_Message)+5);
}

void PC_Send_RC(void)
{
	u8 i=0;
	u8 sum = 0;
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0x02;
	data_to_send[3]=sizeof(RC_CtrlData);
	memcpy(data_to_send+4,(u8*)&RC_CtrlData,sizeof(RC_CtrlData));
	for(i=0;i<sizeof(RC_CtrlData)+4;i++)
		sum += data_to_send[i];
	data_to_send[sizeof(RC_CtrlData)+4]=sum;
	PC_Send(data_to_send, sizeof(RC_CtrlData)+5);
}
void PC_SendMotor(s16 m_1,s16 m_2,s16 m_3,s16 m_4,s16 m_5,s16 m_6,s16 m_7,s16 m_8)
{
	u8 i=0;
	u8 sum = 0;
	PC_Send_Motor_t	PC_Send_Motor;
	PC_Send_Motor.motor1=m_1*CHASSIS_DECELE_RATIO;
	PC_Send_Motor.motor2=m_2*CHASSIS_DECELE_RATIO;
	PC_Send_Motor.motor3=m_3*CHASSIS_DECELE_RATIO;
	PC_Send_Motor.motor4=m_4*CHASSIS_DECELE_RATIO;
	PC_Send_Motor.motor5=m_5*PIT_DECELE_RATIO;
	PC_Send_Motor.motor6=m_6*YAW_DECELE_RATIO;
	PC_Send_Motor.motor7=m_7;
	PC_Send_Motor.motor8=m_8;

	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0x03;
	data_to_send[3]=sizeof(PC_Send_Motor);
	memcpy(data_to_send+4,(u8*)&PC_Send_Motor,sizeof(PC_Send_Motor));
	for(i=0;i<sizeof(PC_Send_Motor)+4;i++)
		sum += data_to_send[i];
	data_to_send[sizeof(PC_Send_Motor)+4]=sum;
	PC_Send(data_to_send, sizeof(PC_Send_Motor)+5);
}

