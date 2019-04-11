#include "main.h"
#include "stdlib.h"
#include "string.h"
#define caliMaxTime 100
uint16_t can_encoder_flag;
u8 yaw_cali_count = 0;
u8 pitch_cali_count = 0;
uint32_t yaw_cali_sum;
uint32_t pitch_cali_sum;
volatile Encoder CM1Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder CM2Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder CM3Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder CM4Encoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder GMYawEncoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder GMPitchEncoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder DialingEncoder = {0,0,0,0,0,0,0,0,0};
PC_control_t PC_control={0.0f,0.0f,0.0f,0.0f,0.0f};

RC_Ctrl_t RC_CtrlData;   //remote control data
u8 checkdata_to_send,checksum_to_send,send_check=0;
u8 send_pid1=0,send_pid2=0,send_pid3=0;


/**
  * @brief 基本串口通讯协议解析
  * @param data_buf	包含完整一帧数据的数组的指针
	* @param _len		帧总长
  * @retval None
  * @details 上层硬件发来的信号或是地面站发来的信号的解析函数
  */
void BasicProtocolAnalysis(u8 const *data_buf,int _len)
{
	u8 sum = 0;
	u8 i;
	for(i=0;i<(_len-1);i++)														//求和校验
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+_len-1)))		return;						//校验不成功则return
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;	//帧头校验不成功则return
	if(*(data_buf+2)==0X01)														//分析帧类型
	{
		if(*(data_buf+4)==0X01)
		{
			IMU_ACCERDataCali();
			
		}
		else if(*(data_buf+4)==0X02)
		{
			IMU_GYRODataCali();
		}
		else if(*(data_buf+4)==0X03)
		{
			IMU_ACCERDataCali();	
			IMU_GYRODataCali();	
		}
		else if(*(data_buf+4)==0X04)
		{
			IMU_MAGDataCali();

		}
		else if (*(data_buf+4)==0X05)
		{
			ParaSavingFlag=1;
		}
		else if((*(data_buf+4)>=0X021)&&(*(data_buf+4)<=0X26))
		{
		}
		else if(*(data_buf+4)==0X20)
		{
		}
	}
	if(*(data_buf+2)==0X22){
			//NS=(enum PendulumMode)(*(data_buf+4));
		}
			if(*(data_buf+2)==0X21){
			//NS=Stop;
		}
			if (*(data_buf+2)==0X25)	//PC_Control Info
			{
				memcpy((u8*)(&PC_control), (data_buf+4), sizeof(PC_control_t));
				FeedDog(DEVICE_INDEX_PC);
			}
if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)
		{
			send_pid1 = 1;
			send_pid2 = 1;
			send_pid3 = 1;
		}
		if(*(data_buf+4)==0XA1)	
		{
			ParametersInit();
		}
	}
		if(*(data_buf+2)==0X10)								//PID1
    {
        PID_arg[0].kp  = 0.01*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        PID_arg[0].ki  = 0.01*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        PID_arg[0].kd  = 0.01*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        PID_arg[1].kp = 0.01*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        PID_arg[1].ki = 0.01*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        PID_arg[1].kd = 0.01*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        PID_arg[2].kp 	= 0.01*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        PID_arg[2].ki 	= 0.01*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        PID_arg[2].kd 	= 0.01*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
				if(send_check == 0)
				{
					send_check = 1;
					checkdata_to_send = *(data_buf+2);
					checksum_to_send = sum;
				}
    }
		 if(*(data_buf+2)==0X11)								//PID2
    {
        PID_arg[3].kp  = 0.01*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        PID_arg[3].ki  = 0.01*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        PID_arg[3].kd  = 0.01*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        PID_arg[4].kp = 0.01*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        PID_arg[4].ki = 0.01*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        PID_arg[4].kd = 0.01*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
				PID_arg[5].kp 	= 0.01*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        PID_arg[5].ki 	= 0.01*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        PID_arg[5].kd 	= 0.01*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
						if(send_check == 0)
				{
					send_check = 1;
					checkdata_to_send = *(data_buf+2);
					checksum_to_send = sum;
				}
    }
	
    	if(*(data_buf+2)==0X03)
	{

		
	}
    if(*(data_buf+2)==0X12)								//PID3
    {	
			  PID_arg[6].kp  = 0.01*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        PID_arg[6].ki  = 0.01*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        PID_arg[6].kd  = 0.01*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        PID_arg[7].kp = 0.01*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        PID_arg[7].ki = 0.01*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        PID_arg[7].kd = 0.01*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
       	if(send_check == 0)
				{
					send_check = 1;
					checkdata_to_send = *(data_buf+2);
					checksum_to_send = sum;
				}
    }
	if(*(data_buf+2)==0X13)								//PID4
	{
		   	if(send_check == 0)
				{
					send_check = 1;
					checkdata_to_send = *(data_buf+2);
					checksum_to_send = sum;
				}
	}
	if(*(data_buf+2)==0X14)								//PID5
	{
						if(send_check == 0)
				{
					send_check = 1;
					checkdata_to_send = *(data_buf+2);
					checksum_to_send = sum;
				}
	}
	if(*(data_buf+2)==0X15)								//PID6
	{
						if(send_check == 0)
				{
					send_check = 1;
					checkdata_to_send = *(data_buf+2);
					checksum_to_send = sum;
				}
	}


}

u8 WHEEL_STATE = WHEEL_OFF;//摩擦轮状态


/**
  * @brief 对遥控器解析结果进行反应
  * @param None
  * @retval None
  */

void RcDataAnalysis(RC_Ctrl_t *rc)
{
	static u16 cancel_cnt = 0;//鼠标模式下右键计时超过1.5s关闭摩擦轮
	float __temp;
	if (IsDeviceLost(DEVICE_INDEX_PC))
	{
		if (GetRcMode()==RC_KEY_RCMODE)
		{
			if(my_abs(rc->rc.ch1-CHANNELMIDDLE)>IDLE_RC_VALUE)
			{
				__temp=GimbalPitchPosRef-(rc->rc.ch1-CHANNELMIDDLE)*RC_PITCHSCALE;
			}
			else
			{
				__temp=GimbalPitchPosRef;
			}
			GimbalPitchPosRef=LIMIT(__temp,PITCH_MIN,PITCH_MAX);
			if(my_abs(rc->rc.ch0-CHANNELMIDDLE)>IDLE_RC_VALUE)
			{
				__temp=GimbalYawPosRef-(rc->rc.ch0-CHANNELMIDDLE)*RC_YAWSCALE;
			}
			else
			{
				__temp=GimbalYawPosRef;
			}
			GimbalYawPosRef=LIMIT(__temp,-YAW_MAX-Yaw,YAW_MAX-Yaw);

			ChassisGoToward=(rc->rc.ch3-CHANNELMIDDLE)*RC_TOWARD_SCALE;
			ChassisGoLeftRight=(rc->rc.ch2-CHANNELMIDDLE)*RC_LEFTRIGHT_SCALE;
			if(rc->rc.s1 == 3)
			{
				WHEEL_STATE = WHEEL_ON;
			}
			else
			{
				WHEEL_STATE = WHEEL_OFF;
			}
		}
		else if (GetRcMode()==RC_KEY_KEYBOARD)
		{
			if(rc->key.v & KEY_W)//按下w键
			{
				ChassisGoToward = MAXTOWARDSPEED * RampCalculate(&RcKeyTowardRamp);
			}
			else if(rc->key.v & KEY_S)
			{
				ChassisGoToward = -MAXTOWARDSPEED * RampCalculate(&RcKeyTowardRamp);
			}
			else
			{
				RampReset(&RcKeyTowardRamp);
				ChassisGoToward = 0;
			}
			if(rc->key.v & KEY_A)
			{
				ChassisGoLeftRight = MAXLEFTRIGHTSPEED * RampCalculate(&RcKeyLeftRightRamp);
			}
			else if(rc->key.v & KEY_D)
			{
				ChassisGoLeftRight = -MAXLEFTRIGHTSPEED * RampCalculate(&RcKeyLeftRightRamp);
			}
			else
			{
				RampReset(&RcKeyLeftRightRamp);
				ChassisGoLeftRight = 0;
			}
			GimbalYawPosRef = LIMIT(GimbalYawPosRef-(rc->mouse.x)*MOUSERESPONCERATE,-YAW_MAX-Yaw,YAW_MAX-Yaw); 
			GimbalPitchPosRef = LIMIT(GimbalPitchPosRef-(rc->mouse.y)*MOUSERESPONCERATE,PITCH_MIN,PITCH_MAX);
			if(rc->mouse.press_r == 1)
			{
				cancel_cnt++;
				if(WHEEL_STATE == WHEEL_OFF&&cancel_cnt==1)
				{
					WHEEL_STATE = WHEEL_ON;
					LASER_ON();
				}
				else if(cancel_cnt >= 50)
				{
					WHEEL_STATE = WHEEL_OFF;
					LASER_OFF(); 
				}
			}
			else
			{
				cancel_cnt = 0;
			}
		}
	}
	else 
	{
		if (PC_control.Valid_flag&PC_CONTORL_CHASSIS_X_VALID)
		{
			ChassisGoToward=PC_control.chassis_x_speed;
		}
		if(PC_control.Valid_flag&PC_CONTORL_CHASSIS_Y_VALID)
		{
			ChassisGoLeftRight=PC_control.chassis_y_speed;
		}
		if (PC_control.Valid_flag&PC_CONTORL_GIMBAL_PITCH_VALID)
		{
			__temp=GMPitchEncoder.ecd_angle+PC_control.Pitch_change;
			GimbalPitchPosRef=LIMIT(__temp,PITCH_MIN,PITCH_MAX);
		}
		if (PC_control.Valid_flag&PC_CONTORL_GIMBAL_YAW_VALID)
		{
			__temp=Yaw+PC_control.Yaw_change;
			GimbalYawPosRef=LIMIT(__temp,-YAW_MAX-Yaw,YAW_MAX-Yaw);
		}
		if (GetRcMode()==RC_KEY_RCMODE)
		{
			if (!(PC_control.Valid_flag&PC_CONTORL_CHASSIS_X_VALID))
			{
				ChassisGoToward=(rc->rc.ch3-CHANNELMIDDLE)*RC_TOWARD_SCALE;
			}
		
			if(!(PC_control.Valid_flag&PC_CONTORL_CHASSIS_Y_VALID))
			{
				ChassisGoLeftRight=(rc->rc.ch2-CHANNELMIDDLE)*RC_LEFTRIGHT_SCALE;
			}
			if (!(PC_control.Valid_flag&PC_CONTORL_GIMBAL_PITCH_VALID))
			{
				if(my_abs(rc->rc.ch1-CHANNELMIDDLE)>IDLE_RC_VALUE)
				{
					__temp=GimbalPitchPosRef-(rc->rc.ch1-CHANNELMIDDLE)*RC_PITCHSCALE;
				}
				else
				{
					__temp=GimbalPitchPosRef;
				}
				GimbalPitchPosRef=LIMIT(__temp,PITCH_MIN,PITCH_MAX);
			}
			if (!(PC_control.Valid_flag&PC_CONTORL_GIMBAL_YAW_VALID))
			{
				if(my_abs(rc->rc.ch0-CHANNELMIDDLE)>IDLE_RC_VALUE)
				{
					__temp=GimbalYawPosRef-(rc->rc.ch0-CHANNELMIDDLE)*RC_YAWSCALE;
				}
				else
				{
					__temp=GimbalYawPosRef;
				}
				GimbalYawPosRef=LIMIT(__temp,-YAW_MAX-Yaw,YAW_MAX-Yaw);
			}
			if(rc->rc.s1 == 3)
			{
				WHEEL_STATE = WHEEL_ON;
			}
			else
			{
				WHEEL_STATE = WHEEL_OFF;
			}
		}
		else if (GetRcMode()==RC_KEY_KEYBOARD)
		{
			
			
			if (!(PC_control.Valid_flag&PC_CONTORL_CHASSIS_X_VALID))
			{
				if(rc->key.v & KEY_W)//按下w键
				{
					ChassisGoToward = MAXTOWARDSPEED * RampCalculate(&RcKeyTowardRamp);
				}
				else if(rc->key.v & KEY_S)
				{
					ChassisGoToward = -MAXTOWARDSPEED * RampCalculate(&RcKeyTowardRamp);
				}
				else
				{
					RampReset(&RcKeyTowardRamp);
					ChassisGoToward = 0;
				}
			}
			if (!(PC_control.Valid_flag&PC_CONTORL_CHASSIS_Y_VALID))
			{
				if(rc->key.v & KEY_A)
				{
					ChassisGoLeftRight = MAXLEFTRIGHTSPEED * RampCalculate(&RcKeyLeftRightRamp);
				}
				else if(rc->key.v & KEY_D)
				{
					ChassisGoLeftRight = -MAXLEFTRIGHTSPEED * RampCalculate(&RcKeyLeftRightRamp);
				}
				else
				{
					RampReset(&RcKeyLeftRightRamp);
					ChassisGoLeftRight = 0;
				}
			}
			if (!(PC_control.Valid_flag&PC_CONTORL_GIMBAL_YAW_VALID))
			{
				GimbalYawPosRef = LIMIT(GimbalYawPosRef-(rc->mouse.x)*MOUSERESPONCERATE,-YAW_MAX-Yaw,YAW_MAX-Yaw);
			}
			if (!(PC_control.Valid_flag&PC_CONTORL_GIMBAL_PITCH_VALID))
			{
				GimbalPitchPosRef = LIMIT(GimbalPitchPosRef-(rc->mouse.y)*MOUSERESPONCERATE,PITCH_MIN,PITCH_MAX);
			}
				if(rc->mouse.press_r == 1)
				{
					cancel_cnt++;
					if(WHEEL_STATE == WHEEL_OFF&&cancel_cnt==1)
					{
						WHEEL_STATE = WHEEL_ON;
						LASER_ON();
					}
					else if(cancel_cnt >= 50)
					{
						WHEEL_STATE = WHEEL_OFF;
						LASER_OFF(); 
					}
				}
			else
			{
				cancel_cnt = 0;
			}
		}
	
	
	
	}
//	else
//	{
//		__temp=GMYawEncoder.ecd_angle+PC_control.Pitch_change;
//		GimbalPitchPosRef=LIMIT(__temp,PITCH_MIN,PITCH_MAX);
//		__temp=Yaw+PC_control.Yaw_change;
//		GimbalYawPosRef=LIMIT(__temp,-YAW_MAX-Yaw,YAW_MAX-Yaw);
//		ChassisGoToward=(rc->rc.ch3-CHANNELMIDDLE)*RC_TOWARD_SCALE;
//		ChassisGoLeftRight=(rc->rc.ch2-CHANNELMIDDLE)*RC_LEFTRIGHT_SCALE;
//	}
	if (GetRcMode()==RC_KEY_STOP)
	{
	   SysMode = SYS_STOPSTATE;	
	}
	
}

/**
  * @brief 遥控器解析函数
  * @param pData	完整一帧接收机发来的数据
  * @param _len		帧总长
  * @retval None
  */
void RcProtocolAnalysis(u8 *pData,int _len)
{
	if(pData == 0)
    {
        return;
    }
    
    RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; 
    RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
    RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |
                         ((int16_t)pData[4] << 10)) & 0x07FF;
    RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
    
    RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
    RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);

    RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
    RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
    RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);    

    RC_CtrlData.mouse.press_l = pData[12];
    RC_CtrlData.mouse.press_r = pData[13];
 
    RC_CtrlData.key.v = ((int16_t)pData[15] << 8)|((int16_t)pData[14]);// | ((int16_t)pData[15] << 8);
		RcDataAnalysis(&RC_CtrlData);
}

/**
  * @brief 裁判系统解析
  * @param _item	完整一帧裁判系统的数据
	* @param _len		帧总长
  * @retval None
  */
void RefereeProtocolAnalysis(u8 *_item,int _len)
{


}

/**
  * @brief CAN数据解析
  * @param None
  * @retval None
  */
void CanProtocolAnalysis(CanRxMsg * msg)
{
  switch(msg->StdId)
		{
				case CAN_BUS2_MOTOR1_FEEDBACK_MSG_ID:
				{
					FeedDog(DEVICE_INDEX_MOTOR1);
					if(((can_encoder_flag>>DEVICE_INDEX_MOTOR1) & 0x0001) == 0)//对应位为0代表未获取编码器初始值
					{
				   GetEncoderBias(&CM1Encoder ,msg);
					 can_encoder_flag |= (1<<DEVICE_INDEX_MOTOR1);//获取初始值后将对应位置1
					}
					else
					EncoderProcess(&CM1Encoder ,msg);                      
				}break;
				case CAN_BUS2_MOTOR2_FEEDBACK_MSG_ID:
				{
					FeedDog(DEVICE_INDEX_MOTOR2);
					if(((can_encoder_flag>>DEVICE_INDEX_MOTOR2) & 0x0001) == 0)
					{
				   GetEncoderBias(&CM2Encoder ,msg);
					 can_encoder_flag |= (1<<DEVICE_INDEX_MOTOR2);
					}
					else
					EncoderProcess(&CM2Encoder ,msg); 
				}break;
				case CAN_BUS2_MOTOR3_FEEDBACK_MSG_ID:
				{
					FeedDog(DEVICE_INDEX_MOTOR3);
					if(((can_encoder_flag>>DEVICE_INDEX_MOTOR3) & 0x0001) == 0)
					{
				   GetEncoderBias(&CM3Encoder ,msg);
					 can_encoder_flag |= (1<<DEVICE_INDEX_MOTOR3);
					}
					else
					EncoderProcess(&CM3Encoder ,msg);   
				}break;
				case CAN_BUS2_MOTOR4_FEEDBACK_MSG_ID:
				{
					FeedDog(DEVICE_INDEX_MOTOR4);
					if(((can_encoder_flag>>DEVICE_INDEX_MOTOR4) & 0x0001) == 0)
					{
				   GetEncoderBias(&CM4Encoder ,msg);
					 can_encoder_flag |= (1<<DEVICE_INDEX_MOTOR4);
					}
					else
					EncoderProcess(&CM4Encoder ,msg); 
				}break;
				case CAN_BUS2_MOTOR7_FEEDBACK_MSG_ID:
				{
					FeedDog(DEVICE_INDEX_DIALING);
					if(((can_encoder_flag>>DEVICE_INDEX_DIALING) & 0x0001) == 0)
					{
				   GetEncoderBias(&DialingEncoder ,msg);
					 can_encoder_flag |= (1<<DEVICE_INDEX_DIALING);
					}
					else
					EncoderProcess(&DialingEncoder ,msg); 
				}break;
				case CAN_BUS2_MOTOR5_FEEDBACK_MSG_ID:
				{
					FeedDog(DEVICE_INDEX_MOTOR5);
					if((CALIFLAG & GIMBALYAWCALING) == GIMBALYAWCALING)
					{
						if(yaw_cali_count < caliMaxTime)
						{
							yaw_cali_sum += ((msg->Data[0]<<8)|msg->Data[1]);
							yaw_cali_count++;
						}
						else
						{
							AllDataUnion.AllData.GimbalYawOffset = yaw_cali_sum/caliMaxTime;
							yaw_cali_count = 0;
							CALIFLAG &=~ GIMBALYAWCALING;
						}
					}
					else if(((can_encoder_flag>>DEVICE_INDEX_MOTOR5) & 0x0001) == 0)
					{
				   GMYawEncoder.ecd_bias =AllDataUnion.AllData.GimbalYawOffset;
					 can_encoder_flag |= (1<<DEVICE_INDEX_MOTOR5);
					}
					else
					EncoderProcess(&GMYawEncoder,msg); 
						// 比较保存编码器的值和偏差值，如果编码器的值和初始偏差之间差距超过阈值，将偏差值做处理，防止出现云台反方向运动
					if(GetWSCurrent() == SYS_PREPARESTATE)   //准备阶段要求二者之间的差值一定不能大于阈值，否则肯定是出现了临界切换
					 {
							 if((GMYawEncoder.ecd_bias - GMYawEncoder.ecd_value) <-4000)
							 {
								  GMYawEncoder.ecd_bias =AllDataUnion.AllData.GimbalYawOffset  + 8192;
							 }
							 else if((GMYawEncoder.ecd_bias - GMYawEncoder.ecd_value) > 4000)
							 {
								  GMYawEncoder.ecd_bias = AllDataUnion.AllData.GimbalYawOffset - 8192;
							 }
					 }
				}break;
				case CAN_BUS2_MOTOR6_FEEDBACK_MSG_ID:
				{
          FeedDog(DEVICE_INDEX_MOTOR6);
					if((CALIFLAG & GIMBALPITCHCALING) == GIMBALPITCHCALING)
					{
						if(pitch_cali_count < caliMaxTime)
						{
							pitch_cali_sum += ((msg->Data[0]<<8)|msg->Data[1]);
							pitch_cali_count++;
						}
						else
						{
							AllDataUnion.AllData.GimbalPitchOffset = pitch_cali_sum/caliMaxTime;
							pitch_cali_count = 0;
							CALIFLAG &=~ GIMBALPITCHCALING;
						}
					}
					else if(((can_encoder_flag>>DEVICE_INDEX_MOTOR6) & 0x0001) == 0)
					{
				   GMPitchEncoder.ecd_bias =AllDataUnion.AllData.GimbalPitchOffset;
					 can_encoder_flag |= (1<<DEVICE_INDEX_MOTOR6);
					}
					else
					EncoderProcess(&GMPitchEncoder,msg); 	
					if(GetWSCurrent() == SYS_PREPARESTATE)  
					 {
							 if((GMPitchEncoder.ecd_bias - GMPitchEncoder.ecd_value) <-4000)
							 {
								  GMPitchEncoder.ecd_bias =AllDataUnion.AllData.GimbalPitchOffset  + 8192;
							 }
							 else if((GMPitchEncoder.ecd_bias - GMPitchEncoder.ecd_value) > 4000)
							 {
								  GMPitchEncoder.ecd_bias = AllDataUnion.AllData.GimbalPitchOffset - 8192;
							 }
					 }
				}break;		
			}				
}
