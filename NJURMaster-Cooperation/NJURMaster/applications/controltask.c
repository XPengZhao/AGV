#include "main.h"


//values for PID Calculate.
float GimbalPitchPosRef=0.0f;
float GimbalPitchGyrRef=0.0f;
float GimbalYawPosRef=0.0f;
float GimbalYawGyrRef=0.0f;
float ChassisGoToward=0.0f;
float ChassisGoLeftRight=0.0f;
float ChassisMotorSpeed1,ChassisMotorSpeed2,ChassisMotorSpeed3,ChassisMotorSpeed4;

float ChassisRotateOut=0.0f;
float CMOutput1,CMOutput2,CMOutput3,CMOutput4;
float GMYawOutput,GMPitchOutput;
unsigned int test_fire_speed = 1500;
int16_t exp_speed[4];
/**
  * @brief 底盘控制程序
  * @param _T程序调用周期(s)
  * @retval None
  * @details 	根据当前的期望底盘移动速度和当前底盘功率
	*						对四个底盘电机的电流进行控制
  */
void ChassisControl(float _T)
{	

	if (SysMode!=SYS_CALISTATE&&SysMode!=SYS_STOPSTATE&&SysMode!=SYS_PREPARESTATE)
	{
		switch(ControlMode)
		{
			case MC_NORMAL:
				ChassisRotateOut=PID_calculate( _T,            //周期（单位：秒）
																				0,				//前馈值
																				0,				//期望值（设定值）
																				GMYawEncoder.ecd_angle,			//反馈值（）
																				&Chassis_Rot_PID_arg, //PID参数结构体
																				&Chassis_Rot_PID_val,	//PID数据结构体
																				0.2		//integration limit，积分限幅
																				 );
				break;
			
			case MC_MODE1:
				
				break;
			
			case MC_MODE2:
				
				break;
			
			default:
				break;
		}
	}
	else
	{
		ChassisRotateOut=0.0f;
		ChassisGoToward=0.0f;
		ChassisGoLeftRight=0.0f;	
	}
	mecanum_calc(ChassisGoToward, -ChassisGoLeftRight, ChassisRotateOut, exp_speed);
//	ChassisMotorSpeed1=ChassisGoToward*0.075f-ChassisGoLeftRight*0.075f+ChassisRotateOut;
//	ChassisMotorSpeed2=ChassisGoToward*0.075f+ChassisGoLeftRight*0.075f+ChassisRotateOut;
//	ChassisMotorSpeed3=-ChassisGoToward*0.075f+ChassisGoLeftRight*0.075f+ChassisRotateOut;
//	ChassisMotorSpeed4=-ChassisGoToward*0.075f-ChassisGoLeftRight*0.075f+ChassisRotateOut;
	CMOutput1=PID_calculate( 			_T,            //周期（单位：秒）
																0,				//前馈值
																exp_speed[0],				//期望值（设定值）
																CM1Encoder.filter_rate,			//反馈值（）
																&Chassis_Vec_PID_arg, //PID参数结构体
																&Chassis_Vec_PID_val1,	//PID数据结构体
															 	0.2		//integration limit，积分限幅
																 );
	CMOutput2=PID_calculate( 			_T,            //周期（单位：秒）
																0,				//前馈值
																exp_speed[1],				//期望值（设定值）
																CM2Encoder.filter_rate,			//反馈值（）
																&Chassis_Vec_PID_arg, //PID参数结构体
																&Chassis_Vec_PID_val2,	//PID数据结构体
																0.2		//integration limit，积分限幅
																 );
	CMOutput3=PID_calculate( 			_T,            //周期（单位：秒）
																0,				//前馈值
																exp_speed[2],				//期望值（设定值）
																CM3Encoder.filter_rate,			//反馈值（）
																&Chassis_Vec_PID_arg, //PID参数结构体
																&Chassis_Vec_PID_val3,	//PID数据结构体
																0.2		//integration limit，积分限幅
																 );
	CMOutput4=PID_calculate( 			_T,            //周期（单位：秒）
																0,				//前馈值
																exp_speed[3],				//期望值（设定值）
																CM4Encoder.filter_rate,			//反馈值（）
																&Chassis_Vec_PID_arg, //PID参数结构体
																&Chassis_Vec_PID_val4,	//PID数据结构体
																0.2		//integration limit，积分限幅
																 );

	if (SysMode!=SYS_CALISTATE&&SysMode!=SYS_STOPSTATE)
	{
		ChassisSpeedSet(CAN1,CMOutput1,CMOutput2,CMOutput3,CMOutput4);
		//ChassisSpeedSet(CAN1,0,0,0,0);
	}
	else
	{
		ChassisSpeedSet(CAN1,0,0,0,0);
	}
}
	float yaw_speed=0,pitch_speed=0;
/**
  * @brief 云台控制程序
  * @param _T程序调用周期
  * @retval None
  * @details 根据当前云台的期望角度来控制两个云台电机的输出电流
  */
void GimbalControl(float _T)
{

   if (SysMode!=SYS_CALISTATE&&SysMode!=SYS_STOPSTATE)
	 {
		 if (SysMode==SYS_NORMALSTATE)
		 {
				if (ControlMode==MC_NORMAL)
				{
				 yaw_speed = PID_calculate( 			_T,            //周期（单位：秒）
																		0,				//前馈值
																		GimbalYawPosRef,				//期望值（设定值）
																		-Yaw,			//反馈值（）
																		&GimbalYaw_Pos_PID_arg, //PID参数结构体
																		&GimbalYaw_Pos_PID_val,	//PID数据结构体
																		0.2		//integration limit，积分限幅
																		 );
				 GMYawOutput = PID_calculate( 			_T,            //周期（单位：秒）
																		0,				//前馈值
																		yaw_speed,				//期望值（设定值）
																		-MPU6500_Gyro.z,			//反馈值（）
																		&GimbalYaw_Vec_PID_arg, //PID参数结构体
																		&GimbalYaw_Vec_PID_val,	//PID数据结构体
																		0.2		//integration limit，积分限幅
																		 );
				 pitch_speed = PID_calculate( 			_T,            //周期（单位：秒）
																		0,				//前馈值
																		GimbalPitchPosRef,				//期望值（设定值）
																		GMPitchEncoder.ecd_angle,			//反馈值（）
																		&GimbalPitch_Pos_PID_arg, //PID参数结构体
																		&GimbalPitch_Pos_PID_val,	//PID数据结构体
																		0.2		//integration limit，积分限幅
																		 );
				 GMPitchOutput = PID_calculate( 			_T,            //周期（单位：秒）
																		0,				//前馈值
																		pitch_speed,				//期望值（设定值）
																		MPU6500_Gyro.x,			//反馈值（）
																		&GimbalPitch_Vec_PID_arg, //PID参数结构体
																		&GimbalPitch_Vec_PID_val,	//PID数据结构体
																		0.2		//integration limit，积分限幅
																		 );
																		 
				 GimbalCurrentSet(CAN1,GMYawOutput,GMPitchOutput,0);
				}
				else if (ControlMode==MC_MODE1)
				{
				
				
				}
				else if (ControlMode==MC_MODE2)
				{
				
				
				}
				else
				{
				
				}
		 }
		 else if (SysMode==SYS_PREPARESTATE)
		 {
			 yaw_speed = PID_calculate( 			_T,            //周期（单位：秒）
																	0,				//前馈值
																	0,				//期望值（设定值）
																	GMYawEncoder.ecd_angle,			//反馈值（）
																	&GimbalYaw_Pos_PID_arg, //PID参数结构体
																	&GimbalYaw_Pos_PID_val,	//PID数据结构体
																	0.2		//integration limit，积分限幅
																	 );
			 GMYawOutput = PID_calculate( 			_T,            //周期（单位：秒）
																	0,				//前馈值
																	yaw_speed,				//期望值（设定值）
																	-MPU6500_Gyro.z,			//反馈值（）
																	&GimbalYaw_Vec_PID_arg, //PID参数结构体
																	&GimbalYaw_Vec_PID_val,	//PID数据结构体
																	0.2		//integration limit，积分限幅
																	);
			  pitch_speed = PID_calculate( 			_T,            //周期（单位：秒）
																	0,				//前馈值
																	0,				//期望值（设定值）
																	GMPitchEncoder.ecd_angle,			//反馈值（）
																	&GimbalPitch_Pos_PID_arg, //PID参数结构体
																	&GimbalPitch_Pos_PID_val,	//PID数据结构体
																	0.2		//integration limit，积分限幅
																	 );
			 GMPitchOutput = PID_calculate( 			_T,            //周期（单位：秒）
																	0,				//前馈值
																	pitch_speed,				//期望值（设定值）
																	MPU6500_Gyro.x,			//反馈值（）
																	&GimbalPitch_Vec_PID_arg, //PID参数结构体
																	&GimbalPitch_Vec_PID_val,	//PID数据结构体
																	0.2		//integration limit，积分限幅
																	 );
			GimbalYawPosRef=-Yaw;
			GimbalPitchPosRef=0.0f;
			 GimbalCurrentSet(CAN1,GMYawOutput,GMPitchOutput,0);
		 }
	 }
	 else
	 {
		 GimbalCurrentSet(CAN1,0,0,0);
	 }
}

/**
  * @brief 发射机构控制程序
  * @param _T程序调用周期(s)
  * @retval None
  * @details 根据给的遥控器或者其他信号控制摩擦轮和拨单电机的转速
  */
void FireControl(float _T)
{
	if (SysMode!=SYS_CALISTATE&&SysMode!=SYS_STOPSTATE)
	{
		 if(WHEEL_STATE == WHEEL_ON)
		 { 
			 SetFrictionWheelSpeed(1500);  
		 }
		 else
		 {
			 SetFrictionWheelSpeed(1000);
		 }
	 }
}

