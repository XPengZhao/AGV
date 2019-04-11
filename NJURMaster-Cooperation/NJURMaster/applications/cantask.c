#include "main.h"
#include "string.h"

/**
  * @brief mecanum chassis velocity decomposition
  * @param input : ?=+vx(mm/s)  ?=+vy(mm/s)  ccw=+vw(deg/s)
  *        output: every wheel speed(rpm)
  * @note  1=FR 2=FL 3=BL 4=BR
  */
int rotation_center_gimbal = 0;
void mecanum_calc(float vx, float vy, float vw, int16_t speed[])
{
  static float rotate_ratio_fr;
  static float rotate_ratio_fl;
  static float rotate_ratio_bl;
  static float rotate_ratio_br;
  static float wheel_rpm_ratio;
  
  if (rotation_center_gimbal)
  {
//    rotate_ratio_fr = ((glb_struct.wheel_base+glb_struct.wheel_track)/2.0f \
//                        - glb_struct.gimbal_x_offset + glb_struct.gimbal_y_offset)/RADIAN_PER_RAD;
//    rotate_ratio_fl = ((glb_struct.wheel_base+glb_struct.wheel_track)/2.0f \
//                        - glb_struct.gimbal_x_offset - glb_struct.gimbal_y_offset)/RADIAN_PER_RAD;
//    rotate_ratio_bl = ((glb_struct.wheel_base+glb_struct.wheel_track)/2.0f \
//                        + glb_struct.gimbal_x_offset - glb_struct.gimbal_y_offset)/RADIAN_PER_RAD;
//    rotate_ratio_br = ((glb_struct.wheel_base+glb_struct.wheel_track)/2.0f \
//                        + glb_struct.gimbal_x_offset + glb_struct.gimbal_y_offset)/RADIAN_PER_RAD;
  }
  else
  {
    rotate_ratio_fr = ((WHEELBASE+WHEELTRACK)/2.0f)/RADIAN_PER_RAD;
    rotate_ratio_fl = rotate_ratio_fr;
    rotate_ratio_bl = rotate_ratio_fr;
    rotate_ratio_br = rotate_ratio_fr;
  }
  wheel_rpm_ratio = 60.0f/(PERIMETER*CHASSIS_DECELE_RATIO);
  
  
  vx = LIMIT(vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);  //mm/s
  vy = LIMIT(vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);  //mm/s
  vw = LIMIT(vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED);  //deg/s
  
  int16_t wheel_rpm[4];
  float   max = 0;
  
  wheel_rpm[0] = (-vx - vy + vw * rotate_ratio_fr) * wheel_rpm_ratio;
  wheel_rpm[1] = ( vx - vy + vw * rotate_ratio_fl) * wheel_rpm_ratio;
  wheel_rpm[2] = ( vx + vy + vw * rotate_ratio_bl) * wheel_rpm_ratio;
  wheel_rpm[3] = (-vx + vy + vw * rotate_ratio_br) * wheel_rpm_ratio;

  //find max item
  for (uint8_t i = 0; i < 4; i++)
  {
    if (my_abs(wheel_rpm[i]) > max)
      max = my_abs(wheel_rpm[i]);
  }
  //equal proportion
  if (max > MAX_WHEEL_RPM)
  {
    float rate = MAX_WHEEL_RPM / max;
    for (uint8_t i = 0; i < 4; i++)
      wheel_rpm[i] *= rate;
  }
  memcpy(speed, wheel_rpm, 4*sizeof(int16_t));
}


/**
  * @brief 根据编码器传回来的数据更新相应encoder结构体值
  * @param Encoder *v底盘和云台编码器
	* @param CanRxMsg * msg can数据帧
  * @retval None
  */
void EncoderProcess(volatile Encoder *v, CanRxMsg * msg)
{
	int i=0;
	int32_t temp_sum = 0;    
	v->last_raw_value = v->raw_value;
	v->raw_value = (msg->Data[0]<<8)|msg->Data[1];
	v->diff = v->raw_value - v->last_raw_value;
	if(v->diff < -3000)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if(v->diff>3000)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff- 8192;
	}		
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	//计算得到连续的编码器输出值
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	if(v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	//计算速度平均值
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}
	v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);					
}

/**
  * @brief 根据编码器传回来的值更新相应encoder结构体的编码器初值
  * @param Encoder *v底盘电机编码器
	* @param CanRxMsg * msg can数据帧
  * @retval None
  */
void GetEncoderBias(volatile Encoder *v, CanRxMsg * msg)
{
	v->ecd_bias = (msg->Data[0]<<8)|msg->Data[1];  //保存初始编码器值作为偏差  
	v->ecd_value = v->ecd_bias;
	v->last_raw_value = v->ecd_bias;
}

/**
  * @brief setting the acceleration of each motor
  * @param CANx CAN1 or CAN2
	* @param cm*_iq expecation acceleration of each motor
  * @retval None
  */
void ChassisSpeedSet(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
{
		CanTxMsg tx_message;
    tx_message.StdId = 0x200;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (uint8_t)(cm1_iq >> 8);
    tx_message.Data[1] = (uint8_t)cm1_iq;
    tx_message.Data[2] = (uint8_t)(cm2_iq >> 8);
    tx_message.Data[3] = (uint8_t)cm2_iq;
    tx_message.Data[4] = (uint8_t)(cm3_iq >> 8);
    tx_message.Data[5] = (uint8_t)cm3_iq;
    tx_message.Data[6] = (uint8_t)(cm4_iq >> 8);
    tx_message.Data[7] = (uint8_t)cm4_iq;
    CAN_Transmit(CANx,&tx_message);
}

/**
  * @brief Set the acceleration of Gimbal and slibling motors
  * @param CANx CAN1 or CAN2 it's depended on which CAN you choose 
  * @param gimbal_yaw_iq  the expected acceleration of yaw motor
  * @param gimbal_pitch_iq  the expected acceleration of pitch motor
  * @param gimbal_dialing_iq	the expected acceleration of dialing motor
  * @retval None
  */
void GimbalCurrentSet(CAN_TypeDef *CANx, int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq, int16_t dialing_iq)
{
    CanTxMsg tx_message;    
    tx_message.StdId = 0x1FF;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (unsigned char)(gimbal_yaw_iq >> 8);
    tx_message.Data[1] = (unsigned char)gimbal_yaw_iq;
    tx_message.Data[2] = (unsigned char)(gimbal_pitch_iq >> 8);
    tx_message.Data[3] = (unsigned char)gimbal_pitch_iq;
    tx_message.Data[4] = (unsigned char)(dialing_iq >> 8);
    tx_message.Data[5] = (unsigned char)dialing_iq;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);
}

