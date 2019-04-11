#ifndef _CANTASK_H_
#define _CANTASK_H_

#include "main.h"
#define CAN_BUS2_MOTOR1_FEEDBACK_MSG_ID           0x201
#define CAN_BUS2_MOTOR2_FEEDBACK_MSG_ID           0x202 
#define CAN_BUS2_MOTOR3_FEEDBACK_MSG_ID           0x203
#define CAN_BUS2_MOTOR4_FEEDBACK_MSG_ID           0x204
#define CAN_BUS2_MOTOR5_FEEDBACK_MSG_ID           0x205
#define CAN_BUS2_MOTOR6_FEEDBACK_MSG_ID           0x206
#define CAN_BUS2_MOTOR7_FEEDBACK_MSG_ID           0x207

#define RATE_BUF_SIZE 6

void EncoderProcess(volatile Encoder *v, CanRxMsg * msg);
void GetEncoderBias(volatile Encoder *v, CanRxMsg * msg);
void ChassisSpeedSet(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);
void GimbalCurrentSet(CAN_TypeDef *CANx, int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq, int16_t dialing_iq);
void mecanum_calc(float vx, float vy, float vw, int16_t speed[]);

#endif
