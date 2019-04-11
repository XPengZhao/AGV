#ifndef _CONTROLTASK_H_
#define _CONTROLTASK_H_

void ChassisControl(float _T);
void GimbalControl(float _T);
void FireControl(float _T);

extern unsigned int test_fire_speed;


extern float GimbalPitchPosRef,GimbalPitchGyrRef,GimbalYawPosRef,
						GimbalYawGyrRef,ChassisGoToward,ChassisGoLeftRight;
extern float ChassisMotorSpeed1,ChassisMotorSpeed2,ChassisMotorSpeed3,ChassisMotorSpeed4;

extern float ChassisRotateOut;
extern float CMOutput1,CMOutput2,CMOutput3,CMOutput4;

#endif
