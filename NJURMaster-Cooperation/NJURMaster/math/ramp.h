#ifndef __RAMP_H_
#define __RAMP_H_
#include "stm32f4xx.h"
typedef __packed struct
{
	float Num_Max;
	float	Num_Current;
}_ramp_st;
void RampReset(_ramp_st * ramp);
float RampCalculate(_ramp_st * ramp);
extern _ramp_st RcKeyTowardRamp,RcKeyLeftRightRamp,RcKeyRotateRamp;
#endif
