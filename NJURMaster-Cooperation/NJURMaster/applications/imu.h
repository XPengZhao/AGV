#ifndef _IMU_H_
#define _IMU_H_
#include "mymath.h"
#include "stm32f4xx.h"
typedef struct 
{
	xyz_f_t err;
	xyz_f_t err_tmp;
	xyz_f_t err_lpf;
	xyz_f_t err_Int;
	xyz_f_t g;
	
}ref_t;
void IMUupdate(float half_T,float gx, float gy, float gz, float ax, float ay, float az,float *rol,float *pit,float *yaw);
void AHRSInit(float ax,float ay,float az,float mx,float my,float mz);
void AHRSUpdate(float dt,float ax,float ay,float az,float gx,float gy,float gz,float mx,float my,float mz,float *pit,float *rol,float *yaw,float twoKp,float twoKi);
float invSqrt(float number) ;
extern float Roll,Pitch,Yaw;
extern float ref_q[4];
#define ANGLE_TO_RADIAN 0.01745329f

#endif
