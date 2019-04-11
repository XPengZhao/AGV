#ifndef _SELFCHECK_H_
#define _SELFCHECK_H_
#include "stm32f4xx.h"


#define SELF_CHECK_ITEM_NUM 13u

#define DEVICE_INDEX_RC                        0u   //green:green:green 1:1:1
#define DEVICE_INDEX_IMU                       1u    //red always on
#define DEVICE_INDEX_ZGYRO                     2u    //
#define DEVICE_INDEX_MOTOR1                    3u    //green:red:green 1 1 1 
#define DEVICE_INDEX_MOTOR2                    4u    //green:red:green 1 2 1 
#define DEVICE_INDEX_MOTOR3                    5u    //green:red:green 1 3 1 
#define DEVICE_INDEX_MOTOR4                    6u    //green:red:green 1 4 1 
#define DEVICE_INDEX_MOTOR5                    7u    //green:red:green 1 5 1 
#define DEVICE_INDEX_MOTOR6                    8u    //green:red:green 1 6 1 
#define DEVICE_INDEX_NOCALI            		 9u    //red:red:red 1:1:1
#define DEVICE_INDEX_TIMEOUT            		 10u    //red:red:red 1:1:1
#define DEVICE_INDEX_PC												11u
#define DEVICE_INDEX_DIALING									12u

#define LOST_ERROR_RC									(1<<DEVICE_INDEX_RC)		//rc lost 
#define LOST_ERROR_IMU									(1<<DEVICE_INDEX_IMU)		//mpu6050 error
#define LOST_ERROR_ZGYRO								(1<<DEVICE_INDEX_ZGYRO)		//can1 zyro error
#define LOST_ERROR_MOTOR1								(1<<DEVICE_INDEX_MOTOR1)		//motor1 error
#define LOST_ERROR_MOTOR2								(1<<DEVICE_INDEX_MOTOR2)		//
#define LOST_ERROR_MOTOR3								(1<<DEVICE_INDEX_MOTOR3)		//
#define LOST_ERROR_MOTOR4								(1<<DEVICE_INDEX_MOTOR4)		//
#define LOST_ERROR_MOTOR5								(1<<DEVICE_INDEX_MOTOR5)		//
#define LOST_ERROR_MOTOR6								(1<<DEVICE_INDEX_MOTOR6)		//
#define LOST_ERROR_NOCALI  						        (1<<DEVICE_INDEX_NOCALI)		//nocali error
#define LOST_ERROR_TIMEOUT 						        (1<<DEVICE_INDEX_TIMEOUT)		
#define LOST_ERROR_PC										(1<<DEVICE_INDEX_PC)	
#define LOST_ERROR_DIALING									(1<<DEVICE_INDEX_DIALING)	
void DogInit(void);
void FeedDog(u8 _dog_index);
void CheckDog(void);
u8 IsDeviceLost(int _dog_index);
extern u32 SelfCheckErrorFlag;



#endif
