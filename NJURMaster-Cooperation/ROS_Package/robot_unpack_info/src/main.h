#ifndef _MAIN_H_
#define _MAIN_H_

typedef  struct
{
	float Pitch;
	float Roll;
	float Yaw;
    float Gimbal_Pitch;
    float Gimbal_Yaw;
	uint32_t 	state;
} __attribute__ ((packed))  PC_Send_IMU_t;

typedef  struct
{
	int16_t motor1;
	int16_t motor2;
	int16_t motor3;
	int16_t motor4;
	int16_t motor5;
	int16_t motor6;
	int16_t motor7;
	int16_t motor8;
} __attribute__ ((packed)) PC_Send_Motor_t;

typedef   struct
{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int8_t s1;
	int8_t s2;
} __attribute__ ((packed))  Remote;
typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t last_press_l;
	uint8_t last_press_r;
	uint8_t press_l;
	uint8_t press_r;
} __attribute__ ((packed))  Mouse;	
typedef	 struct
{
	uint16_t v;
} __attribute__ ((packed)) Key;

typedef  struct
{
	Remote rc;
	Mouse mouse;
	Key key;
} __attribute__ ((packed))  RC_Ctrl_t;




#define SELF_CHECK_ITEM_NUM 12u

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
#define DEVICE_INDEX_MOTOR7                                                12u

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
#define LOST_ERROR_MOTOR7                                        (1<<DEVICE_INDEX_MOTOR7)

#endif
