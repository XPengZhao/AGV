#ifndef _SYS_CONFIG_H_
#define _SYS_CONFIG_H_



//chassis  definiiton




/************************ chassis parameter ****************************/
/* the radius of wheel(mm) */
#define RADIUS     76
/* the perimeter of wheel(mm) */
#define PERIMETER  478

/* wheel track distance(mm) */
#define WHEELTRACK 403
/* wheelbase distance(mm) */
#define WHEELBASE  385

/* gimbal is relative to chassis center x axis offset(mm) */
#define GIMBAL_X_OFFSET 150
/* gimbal is relative to chassis center y axis offset(mm) */
#define GIMBAL_Y_OFFSET 0

/* chassis motor use 3508 default */
/* define CHASSIS_EC60 to use EC60 */
#define CHASSIS_3510

#ifdef CHASSIS_EC60
  /* chassis motor use EC60 */
  /* the deceleration ratio of chassis motor */
  #define CHASSIS_DECELE_RATIO (1.0f)
  /* single 3508 motor maximum speed, unit is rpm */
  #define MAX_WHEEL_RPM        400   //440rpm = 3500mm/s
  /* chassis maximum translation speed, unit is mm/s */
  #define MAX_CHASSIS_VX_SPEED 3300  //415rpm
  #define MAX_CHASSIS_VY_SPEED 3300
  /* chassis maximum rotation speed, unit is degree/s */
  #define MAX_CHASSIS_VR_SPEED 300
#else
  /* chassis motor use 3508 */
  /* the deceleration ratio of chassis motor */
  #define CHASSIS_DECELE_RATIO (1.0f/19.0f)
  /* single 3508 motor maximum speed, unit is rpm */
  #define MAX_WHEEL_RPM        8500  //8347rpm = 3500mm/s
  /* chassis maximum translation speed, unit is mm/s */
  #define MAX_CHASSIS_VX_SPEED 3300  //8000rpm
  #define MAX_CHASSIS_VY_SPEED 3300
  /* chassis maximum rotation speed, unit is degree/s */
  #define MAX_CHASSIS_VR_SPEED 300   //5000rpm
#endif

/************************** gimbal parameter *****************************/
/* the ratio of motor encoder value translate to degree */
#define ENCODER_ANGLE_RATIO    (8192.0f/360.0f)
/* the deceleration ratio of pitch axis motor */
#define PIT_DECELE_RATIO       1.0f
/* the deceleration ratio of yaw axis motor */
#define YAW_DECELE_RATIO       1.0f    //(5.0f/8.0f)
/* the positive direction of pitch axis motor */
#define PIT_MOTO_POSITIVE_DIR  1.0f
/* the positive direction of yaw axis motor */
#define YAW_MOTO_POSITIVE_DIR  1.0f
/* the positive direction of tirgger motor */
#define TRI_MOTO_POSITIVE_DIR  1.0f


/************************rc parameter*************************/

#define PITCH_MIN (-8.0f)
#define PITCH_MAX (27.0f)
#define YAW_MAX   (35.0f)
#define CHANNELMIDDLE	(1024)
#define RC_TOWARD_SCALE (1.0f)
#define RC_LEFTRIGHT_SCALE (1.0f)

#define RC_PITCHSCALE (0.02f)
#define RC_YAWSCALE (0.003f)
#define MAXTOWARDSPEED (660*RC_TOWARD_SCALE)
#define MAXLEFTRIGHTSPEED (660*RC_LEFTRIGHT_SCALE)
#define MOUSERESPONCERATE (0.1f) // Û±Í¡È√Ù∂»




/******************************PID default parameter*****************/
////////////////////////////		kp		ki		kd		k_pre_d		inc_hz		k_inc_d_norm		k_ff
#define CHASSIS_Rot_PID_OFF 	{	1.8f,	0.0f,	0.0f,	0.0f,				0.0f,		0.0f,						0.0f	}
#define CHASSIS_Vec_PID_OFF 	{	2.0f,	0.0f,	0.0f,	0.0f,				0.0f,		0.0f,						0.0f	}
#define GIMBALP_Pos_PID_OFF 	{	6.0f,	0.0f,	0.0f,	0.0f,				0.0f,		0.0f,						0.0f	}
#define GIMBALP_Vec_PID_OFF 	{	20.0f,	0.0f,	0.0f,	0.0f,				0.0f,		0.0f,						0.0f	}
#define GIMBALY_Pos_PID_OFF 	{	20.0f,	1.0f,	0.0f,	0.0f,				0.0f,		0.0f,						0.0f	}
#define GIMBALY_Vec_PID_OFF 	{	20.0f,	0.0f,	0.0f,	0.0f,				0.0f,		0.0f,						0.0f	}
#define SLIBLIN_Pos_PID_OFF 	{	1.0f,	0.0f,	0.0f,	0.0f,				0.0f,		0.0f,						0.0f	}
#define SLIBLIN_Vec_PID_OFF 	{	1.0f,	0.0f,	0.0f,	0.0f,				0.0f,		0.0f,						0.0f	}

#define RADIAN_PER_RAD (57.29578f)
#endif


