#ifndef _PARAMETERS_H_
#define _PARAMETERS_H_
#include "stm32f4xx.h"
#include "mymath.h"
#include "pid.h"
#define PIDGROUPLEN 8
#define Chassis_Rot_PID_arg ( *PID_arg )
#define Chassis_Vec_PID_arg ( *(PID_arg+1) )
#define GimbalPitch_Pos_PID_arg ( *(PID_arg+2) )
#define GimbalPitch_Vec_PID_arg ( *(PID_arg+3) )
#define GimbalYaw_Pos_PID_arg ( *(PID_arg+4) )
#define GimbalYaw_Vec_PID_arg ( *(PID_arg+5) )
#define Slibing_Pos_PID_arg ( *(PID_arg+6) )
#define Slibing_Vec_PID_arg ( *(PID_arg+7) )

#define Chassis_Rot_PID_val ( *PID_val )
#define Chassis_Vec_PID_val1 ( *(PID_val+1) )
#define Chassis_Vec_PID_val2 ( *(PID_val+2) )
#define Chassis_Vec_PID_val3 ( *(PID_val+3) )
#define Chassis_Vec_PID_val4 ( *(PID_val+4) )
#define GimbalPitch_Pos_PID_val ( *(PID_val+5) )
#define GimbalPitch_Vec_PID_val ( *(PID_val+6) )
#define GimbalYaw_Pos_PID_val ( *(PID_val+7) )
#define GimbalYaw_Vec_PID_val ( *(PID_val+8) )
#define Slibing_Pos_PID_val ( *(PID_val+9) )
#define Slibing_Vec_PID_val ( *(PID_val+10) )


void ParametersInit(void);
void SensorOffsetInit(void);
void ParametersSave(void);
void SensorsOffsetSave(void);
void IMU_GYRODataCali(void);
void IMU_ACCERDataCali(void);
void IMU_MAGDataCali(void);
void GimbalDataCali(void);

extern u8 CALIFLAG;
#define IMU_GYROCALING 0x01
#define IMU_ACCERCALING 0x02
#define IMU_MAGCALING 0x04

#define GIMBALPITCHCALING 0x08
#define GIMBALYAWCALING 0x10


typedef __packed struct 
{
	xyz_f_t GYRO_Offset;
	xyz_f_t ACCER_Offset;
	xyz_f_t MAG_Offset;
}IMUSensor_OffSet__;

typedef __packed struct
{
	u8 savedflag;
	IMUSensor_OffSet__ imu_offset;
	uint16_t GimbalPitchOffset;
	uint16_t GimbalYawOffset;
	_PID_arg_st PID_ARG[PIDGROUPLEN];
}AllDataOffset__;
typedef union
{
	u8 data[sizeof(AllDataOffset__)];
	AllDataOffset__ AllData;

}AllDataUnion__;
extern u8 ParaSavingFlag;
extern u8 CALIFLAG;
extern IMUSensor_OffSet__ IMUSensor_Offset;
extern AllDataUnion__ AllDataUnion; 
extern u8 ParamSavedFlag;
extern _PID_arg_st PID_arg[PIDGROUPLEN];
extern _PID_val_st PID_val[PIDGROUPLEN+3];
#endif
