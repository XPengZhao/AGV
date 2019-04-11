#ifndef _MODE_DUTY_H_
#define _MODE_DUTY_H_
#include "stm32f4xx.h"
enum
{
	SYS_STOPSTATE=0,
	SYS_PREPARESTATE,
	SYS_NORMALSTATE,
	SYS_CALISTATE
};
enum
{
	MC_NORMAL=0,			//正常情况，云台先动，底盘随动
	MC_MODE1,					//云台受控制，底盘不受控制，并周期转动
	MC_MODE2,					//底盘前后左右可控，云台受控
	MC_MODE3					//保留

};
#define SYS_PREPARETIME 5000
extern u8 SysMode,ControlMode;
void WorkStateFSM(u32 sys);
u8 GetWSCurrent(void);
#endif
