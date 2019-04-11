#ifndef _APDRIVER_H_
#define _APDRIVER_H_
#include "stdint.h"
typedef struct
{
    float Chassis_speed_x;
    float Chassis_speed_y;
    float Chassis_speed_rotate;
    float Gimbal_delta_pitch;
    float Gimbal_delta_yaw;
	uint16_t ControlValid;
} __attribute__ ((packed)) Robot_Control_t;
#endif
