#ifndef _LASER_H_
#define _LASER_H_
#define LASER_ON() GPIO_SetBits(GPIOG, GPIO_Pin_13)
#define LASER_OFF() GPIO_ResetBits(GPIOG, GPIO_Pin_13)
#include "stm32f4xx.h"

void Laser_Configuration(void);

#endif
