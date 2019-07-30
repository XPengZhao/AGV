#ifndef __TIMER_H
#define __TIMER_H
#include <sys.h>	 
void TIM3_Cap_Init(u16 arr,u16 psc);
void Read_Distane(void);
void TIM3_IRQHandler(void);
#endif
