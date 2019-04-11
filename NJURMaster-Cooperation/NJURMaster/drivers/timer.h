#ifndef _TIMER_H_
#define _TIMER_H_
#include "stm32f4xx.h"
#define Get_Time_Micros() (TIM5->CNT)
#define INERLOOPLENGTH 11

#define Task_1ms_Time 0
#define Task_2ms_Time 1
#define Task_5ms_Time 2
#define Task_10ms_Time 3
#define Task_20ms_Time 4
#define Task_50ms_Time 5
#define DutyLoop_Time 6
#define Pitch_Time 7
#define Roll_Time 8
#define Reset_Time 9
#define CalcV_Time 10

#define delay_us(x) (Delay_us(x))
#define delay_ms(x) (Delay_ms(x))
void TIM5_Configuration(void);
//uint32_t Get_Time_Micros(void);
void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);
void TIM6_Configuration(void);
void TIM6_Start(void);
uint32_t GetInnerLoop(int loop);
void InnerLoopInit(void);
#endif
