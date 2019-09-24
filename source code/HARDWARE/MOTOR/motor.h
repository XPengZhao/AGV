#ifndef __MOTOR_H
#define __MOTOR_H
#include <sys.h>	 

#define PWMD   TIM8->CCR4  
#define PWMC   TIM8->CCR3  
#define PWMB   TIM8->CCR2 
#define PWMA   TIM8->CCR1 
#define IND   PBout(1)  
#define INC   PCout(5)  
#define INB   PBout(0)  
#define INA   PCout(4)  

void MiniBalance_PWM_Init(u16 arr,u16 psc);
void MiniBalance_Motor_Init(void);
#endif
