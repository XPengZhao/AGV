#ifndef __USRATX_H
#define __USRATX_H 
#include "sys.h"	  	
void usart3_send(u8 data);
void uart3_init(u32 pclk2,u32 bound);
int USART3_IRQHandler(void);

#endif

