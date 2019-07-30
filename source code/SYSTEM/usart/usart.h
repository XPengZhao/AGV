#ifndef __USART_H
#define __USART_H
#include "sys.h"
#include "stdio.h"	
void usart1_send(u8 data);
void uart_init(u32 pclk2,u32 bound);

void usart2_send(u8 data);
void uart2_init(u32 pclk2,u32 bound);
u8 click_RC (void);
void USART_TX(void);
#endif	   












