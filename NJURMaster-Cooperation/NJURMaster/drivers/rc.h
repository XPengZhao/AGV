#ifndef _USART1_H
#define _USART1_H

#include "stm32f4xx.h"

#define USART1_DMA_RX_LEN 30u
#define RC_FRAME_LENGTH 18u


void Rc_Init(void);
void Usart1_Init(void);
u8 GetRcMode(void);

#endif


