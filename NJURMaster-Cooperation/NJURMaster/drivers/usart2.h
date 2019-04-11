#ifndef _USART2_H
#define _USART2_H

#include "stm32f4xx.h"
void Usart2_Init(u32 br_num);
void Usart2_Send(unsigned char *DataToSend ,u8 data_num);
#endif

