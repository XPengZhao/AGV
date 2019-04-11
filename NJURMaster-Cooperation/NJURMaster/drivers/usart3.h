#ifndef _USART3_H
#define _USART3_H

#include "stm32f4xx.h"
void Usart3_Init(u32 br_num);
void Usart3_Send(unsigned char *DataToSend ,u8 data_num);
#endif

