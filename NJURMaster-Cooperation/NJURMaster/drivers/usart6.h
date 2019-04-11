#ifndef _USART6_H_
#define _USART6_H_
#include "stm32f4xx.h"

#define  BSP_USART6_DMA_RX_BUF_LEN                50u
#define  RS_FRAME_LENGTH                          44u

void Usart6_Init(u32 br_num);
void Usart6_Send(unsigned char *DataToSend ,u8 data_num);


#endif
