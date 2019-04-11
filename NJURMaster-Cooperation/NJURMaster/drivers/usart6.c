#include "main.h"
#include "usart6.h"

#define USART6DMA_ENABLE 1;

/*******串口1变量********************/
static uint8_t _USART6_DMA_RX_BUF[2][BSP_USART6_DMA_RX_BUF_LEN];//DMA的两个缓冲区，缓冲区稍大于遥控器数据帧
static uint8_t _USART6_DMA_TX_BUF[256] = {0};
static uint8_t _USART6_TX_BUF[256] = {0};

/**
  * @brief 串口6初始化 
  * @param BaudRate
  * @retval None
  * @details DMA发送，RXNE接收中断
  */
void Usart6_Init(u32 br_num)
{ 
    //USART6->RX: PG9 
    //USART6->TX: PG14
	  GPIO_InitTypeDef  gpio;
    USART_InitTypeDef usart6;
    NVIC_InitTypeDef nvic;
    DMA_InitTypeDef dma;
    
    //initialize clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART6);
    GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_USART6);
   //initialize gpio
    gpio.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOG,&gpio);

    //initialize usart6
    usart6.USART_BaudRate = br_num;
    usart6.USART_WordLength = USART_WordLength_8b;
    usart6.USART_StopBits = USART_StopBits_1;
    usart6.USART_Parity = USART_Parity_No;
    usart6.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
    usart6.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART6,&usart6);

    USART_ITConfig(USART6,USART_IT_IDLE,ENABLE);      //usart rx idle interrupt enabled
    USART_Cmd(USART6,ENABLE);

    //initialize interrupt
    nvic.NVIC_IRQChannel = USART6_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&nvic);
#ifdef USART6DMA_ENABLE
    //initialize dma to receive data 
    DMA_DeInit(DMA2_Stream1);
    DMA_StructInit(&dma);
    dma.DMA_Channel = DMA_Channel_5;
    dma.DMA_PeripheralBaseAddr = (uint32_t)(&USART6->DR);
    dma.DMA_Memory0BaseAddr = (uint32_t)&_USART6_DMA_RX_BUF[0][0];
    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize = sizeof(_USART6_DMA_RX_BUF)/2;            // sizeof(array[r][c]) = sizeof(typevalue)*r*c
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_Medium;
    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream1, &dma);
    
    USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
    //enable double buffer mode
    DMA_DoubleBufferModeConfig(DMA2_Stream1, (uint32_t)&_USART6_DMA_RX_BUF[1][0], DMA_Memory_0);   //first used memory configuration
    DMA_DoubleBufferModeCmd(DMA2_Stream1, ENABLE);
    //enable dma stream2
    DMA_Cmd(DMA2_Stream1, ENABLE);

    //initialize dma to send data
    DMA_DeInit(DMA2_Stream6);
    dma.DMA_Channel = DMA_Channel_5;
    dma.DMA_PeripheralBaseAddr = (uint32_t)(&USART6->DR);
    dma.DMA_Memory0BaseAddr = (uint32_t)&_USART6_DMA_TX_BUF[0];
    dma.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    dma.DMA_BufferSize = sizeof(_USART6_DMA_TX_BUF);            
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Normal;
    dma.DMA_Priority = DMA_Priority_Medium;
    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream6, &dma);
    USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE);
    //enable dma stream6
    DMA_Cmd(DMA2_Stream6, ENABLE);
#endif
    return;
}

/**
  * @brief 串口6中断
  * @param None
  * @retval None
  * @details RXNE中断，由此进入Usart6通讯协议解析
  */
void USART6_IRQHandler(void)
{
//	u8 com_data;
	static uint32_t this_time_rx_len = 0;
//	if(USART6->SR & USART_SR_ORE)
//	{
//		com_data = USART6->DR;
//	}
	if(USART_GetITStatus(USART6,USART_IT_IDLE) != RESET)
	{
		//USART_ClearITPendingBit(USART6,USART_IT_IDLE);
		(void)USART6->SR;
		(void)USART6->DR;
    //Target is Memory0
    if(DMA_GetCurrentMemoryTarget(DMA2_Stream1) == 0)
    {
      DMA_Cmd(DMA2_Stream1,DISABLE);
      this_time_rx_len = BSP_USART6_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA2_Stream1);  //get the length of data that DMA has transferred
      DMA2_Stream1->NDTR = (uint16_t)BSP_USART6_DMA_RX_BUF_LEN;                             //relocate the dma memory pointer to the beginning position
      DMA2_Stream1->CR |= (uint32_t)(DMA_SxCR_CT);                                          //we select the memory1 to receive data
      DMA_Cmd(DMA2_Stream1,ENABLE);
      if(this_time_rx_len <= RS_FRAME_LENGTH)       //
      {
		      Usart6_DataPrepare(_USART6_DMA_RX_BUF[0]);
      }
    }
    //Target is Memory1
    else
    {
      DMA_Cmd(DMA2_Stream1,DISABLE);
      this_time_rx_len = BSP_USART6_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA2_Stream1);  //get the length of data that DMA has transferred
      DMA2_Stream1->NDTR = (uint16_t)BSP_USART6_DMA_RX_BUF_LEN;                             //relocate the dma memory pointer to the beginning position
      DMA2_Stream1->CR &= ~(uint32_t)(DMA_SxCR_CT);                                         //we select the memory1 to receive data
      DMA_Cmd(DMA2_Stream1,ENABLE);
      if(this_time_rx_len <= RS_FRAME_LENGTH)       //
      {  
		      Usart6_DataPrepare(_USART6_DMA_RX_BUF[1]);
      }
    }
	}
}

/**
  * @brief 串口6的DMA发送函数，发送一组数据
  * @param DataToSend 要发送数据的数组的指针
	* @param data_num 要发送的数据的个数
  * @retval None
  */
void Usart6_Send(unsigned char *DataToSend ,u8 data_num)
{
	u8 i;
	static u16 n_remain=0;
	static u8 last_len=0;
    static u8 rear = 0;
	
	DMA_Cmd(DMA2_Stream6, DISABLE);
	DMA_ClearFlag(DMA2_Stream6,DMA_FLAG_TCIF6);
	n_remain = DMA_GetCurrDataCounter(DMA2_Stream6);

  //write the remain data to dma buffer
	for(i=0;i<(u8)n_remain;i++)
	{
		_USART6_DMA_TX_BUF[i]=_USART6_TX_BUF[((u8)(last_len-n_remain+i))];
	}

  //write the new data to dma buffer
	for (i = n_remain;i<(u8)(n_remain+data_num);i++)
	{
		_USART6_DMA_TX_BUF[i]=*(DataToSend+i-n_remain);
	}

  //copy tha new data to data buffer
  for(i = 0;i<(u8)(data_num);i++)
	{
		_USART6_TX_BUF[rear++] = *(DataToSend+i);
	}
  //assign current length of data to last length
	last_len = data_num;
	while (DMA_GetCmdStatus(DMA2_Stream6) != DISABLE){}	
	DMA2_Stream6->NDTR = (uint16_t)(n_remain+data_num);        
	DMA_Cmd(DMA2_Stream6, ENABLE);        
}


