#include "main.h"
/*******串口3变量*********************/
u8 Rx_3_Buf[256];	
u8 Tx3Buffer[256];
u8 Tx3Counter=0;
u8 count3=0; 
u8 Tx3DMABuffer[256]={0};
/***********************************/
/**
  * @brief 串口3初始化 
  * @param BaudRate
  * @retval None
  * @details	PD8	Tx
	*						PD9	Rx
	*						BaudRate	115200
	*						使能DMA发送，RXNE中断
  */
void Usart3_Init(u32 br_num)
{ 
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);
	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 

	USART_InitStructure.USART_BaudRate = br_num;     
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; 
	USART_InitStructure.USART_StopBits = USART_StopBits_1; 
	USART_InitStructure.USART_Parity = USART_Parity_No; 
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; 
	USART_ClockInitStruct.USART_Clock = USART_Clock_Disable; 
	USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge; 
	USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable;
	
	USART_Init(USART3, &USART_InitStructure);
	USART_ClockInit(USART3, &USART_ClockInitStruct);

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART3, ENABLE); 
	USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);

	DMA_DeInit(DMA1_Stream3);
	
	while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE){}//等待DMA可配置 
	
  /* 配置 DMA Stream */
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART3->DR;//DMA外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)Tx3DMABuffer;//DMA 存储器0地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//存储器到外设模式
  DMA_InitStructure.DMA_BufferSize = 0;//数据传输量 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 使用普通模式 
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//中等优先级
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
  DMA_Init(DMA1_Stream3, &DMA_InitStructure);//初始化DMA Stream
	
}

/**
  * @brief 串口3中断
  * @param None
  * @retval None
  * @details RXNE中断，由此进入Usart3通讯协议解析
  */
void USART3_IRQHandler(void)
{
	u8 com_data;
	
	if(USART3->SR & USART_SR_ORE)
	{
		com_data = USART3->DR;
	}
	if( USART_GetITStatus(USART3,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);

		com_data = USART3->DR;
		Usart3_DataPrepare(com_data);
	}
}

/**
  * @brief 串口3的DMA发送函数，发送一组数据
  * @param DataToSend 要发送数据的数组的指针
	* @param data_num 要发送的数据的个数
  * @retval None
  */
void Usart3_Send(unsigned char *DataToSend ,u8 data_num)
{
  u8 i;
	static uint16_t num=0;
	static u8 len=0;
	
	DMA_Cmd(DMA1_Stream3, DISABLE);
	DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);//清除DMA1_Steam3传输完成标志
	num = DMA_GetCurrDataCounter(DMA1_Stream3);
	for(i=0;i<data_num;i++)
	{
		Tx3Buffer[count3++] = *(DataToSend+i);
	}
	for (i=0;i<(u8)num;i++)
	{
		Tx3DMABuffer[i]=Tx3Buffer[((u8)(len-num+i))];
	}
	for (;i<(u8)(num+data_num);i++)
	{
		Tx3DMABuffer[i]=*(DataToSend+i-num);
	}
	len=count3;
	while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE){}	//确保DMA可以被设置  
	DMA1_Stream3->NDTR = (uint16_t)(num+data_num);          //数据传输量  
	DMA_Cmd(DMA1_Stream3, ENABLE);       

}


