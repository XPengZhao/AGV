#include "main.h"

/*******串口2变量*********************/
u8 Rx_2_Buf[256];	
u8 Tx2Buffer[256];
u8 Tx2Counter=0;
u8 count2=0; 
u8 Tx2DMABuffer[256]={0};
/***********************************/

/**
  * @brief 串口2初始化
  * @param BaudRate
  * @retval None
  * @details	BaudRate	115200
	*						使能DMA发送，RXNE接收中断
	*						TX	PD5			PA2
	*						RX	PD6			PA3
  */
void Usart2_Init(u32 br_num)
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

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
	
	USART_Init(USART2, &USART_InitStructure);
	USART_ClockInit(USART2, &USART_ClockInitStruct);

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART2, ENABLE); 
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);

	DMA_DeInit(DMA1_Stream6);
	
	while (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE){}//等待DMA可配置 
	
  /* 配置 DMA Stream */
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART2->DR;//DMA外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)Tx2DMABuffer;//DMA 存储器0地址
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
  DMA_Init(DMA1_Stream6, &DMA_InitStructure);//初始化DMA Stream
	
}

/**
  * @brief 串口2中断程序
  * @param None
  * @retval None
  * @details RXNE中断，每接收到一个字节即进入一次中断，随后进入解析函数
  */
void USART2_IRQHandler(void)
{
	u8 com_data;
	if( USART_GetITStatus(USART2,USART_IT_RXNE) )				//接收中断
	{
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
		com_data = USART2->DR;
		Usart2_DataPrepare(com_data);											//数据解析
	}

}

/**
  * @brief 串口2的DMA发送函数，发送一组数据
  * @param DataToSend 要发送数据的数组的指针
	* @param data_num 要发送的数据的个数
  * @retval None
  */
void Usart2_Send(unsigned char *DataToSend ,u8 data_num)
{
	u8 i;
	static uint16_t num=0;
	static u8 len=0;
	
	DMA_Cmd(DMA1_Stream6, DISABLE);
	DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);//清除DMA2_Steam7传输完成标志
	num = DMA_GetCurrDataCounter(DMA1_Stream6);
	for(i=0;i<data_num;i++)
	{
		Tx2Buffer[count2++] = *(DataToSend+i);
	}
	for (i=0;i<(u8)num;i++)
	{
		Tx2DMABuffer[i]=Tx2Buffer[((u8)(len-num+i))];
	}
	for (;i<(u8)(num+data_num);i++)
	{
		Tx2DMABuffer[i]=*(DataToSend+i-num);
	}
	len=count2;
	while (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE){}	//确保DMA可以被设置  
	DMA1_Stream6->NDTR = (uint16_t)(num+data_num);          //数据传输量  
	DMA_Cmd(DMA1_Stream6, ENABLE);       
}


