
#include "usartx.h"

/**************************实现函数**********************************************
*功    能:		usart4发送一个字节
*********************************************************************************/
void usart3_send(u8 data)
{
	USART3->DR = data;
	while((USART3->SR&0x40)==0);	
}
/**************************************************************************
函数功能：串口3初始化
入口参数：pclk2:PCLK2 时钟频率(Mhz)    bound:波特率
返回  值：无
**************************************************************************/
void uart3_init(u32 pclk2,u32 bound)
{  	 
float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV
	mantissa=temp;				 //得到整数部分
	fraction=(temp-mantissa)*16; //得到小数部分	 
  mantissa<<=4;
	mantissa+=fraction; 
	

	RCC->APB2ENR|=1<<0;    //开启辅助时钟
	RCC->APB2ENR|=1<<4;   //使能PORTC口时钟  
	RCC->APB1ENR|=1<<18;  //使能串口时钟 
	GPIOC->CRH&=0XFFFF00FF; 
	GPIOC->CRH|=0X00008B00;//IO状态设置
	GPIOC->ODR|=1<<10;	 
  AFIO->MAPR|=1<<4;      //部分重映射

	RCC->APB1RSTR|=1<<18;   //复位串口1
	RCC->APB1RSTR&=~(1<<18);//停止复位	   	   
	//波特率设置
 	USART3->BRR=mantissa; // 波特率设置	 
	USART3->CR1|=0X200C;  //1位停止,无校验位.
	//使能接收中断
	USART3->CR1|=1<<8;    //PE中断使能
	USART3->CR1|=1<<5;    //接收缓冲区非空中断使能	    	
	MY_NVIC_Init(0,1,USART3_IRQn,2);//组2，最低优先级 
}

/**************************************************************************
函数功能：串口3接收中断
入口参数：无
返回  值：无
**************************************************************************/
int USART3_IRQHandler(void)
{	
	if(USART3->SR&(1<<5))//接收到数据
	{	      
				u8 temp;
				static u8 count,last_data,last_last_data,Usart_ON_Count;
		    if(Usart_ON_Flag==0)
				{	
		    if(++Usart_ON_Count>10)Usart_ON_Flag=1;
				}
				temp=USART3->DR;
				Show_Data_Mb=temp;
				   if(Usart_Flag==0)
						{	
						if(last_data==0xfe&&last_last_data==0xff) 
						Usart_Flag=1,count=0;	
						}
					 if(Usart_Flag==1)
						{	
							Urxbuf[count]=temp;     
							count++;                
							if(count==8)Usart_Flag=0;
						}
						last_last_data=last_data;
						last_data=temp;
   }
return 0;	
}



