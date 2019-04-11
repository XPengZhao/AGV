#include "main.h"


/**
  * @brief TIM5初始化
  * @param None
  * @retval None
  * @details 	TIM5是一个32位的寄存器，可以用来纯粹的计时，记录从系统开始到现在所
	*						经过的微秒数
  */
void TIM5_Configuration(void)										
{
   TIM_TimeBaseInitTypeDef tim;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
    tim.TIM_Period = 0xFFFFFFFF;
    tim.TIM_Prescaler = 90 - 1;	 //1M 的时钟  
    tim.TIM_ClockDivision = TIM_CKD_DIV1;	
    tim.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_ARRPreloadConfig(TIM5, ENABLE);	
    TIM_TimeBaseInit(TIM5, &tim);
    TIM_Cmd(TIM5,ENABLE);	
}

/**
  * @brief TIM5的溢出中断
  * @param None
  * @retval None
  * @details 执行清除中断位的操作
  */
void TIM5_IRQHandler(void)										
{
	  if (TIM_GetITStatus(TIM5,TIM_IT_Update)!= RESET) 
		{
			  TIM_ClearITPendingBit(TIM5,TIM_IT_Update);
        TIM_ClearFlag(TIM5, TIM_FLAG_Update);
		}
} 

/**
  * @brief 微秒级延时
  * @param None
  * @retval None
  */
void Delay_us(uint32_t us)										//用TIM2的计数值来做到精确延时
{
    uint32_t now = Get_Time_Micros();
    while (Get_Time_Micros() - now < us);
}

/**
  * @brief 毫秒级延时
  * @param None
  * @retval None
  */
void Delay_ms(uint32_t ms)
{
    while (ms--)
        Delay_us(1000);
}

/**
  * @brief TIM6初始化
  * @param None
  * @retval None
  * @details	TIM6用于产生1ms中断
  */
void TIM6_Configuration(void)							
{
    TIM_TimeBaseInitTypeDef  tim;
    NVIC_InitTypeDef         nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
    
    nvic.NVIC_IRQChannel = TIM6_DAC_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3;
    nvic.NVIC_IRQChannelSubPriority = 3;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    tim.TIM_Prescaler = 90-1;        //90M internal clock
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_Period = 1000 - 1;  //1ms,1000Hz
    TIM_TimeBaseInit(TIM6,&tim);
}

/**
  * @brief 使能TIM6中断
  * @param None
  * @retval None
  * @details	TIM6中断由此开始
  */
void TIM6_Start(void)
{
    TIM_Cmd(TIM6, ENABLE);	 
    TIM_ITConfig(TIM6, TIM_IT_Update,ENABLE);
    TIM_ClearFlag(TIM6, TIM_FLAG_Update);	
}

/**
  * @brief TIM6溢出中断
  * @param None
  * @retval None
  * @details 1ms中断，由此进入Duty_loop
  */
void TIM6_DAC_IRQHandler(void)								
{
	 if (TIM_GetITStatus(TIM6,TIM_IT_Update)!= RESET) 
	  {
			TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
			TIM_ClearFlag(TIM6, TIM_FLAG_Update);
			Duty_loop();
    }

}

/**
  * @brief 初始化GetInnerLoop功能
  * @param None
  * @retval None
  */
void InnerLoopInit(void)
{
	int i=0;
	for (i=0;i<INERLOOPLENGTH;i++)
	{
		GetInnerLoop(i);
	}
}	

/**
  * @brief 得到某函数的精准的调用周期
  * @param 计时序号，在头文件中可以查到
  * @retval 执行周期
  */
uint32_t GetInnerLoop(int loop)								//用于获得精确的函数调用的周期
{
	static uint32_t Time[2][20]={0};//Time[0] is the last time, Time[1] is the new time;
	Time[0][loop] = Time[1][loop];
	Time[1][loop] = Get_Time_Micros();
	return Time[1][loop]-Time[0][loop];
}
