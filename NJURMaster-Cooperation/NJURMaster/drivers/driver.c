#include "main.h"

/**
  * @brief µ×²ãÇý¶¯³õÊ¼»¯
  * @param None
  * @retval None
  */
void All_Init(void)
{
	int i=0;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  
		
	TIM6_Configuration();
	TIM5_Configuration();
	InnerLoopInit();
	Led_Configuration();
	
	Rc_Init();
	Usart2_Init(115200);
	Usart3_Init(115200);
	Usart6_Init(115200);
  Can1_Init();
	Can2_Init();
	PWM_Init();
	EncoderInit();
	
	delay_ms(100);
	SPI5_Init();
	delay_ms(100);
	while(MPU6500_Init())
	{
		delay_ms(100);
	}
	delay_ms(200);
	while (IST8310_Init())
	{
		delay_ms(200);
	}
  
	for (i=0;i<100;i++)
	{
		CheckDog();
		delay_ms(5);
	}

	
	
	TIM6_Start();
}
