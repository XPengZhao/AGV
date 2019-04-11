#include "main.h"

/**
  * @brief main函数，用于初始化执行主循环等
  * @param None
  * @retval 成功返回0，失败返回错误码
  */
int main()
{

	AppInit();
	All_Init();

	while (1)
	{
		if (ParaSavingFlag)
		{
			ParametersSave();
			ParaSavingFlag=0;
		}
		delay_ms(1000);
	}
}

