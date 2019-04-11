#include "main.h"

/**
  * @brief 所有非驱动的初始化
  * @param None
  * @retval None
  */
void AppInit(void)
{
//	DogInit();
	SensorOffsetInit();
	ParametersInit();
}
