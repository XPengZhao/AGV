#include "main.h"


u8 CALIFLAG=0;
u8 ParamSavedFlag=0;
u8 ParaSavingFlag=0;
IMUSensor_OffSet__ IMUSensor_Offset;
AllDataUnion__ AllDataUnion; 
_PID_arg_st PID_arg[PIDGROUPLEN]={CHASSIS_Rot_PID_OFF,
																	CHASSIS_Vec_PID_OFF,
																	GIMBALP_Pos_PID_OFF,
																	GIMBALP_Vec_PID_OFF,
																	GIMBALY_Pos_PID_OFF,
																	GIMBALY_Vec_PID_OFF,
																	SLIBLIN_Pos_PID_OFF,
																	SLIBLIN_Vec_PID_OFF};
_PID_val_st PID_val[PIDGROUPLEN+3];


/**
  * @brief 参数初始化
  * @param None
  * @retval None
  */
void ParametersInit(void)
{
	int i=0;
	if (AllDataUnion.AllData.savedflag==1)
	{
		for(i=0;i<PIDGROUPLEN;i++)
		{
				PID_arg[i]=AllDataUnion.AllData.PID_ARG[i];
		}
	}

}

/**
  * @brief 传感器校准值初始化
  * @param None
  * @retval None
  */
void SensorOffsetInit(void)
{
	BSP_FLASH_Read(ADDR_FLASH_SECTOR_11, AllDataUnion.data, sizeof(AllDataOffset__));
	if (AllDataUnion.AllData.savedflag==1)
	{
		IMUSensor_Offset = AllDataUnion.AllData.imu_offset;
		
		ParamSavedFlag = 1;
//		for(i=0;i<PIDGROUPLEN;i++)
//		{
//			PID_arg[i]=AllDataUnion.AllData.PID_ARG[i];
//		}
	}

}

/**
  * @brief 参数保存至FALSH
  * @param None
  * @retval None
  */
void ParametersSave(void)
{
	int i=0;
	for (i=0;i<PIDGROUPLEN;i++)
	{
		AllDataUnion.AllData.PID_ARG[i]=PID_arg[i];
	}
	AllDataUnion.AllData.savedflag=1;
	AllDataUnion.AllData.imu_offset=IMUSensor_Offset;
	BSP_FLASH_Write(ADDR_FLASH_SECTOR_11, AllDataUnion.data, sizeof(AllDataOffset__));
}

/**
  * @brief 传感器校准值保存至FLASH
  * @param None
  * @retval None
  */
void SensorsOffsetSave(void)
{


}

/**
  * @brief 陀螺仪校准
  * @param None
  * @retval None
  */
void IMU_GYRODataCali(void)
{
	SysMode=SYS_CALISTATE;
	CALIFLAG |= IMU_GYROCALING;
}

/**
  * @brief 加速度计校准
  * @param None
  * @retval None
  */
void IMU_ACCERDataCali(void)
{
	CALIFLAG |= IMU_ACCERCALING;
	GimbalDataCali();
}

/**
  * @brief 磁力计校准
  * @param None
  * @retval None
  */
void IMU_MAGDataCali(void)
{
	CALIFLAG |= IMU_MAGCALING;
}

/**
  * @brief 云台校准
  * @param None
  * @retval None
  */
void GimbalDataCali(void)
{
	CALIFLAG |= (GIMBALYAWCALING|GIMBALPITCHCALING);
}
