#include "main.h"

/**
  * @brief 通过MPU6500写入磁力计
  * @param None
  * @retval None
  */
 static void IST_Reg_Write_By_MPU(uint8_t addr, uint8_t data)
{
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x00);
  Delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_REG, addr);
  Delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_DO, data);
  Delay_ms(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x080 | 0x01);
  Delay_ms(10);
}
 
/**
  * @brief 通过MPU6500读取磁力计
  * @param None
  * @retval None
  */
 static uint8_t IST_Reg_Read_By_MPU(uint8_t addr)
{
  uint8_t data;
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_REG, addr);
  delay_ms(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x80);
  delay_ms(2);
  data = MPU6500_Read_Reg(MPU6500_I2C_SLV4_DI);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x00);
	delay_ms(2);
  return data;
}

/**
  * @brief MPU_Auto_Read配置
  * @param None
  * @retval None
  * @details SLV1写 SLV0读
  */
 static void MPU_Auto_Read_IST_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_ADDR, device_address);
  Delay_ms(40);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
  Delay_ms(40);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
  Delay_ms(40);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
  Delay_ms(40);
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_REG, reg_base_addr);
  Delay_ms(40);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x04);
 Delay_ms(40);
  
  MPU6500_Write_Reg(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
  Delay_ms(40);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
  Delay_ms(40);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
  Delay_ms(40);
}

/**
  * @brief IST8310初始化
  * @param None
  * @retval 初始化成功则返回0
  * @details SLV4读取 SLV1写入
  */
u8 IST8310_Init(void)
{
	MPU6500_Write_Reg(USER_CTRL, 0x30);
  Delay_ms(40);
  MPU6500_Write_Reg(MPU6500_I2C_MST_CTRL, 0x0d);
  Delay_ms(40);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);
  Delay_ms(40);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
  Delay_ms(40);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFB, 0x01);
  if(IST8310_DEVICE_ID_A != IST_Reg_Read_By_MPU(IST8310_WHO_AM_I))
    return 1; //error
  Delay_ms(40);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFA, 0x00);
  if(IST_Reg_Read_By_MPU(IST8310_R_CONFA) != 0x00)
    return 2;
  Delay_ms(40);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFB, 0x00);
  if(IST_Reg_Read_By_MPU(IST8310_R_CONFB) != 0x00)
    return 3;
  Delay_ms(40);
  
  IST_Reg_Write_By_MPU(IST8310_AVGCNTL, 0x24);
  if(IST_Reg_Read_By_MPU(IST8310_AVGCNTL) != 0x24)
    return 4;
	Delay_ms(40);
  
  IST_Reg_Write_By_MPU(IST8310_PDCNTL, 0xc0);
  if(IST_Reg_Read_By_MPU(IST8310_PDCNTL) != 0xc0)
    return 5;
 Delay_ms(40);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x00);
  Delay_ms(40);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x00);
  Delay_ms(40);
  
  MPU_Auto_Read_IST_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
  Delay_ms(100);
  return 0;
}

xyz_f_t IST_Raw;
/**
  * @brief 通过SLV0去自动读取ist8310
  * @param None
  * @retval None
  */
uint8_t temp_ist_buff[6];
void IST8310_getRawEX(void)
{
	u8 i=0;
	
	static u8 ADDR_BASE_EXTER = 0x49;
	for(i=0;i<6;i++)
	{
		temp_ist_buff[i]=MPU6500_Read_Reg(ADDR_BASE_EXTER+i);
		delay_us(1);
	}
	IST_Raw.x = BYTE16(s16, temp_ist_buff[1],temp_ist_buff[0]);
	IST_Raw.y	= BYTE16(s16, temp_ist_buff[3],temp_ist_buff[2]);
	IST_Raw.z = BYTE16(s16, temp_ist_buff[5],temp_ist_buff[4]); 
}

/**
  * @brief 校准磁力计
  * @param None
  * @retval None
  */
void IST8310_CALI(void)
{	

	static xyz_f_t	MagMAX = { -100 , -100 , -100 }, MagMIN = { 100 , 100 , 100 };//, MagSum;
	static uint16_t cnt_m=0;
	
	if (CALIFLAG & IMU_MAGCALING)
	{
		BOTH_LED_TOGGLE();
		if(ABS(IST_Raw.x)<800&&ABS(IST_Raw.y)<800&&ABS(IST_Raw.z)<800)
		{
			MagMAX.x = _MAX(IST_Raw.x, MagMAX.x);
			MagMAX.y = _MAX(IST_Raw.y, MagMAX.y);
			MagMAX.z = _MAX(IST_Raw.z, MagMAX.z);
			
			MagMIN.x = _MIN(IST_Raw.x, MagMIN.x);
			MagMIN.y = _MIN(IST_Raw.y, MagMIN.y);
			MagMIN.z = _MIN(IST_Raw.z, MagMIN.z);		
			
			if(cnt_m == CALIBRATING_MAG_CYCLES)
			{
				IMUSensor_Offset.MAG_Offset.x = ((MagMAX.x + MagMIN.x) * 0.5f);
				IMUSensor_Offset.MAG_Offset.y = ((MagMAX.y + MagMIN.y) * 0.5f);
				IMUSensor_Offset.MAG_Offset.z = ((MagMAX.z + MagMIN.z) * 0.5f);
	
	//			MagSum.x = MagMAX.x - MagMIN.x;
	//			MagSum.y = MagMAX.y - MagMIN.y;
	//			MagSum.z = MagMAX.z - MagMIN.z;
				
//				ak8975.Mag_Gain.y = MagSum.x / MagSum.y;
//				ak8975.Mag_Gain.z = MagSum.x / MagSum.z;
//				
//				Param_SaveMagOffset(&ak8975.Mag_Offset);//param_Save();//????
				cnt_m = 0;
				CALIFLAG &= ~(IMU_MAGCALING);
		//		f.msg_id = 3;
		//		f.msg_data = 1;
	
			}
		}
		cnt_m++;
	
	}
}
xyz_f_t MagValue;
/**
  * @brief 得到滤波后的磁力计数据
  * @param None
  * @retval None
  */
void IST8310_Data_Prepare(void)
{
	IST8310_getRawEX();
	IST8310_CALI();
	MagValue.x = -(IST_Raw.x - IMUSensor_Offset.MAG_Offset.x);
	MagValue.y = -(IST_Raw.y - IMUSensor_Offset.MAG_Offset.y);
	MagValue.z = (IST_Raw.z - IMUSensor_Offset.MAG_Offset.z);

}
