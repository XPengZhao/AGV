#include "main.h"

/**
  * @brief MPU6500初始化
  * @param None
  * @retval 成功初始化返回0，否则返回1
  */
u8 MPU6500_Init(void)
{
	if( MPU6500_Read_Reg(WHO_AM_I)== 0x70)			//正确读取到6500的地址
	{		
		delay_ms(100);
		MPU6500_Write_Reg(PWR_MGMT_1,0X80);   		//电源管理,复位MPU6500
		delay_ms(100);
		MPU6500_Write_Reg(SIGNAL_PATH_RESET,0X07);//陀螺仪、加速度计、温度计复位
		delay_ms(100);
		MPU6500_Write_Reg(PWR_MGMT_1,0X01);   		//选择时钟源
		
		delay_ms(100);
		MPU6500_Write_Reg(USER_CTRL,0X30);  
		delay_ms(100);
		MPU6500_Write_Reg(SMPLRT_DIV,0X00);				//采样率1000/(1+0)=1000HZ
		delay_ms(100);
		MPU6500_Write_Reg(GYRO_CONFIG,0X10);  		//陀螺仪测量范围 0X10 正负1000度
		delay_ms(100);
		MPU6500_Write_Reg(CONFIG,0X04);						//低通滤波器 0x02 20hz (9.9ms delay) fs=1khz
		delay_ms(100);
		MPU6500_Write_Reg(ACCEL_CONFIG,0x10); 		//加速度计测量范围 0X10 正负8g
		delay_ms(100);
		MPU6500_Write_Reg(ACCEL_CONFIG2,0x04);		//加速度计速率1khz 滤波器20hz (99.8ms delay)
		delay_ms(100);
		MPU6500_Write_Reg(PWR_MGMT_2,0X00);   		//使能加速度计和陀螺仪
		
		return 0;
	}
	else {return 1;}
}

xyz_s16_t MPU6500_Acc_Raw;
xyz_s16_t MPU6500_Gyro_Raw;

xyz_f_t MPU6500_Acc;
xyz_f_t MPU6500_Gyro;
u8 mpu6500_buf[20];

/**
  * @brief 读取加速度计陀螺仪的原始数据
  * @param None
  * @retval None
  */
void MPU6500_ReadValueRaw(void)
{
	uint8_t i;
	
	MPU6500_CS(0); 																	//使能SPI传输

	SPI5_Read_Write_Byte(ACCEL_XOUT_H|0x80); 				//从加速度计的寄存器开始进行读取陀螺仪和加速度计的值//发送读命令+寄存器号
	
	for(i	=	0;i	<	14;i++)														//一共读取14字节的数据
	{
		mpu6500_buf[i]	=	SPI5_Read_Write_Byte(0xff);	//输入0xff,因为slave不识别
	}	
		MPU6500_Acc_Raw.x = BYTE16(s16, mpu6500_buf[0],  mpu6500_buf[1]);
		MPU6500_Acc_Raw.y = BYTE16(s16, mpu6500_buf[2],  mpu6500_buf[3]);
		MPU6500_Acc_Raw.z = BYTE16(s16, mpu6500_buf[4],  mpu6500_buf[5]);
		MPU6500_Gyro_Raw.x = BYTE16(s16, mpu6500_buf[8],  mpu6500_buf[9]);
		MPU6500_Gyro_Raw.y = BYTE16(s16, mpu6500_buf[10],  mpu6500_buf[11]);
		MPU6500_Gyro_Raw.z = BYTE16(s16, mpu6500_buf[12],  mpu6500_buf[13]);
//		
//		mpu6500_tempreature_temp	=	BYTE16(s16, mpu6500_buf[6],  mpu6500_buf[7]);
//		mpu6500_tempreature	=	(float)(35000+((521+mpu6500_tempreature_temp)*100)/34); // 原来分母为340，现在分子*100，即：扩大1000倍；
//		mpu6500_tempreature = mpu6500_tempreature/1000;                             
	
	
	MPU6500_CS(1);  	    //禁止SPI传输
}

float sum_temp[ITEMS]= {0,0,0,0,0,0,0};
u16 acc_sum_cnt = 0,gyro_sum_cnt = 0;


/**
  * @brief MPU6500数据校准
  * @param None
  * @retval None
  */
void MPU6500_Data_Cali(void)
{
	if (CALIFLAG & IMU_ACCERCALING)
	{
				acc_sum_cnt++;
        sum_temp[A_X] += MPU6500_Acc_Raw.x;
        sum_temp[A_Y] += MPU6500_Acc_Raw.y;
        sum_temp[A_Z] += MPU6500_Acc_Raw.z - 65536/16;   // +-8G
			if (acc_sum_cnt>=OFFSET_AV_NUM)
			{
				IMUSensor_Offset.ACCER_Offset.x=sum_temp[A_X]/OFFSET_AV_NUM;
				IMUSensor_Offset.ACCER_Offset.y=sum_temp[A_Y]/OFFSET_AV_NUM;
				IMUSensor_Offset.ACCER_Offset.z=sum_temp[A_Z]/OFFSET_AV_NUM;
				acc_sum_cnt=0;
				sum_temp[A_X]=sum_temp[A_Y]=sum_temp[A_Z]=0;
				CALIFLAG &= ~((u8)(IMU_ACCERCALING));
			}
			RED_LED_TOGGLE();
	}
	if (CALIFLAG & IMU_GYROCALING)
	{
				gyro_sum_cnt++;
        sum_temp[G_X] += MPU6500_Gyro_Raw.x;
        sum_temp[G_Y] += MPU6500_Gyro_Raw.y;
        sum_temp[G_Z] += MPU6500_Gyro_Raw.z;

        if( gyro_sum_cnt >= OFFSET_AV_NUM )
        {
					
            IMUSensor_Offset.GYRO_Offset.x = sum_temp[G_X]/OFFSET_AV_NUM;
            IMUSensor_Offset.GYRO_Offset.y= sum_temp[G_Y]/OFFSET_AV_NUM;
            IMUSensor_Offset.GYRO_Offset.z = sum_temp[G_Z]/OFFSET_AV_NUM;
            gyro_sum_cnt =0;
            sum_temp[G_X] = sum_temp[G_Y] = sum_temp[G_Z] = sum_temp[TEM] = 0;
						CALIFLAG &= ~((u8)(IMU_GYROCALING));
				}
				GREEN_LED_TOGGLE();
	
	}

}

float mpu6500_tmp[ITEMS];
float mpu_fil_tmp[ITEMS];
s16 FILT_BUF[ITEMS][(FILTER_NUM + 1)];
uint8_t filter_cnt = 0,filter_cnt_old = 0;
/**
  * @brief 对采样到的数据减去offset并作移动平均
  * @param None
  * @retval None
  */
void MPU6500_Data_Prepare(void)
{
	float FILT_TMP[ITEMS] = {0,0,0,0,0,0,0};
	int i=0;
	MPU6500_ReadValueRaw();
	MPU6500_Data_Cali();

	MPU6500_Data_Cali();
	mpu6500_tmp[A_X]=MPU6500_Acc_Raw.x-IMUSensor_Offset.ACCER_Offset.x;
	mpu6500_tmp[A_Y]=MPU6500_Acc_Raw.y-IMUSensor_Offset.ACCER_Offset.y;
	mpu6500_tmp[A_Z]=MPU6500_Acc_Raw.z-IMUSensor_Offset.ACCER_Offset.z;
	mpu6500_tmp[G_X]=MPU6500_Gyro_Raw.x - IMUSensor_Offset.GYRO_Offset.x;//
  mpu6500_tmp[G_Y]=MPU6500_Gyro_Raw.y- IMUSensor_Offset.GYRO_Offset.y;//
  mpu6500_tmp[G_Z]=MPU6500_Gyro_Raw.z - IMUSensor_Offset.GYRO_Offset.z;//
	if( ++filter_cnt > FILTER_NUM )
    {
        filter_cnt = 0;
        filter_cnt_old = 1;
    }
    else
    {
        filter_cnt_old = (filter_cnt == FILTER_NUM)? 0 : (filter_cnt + 1);
    }
		
		FILT_BUF[A_X][filter_cnt] = mpu6500_tmp[A_X];
    FILT_BUF[A_Y][filter_cnt] = mpu6500_tmp[A_Y];
    FILT_BUF[A_Z][filter_cnt] = mpu6500_tmp[A_Z];
    FILT_BUF[G_X][filter_cnt] = mpu6500_tmp[G_X];
    FILT_BUF[G_Y][filter_cnt] = mpu6500_tmp[G_Y];
    FILT_BUF[G_Z][filter_cnt] = mpu6500_tmp[G_Z];

    for(i=0; i<FILTER_NUM; i++)
    {
        FILT_TMP[A_X] += FILT_BUF[A_X][i];
        FILT_TMP[A_Y] += FILT_BUF[A_Y][i];
        FILT_TMP[A_Z] += FILT_BUF[A_Z][i];
        FILT_TMP[G_X] += FILT_BUF[G_X][i];
        FILT_TMP[G_Y] += FILT_BUF[G_Y][i];
        FILT_TMP[G_Z] += FILT_BUF[G_Z][i];
    }
		mpu_fil_tmp[A_X] = (float)( FILT_TMP[A_X] )/(float)FILTER_NUM;
    mpu_fil_tmp[A_Y] = (float)( FILT_TMP[A_Y] )/(float)FILTER_NUM;
    mpu_fil_tmp[A_Z] = (float)( FILT_TMP[A_Z] )/(float)FILTER_NUM;


    mpu_fil_tmp[G_X] = (float)( FILT_TMP[G_X] )/(float)FILTER_NUM;
    mpu_fil_tmp[G_Y] = (float)( FILT_TMP[G_Y] )/(float)FILTER_NUM;
    mpu_fil_tmp[G_Z] = (float)( FILT_TMP[G_Z] )/(float)FILTER_NUM;
		
		MPU6500_Acc.x=mpu_fil_tmp[A_X];
		MPU6500_Acc.y=mpu_fil_tmp[A_Y];
		MPU6500_Acc.z=mpu_fil_tmp[A_Z];
		MPU6500_Gyro.x=mpu_fil_tmp[G_X];
		MPU6500_Gyro.y=mpu_fil_tmp[G_Y];
		MPU6500_Gyro.z=mpu_fil_tmp[G_Z];
		
		MPU6500_Gyro.x=0.0610351563f*MPU6500_Gyro.x*0.5f;
		MPU6500_Gyro.y=0.0610351563f*MPU6500_Gyro.y*0.5f;
		MPU6500_Gyro.z=0.0610351563f*MPU6500_Gyro.z*0.5f;
}




