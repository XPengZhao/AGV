#include "sys.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/ 
u8 Flag_Left,Flag_Right,Flag_Direction=0;             //蓝牙遥控相关的变量
u8 Flag_Stop=1,Flag_Show=0;                           //停止标志位和 显示标志位 默认停止 显示打开
int Encoder_A,Encoder_B,Encoder_C,Encoder_D;          //编码器的脉冲计数
long int Position_A,Position_B,Position_C,Position_D,Rate_A,Rate_B,Rate_C,Rate_D; //PID控制相关变量
int Encoder_A_EXTI;                                   //通过外部中断读取的编码器数据                       
long int Motor_A,Motor_B,Motor_C,Motor_D;             //电机PWM变量
long int Target_A,Target_B,Target_C,Target_D;         //电机目标值
int Voltage;                                          //电池电压采样相关的变量
float Show_Data_Mb;                                   //全局显示变量，用于显示需要查看的数据                         
u8 delay_50,delay_flag;                               //延时相关变量
u8 Run_Flag=0;                                        //蓝牙遥控相关变量和运行状态标志位
u8 rxbuf[8],Urxbuf[8],CAN_ON_Flag=0,Usart_ON_Flag=0,PS2_ON_Flag=0,Usart_Flag,PID_Send,Flash_Send;  //CAN和串口控制相关变量
u8 txbuf[8],txbuf2[8],Turn_Flag;                      //CAN发送相关变量
float Pitch,Roll,Yaw,Move_X,Move_Y,Move_Z;            //三轴角度和XYZ轴目标速度
u16 PID_Parameter[10],Flash_Parameter[10];            //Flash相关数组
float	Position_KP=40,Position_KI=0,Position_KD=40;    //位置控制PID参数
float Velocity_KP=10,Velocity_KI=10;	                //速度控制PID参数
int RC_Velocity=45,RC_Position=3000;                  //设置遥控的速度和位置值
int PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY;  
int Gryo_Z;
int main(void)
{ 
    Stm32_Clock_Init(9);            //=====系统时钟设置
    delay_init(72);                 //=====延时初始化
    JTAG_Set(JTAG_SWD_DISABLE);     //=====关闭JTAG接口
    JTAG_Set(SWD_ENABLE);           //=====打开SWD接口 可以利用主板的SWD接口调试
    LED_Init();                     //=====初始化与 LED 连接的硬件接口
    KEY_Init();                     //=====按键初始化
    if(MODE==0)Run_Flag=1;          //=====启动的过程中，根据模式选择开关确定进入位置模式还是速度模式
    else Run_Flag=0;                //=====启动的过程中，根据模式选择开关确定进入位置模式还是速度模式
    OLED_Init();                    //=====OLED初始化
    uart_init(72,115200);           //=====串口1初始化
    uart2_init(36,9600);            //=====串口2初始化
    uart3_init(36,115200);          //=====串口3初始化 默认与蓝牙连接
    Encoder_Init_TIM2();            //=====编码器接口
    Encoder_Init_TIM3();            //=====编码器接口
    Encoder_Init_TIM4();            //=====初始化编码器C
    Encoder_Init_TIM5();            //=====初始化编码器D
    Adc_Init();                     //=====adc初始化
    IIC_Init();                     //=====IIC初始化
    MPU6050_initialize();           //=====MPU6050初始化	
    DMP_Init();                     //=====初始化DMP     
    MiniBalance_PWM_Init(7199,0);   //=====初始化PWM 10KHZ，用于驱动电机
    if(KEY==0) Flash_Read();        //=====读取Flash里面的参数
    PS2_Init();                     //=====ps2驱动端口初始化
    PS2_SetInit();                  //=====ps2配置初始化,配置“红绿灯模式”，并选择是否可以修改
    EXTI_Init();                    //=====MPU6050 5ms定时中断初始化
    CAN1_Mode_Init(1,2,3,6,0);      //=====CAN初始化
    while(1)
    {
        if(Flash_Send==1)          //写入PID参数到Flash,由app控制该指令
        {
            Flash_Write();	
            Flash_Send=0;	
        }
        if(Flag_Show==0)           //使用MiniBalance APP和OLED显示屏
        {
            APP_Show();
            oled_show();           //===显示屏打开
        }
        else                       //使用MiniBalance上位机 上位机使用的时候需要严格的时序，故此时关闭app监控部分和OLED显示屏
        {
            DataScope();           //开启MiniBalance上位机
        }
        CAN1_SEND();               //CAN发送	
        PS2_Receive();             //PS2接收
        USART_TX();                //串口发送
        delay_flag=1;	
        delay_50=0;
        while(delay_flag);	       //通过MPU6050的INT中断实现的50ms精准延时
    } 
}
