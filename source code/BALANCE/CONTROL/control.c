#include "control.h"	
#include "filter.h"	
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/

u8 Flag_Target,Flag_Change;                             //相关标志位
u8 temp1;                                               //临时变量
float Voltage_Count,Voltage_All;                        //电压采样相关变量
float Gyro_K=0.004;                                     //陀螺仪比例系数
int j;
#define a_PARAMETER          (0.25f)               
#define b_PARAMETER          (0.225f)         
/**************************************************************************
函数功能：小车运动数学模型
入口参数：X Y Z 三轴速度或者位置
返回  值：无
**************************************************************************/
void Kinematic_Analysis(float Vx,float Vy,float Vz)
{
    Target_A = -Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER);
    Target_B = +Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER);
    Target_C = -Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER);
    Target_D = +Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER);
}
/**************************************************************************
函数功能：获取位置控制过程速度值
入口参数：X Y Z 三轴位置变化量
返回  值：无
**************************************************************************/
void Kinematic_Analysis2(float Vx,float Vy,float Vz)
{
    Rate_A = -Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER);
    Rate_B = +Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER);
    Rate_C = -Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER);
    Rate_D = +Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER);
}
/**************************************************************************
函数功能：所有的控制代码都在这里面
         5ms定时中断由MPU6050的INT引脚触发
         严格保证采样和数据处理的时间同步
**************************************************************************/
int EXTI15_10_IRQHandler(void) 
{    
    if(INT==0)
    {
        EXTI->PR=1<<15;                                                       //清除LINE5上的中断标志位
        Flag_Target=!Flag_Target;
        if(delay_flag==1)
        {
            if(++delay_50==10)
                delay_50=0,delay_flag=0;                                       //给主函数提供50ms的精准延时
        }
        if(Flag_Target==1)                                                     //5ms读取一次陀螺仪和加速度计的值
        {
            if(Usart_Flag==0&&PS2_ON_Flag==0&&Usart_ON_Flag==1)
                memcpy(rxbuf,Urxbuf,8*sizeof(u8));                             //如果解锁了串口控制标志位，进入串口控制模式
            Read_DMP();                                                        //===更新姿态
            Key();                                                             //扫描按键变化
            return 0;	                                               
        }                                                       //===10ms控制一次，为了保证M法测速的时间基准，首先读取编码器数据
        Encoder_A=Read_Encoder(2)/20;                                          //===读取编码器的值(速度值)
        Position_A+=Encoder_A;                                                 //===积分得到速度
        Encoder_B=Read_Encoder(3)/20;                                          //===读取编码器的值
        Position_B+=Encoder_B;                                                 //===积分得到速度
        Encoder_C=-Read_Encoder(4)/20;                                         //===读取编码器的值
        Position_C+=Encoder_C;                                                 //===积分得到速度
        Encoder_D=-Read_Encoder(5)/20;                                         //===读取编码器的值
        Position_D+=Encoder_D;                                                 //===积分得到速度
        Read_DMP();                                                            //===更新姿态
        Led_Flash(100);                                                        //===LED闪烁;常规模式 1s改变一次指示灯的状态
        Voltage_All+=Get_battery_volt();                                       //多次采样累积
        if(++Voltage_Count==100)
            Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;             //求平均值 获取电池电压
        if(PS2_KEY==4)
            PS2_ON_Flag=1,CAN_ON_Flag=0,Usart_ON_Flag=0;
        if(CAN_ON_Flag==1||Usart_ON_Flag==1||PS2_ON_Flag==1)
            CAN_N_Usart_Control();                                  //接到串口或者CAN遥控解锁指令之后，使能CAN和串口控制输入
        if(RC_Velocity>0&&RC_Velocity<15)
            RC_Velocity=15;                                                   //避免电机进入低速非线性区

         if(Turn_Off(Voltage)==0)                                             //===如果电池电压不存在异常
         {
            if(Run_Flag==0)//速度模式
            {
                if(CAN_ON_Flag==0&&Usart_ON_Flag==0&&PS2_ON_Flag==0)  
                    Get_RC(Run_Flag);                                        //===串口和CAN控制都未使能，则接收蓝牙遥控指
                Motor_A=Incremental_PI_A(Encoder_A,Target_A);                         //===速度闭环控制计算电机A最终PWM
                Motor_B=Incremental_PI_B(Encoder_B,Target_B);                         //===速度闭环控制计算电机B最终PWM
                Motor_C=Incremental_PI_C(Encoder_C,Target_C);                         //===速度闭环控制计算电机C最终PWM
                Motor_D=Incremental_PI_D(Encoder_D,Target_D);                         //===速度闭环控制计算电机C最终PWM
            }
            else//位置模式
            {
                if(CAN_ON_Flag==0&&Usart_ON_Flag==0&&PS2_ON_Flag==0) //===串口和CAN控制都未使能，则接收蓝牙遥控指令
                {
                    if(Turn_Flag==0) 	Flag_Direction=click_RC();     
                    Get_RC(Run_Flag);
                }
                    Motor_A=Position_PID_A(Position_A,Target_A)>>8;//位置闭环控制，计算电机A速度内环的输入量
                    Motor_B=Position_PID_B(Position_B,Target_B)>>8;//位置闭环控制，计算电机B速度内环的输入量
                    Motor_C=Position_PID_C(Position_C,Target_C)>>8;//位置闭环控制，计算电机C速度内环的输入量
                    Motor_D=Position_PID_D(Position_D,Target_D)>>8;//位置闭环控制，计算电机C速度内环的输入量
                
                if(rxbuf[0]!=2)
                    Count_Velocity();   //这是调节位置控制过程的速度大小
                else
                    Xianfu_Velocity(RC_Velocity,RC_Velocity,RC_Velocity,RC_Velocity); 
                Motor_A=Incremental_PI_A(Encoder_A,-Motor_A);         //===速度闭环控制计算电机A最终PWM
                Motor_B=Incremental_PI_B(Encoder_B,-Motor_B);         //===速度闭环控制计算电机B最终PWM
                Motor_C=Incremental_PI_C(Encoder_C,-Motor_C);         //===速度闭环控制计算电机C最终PWM
                Motor_D=Incremental_PI_D(Encoder_D,-Motor_D);         //===速度闭环控制计算电机C最终PWM
            }
            Xianfu_Pwm(6900);                     //===PWM限幅
            Set_Pwm(Motor_A,Motor_B,Motor_C,Motor_D);     //===赋值给PWM寄存器  
        }
    }
     return 0;
} 


/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d)
{
        int siqu=0;
        if(motor_a>0)    PWMA=motor_a+siqu, INA=0;
        else             PWMA=7200+motor_a-siqu,INA=1;
        
        if(motor_b>0)    PWMB=7200-motor_b-siqu,INB=1;
        else 	         PWMB=-motor_b+siqu,INB=0;
    
        if(motor_c>0)    PWMC=7200-motor_c-siqu,INC=1;
        else 	         PWMC=-motor_c+siqu,INC=0;
    
        if(motor_d>0)     PWMD=motor_d+siqu,IND=0;
        else              PWMD=7200+motor_d-siqu,IND=1;
}

/**************************************************************************
函数功能：限制PWM赋值 
入口参数：幅值
返回  值：无
**************************************************************************/
void Xianfu_Pwm(int amplitude)
{	
    if(Motor_A<-amplitude) Motor_A=-amplitude;
    if(Motor_A>amplitude)  Motor_A=amplitude;
    if(Motor_B<-amplitude) Motor_B=-amplitude;
    if(Motor_B>amplitude)  Motor_B=amplitude;
    if(Motor_C<-amplitude) Motor_C=-amplitude;
    if(Motor_C>amplitude)  Motor_C=amplitude;
    if(Motor_D<-amplitude) Motor_D=-amplitude;
    if(Motor_D>amplitude)  Motor_D=amplitude;
}
/**************************************************************************
函数功能：位置PID控制过程中速度的设置
入口参数：无、幅值
返回  值：无
**************************************************************************/
void Xianfu_Velocity(int amplitude_A,int amplitude_B,int amplitude_C,int amplitude_D)
{	
    if(Motor_A<-amplitude_A) Motor_A=-amplitude_A;	//位置控制模式中，A电机的运行速度
    if(Motor_A>amplitude_A)  Motor_A=amplitude_A;	  //位置控制模式中，A电机的运行速度
    if(Motor_B<-amplitude_B) Motor_B=-amplitude_B;	//位置控制模式中，B电机的运行速度
    if(Motor_B>amplitude_B)  Motor_B=amplitude_B;		//位置控制模式中，B电机的运行速度
    if(Motor_C<-amplitude_C) Motor_C=-amplitude_C;	//位置控制模式中，C电机的运行速度
    if(Motor_C>amplitude_C)  Motor_C=amplitude_C;		//位置控制模式中，C电机的运行速度
    if(Motor_D<-amplitude_D) Motor_D=-amplitude_D;	//位置控制模式中，C电机的运行速度
    if(Motor_D>amplitude_D)  Motor_D=amplitude_D;		//位置控制模式中，C电机的运行速度
}
/**************************************************************************
函数功能：按键修改小车运行状态 
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{	
    u8 tmp;
    tmp=click_N_Double(100); 
    if(tmp==2)Flag_Show=!Flag_Show;//双击控制显示模式
}

/**************************************************************************
函数功能：异常关闭电机
入口参数：电压
返回  值：1：异常  0：正常
**************************************************************************/
u8 Turn_Off( int voltage)
{
    u8 temp;
    if(voltage<2220)//电池电压低于22.2V关闭电机
    {
        temp=1;
        PWMA=0; PWMB=0; PWMC=0; PWMD=0;
        INA=0;  INB=0;  INC=0;  IND=0;
    }
    else
        temp=0;
        return temp;
}

/**************************************************************************
函数功能：绝对值函数
入口参数：long int
返回  值：unsigned int
**************************************************************************/
u32 myabs(long int a)
{
    u32 temp;
    if(a<0)  temp=-a;  
    else temp=a;
    return temp;
}
/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (int Encoder,int Target)
{ 	
     static int Bias,Pwm,Last_bias;
     Bias=Encoder-Target;                //计算偏差
     Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
     if(Pwm>7200)Pwm=7200;
     if(Pwm<-7200)Pwm=-7200;
     Last_bias=Bias;	                   //保存上一次偏差 
     return Pwm;                         //增量输出
}
int Incremental_PI_B (int Encoder,int Target)
{ 	
     static int Bias,Pwm,Last_bias;
     Bias=Encoder-Target;                //计算偏差
     Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
     if(Pwm>7200)Pwm=7200;
     if(Pwm<-7200)Pwm=-7200;
     Last_bias=Bias;	                   //保存上一次偏差 
     return Pwm;                         //增量输出
}
int Incremental_PI_C (int Encoder,int Target)
{ 	
     static int Bias,Pwm,Last_bias;
     Bias=Encoder-Target;                                  //计算偏差
     Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
     if(Pwm>7200)Pwm=7200;
     if(Pwm<-7200)Pwm=-7200;
     Last_bias=Bias;	                   //保存上一次偏差 
     return Pwm;                         //增量输出
}
int Incremental_PI_D (int Encoder,int Target)
{ 	
     static int Bias,Pwm,Last_bias;
     Bias=Encoder-Target;                                  //计算偏差
     Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //增量式PI控制器
     if(Pwm>7200)Pwm=7200;
     if(Pwm<-7200)Pwm=-7200;
     Last_bias=Bias;	                   //保存上一次偏差 
     return Pwm;                         //增量输出
}
/**************************************************************************
函数功能：位置式PID控制器
入口参数：编码器测量位置信息，目标位置
返回  值：电机PWM
根据位置式离散PID公式 
pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  
∑e(k)代表e(k)以及之前的偏差的累积和;其中k为1,2,,k;
pwm代表输出
**************************************************************************/
int Position_PID_A (int Encoder,int Target)
{ 	
    static float Bias,Pwm,Integral_bias,Last_Bias;
    Bias=Encoder-Target;                                  //计算偏差
    Integral_bias+=Bias;	                                 //求出偏差的积分
    if(Integral_bias>100000)Integral_bias=10000;
    if(Integral_bias<-100000)Integral_bias=-10000;
    Pwm=Position_KP*Bias+Position_KI/100*Integral_bias+Position_KD*(Bias-Last_Bias);       //位置式PID控制器
    Last_Bias=Bias;                                       //保存上一次偏差 
    return Pwm;                                           //增量输出
}
int Position_PID_B (int Encoder,int Target)
{
    static float Bias,Pwm,Integral_bias,Last_Bias;
    Bias=Encoder-Target;                                  //计算偏差
    Integral_bias+=Bias;	                                 //求出偏差的积分
    if(Integral_bias>100000)Integral_bias=10000;
    if(Integral_bias<-100000)Integral_bias=-10000;
    Pwm=Position_KP*Bias+Position_KI/100*Integral_bias+Position_KD*(Bias-Last_Bias);       //位置式PID控制器
    Last_Bias=Bias;                                       //保存上一次偏差 
    return Pwm;                                           //增量输出
}

int Position_PID_C (int Encoder,int Target)
{
    static float Bias,Pwm,Integral_bias,Last_Bias;
    Bias=Encoder-Target;                                  //计算偏差
    Integral_bias+=Bias;	                                 //求出偏差的积分
    if(Integral_bias>100000)Integral_bias=10000;
    if(Integral_bias<-100000)Integral_bias=-10000;
    Pwm=Position_KP*Bias+Position_KI/100*Integral_bias+Position_KD*(Bias-Last_Bias);       //位置式PID控制器
    Last_Bias=Bias;                                       //保存上一次偏差 
    return Pwm;                                           //增量输出
}

int Position_PID_D (int Encoder,int Target)
{ 	
    static float Bias,Pwm,Integral_bias,Last_Bias;
    Bias=Encoder-Target;                                  //计算偏差
    Integral_bias+=Bias;	                                 //求出偏差的积分
    if(Integral_bias>100000)Integral_bias=10000;
    if(Integral_bias<-100000)Integral_bias=-10000;
    Pwm=Position_KP*Bias+Position_KI/100*Integral_bias+Position_KD*(Bias-Last_Bias);       //位置式PID控制器
    Last_Bias=Bias;                                       //保存上一次偏差 
    return Pwm;                                           //增量输出
}
/**************************************************************************
函数功能：通过串口指令对小车进行遥控
入口参数：串口指令
返回  值：无
**************************************************************************/
void Get_RC(u8 mode)
{

    float step=0.5;  //设置速度控制步进值。
    u8 Flag_Move=1;
      if(mode==0)//速度
        {	
                 switch(Flag_Direction)   //方向控制
                 {
                 case 1:  Move_X=0;           Move_Y+=step;  Flag_Move=1;               break;
                 case 2:  Move_X+=step;       Move_Y+=step;  Flag_Move=1;               break;
                 case 3:  Move_X+=step;       Move_Y=0;      Flag_Move=1;               break;
                 case 4:  Move_X+=step;       Move_Y-=step;  Flag_Move=1;               break;
                 case 5:  Move_X=0;           Move_Y-=step;  Flag_Move=1;               break;
                 case 6:  Move_X-=step;       Move_Y-=step;  Flag_Move=1;               break;
                 case 7:  Move_X-=step;       Move_Y=0;      Flag_Move=1;               break;
                 case 8:  Move_X-=step;       Move_Y+=step;  Flag_Move=1;               break; 
                 default: Flag_Move=0;        Move_X=Move_X/1.05;	Move_Y=Move_Y/1.05;	  break;	 
             }
                 
            if(Flag_Move==0)		//如果无方向控制指令	 ，检查转向控制状态
            {	
                if(Flag_Left==1)       Move_Z-=step,Gyro_K=0;    //左自旋   
                else if(Flag_Right==1) Move_Z+=step,Gyro_K=0;    //右自旋		
                else 		           Move_Z=0,Gyro_K=0.004;    //停止
            }	
                if(Flag_Move==1)        Flag_Left=0,Flag_Right=0,Move_Z=0;
                if(Move_X<-RC_Velocity) Move_X=-RC_Velocity;	   //速度控制限幅
                if(Move_X>RC_Velocity)  Move_X=RC_Velocity;	     
                if(Move_Y<-RC_Velocity) Move_Y=-RC_Velocity;	
                if(Move_Y>RC_Velocity)  Move_Y=RC_Velocity;	 
                if(Move_Z<-RC_Velocity) Move_Z=-RC_Velocity;	
                if(Move_Z>RC_Velocity)  Move_Z=RC_Velocity;	 
             
     }
         else if(mode==1)//位置模式
        {	
                 switch(Flag_Direction)   //方向控制
                 {
                 case 1:  Move_Y+=RC_Position; Flag_Change=1;break;
                 case 2:  Move_X+=RC_Position; Flag_Change=2; Move_Y+=RC_Position; break;
                 case 3:  Move_X+=RC_Position; Flag_Change=3;break;
                 case 4:  Move_X+=RC_Position; Flag_Change=4;Move_Y-=RC_Position;break;
                 case 5:  Move_Y-=RC_Position; Flag_Change=5;break;
                 case 6:  Move_X-=RC_Position; Flag_Change=6;Move_Y-=RC_Position; break;
                 case 7:  Move_X-=RC_Position; Flag_Change=7; break;
                 case 8:  Move_X-=RC_Position; Flag_Change=8;Move_Y+=RC_Position;break;			 
                 case 9:  Move_Z-=RC_Position; Flag_Change=9; break;
                 case 10: Move_Z+=RC_Position; Flag_Change=10;break;			 
                 default: break;	 
             }
     }
         Kinematic_Analysis(Move_X,Move_Y,Move_Z);//得到控制目标值，进行运动学分析
}
/**************************************************************************
函数功能：每个电机位置控制过程速度计算
入口参数：无
返回  值：无
**************************************************************************/
void Count_Velocity(void)
{
    static double Last_Target_X,Last_Target_Y,Last_Target_Z,Divider;
    double Bias_X,Bias_Y,Bias_Z;
    Bias_X=(Move_X-Last_Target_X);  //求X轴位移量
    Bias_Y=(Move_Y-Last_Target_Y);	//求Y轴位移量
    Bias_Z=(Move_Z-Last_Target_Z);	//求Z轴位移量
    if(Bias_X!=0||Bias_Y!=0||Bias_Z!=0)Divider=sqrt(Bias_X*Bias_X+Bias_Y*Bias_Y+Bias_Z*Bias_Z);
    if(Bias_X!=0||Bias_Y!=0||Bias_Z!=0) Kinematic_Analysis2(Bias_X,Bias_Y,Bias_Z);

    Xianfu_Velocity(RC_Velocity*myabs(Rate_A)/Divider,RC_Velocity*myabs(Rate_B)/Divider,RC_Velocity*myabs(Rate_C)/Divider,RC_Velocity*myabs(Rate_D)/Divider); 
    Last_Target_X=Move_X;   //保存X轴上一次的位置信息，便于调用
    Last_Target_Y=Move_Y;   //保存Y轴上一次的位置信息，便于调用
    Last_Target_Z=Move_Z;   //保存Z轴上一次的位置信息，便于调用
}
/**************************************************************************
函数功能：接收CAN或者串口控制指令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void CAN_N_Usart_Control(void)
{
    int flag_1, flag_2,flag_3,flag_4;
    int Yuzhi=20;
    int LX,LY,RX;
    if(Run_Flag==0)//速度模式
    {
        if(CAN_ON_Flag==1||Usart_ON_Flag==1) 
        {
            if(rxbuf[0]==1)
            {
                if((rxbuf[7]&0x04)==0)flag_1=1;  else flag_1=-1;  //方向控制位
                if((rxbuf[7]&0x02)==0)flag_2=1;  else flag_2=-1;  //方向控制位
                if((rxbuf[7]&0x01)==0)flag_3=1;  else flag_3=-1;  //方向控制位
                Move_X=flag_1*(rxbuf[1]*256+rxbuf[2]);
                Move_Y=flag_2*(rxbuf[3]*256+rxbuf[4]);	
                Move_Z=flag_3*(rxbuf[5]*256+rxbuf[6]);	
                Kinematic_Analysis(Move_X,Move_Y,Move_Z);//进行运动学分析
                Gyro_K=0;    
            }
            if(rxbuf[0]==2)
            {
                if((rxbuf[7]&0x08)==0)flag_1=1;  else flag_1=-1;  //方向控制位
                if((rxbuf[7]&0x04)==0)flag_2=1;  else flag_2=-1;  //方向控制位
                if((rxbuf[7]&0x02)==0)flag_3=1;  else flag_3=-1;  //方向控制位
                if((rxbuf[7]&0x01)==0)flag_4=1;  else flag_4=-1;  //方向控制位
                
                Target_A=flag_1*rxbuf[1];
                Target_B=flag_2*rxbuf[2];
                Target_C=flag_3*rxbuf[3];
                Target_D=flag_4*rxbuf[4];
            }
        }
        else if (PS2_ON_Flag==1)
        {
            LX=PS2_LX-128;
            LY=PS2_LY-128;
            RX=PS2_RX-128;
            if(LX>-Yuzhi&&LX<Yuzhi)LX=0;
            if(LY>-Yuzhi&&LY<Yuzhi)LY=0;
            if(RX>-Yuzhi&&RX<Yuzhi)RX=0;
            Move_X=LX*RC_Velocity/200;
            Move_Y=-LY*RC_Velocity/200;	
            Move_Z=-RX*RC_Velocity/200;		 
//		 if(Move_X<-RC_Velocity) Move_X=-RC_Velocity;	   //速度控制限幅
//		if(Move_X>RC_Velocity)  Move_X=RC_Velocity;	     
//		if(Move_Y<-RC_Velocity) Move_Y=-RC_Velocity;	
//		if(Move_Y>RC_Velocity)  Move_Y=RC_Velocity;	 
//		if(Move_Z<-RC_Velocity) Move_Z=-RC_Velocity;	
//		if(Move_Z>RC_Velocity)  Move_Z=RC_Velocity;	 
            Kinematic_Analysis(Move_X,Move_Y,Move_Z),Gyro_K=0;    //进行运动学分析 
        }

    }
    else if(Run_Flag==1)//位置模式
    {
        if(rxbuf[0]==1)
        {
            if((rxbuf[7]&0x04)==0)flag_1=1;  else flag_1=-1;  //方向控制位
            if((rxbuf[7]&0x02)==0)flag_2=1;  else flag_2=-1;  //方向控制位
            if((rxbuf[7]&0x01)==0)flag_3=1;  else flag_3=-1;  //方向控制位
            Move_X=flag_1*(rxbuf[1]*256+rxbuf[2]);
            Move_Y=flag_2*(rxbuf[3]*256+rxbuf[4]);	
            Move_Z=flag_3*(rxbuf[5]*256+rxbuf[6]);	
            Kinematic_Analysis(Move_X,Move_Y,Move_Z);//进行运动学分析
            Gyro_K=0;    
        }
     }
}
