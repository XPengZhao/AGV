#include "CAN.h"
#include "led.h"
#include "delay.h"
#include "usart.h"
//作者：平衡小车之家
//我的淘宝小店：http://shop114407458.taobao.com/
//版权所有 盗版必究
        
//CAN1初始化
//tsjw:重新同步跳跃时间单元.范围:1~3;
//tbs2:时间段2的时间单元.范围:1~8;
//tbs1:时间段1的时间单元.范围:1~16;
//brp :波特率分频器.范围:1~1024;(实际要加1,也就是1~1024) tq=(brp)*tpclk1
//注意以上参数任何一个都不能设为0,否则会乱.
//波特率=Fpclk1/((tbs1+tbs2+1)*brp);
//mode:0,普通模式;1,回环模式;
//Fpclk1的时钟在初始化的时候设置为36M,如果设置CAN1_Normal_Init(1,2,3,12,1);
//则波特率为:   36/(1+3+2)/12=500K  
//返回值:0,初始化OK;
//    其他,初始化失败;
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
    u16 i=0;
    if(tsjw==0||tbs2==0||tbs1==0||brp==0)return 1;
    tsjw-=1;//先减去1.再用于设置
    tbs2-=1;
    tbs1-=1;
    brp-=1;

    RCC->APB2ENR|=1<<0;    //开启辅助时钟
    RCC->APB2ENR|=1<<3;    //使能PORTB时钟	 
    RCC->APB1ENR|=1<<25;//使能CAN1时钟 CAN1使用的是APB1的时钟(max:36M)
    
    GPIOB->CRH&=0XFFFFFF00; 
    GPIOB->CRH|=0X000000B8;//PB8 RX,PB9 TX推挽输出   	 
    GPIOB->ODR|=3<<8;
    AFIO->MAPR|=2<<13;      //CAN部分重映射

    CAN1->MCR=0x0000;	//退出睡眠模式(同时设置所有位为0)
    CAN1->MCR|=1<<0;		//请求CAN1进入初始化模式
    while((CAN1->MSR&1<<0)==0)
    {
        i++;
        if(i>100)return 2;//进入初始化模式失败
    }
    CAN1->MCR|=0<<7;		//非时间触发通信模式
    CAN1->MCR|=0<<6;		//软件自动离线管理
    CAN1->MCR|=0<<5;		//睡眠模式通过软件唤醒(清除CAN1->MCR的SLEEP位)
    CAN1->MCR|=1<<4;		//禁止报文自动传送
    CAN1->MCR|=0<<3;		//报文不锁定,新的覆盖旧的
    CAN1->MCR|=0<<2;		//优先级由报文标识符决定
    CAN1->BTR=0x00000000;//清除原来的设置.
    CAN1->BTR|=mode<<30;	//模式设置 0,普通模式;1,回环模式;
    CAN1->BTR|=tsjw<<24; //重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位
    CAN1->BTR|=tbs2<<20; //Tbs2=tbs2+1个时间单位
    CAN1->BTR|=tbs1<<16;	//Tbs1=tbs1+1个时间单位
    CAN1->BTR|=brp<<0;  	//分频系数(Fdiv)为brp+1
                        //波特率:Fpclk1/((Tbs1+Tbs2+1)*Fdiv)
    CAN1->MCR&=~(1<<0);	//请求CAN1退出初始化模式
    while((CAN1->MSR&1<<0)==1)
    {
        i++;
        if(i>0XFFF0)return 3;//退出初始化模式失败
    }
    //过滤器初始化
    CAN1->FMR|=1<<0;			//过滤器组工作在初始化模式
    CAN1->FA1R&=~(1<<0);		//过滤器0不激活
    CAN1->FS1R|=1<<0; 		//过滤器位宽为32位.
    CAN1->FM1R|=0<<0;		//过滤器0工作在标识符屏蔽位模式
    CAN1->FFA1R|=0<<0;		//过滤器0关联到FIFO0
    CAN1->sFilterRegister[0].FR1=0X00000000;//32位ID
    CAN1->sFilterRegister[0].FR2=0X00000000;//32位MASK
    CAN1->FA1R|=1<<0;		//激活过滤器0
    CAN1->FMR&=0<<0;			//过滤器组进入正常模式

#if CAN1_RX0_INT_ENABLE
     //使用中断接收
    CAN1->IER|=1<<1;			//FIFO0消息挂号中断允许.	    
    MY_NVIC_Init(1,0,USB_LP_CAN1_RX0_IRQn,2);//组2
#endif
    return 0;
}  
//作者：平衡小车之家
//我的淘宝小店：http://shop114407458.taobao.com/
//版权所有 盗版必究
//id:标准ID(11位)/扩展ID(11位+18位)	    
//ide:0,标准帧;1,扩展帧
//rtr:0,数据帧;1,远程帧
//len:要发送的数据长度(固定为8个字节,在时间触发模式下,有效数据为6个字节)
//*dat:数据指针.
//返回值:0~3,邮箱编号.0XFF,无有效邮箱.
u8 CAN1_Tx_Msg(u32 id,u8 ide,u8 rtr,u8 len,u8 *dat)
{	   
    u8 mbox;	  
    if(CAN1->TSR&(1<<26))mbox=0;			//邮箱0为空
    else if(CAN1->TSR&(1<<27))mbox=1;	//邮箱1为空
    else if(CAN1->TSR&(1<<28))mbox=2;	//邮箱2为空
    else return 0XFF;					//无空邮箱,无法发送 
    CAN1->sTxMailBox[mbox].TIR=0;		//清除之前的设置
    if(ide==0)	//标准帧
    {
        id&=0x7ff;//取低11位stdid
        id<<=21;		  
    }else		//扩展帧
    {
        id&=0X1FFFFFFF;//取低32位extid
        id<<=3;									   
    }
    CAN1->sTxMailBox[mbox].TIR|=id;		 
    CAN1->sTxMailBox[mbox].TIR|=ide<<2;	  
    CAN1->sTxMailBox[mbox].TIR|=rtr<<1;
    len&=0X0F;//得到低四位
    CAN1->sTxMailBox[mbox].TDTR&=~(0X0000000F);
    CAN1->sTxMailBox[mbox].TDTR|=len;		   //设置DLC.
    //待发送数据存入邮箱.
    CAN1->sTxMailBox[mbox].TDHR=(((u32)dat[7]<<24)|
                                ((u32)dat[6]<<16)|
                                 ((u32)dat[5]<<8)|
                                ((u32)dat[4]));
    CAN1->sTxMailBox[mbox].TDLR=(((u32)dat[3]<<24)|
                                ((u32)dat[2]<<16)|
                                 ((u32)dat[1]<<8)|
                                ((u32)dat[0]));
    CAN1->sTxMailBox[mbox].TIR|=1<<0; //请求发送邮箱数据
    return mbox;
}
//获得发送状态.
//mbox:邮箱编号;
//返回值:发送状态. 0,挂起;0X05,发送失败;0X07,发送成功.
u8 CAN1_Tx_Staus(u8 mbox)
{	
    u8 sta=0;
    switch (mbox)
    {
        case 0: 
            sta |= CAN1->TSR&(1<<0);			//RQCP0
            sta |= CAN1->TSR&(1<<1);			//TXOK0
            sta |=((CAN1->TSR&(1<<26))>>24);	//TME0
            break;
        case 1: 
            sta |= CAN1->TSR&(1<<8)>>8;		//RQCP1
            sta |= CAN1->TSR&(1<<9)>>8;		//TXOK1
            sta |=((CAN1->TSR&(1<<27))>>25);	//TME1	   
            break;
        case 2: 
            sta |= CAN1->TSR&(1<<16)>>16;	//RQCP2
            sta |= CAN1->TSR&(1<<17)>>16;	//TXOK2
            sta |=((CAN1->TSR&(1<<28))>>26);	//TME2
            break;
        default:
            sta=0X05;//邮箱号不对,肯定失败.
        break;
    }
    return sta;
} 
//得到在FIFO0/FIFO1中接收到的报文个数.
//fifox:0/1.FIFO编号;
//返回值:FIFO0/FIFO1中的报文个数.
u8 CAN1_Msg_Pend(u8 fifox)
{
    if(fifox==0)return CAN1->RF0R&0x03; 
    else if(fifox==1)return CAN1->RF1R&0x03; 
    else return 0;
}
//接收数据
//fifox:邮箱号
//id:标准ID(11位)/扩展ID(11位+18位)	    
//ide:0,标准帧;1,扩展帧
//rtr:0,数据帧;1,远程帧
//len:接收到的数据长度(固定为8个字节,在时间触发模式下,有效数据为6个字节)
//dat:数据缓存区
void CAN1_Rx_Msg(u8 fifox,u32 *id,u8 *ide,u8 *rtr,u8 *len,u8 *dat)
{
    *ide=CAN1->sFIFOMailBox[fifox].RIR&0x04;//得到标识符选择位的值  
    if(*ide==0)//标准标识符
    {
        *id=CAN1->sFIFOMailBox[fifox].RIR>>21;
    }else	   //扩展标识符
    {
        *id=CAN1->sFIFOMailBox[fifox].RIR>>3;
    }
    *rtr=CAN1->sFIFOMailBox[fifox].RIR&0x02;	//得到远程发送请求值.
    *len=CAN1->sFIFOMailBox[fifox].RDTR&0x0F;//得到DLC
     //*fmi=(CAN1->sFIFOMailBox[FIFONumber].RDTR>>8)&0xFF;//得到FMI
    //接收数据
    dat[0]=CAN1->sFIFOMailBox[fifox].RDLR&0XFF;
    dat[1]=(CAN1->sFIFOMailBox[fifox].RDLR>>8)&0XFF;
    dat[2]=(CAN1->sFIFOMailBox[fifox].RDLR>>16)&0XFF;
    dat[3]=(CAN1->sFIFOMailBox[fifox].RDLR>>24)&0XFF;    
    dat[4]=CAN1->sFIFOMailBox[fifox].RDHR&0XFF;
    dat[5]=(CAN1->sFIFOMailBox[fifox].RDHR>>8)&0XFF;
    dat[6]=(CAN1->sFIFOMailBox[fifox].RDHR>>16)&0XFF;
    dat[7]=(CAN1->sFIFOMailBox[fifox].RDHR>>24)&0XFF;    
    if(fifox==0)CAN1->RF0R|=0X20;//释放FIFO0邮箱
    else if(fifox==1)CAN1->RF1R|=0X20;//释放FIFO1邮箱	 
}

#if CAN1_RX0_INT_ENABLE	//使能RX0中断
//中断服务函数
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    u8 i;
    u32 id;
    u8 ide,rtr,len;     
    u8 ON_rxbuf[8]={10,12,15,19,24,30,37,0};

    u8 temp_rxbuf[8];
     CAN1_Rx_Msg(0,&id,&ide,&rtr,&len,temp_rxbuf);
    for(i=0;i<=7;i++) //用于解锁CAN控制的起始语句
    {
        if(i==7)
        {	
            CAN_ON_Flag=temp_rxbuf[i];
            break;
        }
        if(temp_rxbuf[i]!=ON_rxbuf[i]) 
            break;
    }
    if(id==0x121&&CAN_ON_Flag)    memcpy(rxbuf,temp_rxbuf,8*sizeof(u8));
}
#endif

//CAN1发送一组数据(固定格式:ID为0X601,标准帧,数据帧)	
//len:数据长度(最大为8)				     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//		 其他,失败;
u8 CAN1_Send_Msg(u8* msg,u8 len)
{	
    u8 mbox;
    u16 i=0;	  	 						       
  mbox=CAN1_Tx_Msg(0X601,0,0,len,msg);   
    while((CAN1_Tx_Staus(mbox)!=0X07)&&(i<0XFFF))i++;//等待发送结束
    if(i>=0XFFF)return 1;							//发送失败?
    return 0;										//发送成功;
}
//CAN1口接收数据查询
//buf:数据缓存区;	 
//返回值:0,无数据被收到;
//其他,接收的数据长度;
u8 CAN1_Receive_Msg(u8 *buf)
{		   		   
    u32 id;
    u8 ide,rtr,len; 
    if(CAN1_Msg_Pend(0)==0)return 0;			//没有接收到数据,直接退出 	 
      CAN1_Rx_Msg(0,&id,&ide,&rtr,&len,buf); 	//读取数据
    if(id!=0x12||ide!=0||rtr!=0)len=0;		//接收错误	   
    return len;	
}


u8 CAN1_Send_MsgTEST(u8* msg,u8 len)
{	
    u8 mbox;
    u16 i=0;	  	 						       
    mbox=CAN1_Tx_Msg(0X701,0,0,len,msg);   
    while((CAN1_Tx_Staus(mbox)!=0X07)&&(i<0XFFF))i++;//等待发送结束
    if(i>=0XFFF)return 1;							//发送失败?
    return 0;										//发送成功;
}

/* 
*  函数名称：CAN1_Send_Numm(u32 id,u8* msg)
*  入口参数：id  ID号，msg  被输送数组的名称
*  函数输出： 1，发送失败；   0，发送成功  
*  函数功能：给给定的id发送一个数组的命令
*/

u8 CAN1_Send_Num(u32 id,u8* msg)
{
    u8 mbox;
    u16 i=0;	  	 						       
  mbox=CAN1_Tx_Msg(id,0,0,8,msg);   
    while((CAN1_Tx_Staus(mbox)!=0X07)&&(i<0XFFF))i++;//等待发送结束
    if(i>=0XFFF)return 1;							//发送失败?
    return 0;
}
 
void CAN1_SEND(void) 
{
        u8 Direction_A,Direction_B,Direction_C,Direction_D;
          u16 Temp_Pitch,Temp_Roll,Temp_Yaw,Temp_GZ;
         static u8 Flag_Send;
               if(Encoder_A>0) Direction_A=0;
        else if(Encoder_A<0) Direction_A=2;
          else                 Direction_A=1;
    
                 if(Encoder_B>0) Direction_B=0;
        else if(Encoder_B<0) Direction_B=2;
          else                 Direction_B=1;     
    
                 if(Encoder_C>0) Direction_C=0;
        else if(Encoder_C<0) Direction_C=2;
          else                 Direction_C=1;
    
                   if(Encoder_D>0) Direction_D=0;
        else if(Encoder_D<0) Direction_D=2;
          else                 Direction_D=1;
    
          Temp_Pitch=Pitch*100+30000;
          Temp_Roll=  Roll*100+30000;
          Temp_Yaw=    Yaw*100+30000;
          Temp_GZ=gyro[2]+32768;
          Flag_Send=!Flag_Send;
        if(Flag_Send==0)//分两次发送全部的数据，第一次
                {	
              txbuf[0]=Temp_Pitch>>8;
                txbuf[1]=Temp_Pitch&0x00ff;		
                txbuf[2]=Temp_Roll>>8;
                txbuf[3]=Temp_Roll&0x00ff;
                txbuf[4]=Temp_Yaw>>8;
                txbuf[5]=Temp_Yaw&0x00ff;
                txbuf[6]=Temp_GZ>>8;	
              txbuf[7]=Temp_GZ&0x00ff;	
                CAN1_Send_Num(0x100,txbuf);	
                }
                else//第二次
                {	
        txbuf[0]=abs(Encoder_A);  
                txbuf[1]=Direction_A; 
                txbuf[2]=abs(Encoder_B);  
                txbuf[3]=Direction_B;
                txbuf[4]=abs(Encoder_C);
                txbuf[5]=Direction_C;
                txbuf[6]=abs(Encoder_D);
                txbuf[7]=Direction_D;
          CAN1_Send_Num(0x101,txbuf);
          }
}



