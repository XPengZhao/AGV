/*************************************************************************
    > File Name: main.cpp
    > Author: FanMing Luo
    > Mail: 151190065@smail.nju.edu.cn 
    > Created Time: 2017年01月12日 星期四 20时43分36秒
 ************************************************************************/

#include <iostream>
#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include <queue>
#include <assert.h>
#include "robot_protocol_msgs/RobotMotor.h"
#include "robot_protocol_msgs/RobotError.h"
#include "robot_protocol_msgs/RobotRC.h"
#include "robot_protocol_msgs/RobotOdometer.h"
#include "robot_protocol_msgs/RobotIMU.h"
#include "robot_protocol_msgs/RobotGimbal.h"

#include "main.h"

ros::Publisher pub_Motor;
ros::Publisher pub_Error;
ros::Publisher pub_RC;
ros::Publisher pub_Odometer;
ros::Publisher pub_IMU;
ros::Publisher pub_Gimbal;

robot_protocol_msgs::RobotMotor msg_Motor;
robot_protocol_msgs::RobotError msg_Error;
robot_protocol_msgs::RobotRC msg_RC;
robot_protocol_msgs::RobotOdometer msg_Odometer;
robot_protocol_msgs::RobotIMU msg_IMU;
robot_protocol_msgs::RobotGimbal msg_Gimbal;

 RC_Ctrl_t RC;
 PC_Send_Motor_t Motor;
 PC_Send_IMU_t IMU;

void IMU_Data_Process(unsigned char * _data, unsigned char _len)
{
	memcpy((unsigned char *)&IMU,_data,sizeof(IMU));
	msg_IMU.pitch=IMU.Pitch;
	msg_IMU.yaw=IMU.Yaw;
	msg_IMU.roll=IMU.Roll;
    msg_Gimbal.Pitch=IMU.Gimbal_Pitch;
    msg_Gimbal.Yaw=IMU.Gimbal_Yaw;
	uint32_t tmp=IMU.state;
    msg_Error.RC=1;
//    std::cout<<tmp<<std::endl;
	if (tmp&LOST_ERROR_RC) {msg_Error.RC=1;} else {msg_Error.RC=0; }	
    if (tmp&LOST_ERROR_IMU) {msg_Error.IMU=1;} else {msg_Error.IMU=0; }
	if (tmp& LOST_ERROR_ZGYRO) {msg_Error.ZGYRO=1;} else {msg_Error.ZGYRO=0; }
	if (tmp&LOST_ERROR_MOTOR1) {msg_Error.MOTOR1=1;} else {msg_Error.MOTOR1=0; }
	if (tmp&LOST_ERROR_MOTOR2) {msg_Error.MOTOR2=1;} else {msg_Error.MOTOR2=0; }
	if (tmp&LOST_ERROR_MOTOR3) {msg_Error.MOTOR3=1;} else {msg_Error.MOTOR3=0; }
	if (tmp&LOST_ERROR_MOTOR4) {msg_Error.MOTOR4=1;} else {msg_Error.MOTOR4=0; }
	if (tmp&LOST_ERROR_MOTOR5) {msg_Error.MOTOR5=1;} else {msg_Error.MOTOR5=0; }
	if (tmp&LOST_ERROR_MOTOR6) {msg_Error.MOTOR6=1;} else {msg_Error.MOTOR6=0; }
	if (tmp&LOST_ERROR_NOCALI) {msg_Error.NOCALI=1;} else {msg_Error.NOCALI=0; }
	if (tmp&LOST_ERROR_TIMEOUT) {msg_Error.TIMEOUT=1;} else {msg_Error.TIMEOUT=0; }	
	if (tmp&LOST_ERROR_PC) {msg_Error.PC=1;} else {msg_Error.PC=0; }
    if (tmp&LOST_ERROR_MOTOR7) {msg_Error.MOTOR7=1;} else {msg_Error.MOTOR7=0; }
    pub_Gimbal.publish(msg_Gimbal);	
	pub_IMU.publish(msg_IMU);
	pub_Error.publish(msg_Error);
}

void Motor_Data_Process(unsigned char *_data, unsigned char _len)
{
	memcpy((unsigned char *)&Motor,_data,sizeof( PC_Send_Motor_t));
	msg_Motor.Motor1=Motor.motor1;
	msg_Motor.Motor2=Motor.motor2;
	msg_Motor.Motor3=Motor.motor3;
	msg_Motor.Motor4=Motor.motor4;
	msg_Motor.Motor5=Motor.motor5;
	msg_Motor.Motor6=Motor.motor6;
	msg_Motor.Motor7=Motor.motor7;
	msg_Motor.Motor8=Motor.motor8;	
	pub_Motor.publish(msg_Motor);
}

void RC_Data_Process(unsigned char *_data, unsigned char _len)
{
	memcpy((unsigned char *)&RC,_data,sizeof( RC_Ctrl_t));
	msg_RC.RC_ch0=RC.rc.ch0;
	msg_RC.RC_ch1=RC.rc.ch1;
	msg_RC.RC_ch2=RC.rc.ch2;
	msg_RC.RC_ch3=RC.rc.ch3;
	msg_RC.RC_key1=RC.rc.s1;
	msg_RC.RC_key2=RC.rc.s2;
	msg_RC.Key=RC.key.v;
	msg_RC.mouse_left_click=RC.mouse.press_l;
	msg_RC.mouse_right_click=RC.mouse.press_r;
	msg_RC.mouse_x=RC.mouse.x;
	msg_RC.mouse_y=RC.mouse.y;
	msg_RC.mouse_z=RC.mouse.z;

	pub_RC.publish(msg_RC);
	
}

std::queue <unsigned char> all;
unsigned char frame_type=0;
unsigned char frame_length=0;
void info (const std_msgs::UInt8MultiArray &msg){
for (int i=0;i<msg.layout.dim[0].size ;i++)all.push(msg.data[i]);
static int step=0;
while (all.size()>0){
	switch (step){
		case 0:{
			if (all.front()==170){step=1;}
			else{step=0;}
			break;
		}
		case 1:{
			if (all.front()==170){step=2;}
			else{step=0;}

			break;
		}
		case 2:{
			frame_type=all.front();
			step=3;
			break;
		}
		case 3:{
			frame_length=all.front();
			step=4;
			break;
		}
		case 4:{
			if (all.size()<frame_length+1) return;
			else {
				unsigned char *data_buf_tmp=(unsigned char *)malloc(frame_length+1 * sizeof(unsigned char));
				for (int i=0;i<frame_length+1;i++)
				{
					data_buf_tmp[i]=all.front();
					all.pop();
				}
				unsigned char _sum=0;
				_sum+=((unsigned char )170);
				_sum+=((unsigned char )170);
				_sum+=(frame_type);
				_sum+=(frame_length);
				for (int i=0;i<frame_length;i++)
				{
					_sum+=data_buf_tmp[i];
				}
                if (_sum==data_buf_tmp[frame_length])
				{
					switch(frame_type)
					{
						case 0x01:				//IMU
						{
                            
							IMU_Data_Process(data_buf_tmp,frame_length);
							break;
						}
						case 0x02:				//RC
						{
							RC_Data_Process(data_buf_tmp,frame_length);
							break;
						}
						case 0x03:				//MOTOR
						{
							Motor_Data_Process(data_buf_tmp,frame_length);
						}
						
						default:
							break;	
					}
					step = 0 ;
					continue;
				}
				else 
				{
					step = 0;
					continue;
				}
			}
		}

	}
	all.pop();

}
}


int main(int argc, char **argv){
	ros::init(argc, argv, "robot_unpack_info");
    ros::NodeHandle n;
    pub_IMU = n.advertise<robot_protocol_msgs::RobotIMU>("RobotIMU", 1);
	pub_Motor= n.advertise<robot_protocol_msgs::RobotMotor>("RobotMotor", 1);
	pub_Error= n.advertise<robot_protocol_msgs::RobotError>("RobotError", 1);
	pub_RC= n.advertise<robot_protocol_msgs::RobotRC>("RobotRC", 1);
	pub_Gimbal= n.advertise<robot_protocol_msgs::RobotGimbal>("RobotGimbal", 1);
	pub_Odometer= n.advertise<robot_protocol_msgs::RobotOdometer>("RobotOdometer", 1);
	ros::Subscriber sub = n.subscribe("uartRE", 1000, &info); 
	ros::spin();

	return 0;
}
