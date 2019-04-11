#include <iostream>
#include "main.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Empty.h"
#include "std_msgs/UInt8.h"
#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"
#include "ros/ros.h"
#include "robot_protocol_msgs/RobotControl.h"
#include "string.h"
#include <string>
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
ros::Publisher pub;
ros::Subscriber sub,sub1,sub2,sub3,sub4;
ros::ServiceServer service,service1,service2,service3;
Robot_Control_t RM_Control={0.0f,0.0f,0.0f,0.0f,0.0f};
std_msgs::UInt8MultiArray m;
//callback function
void RobotControl(const robot_protocol_msgs::RobotControl &msg){
	unsigned char _tmp[sizeof(Robot_Control_t)+5]={0};
    m.layout.dim[0].label = "control";
    m.layout.dim[0].size =sizeof(Robot_Control_t)+5;
    m.data.resize(sizeof(Robot_Control_t)+5);
	m.data[sizeof(Robot_Control_t)+4]=0;
    _tmp[0]=0xaa;_tmp[1]=0xaf;_tmp[2]=0x25;_tmp[3]=sizeof(Robot_Control_t);
    RM_Control.Chassis_speed_x=msg.Chassis_speed_x;
    RM_Control.Chassis_speed_y=msg.Chassis_speed_y;
    RM_Control.Chassis_speed_rotate=msg.Chassis_speed_rotate;
    RM_Control.Gimbal_delta_pitch=msg.Gimbal_delta_pitch;
    RM_Control.Gimbal_delta_yaw=msg.Gimbal_delta_yaw;
	RM_Control.ControlValid = msg.ControlValid;
	memcpy(_tmp+4,(unsigned char*)&(RM_Control),sizeof(RM_Control));
    for (int i=0;i<sizeof(Robot_Control_t)+4;i++)_tmp[sizeof(Robot_Control_t)+4]+=_tmp[i];
    for (int i=0;i<sizeof(Robot_Control_t)+5;i++)m.data[i]=_tmp[i];
	pub.publish(m);
}

int main(int argc,char **argv){

	ros::init(argc, argv, "robot_pack_info");
    ros::NodeHandle n;
    m.layout.dim.push_back(std_msgs::MultiArrayDimension());
    m.layout.dim[0].stride = 1;
    pub = n.advertise<std_msgs::UInt8MultiArray>("array_write", 1000);
    sub = n.subscribe("RobotControl", 1, &RobotControl);
    
    
    while(ros::ok())
    ros::spin();

	return 0;
}
