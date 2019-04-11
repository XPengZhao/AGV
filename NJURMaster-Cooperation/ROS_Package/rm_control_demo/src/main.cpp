#include <iostream>
#include "string.h"
#include "robot_protocol_msgs/RobotControl.h"
#include "robot_protocol_msgs/RobotError.h"
#include "robot_protocol_msgs/RobotRC.h"
#include "robot_protocol_msgs/RobotOdometer.h"
#include "robot_protocol_msgs/RobotIMU.h"
#include "robot_protocol_msgs/RobotGimbal.h"
#include "robot_protocol_msgs/RobotMotor.h"
#include "ros/ros.h"
#include "key.h"
#include "main.h"
ros::Publisher Control_info_pub;
ros::Subscriber IMU_sub,
                Motor_sub,
                Error_sub,
                RC_sub,
                Gimbal_sub;
robot_protocol_msgs::RobotControl Control_msg;
#define PRINT_IMU_DATA 1
#define PRINT_MOTOR_DATA 0
#define PRINT_ERROR_DATA 0
#define PRINT_RC_DATA 0
#define PRINT_GIMBAL_DATA 0
void IMU_callback(const robot_protocol_msgs::RobotIMU &msg)
{
#if PRINT_IMU_DATA
    std::cout<<"Pitch is "
                <<msg.pitch
                <<", roll is "
                <<msg.roll
                <<", yaw is "
                <<msg.yaw
                <<std::endl;
#endif
}

void Motor_callback(const robot_protocol_msgs::RobotMotor &msg)
{
#if PRINT_MOTOR_DATA
    std::cout<<"ChassisMotor1 is "
                <<msg.Motor1
                <<" ChassisMotor2 is "
                <<msg.Motor2
                <<" ChassisMotor3 is "
                <<msg.Motor3
                <<" ChassisMotor4 is "
                <<msg.Motor4
                <<" GimbalMotor Pitch is  "
                <<msg.Motor5
                <<" GimbalMotor Yaw is "
                <<msg.Motor6
                <<std::endl;

#endif

}

void Error_callback(const robot_protocol_msgs::RobotError &msg)
{
#if PRINT_ERROR_DATA
    
#endif
}

void RC_callback(const robot_protocol_msgs::RobotRC &msg)
{
#if PRINT_RC_DATA

#endif
}

void Gimbal_callback(const robot_protocol_msgs::RobotGimbal &msg)
{

}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"robot_control_demo");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);
    Control_info_pub = n.advertise<robot_protocol_msgs::RobotControl>("RobotControl",1);
    IMU_sub     =   n.subscribe("RobotIMU",1,&IMU_callback);
    Motor_sub   =   n.subscribe("RobotMotor",1,&Motor_callback);
    Error_sub   =   n.subscribe("RobotError",1,&Error_callback);
    RC_sub      =   n.subscribe("RobotRC",1,&RC_callback);
    Gimbal_sub  =   n.subscribe("RobotGimbal",1,&Gimbal_callback);
    while (ros::ok())
    {
	Control_msg.ControlValid=0x00;
        Control_msg.Chassis_speed_x=100.0f;
//	Control_msg.ControlValid|=CHASSIS_X_VALID;
        Control_msg.Chassis_speed_y=200.0f;
//	Control_msg.ControlValid|=CHASSIS_Y_VALID;
        Control_msg.Chassis_speed_rotate=300.0f;
//	Control_msg.ControlValid|=CHASSIS_R_VALID;
        Control_msg.Gimbal_delta_pitch=2.0f;
	Control_msg.ControlValid|=GIMBAL_PITCH_VALID;
        Control_msg.Gimbal_delta_yaw=1.0f;
	Control_msg.ControlValid|=GIMBAL_YAW_VALID;
        Control_info_pub.publish(Control_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

}


