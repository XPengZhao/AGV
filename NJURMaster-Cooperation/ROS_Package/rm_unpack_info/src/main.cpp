/*************************************************************************
    > File Name: main.cpp
    > Author: FanMing Luo
    > Mail: 151190065@smail.nju.edu.cn 
    > Created Time: 2017年01月12日 星期四 20时43分36秒
 ************************************************************************/
#include "infantry.h"
#include <iostream>
#include "ros/ros.h"
#include "stdlib.h"
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
#include "robot_protocol_msgs/RobotControl.h"
#include "robot_protocol_msgs/RmRc.h"

#include "main.h"
ros::Publisher pub_Gimbal, pub_RC;
robot_protocol_msgs::RobotGimbal msg_Gimbal;
robot_protocol_msgs::RmRc msg_RC;
ros::Publisher pub;
std_msgs::UInt8MultiArray m;
UnpackStep unpack_step_e_;
unsigned char protocol_packet_[PACK_MAX_SIZE];
int index_, byte_, data_length_;
GimbalControl gimbal_control_data_;
ChassisControl chassis_control_data_;
FrameHeader computer_frame_header_;
GameInfo game_information_;
HurtData robot_hurt_data_;
ShootData real_shoot_data_;
RfidData rfid_data_;
RealPowerData real_power_data_;
GameResult game_result_data_;
ChassisInfo chassis_information_;
GimbalInfo gimbal_information_;
ShootInfo shoot_task_data_;
InfantryError global_error_data_;
GameBuff get_buff_data_;
ServerToUser student_download_data_;
ConfigMessage config_response_data_;
CalibrateResponse cali_response_data_;
RcInfo rc_info_data_;
VersionInfo version_info_data_;
ShootControl shoot_task_control_;
void info (const std_msgs::UInt8MultiArray &msg){
    int read_len_ = msg.layout.dim[0].size;
    int static_len_ = msg.layout.dim[0].size;
    unsigned char * rx_buf_ = (unsigned char *)malloc(read_len_ * sizeof ( unsigned char ));
    for (int i = 0;i < read_len_; i++)
    {
        rx_buf_[i] = msg.data[i];
    }
    if (read_len_ > 0) {
        while (read_len_--) {
	    
            byte_ = rx_buf_[static_len_ - read_len_];
            switch (unpack_step_e_) {
                case STEP_HEADER_SOF: {
                    if (byte_ == UP_REG_ID) {
                        protocol_packet_[index_++] = byte_;
                        unpack_step_e_ = STEP_LENGTH_LOW;
                    } else {
                        index_ = 0;
                    }
                }
                    break;
                case STEP_LENGTH_LOW: {
                    data_length_ = byte_;
                    protocol_packet_[index_++] = byte_;
                    unpack_step_e_ = STEP_LENGTH_HIGH;
                }
                    break;
                case STEP_LENGTH_HIGH: {
                    data_length_ |= (byte_ << 8);
                    protocol_packet_[index_++] = byte_;
                    if (data_length_ < (PROTOCAL_FRAME_MAX_SIZE - HEADER_LEN - CMD_LEN - CRC_LEN)) {
                        unpack_step_e_ = STEP_FRAME_SEQ;
                    } else {
                        LOG_WARNING << "Data length too big, it's" << data_length_<<"\n";
                        unpack_step_e_ = STEP_HEADER_SOF;
                        index_ = 0;
                    }
                }
                    break;
                case STEP_FRAME_SEQ: {
                    protocol_packet_[index_++] = byte_;
                    unpack_step_e_ = STEP_HEADER_CRC8;
                }
                    break;
                case STEP_HEADER_CRC8: {
                    protocol_packet_[index_++] = byte_;
                    bool crc8_result = VerifyCrcOctCheckSum(protocol_packet_, HEADER_LEN);
                    if (!crc8_result) {
                        LOG_WARNING << "CRC 8 error";
                    }
                    if ((index_ == HEADER_LEN) && crc8_result) {
                        if (index_ < HEADER_LEN) {
                            LOG_WARNING << "CRC 8 index less.";
                        }
                        unpack_step_e_ = STEP_DATA_CRC16;
                    } else {
                        unpack_step_e_ = STEP_HEADER_SOF;
                        index_ = 0;
                    }
                }
                    break;
                case STEP_DATA_CRC16: {
                    if (index_ < (HEADER_LEN + CMD_LEN + data_length_ + CRC_LEN)) {
                        protocol_packet_[index_++] = byte_;
                    } else if (index_ > (HEADER_LEN + CMD_LEN + data_length_ + CRC_LEN)) {
                        LOG_WARNING << "Index Beyond";
                    }
                    if (index_ == (HEADER_LEN + CMD_LEN + data_length_ + CRC_LEN)) {
                        unpack_step_e_ = STEP_HEADER_SOF;
                        index_ = 0;
                        if (VerifyCrcHexCheckSum(protocol_packet_, HEADER_LEN + CMD_LEN + data_length_ + CRC_LEN)) {
                            DataHandle();
                        } else {
                            LOG_WARNING << "CRC16 error";
                        }
                    }
                }
                    break;
                default: {
                    LOG_WARNING << "Unpack not well";
                    unpack_step_e_ = STEP_HEADER_SOF;
                    index_ = 0;
                }
                    break;
            }
        }
    }
    
    free(rx_buf_);
}

int debug_flag_ = 1;
void DataHandle() {
    auto *p_header = (FrameHeader *) protocol_packet_;
    uint16_t data_length = p_header->data_length;
    uint16_t cmd_id = *(uint16_t *) (protocol_packet_ + HEADER_LEN);
    uint8_t *data_addr = protocol_packet_ + HEADER_LEN + CMD_LEN;
    switch (cmd_id) {
        case GAME_INFO_ID: memcpy(&game_information_, data_addr, data_length);
            break;
        case REAL_BLOOD_DATA_ID: memcpy(&robot_hurt_data_, data_addr, data_length);
            
            break;
        case REAL_SHOOT_DATA_ID: memcpy(&real_shoot_data_, data_addr, data_length);
            break;
        case REAL_POWER_DATA_ID:memcpy(&real_power_data_, data_addr, data_length);
            //std::cout<< real_power_data_.chassis_power<<"\n";
            break;
        case REAL_RFID_DATA_ID: memcpy(&rfid_data_, data_addr, data_length);
            break;
        case GAME_RESULT_ID: memcpy(&game_result_data_, data_addr, data_length);
            break;
        case GAIN_BUFF_ID: memcpy(&get_buff_data_, data_addr, data_length);
            break;
        case SERVER_TO_USER_ID: memcpy(&student_download_data_, data_addr, data_length);
            break;
        case CHASSIS_DATA_ID: {
            
        }
            break;
        case GIMBAL_DATA_ID: memcpy(&gimbal_information_, data_addr, data_length);
            msg_Gimbal.Pitch = gimbal_information_.pit_relative_angle;
            msg_Gimbal.Yaw = gimbal_information_.yaw_relative_angle;
            pub_Gimbal.publish(msg_Gimbal);
            break;
        case SHOOT_TASK_DATA_ID: memcpy(&shoot_task_data_, data_addr, data_length);
            
            break;
        case INFANTRY_ERR_ID: memcpy(&global_error_data_, data_addr, data_length);
            
            break;
        case CONFIG_RESPONSE_ID: memcpy(&config_response_data_, data_addr, data_length);
            
            break;
        case CALI_RESPONSE_ID: memcpy(&cali_response_data_, data_addr, data_length);
            
            break;
        case REMOTE_CTRL_INFO_ID: memcpy(&rc_info_data_, data_addr, data_length);
            msg_RC.ch1 = rc_info_data_.ch1;msg_RC.ch2 = rc_info_data_.ch2;
            msg_RC.ch3 = rc_info_data_.ch3;msg_RC.ch4 = rc_info_data_.ch4;
            msg_RC.sw1 = rc_info_data_.sw1;msg_RC.sw2 = rc_info_data_.sw2;
            msg_RC.x = rc_info_data_.Mouse.x;msg_RC.y = rc_info_data_.Mouse.y;
            msg_RC.z = rc_info_data_.Mouse.z;msg_RC.l = rc_info_data_.Mouse.l;
            msg_RC.r = rc_info_data_.Mouse.r;
           /* msg_RC.W = rc_info_data_.Keyboard.Bit.W;msg_RC.S = rc_info_data_.Keyboard.Bit.S;
            msg_RC.A = rc_info_data_.Keyboard.Bit.A;msg_RC.D = rc_info_data_.Keyboard.Bit.D;
            msg_RC.SHIFT = rc_info_data_.Keyboard.Bit.SHIFT;msg_RC.CTRL = rc_info_data_.Keyboard.Bit.CTRL;
            msg_RC.Q = rc_info_data_.Keyboard.Bit.Q;msg_RC.E = rc_info_data_.Keyboard.Bit.E;
            msg_RC.R = rc_info_data_.Keyboard.Bit.R;msg_RC.F = rc_info_data_.Keyboard.Bit.F;
            msg_RC.G = rc_info_data_.Keyboard.Bit.G;msg_RC.Z = rc_info_data_.Keyboard.Bit.Z;
            msg_RC.X = rc_info_data_.Keyboard.Bit.X;msg_RC.C = rc_info_data_.Keyboard.Bit.C;
            msg_RC.V = rc_info_data_.Keyboard.Bit.V;msg_RC.B = rc_info_data_.Keyboard.Bit.B;
            */
            msg_RC.keyboard = rc_info_data_.Keyboard.key_code;
            pub_RC.publish(msg_RC);
            break;
        case BOTTOM_VERSION_ID: memcpy(&version_info_data_, data_addr, data_length);
            
            break;
        default:
            break;
    }
}

void SendDataHandle(uint16_t cmd_id,
                                   uint8_t *topack_data,
                                   uint8_t *packed_data,
                                   uint16_t len
                                   ) {
    FrameHeader *p_header = (FrameHeader *) packed_data;
    p_header->sof = UP_REG_ID;
    p_header->data_length = len;
    memcpy(packed_data + HEADER_LEN, (uint8_t *) &cmd_id, CMD_LEN);
    AppendCrcOctCheckSum(packed_data, HEADER_LEN);
    memcpy(packed_data + HEADER_LEN + CMD_LEN, topack_data, len);
    AppendCrcHexCheckSum(packed_data, HEADER_LEN + CMD_LEN + CRC_LEN + len);
}


unsigned char pack[PACK_MAX_SIZE];
void RobotControl(const robot_protocol_msgs::RobotControl &msg){
    GimbalControl gimbal_control_data;
    gimbal_control_data.pit_ref = msg.Gimbal_delta_pitch;
    gimbal_control_data.yaw_ref = msg.Gimbal_delta_yaw;
    gimbal_control_data.visual_valid = 1;
    
    int length = sizeof(GimbalControl), total_length = length + HEADER_LEN + CMD_LEN + CRC_LEN;
    std::cout<<"Sending byte "<<total_length<<std::endl;
    SendDataHandle(GIMBAL_CTRL_ID, (uint8_t *) &gimbal_control_data, pack, length);

    m.layout.dim[0].label = "control";
    m.layout.dim[0].size =total_length;
    m.data.resize(total_length);
    
    for (int i = 0; i < total_length; i++)
    {
        m.data[i]=pack[i];
    }
    
    pub.publish(m);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "rm_unpack_info");
    ros::NodeHandle n;
    m.layout.dim.push_back(std_msgs::MultiArrayDimension());
    m.layout.dim[0].stride = 1;
	ros::Subscriber sub = n.subscribe("uartRE", 1000, &info);
    ros::Subscriber sub2 = n.subscribe("RobotControl", 1, &RobotControl);
    pub = n.advertise<std_msgs::UInt8MultiArray>("array_write", 1000);
    pub_Gimbal= n.advertise<robot_protocol_msgs::RobotGimbal>("RobotGimbal", 1);
    pub_RC =n.advertise<robot_protocol_msgs::RmRc>("RMRC", 1);
	ros::spin();

	return 0;
}




uint8_t GetCrcOctCheckSum(uint8_t *message, uint32_t length, uint8_t crc) {
    uint8_t index;
    while (length--) {
        index = crc ^ (*message++);
        crc = kCrcOctTable[index];
    }
    return (crc);
}

bool VerifyCrcOctCheckSum(uint8_t *message, uint16_t length) {
    uint8_t expected = 0;
    if ((message == 0) || (length <= 2)) {
        LOG_WARNING << "Verify CRC8 false";
        return false;
    }
    expected = GetCrcOctCheckSum(message, length - 1, kCrc8);
    return (expected == message[length - 1]);
}

void AppendCrcOctCheckSum(uint8_t *message, uint16_t length) {
    uint8_t crc = 0;
    if ((message == 0) || (length <= 2)) {
        LOG_WARNING << "Append CRC8 NULL";
        return;
    };
    crc = GetCrcOctCheckSum(message, length - 1, kCrc8);
    message[length - 1] = crc;
}

uint16_t GetCrcHexCheckSum(uint8_t *message, uint32_t length, uint16_t crc) {
    uint8_t data;
    if (message == NULL) {
        return 0xFFFF;
    }
    while (length--) {
        data = *message++;
        (crc) = ((uint16_t) (crc) >> 8) ^ kCrcTable[((uint16_t) (crc) ^ (uint16_t) (data)) & 0x00ff];
    }
    return crc;
}

bool VerifyCrcHexCheckSum(uint8_t *message, uint32_t length) {
    uint16_t expected = 0;
    if ((message == NULL) || (length <= 2)) {
        LOG_WARNING << "Verify CRC16 bad";
        return false;
    }
    expected = GetCrcHexCheckSum(message, length - 2, kCrc);
    return ((expected & 0xff) == message[length - 2] && ((expected >> 8) & 0xff) == message[length - 1]);
}

void AppendCrcHexCheckSum(uint8_t *message, uint32_t length) {
    uint16_t crc = 0;
    if ((message == NULL) || (length <= 2)) {
        LOG_WARNING << "Append CRC 16 NULL";
        return;
    }
    crc = GetCrcHexCheckSum(message, length - 2, kCrc);
    message[length - 2] = (uint8_t) (crc & 0x00ff);
    message[length - 1] = (uint8_t) ((crc >> 8) & 0x00ff);
}
