#ifndef __REFEREESYSTEM_H
#define __REFEREESYSTEM_H

#include<stdint.h>

#define    ROBOTSTATE   0x0001
#define    ROBOTHURT    0x0002
#define    SHOOTDATA    0x0003
#define    POWERHEAT    0x0004
#define    RFIDDETECT   0x0005
#define    GAMERESULT   0x0006
#define    BUFF         0x0007
#define    RESERVED     0x0100


typedef __packed struct{
    uint8_t validFlag;
    float x;
    float y;
    float z;
    float yaw;
}position_t;

//Cmd_ID = 0x0001
typedef __packed struct{
    uint16_t stageRemainTime;   //remain time: xx s 
    uint8_t gameProcess;        //0: it hasn't start    1:ready     2:self-inspect      3:count down in 5s      4:game      5:result
    uint8_t robotLevel;
    uint16_t remainHP;
    uint16_t maxHP;
    position_t position;
}extGameRobotState_t;

//Cmd_ID = 0x0002
typedef __packed struct{
    uint8_t armorType;
    uint8_t hurtType;   
}extRobotHurt_t;

//Cmd_ID = 0x0003
typedef __packed struct{
    uint8_t bulletType;         //1:17 mm bullet        2:42 mm bullet
    uint8_t bulletFreq;
    float bulletSpeed;
    float reserved;
}extShootData_t;

//Cmd_ID = 0x0004
typedef __packed struct{
    float chassisVolt;
    float chassisCurrent;
    float chassisPower;
    float chassisPowerBuffer;
    uint16_t shooterHeat0;      //the heat quantity of 17 mm shooter
    uint16_t shooterHeat1;      //the heat quantity of 42 mm shooter
}extPowerHeatData_t;

//Cmd_ID = 0x0005
typedef __packed struct{
    uint8_t cardType;
    uint8_t cardIndex;
}extRfidDetect_t;

//Cmd_ID = 0x0006
typedef __packed struct{
    uint8_t winner;         //0:the match endded in a draw      1:Red wins      2:Blue wins
}extGameResult_t;

//Cmd_ID = 0x0007
typedef __packed struct{
    uint8_t buffType;
    uint8_t buffAddition;
}extGetBuff_t;

//Cmd_ID = 0x01000
typedef __packed struct{
    float data1;
    float data2;
    float data3;
    uint8_t mask;
}extUserData_t;

typedef __packed struct{
    extGameRobotState_t extGameRobotState;
    extRobotHurt_t	extRobotHurt;
    extShootData_t	extShootData;
    extPowerHeatData_t extPowerHeatData;
    extRfidDetect_t extRfidDetect;
    extGameResult_t extGameResult;
    extGetBuff_t extGetBuff;
    extUserData_t userData;
}RefereeSystem_t;

#endif
