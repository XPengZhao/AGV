#ifndef _PROTOCOL_H_
#define  _PROTOCOL_H_
#include "stm32f4xx.h"
#include "can.h"
#define IDLE_RC_VALUE 10
void BasicProtocolAnalysis(u8 const *data_buf,int _len);
void RcProtocolAnalysis(u8 *_item,int _len);
void RefereeProtocolAnalysis(u8 *_item,int _len);
void CanProtocolAnalysis(CanRxMsg * msg);

extern volatile Encoder CM1Encoder;
extern volatile Encoder CM2Encoder;
extern volatile Encoder CM3Encoder;
extern volatile Encoder CM4Encoder;
extern volatile Encoder GMYawEncoder;
extern volatile Encoder GMPitchEncoder;
extern volatile Encoder DialingEncoder;
typedef __packed struct
{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int8_t s1;
	int8_t s2;
}Remote;
typedef __packed struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t last_press_l;
	uint8_t last_press_r;
	uint8_t press_l;
	uint8_t press_r;
}Mouse;	
typedef	__packed struct
{
	uint16_t v;
}Key;

typedef __packed struct
{
	Remote rc;
	Mouse mouse;
	Key key;
}RC_Ctrl_t;
extern RC_Ctrl_t RC_CtrlData;
extern u8 checkdata_to_send,checksum_to_send,send_check;
extern u8 send_pid1,send_pid2,send_pid3;
#define KEY_W 0x01
#define KEY_S 0x02
#define KEY_A 0x04
#define KEY_D 0x08
#define KEY_SHIFT 0x10
#define KEY_CTRL 0x20
#define KEY_Q 0x40
#define KEY_E 0x80
#define KEY_R 0x100
#define KEY_F 0x200
#define KEY_G 0x400
#define KEY_Z 0x800
#define KEY_X 0x1000
#define KEY_C 0x2000
#define KEY_V 0x4000
#define KEY_B 0x8000


enum
{
	WHEEL_ON = 0,
	WHEEL_OFF
};
extern u8 WHEEL_STATE;

#define RC_KEY_STOP (2)
#define RC_KEY_RCMODE (3)
#define RC_KEY_KEYBOARD (1)

#endif
