#ifndef _KEY_H_
#define _KEY_H_
#include <iostream>
#include "ros/ros.h"
char printKey(uint32_t);

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

#endif
