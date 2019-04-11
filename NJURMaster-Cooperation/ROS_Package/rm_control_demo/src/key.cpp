#include "key.h"
#include <iostream>
#include "ros/ros.h"

char printKey(uint32_t  _temp){ 
	if (_temp==KEY_W)return 'W';
	else if(_temp==KEY_A)return 'A';
	else if(_temp==KEY_S)return 'S';
	else if(_temp==KEY_D)return 'D';
	else if(_temp==KEY_F)return 'F';
	else if(_temp==KEY_G)return 'G';
	else if(_temp==KEY_Z)return 'Z';
	else if(_temp==KEY_X)return 'X';
	else if(_temp==KEY_C)return 'C';
	else if(_temp==KEY_V)return 'V';
	else if(_temp==KEY_B)return 'B';
	else if(_temp==KEY_Q)return 'Q';
	else if(_temp==KEY_E)return 'E';
	else if(_temp==KEY_R)return 'R';
/*	else if(_temp==KEY_CTRL)printf("CTRL\n");
	else if(_temp==KEY_SHIFT)printf("SHIFT\n");
	else if(_temp==(KEY_CTRL|KEY_A))printf("CTRL A\n");
	else if(_temp==(KEY_CTRL|KEY_S))printf("CTRL S\n");
	else if(_temp==(KEY_CTRL|KEY_D))printf("CTRL D\n");
	else if(_temp==(KEY_CTRL|KEY_F))printf("CTRL F\n");
	else if(_temp==(KEY_CTRL|KEY_G))printf("CTRL G\n");
	else if(_temp==(KEY_CTRL|KEY_Z))printf("CTRL Z\n");
	else if(_temp==(KEY_CTRL|KEY_X))printf("CTRL X\n");
	else if(_temp==(KEY_CTRL|KEY_C))printf("CTRL C\n");
	else if(_temp==(KEY_CTRL|KEY_V))printf("CTRL V\n");
	else if(_temp==(KEY_CTRL|KEY_B))printf("CTRL B\n");
	else if(_temp==(KEY_CTRL|KEY_Q))printf("CTRL Q\n");
	else if(_temp==(KEY_CTRL|KEY_E))printf("CTRL E\n");
	else if(_temp==(KEY_CTRL|KEY_R))printf("CTRL R\n");
	else if(_temp==(KEY_CTRL|KEY_W))printf("CTRL W\n");
*/	
	
}
