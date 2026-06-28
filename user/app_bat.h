#ifndef APP_BAT_H
#define APP_BAT_H

#include "stm32f10x.h"                  // Device header
//先引用单片机的头文件

//在.h文件中，写外部能够调用函数的声明
//函数的具体在.c中实现
void App_Bat_Init(void);
float App_Bat_Get(void);//声明编程接口，用于获取电压vbat的值
void App_Bat_Proc(void);//根据电池剩余电量实现对LED的控制函数

//为什么宏定义可以防止重复重复引用
//C语言中学习
#endif  //宏定义的作用是防止头文件被重复引用
