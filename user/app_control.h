#ifndef APP_CONTROL_H
#define APP_CONTROL_H
#include "stm32f10x.h"

//总结这么多代码，一般应用层文件中
//主要包括初始化函数和进程函数（有的是不需要进程函数的比如app_usart2）

//
//@简介：平衡车控制相关的代码
//@即包括倒立摆的单片机内部程序实现以及速度环的单片机内部程序实现，和遥控器控制部分的程序
void App_Control_Init(void);//初始化函数

void App_Control_Proc(void);//进程函数

void App_Control_Reset(void);//对整个控制系统进行复位，否则上电时，轮胎会疯转

void App_Control_SetMoveSpeed(float MoveSpeed);//改变平衡车的车速

void App_Control_SetTurnSpeed(float TurnSpeed);//改变平衡车的转速

#endif
