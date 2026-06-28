#ifndef APP_MOTOR_H
#define APP_MOTOR_H
#include "stm32f10x.h"                  // Device header
//预编译的宏可以防止重复定义
//
//@简介：此文件及其.c文件用于对电机调速
//


void App_Motor_Init(void);//对电机进行初始化
void App_Motor_Proc(void);//电机的进程函数，需要每隔一段时间进行PID计算

void App_Motor_SetOmega_L(float Omega);//用户设置左电机的转速SP_L
void App_Motor_SetOmega_R(float Omega);//用户设置右电机的转速SP_R

void App_Motor_Cmd(uint8_t On);//实现对电机开关的控制
#endif
