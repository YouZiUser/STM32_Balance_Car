#ifndef __APP_PWM_H
#define __APP_PWM_H

#include "stm32f10x.h"                  // Device header

void App_PWM_Init(void);//PWM初始化
void App_PWM_Cmd(uint8_t on);//按键控制
void App_PWM_Set_L(float Duty);//设置左电机占空比，Duty是占空比的英文
void App_PWM_Set_R(float Duty);//设置右电机占空比
#endif
