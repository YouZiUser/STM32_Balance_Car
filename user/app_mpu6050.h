#ifndef __APP_MPU6050_H
#define __APP_MPU6050_H

#include "stm32f10x.h"                  // Device header


void App_MPU6050_Init(void);//MPU6050初始化

void App_MPU6050_Proc(void);//在这个函数中解算欧拉角

void App_MPU6050_Update(void);//获取MPU6050的采集的数据


float App_MPU6050_GetAx(void);//获取ax的值
float App_MPU6050_GetAy(void);//获取ay的值
float App_MPU6050_GetAz(void);//获取az的值

float App_MPU6050_GetGx(void);//获取gx的值
float App_MPU6050_GetGy(void);//获取gy的值
float App_MPU6050_GetGz(void);//获取gz的值

float App_MPU6050_GetYaw(void);//获取偏航角YAW的值
float App_MPU6050_GetPitch(void);//获取俯仰角PITCH的值
float App_MPU6050_GetRoll(void);//获取翻滚角ROLL的值

float App_MPU6050_GetTemperature(void);//获取温度计的值
#endif
