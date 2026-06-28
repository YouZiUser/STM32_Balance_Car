#ifndef __APP_PID_H
#define __APP_PID_H
//
//@简介：平衡车的PID实现，可以迁移
//
#include "stm32f10x.h"
//反馈值：FB会实时发生变化


//KP、KI、KD，用户的设定值SP是需要长期被存储的
//需要声明一个结构体存储这些需要被长期保存的数据
//结构体的声明一定要放在头文件中
typedef struct
{  //{}写入具体的名称
	float Kp;//比例系数
	float Ki;//积分项系数
	float Kd;//微分项系数
	float SP;//用户设定值
	
	uint64_t t_k_1;   //即是t[k-1],上次运行PID的时间
	float err_k_1;    //即是err[k-1],上次运行PID的误差
	float err_int_k_1;//即是err[k-1],上次运行的积分值
	
	float UpperLimit;//CO的上限
	float LowerLimit; //CO的下限 都保存在自定义的PID_TypeDef里面
	


}PID_TypeDef;//自定义的结构体名称

//初始化PID
void PID_Init(PID_TypeDef *PID, float Kp, float Ki, float Kd);
//改变设定值SP
void PID_ChangeSP(PID_TypeDef *PID, float SP);
//计算PID的函数
float PID_Compute(PID_TypeDef *PID,float FB);

//用户自定义CO的上下限函数
void PID_LimitConfig(PID_TypeDef *PID,float Upper,float Lower);

//PID复位函数
void PID_Reset(PID_TypeDef *PID);


#endif
