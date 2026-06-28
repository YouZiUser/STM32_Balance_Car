///**
//  ******************************************************************************
//  * @file    pid.c
//  * @author  铁头山羊stm32工作组
//  * @version V1.0.0
//  * @date    2023年2月24日
//  * @brief   pid算法库
//  ******************************************************************************
//  * @attention
//  * Copyright (c) 2022 - 2023 东莞市明玉科技有限公司. 保留所有权力.
//  ******************************************************************************
//*/

//#include "pid.h"

//#define INVALID_TICK 0xffffffffffffffff

////
//// @简介：PID算法初始化
//// @参数：PID - PID算法句柄
//// @返回：空
////
//void PID_Init(PID_TypeDef *PID, PID_InitTypeDef *PID_InitStruct)
//{
//	PID->Init = *PID_InitStruct;
//	
//	PID->Setpoint = PID->Init.Setpoint;
//	PID->Kp = PID->Init.Kp;
//	PID->Ki = PID->Init.Ki;
//	PID->Kd = PID->Init.Kd;
//	PID->OutputLowerLimit = PID->Init.OutputLowerLimit;
//	PID->OutputUpperLimit = PID->Init.OutputUpperLimit;
//	PID->ITerm = PID->Init.DefaultOutput;
//	PID->LastTime = INVALID_TICK;
//	PID->LastInput = 0;
//	PID->ManualOutput = PID->Init.DefaultOutput;
//	PID->Manual = 0;
//}

////
//// @简介：复位PID算法
//// @参数：PID - PID算法句柄
//// @返回：空
//// @注意：其实现原理是将积分项清零
////
//void PID_Reset(PID_TypeDef *PID)
//{
//	PID->LastTime = INVALID_TICK;
//	PID->ITerm = PID->Init.DefaultOutput;
//}

////
//// @简介：执行一次PID运算并得出控制器当前的输出值
//// @参数：PID - PID算法句柄
//// @参数：Input  - 传感器的输入值
//// @返回：控制器当前的输出值
////
//float PID_Compute1(PID_TypeDef *PID, float Input, uint64_t now)
//{
//	float Output;
//	float timeChange = 0;
//	float error;
//	
//	error	= PID->Setpoint - Input;
//	timeChange = (now - PID->LastTime) * 1.0e-6f;
//	
//	Output = PID->Kp * error;
//	
//	if(PID->LastTime != INVALID_TICK) // 非第一次初始化，或有I或D环节
//	{
//		
//		if(PID->Kd != 0)
//		{
//			PID->DTerm = PID->Kd * (PID->LastInput - Input) / timeChange;
//			Output += PID->DTerm;
//		}
//		
//		if(PID->Ki != 0)
//		{
//			PID->ITerm += PID->Ki * (error + PID->LastError) * 0.5 * timeChange;
//			
//			if(PID->ITerm > PID->OutputUpperLimit)
//			{
//				PID->ITerm = PID->OutputUpperLimit;
//			}
//			else if(PID->ITerm < PID->OutputLowerLimit)
//			{
//				PID->ITerm = PID->OutputLowerLimit;
//			}
//			
//			Output += PID->ITerm;
//		}
//	}
//	
//	if(Output > PID->OutputUpperLimit)
//	{
//		Output = PID->OutputUpperLimit;
//	}
//	else if(Output < PID->OutputLowerLimit)
//	{
//		Output = PID->OutputLowerLimit;
//	}
//	
//	if(PID->Manual != 0) 
//	{
//		Output = PID->ManualOutput;
//	}
//	
//	PID->LastInput = Input;
//	PID->LastTime = now;
//	PID->LastError = error;
//	PID->LastOutput = Output;
//	
//	return Output;
//}

////
//// @简介：执行一次PID运算并得出控制器当前的输出值
////        此方法与PID_Compute1相同，但需要用户提供dInputDt的值
//// @参数：PID - PID算法句柄
//// @参数：Input  - 传感器的输入值
//// @参数：dInputDt - 传感器输入的变化速度 dInput(k) = (Input(k) - Input(k-1)) / dt
//// @返回：控制器当前的输出值
////
//float PID_Compute2(PID_TypeDef *PID, float Input, float dInputDt, uint64_t now)
//{
//	float Output;
//	float timeChange = 0;
//	float error;
//	
//	error	= PID->Setpoint - Input;
//	timeChange = (now - PID->LastTime) * 1.0e-6f;
//	
//	Output = PID->Kp * error;
//	
//	if(PID->LastTime != INVALID_TICK) // 非第一次初始化，或有I或D环节
//	{
//		if(PID->Kd != 0)
//		{
//			PID->DTerm = -PID->Kd * dInputDt;
//			Output += PID->DTerm;
//		}
//		
//		if(PID->Ki != 0)
//		{
//			PID->ITerm += PID->Ki * (error + PID->LastError) * 0.5 * timeChange;
//			
//			if(PID->ITerm > PID->OutputUpperLimit)
//			{
//				PID->ITerm = PID->OutputUpperLimit;
//			}
//			else if(PID->ITerm < PID->OutputLowerLimit)
//			{
//				PID->ITerm = PID->OutputLowerLimit;
//			}
//			
//			Output += PID->ITerm;
//		}
//	}
//	
//	if(Output > PID->OutputUpperLimit)
//	{
//		Output = PID->OutputUpperLimit;
//	}
//	else if(Output < PID->OutputLowerLimit)
//	{
//		Output = PID->OutputLowerLimit;
//	}
//	
//	if(PID->Manual != 0) 
//	{
//		Output = PID->ManualOutput;
//	}
//	
//	PID->LastInput = Input;
//	PID->LastTime = now;
//	PID->LastError = error;
//	PID->LastOutput = Output;
//	
//	return Output;
//}

////
//// @简介：调节PID的参数（Kp, Ki和Kd）
//// @参数：PID - PID算法句柄
//// @参数：NewKp  - 新的比例系数Kp
//// @参数：NewKi  - 新的积分系数Ki
//// @参数：NewKd  - 新的微分系数Kd
//// @返回：空
////
//void PID_ChangeTunings(PID_TypeDef *PID, float NewKp, float NewKi, float NewKd)
//{
//	PID->Kp = NewKp;
//	PID->Ki = NewKi;
//	PID->Kd = NewKd;
//}

////
//// @简介：改变设定值SP
//// @参数：PID      - PID算法句柄
//// @参数：NewSetpoint - 新的设定值
//// @返回：空
////
//void PID_ChangeSetpoint(PID_TypeDef *PID, float NewSetpoint)
//{
//	PID->Setpoint = NewSetpoint;
//}

////
//// @简介：获取设定值SP
//// @参数：PID      - PID算法句柄
//// @返回：SP
////
//float PID_GetSetpoint(PID_TypeDef *PID)
//{
//	return PID->Setpoint;
//}

////
//// @简介：获取当前的PID参数
//// @参数：PID - PID算法句柄
//// @参数：pKpOut - 输出参数，用于输出Kp的值
//// @参数：pKiOut - 输出参数，用于输出Ki的值
//// @参数：pKdOut - 输出参数，用于输出Kd的值
//// @返回：空
////
//void PID_GetTunings(PID_TypeDef *PID, float *pKpOut, float *pKiOut, float *pKdOut)
//{
//	*pKpOut = PID->Kp;
//	*pKiOut = PID->Ki;
//	*pKdOut = PID->Kd;
//}

////
//// @简介：使能/禁止PID
//// @参数：PID   - PID算法句柄
//// @参数：NewState - 非零 - 使能，0 - 禁止
//// @返回：空
//// @注意：PID被禁止后会进入手动模式。手动模式下的输出由PID_ChangeManualOutput设置
////
//void PID_Cmd(PID_TypeDef *PID, uint8_t NewState)
//{
//	if(PID->Manual == 1 && NewState == 1)
//	{
//		PID->ITerm = PID->ManualOutput;
//		if(PID->ITerm > PID->OutputUpperLimit)
//		{
//			PID->ITerm = PID->OutputUpperLimit;
//		}
//		
//		if(PID->ITerm < PID->OutputLowerLimit)
//		{
//			PID->ITerm = PID->OutputLowerLimit;
//		}
//	}
//	
//	PID->Manual = NewState == 0 ? 1 : 0;
//}

////
//// @简介：修改手模式下的输出
//// @参数：PID   - PID算法句柄
//// @参数：NewValue - 手动模式下的输出
//// @返回：空
////
//void PID_ChangeManualOutput(PID_TypeDef *PID, float NewValue)
//{
//	PID->ManualOutput = NewValue;
//}
