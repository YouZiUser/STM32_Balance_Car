///**
//  ******************************************************************************
//  * @file    pid.h
//  * @author  铁头山羊stm32工作组
//  * @version V1.0.0
//  * @date    2024年10月27日
//  * @brief   pid算法库
//  ******************************************************************************
//  * @attention
//  * Copyright (c) 2024 -  东莞市明玉科技有限公司. 保留所有权力.
//  ******************************************************************************
//*/

//#ifndef _PID_H_
//#define _PID_H_

//#include <stdint.h>

//typedef struct{
//	float Kp; 
//	float Ki; 
//	float Kd; 
//	float Setpoint;
//	float OutputUpperLimit;
//	float OutputLowerLimit;
//	float DefaultOutput;
//}PID_InitTypeDef;

//typedef struct{
//	PID_InitTypeDef Init;
//	uint64_t LastTime; // Last time run (us)
//	float LastOutput;
//	float ITerm;
//	float DTerm;
//	float LastInput;
//	float LastError;
//	float Kp;
//	float Ki; 
//	float Kd; 
//	float OutputUpperLimit;
//	float OutputLowerLimit;
//	float Setpoint;
//	uint8_t Manual;
//	float ManualOutput;
//}PID_TypeDef;

// void PID_Init(PID_TypeDef *PID, PID_InitTypeDef *PID_InitStruct);
// void PID_Cmd(PID_TypeDef *PID, uint8_t NewState);
// void PID_Reset(PID_TypeDef *PID);
//float PID_Compute1(PID_TypeDef *PID, float Input, uint64_t Now);
//float PID_Compute2(PID_TypeDef *PID, float Input, float dInput, uint64_t Now);
// void PID_ChangeTunings(PID_TypeDef *PID, float NewKp, float NewKi, float NewKd);
// void PID_GetTunings(PID_TypeDef *PID, float *pKpOut, float *pKiOut, float *pKdOut);
// void PID_ChangeSetpoint(PID_TypeDef *PID, float NewSetpoint);
//float PID_GetSetpoint(PID_TypeDef *PID);
// void PID_ChangeManualOutput(PID_TypeDef *PID, float NewValue);

//#endif
