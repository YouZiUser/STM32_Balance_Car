#include "app_pid.h"
#include "delay.h"
//
//@简介：对PID控制器进行初始化
//@参数：Kp比例系数 Ki积分系数 Kd微分系数
//@参数：SP set point 设定值
void PID_Init(PID_TypeDef *PID, float Kp, float Ki, float Kd)
{
	PID->Kp = Kp;
	PID->Ki = Ki;
	PID->Kd = Kd;
	PID->SP = 0.0f;
	
	
	PID->t_k_1= 0;//在这里对结构体里的变量赋初始值，均赋值为0            //即是t[k-1],上次运行PID的时间
	PID->err_k_1= 0.0f;//每次复位，这三个值都需要赋值为0，在reset中实现  //即是err[k-1],上次运行PID的误差
	PID->err_int_k_1= 0.0f;                                              //即是err[k-1],上次运行的积分值
	
	PID->UpperLimit = +3.4e+38f;//这个值为float类型的最大值 初始值无限大，表示没有限制
	PID->LowerLimit = -3.4e+38f;//这个值为float类型的最小值 初始值无限小，表示没有限制
}
//
//@简介：改变设定值SP
//@参数：SP新的设定值
void PID_ChangeSP(PID_TypeDef *PID, float SP)
{
	PID->SP = SP;
}

//
//@简介：执行一次PID运算
//@参数：FB - 反馈的值，也就是传感器采集的值
//@返回值：PID控制器计算的结果CO
//CO也就是ua(t),用于控制电机的电压
float PID_Compute(PID_TypeDef *PID,float FB)
{
	float err = PID->SP - FB;//当前的err[k]
	
	uint64_t t_k = GetUs();//获取当前时间t[k] 
	
	float delatT  = (t_k - PID->t_k_1) * 1.0e-6f;  //Δt=t[k]-t[k-1]，并将单位由微秒转换为秒
	float err_dev = 0.0f;
	float err_int = 0.0f;
	//判断PID控制器是否为第一次运行，通过判断t[k-1]是否为默认值0,如果为0，跳过if，err_dev = 0.0f err_int = 0.0f;否则执行循环，计算积分和微分
	if (PID->t_k_1 !=0)
	{
		err_dev = (err - PID->err_k_1)/delatT;   //err(k)切线的微分 dev --deviation
		err_int = PID->err_int_k_1 + ((err + PID->err_k_1)*0.5f) * delatT;//err（k）的积分，使用上一时刻积分值加上梯形面积
	}
	
	float COp = PID->Kp * err;//比例器输出
	
	float COi = PID->Ki * err_int;//积分器输出
	
	float COd = PID->Kd * err_dev;//微分器输出
	
	float CO = COp + COi +COd;
	
	//更新相关的值
	PID->t_k_1 = t_k;
	PID->err_k_1 = err;
	PID->err_int_k_1=err_int;
	
	if(CO>PID->UpperLimit) CO = PID->UpperLimit;//限制CO的值，如果CO的值大于上限，则令C0等于上限
	if(CO<PID->LowerLimit) CO = PID->LowerLimit;//限制CO的值，如果CO的值小于小限，则令C0等于下限
	
	//不只要限制控制器的输出CO，还要限制积分项，否则积分一直累加，可能超过CO的上限之后就失效了
	if(PID->err_int_k_1 > PID->UpperLimit) PID->err_int_k_1 = PID->UpperLimit;
	if(PID->err_int_k_1 < PID->LowerLimit) PID->err_int_k_1 = PID->LowerLimit;
	return CO;//直接把计算好的CO作为返回值返回回去即可
}

//
//@简介：改变控制器输出CO的上下限
//@参数：Upper上限值 Lower下限值
//用户自定义CO的上下限函数
void PID_LimitConfig(PID_TypeDef *PID,float Upper,float Lower)
{
	PID->UpperLimit = Upper;
	PID->LowerLimit = Lower;
}

//
//@简介：对PID控制器进行复位
//每次开关电机的时候对这个函数进行使用
void PID_Reset(PID_TypeDef *PID)
{
	PID->t_k_1 = 0.0f;
	PID->err_int_k_1=0.0f;
	PID->err_k_1=0.0f;
}
