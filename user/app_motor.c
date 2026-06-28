#include "app_motor.h"
#include "app_pid.h"
#include "task.h"
#include "app_encoder.h"
#include "app_pwm.h"
#include "app_bat.h"
//
//@简介：电机调速文件
//


//
//@简介：左右电机调速系统的初始化
//
//首先声明两个结构体变量
static PID_TypeDef pid_motor_l;//左电机PID变量
static PID_TypeDef pid_motor_r;//右电机PID变量

void App_Motor_Init(void)//对左右两个电机进行初始化
{

	        //第一个参数是指针类型的变量
	PID_Init(&pid_motor_l, 0.5, 7, 0);//左电机PID初始化  实际上此系统为PI控制器
	PID_LimitConfig(&pid_motor_l,+2.0f,-2.0f);//设置左电机上下限 由于我是170电机且是面包板搭建，尽可能小一些
	
	PID_Init(&pid_motor_r, 0.5, 7, 0);//右电机PID初始化
    PID_LimitConfig(&pid_motor_r,+2.0f,-2.0f);
//	App_Encoder_Init();//使用编码器应该进行初始化，选择在main函数中统一初始化，这里注释掉

}

//
//@简介：电机调速系统的进程函数
//
void App_Motor_Proc(void)
{
	PERIODIC(1)//每1ms计算一次PID
	//1.通过编码器获取左右电机的旋转角速度
	float omega_l = App_Encoder_GetSpeed_L();//获取当前左电机角速度FB_l
	float omega_r = App_Encoder_GetSpeed_R();//获取当前右电机角速度FB_r
	
	//2.使用PID_Compute计算控制器输出
	float CO_l = PID_Compute(&pid_motor_l,omega_l);//左电机控制器输出
	float CO_r = PID_Compute(&pid_motor_r,omega_r);//右电机控制器输出
	
	//实际上CO就是加在电机上的电压
	float ua_l = CO_l;
	float ua_r = CO_r;
	
	//3.通过PWM的占空比模拟出等效电压ua
	//由ua = VBAT(电源电压) *占空比
	//反推出占空比 = ua/VBAT 获取VBAT需要在电源的.c文件中获取
	float vbat = App_Bat_Get();//获取当前电池电压
	
	float duty_l = (ua_l / vbat)* 100.0f;//将左电机占空比转换为百分数
	float duty_r = (ua_r / vbat)* 100.0f;//将右电机占空比转换为百分数
	
	App_PWM_Set_L(duty_l);//设置左电机占空比
	App_PWM_Set_R(duty_r);//设置右电机占空比

}
//
//@简介：用来设置左电机的转速Omega的值  SP_l实际上就是设定值
//@参数：Omega - 表示电机的转速，单位是rad/s
void App_Motor_SetOmega_L(float Omega)
{
	PID_ChangeSP(&pid_motor_l, Omega);
}

//
//@简介：用来设置右电机的转速Omega的值  SP_r实际上就是设定值
//@参数：Omega - 表示电机的转速，单位是rad/s
void App_Motor_SetOmega_R(float Omega)
{
	PID_ChangeSP(&pid_motor_r, Omega);
}

//
//@简介：开关电机
//@参数：On控制电机的开关，0 - 关闭  非零 - 开启 
void App_Motor_Cmd(uint8_t On)
{
	App_PWM_Cmd(On);
	PID_Reset(&pid_motor_l);
	PID_Reset(&pid_motor_r);//开关每启停一次，都需要对左右两电机进行复位
}
