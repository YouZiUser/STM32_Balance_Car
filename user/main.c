#include "stm32f10x.h"
#include "delay.h"
#include "app_bat.h"
#include "app_button.h"
#include "app_pwm.h"
#include "app_motor.h"
#include "app_encoder.h"
#include "delay.h"
#include "app_usart2.h"
#include "task.h"
#include "app_mpu6050.h"
#include "app_control.h"
#include "app_rc.h"
//裸机多任务开发模型
//将多个任务切片，每个切片（每个切片称为进程函数）在while循环中逐次执行
//每个切片之间不允许使用delay延时函数
//否则就不能视为一个完整任务
//一定一定注意，进程函数中，不能使用延时函数


//static float targetOmage;//目标角速度

//static void USART2_Proc(void);//声明一个usart2发送数据的进程函数

int main(void)
{
	//中断优先级分组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
	App_Button_Init();//按钮，控制电机的启动
	App_Encoder_Init();//编码器初始化
	App_PWM_Init();//TB6612FNG双H桥控制
	App_Bat_Init();//电池电压监控
	App_Motor_Init();//电机调速系统（PID）
	App_USART2_Init();//USART2初始化
	App_MPU6050_Init();//MPU6050初始化
	App_Control_Init();//平衡车控制的进程函数
	App_RC_Init();//对遥控器进行初始化
	while(1)
	{
		App_Bat_Proc();
		App_Button_Proc();
		App_Motor_Proc();
		App_MPU6050_Proc();//MPU6050的进程函数用于解算欧拉角
		App_Control_Proc();//平衡车的进程函数
		App_RC_Proc();//遥控器控制平衡车的进程函数
	}
}



