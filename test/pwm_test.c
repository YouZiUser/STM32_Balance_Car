#include "pwm_test.h"
#include "app_pwm.h"
#include "app_button.h"
#include "delay.h"
//
//@简介：对PWM模块进行测试
//		先使能电机
//		再让电机正转30%两秒
//		再让电机正转60%两秒
//		再让电机正转90%两秒
//		关闭电机
void PWM_Test(void)
{
	App_PWM_Init();
	App_PWM_Cmd(1);//开启电机
	App_PWM_Set_L(30);//30%正转
	Delay(2000);
//	App_PWM_Set_L(60);//60%正转
//	Delay(2000);
//	App_PWM_Set_L(90);//90%正转
//	Delay(2000);
	
	App_PWM_Cmd(0);//关闭电机
	while(1)
	{
	
	
	
	
	}


}
