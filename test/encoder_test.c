#include "encoder_test.h"
#include "app_encoder.h"
#include "usart.h"
#include "app_usart2.h"
#include "delay.h"
//
//@简介：对编码器的位置信号进行测试
//通过调试用的串口，把编码器的当前位置显示在vofa上
void Encoder_Test(void)
{
	App_USART2_Init();
	App_Encoder_Init();
		while(1)
	{
		float pos_left  = App_Encoder_GetPos_L();//左轮胎旋转角度
		float pos_right = App_Encoder_GetPos_R();//右轮胎旋转角度
		
		My_USART_Printf(USART2,"%f,%f\n",pos_left,pos_right);
		
		Delay(50);
	}
}

static float last_pos_l = 0.0f;
static float last_pos_r = 0.0f;

//
//@简介：对M法测速进行测试
//
void Encoder_M_Method_Test(void)
{
	App_USART2_Init();
	App_Encoder_Init();//对编码器进行初始化
	while(1)
	{
		Delay(1);
		
		float pos_l = App_Encoder_GetPos_L();
		float pos_r = App_Encoder_GetPos_R();
		
		float M_l = pos_l - last_pos_l;
		float M_r = pos_r - last_pos_r;
		
		//M_l 和 M_r其实是已经换算好的，因为App_Encoder_GetPos_L()   App_Encoder_GetPos_R()已经换算好了
		float omega_l = (M_l / 0.001f);    
		float omega_r = (M_r / 0.001f);
		last_pos_l = pos_l;
		last_pos_r = pos_r;
		My_USART_Printf(USART2,"%f,%f,%f,%f\n",pos_l,pos_r,omega_l,omega_r);
	}

}

//
//@简介：对T法测速进行测试
//
void Encoder_T_Method_Test(void)
{
	App_USART2_Init();
	App_Encoder_Init();//对编码器进行初始化
	while(1)
	{		
		Delay(1);
		float omega_l =App_Encoder_GetSpeed_L();    
		float omega_r = App_Encoder_GetSpeed_R();
		My_USART_Printf(USART2,"%f,%f\n",omega_l,omega_r);
	}

}
