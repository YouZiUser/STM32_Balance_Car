#include "app_button.h"
#include "button.h"
#include "app_pwm.h"
#include "app_motor.h"
#include "app_control.h"
//就相当于自己定义了一个外设
static Button_TypeDef userKey;  //声明一个变量代表用户按钮


static void OnUserKey_Clicked(uint8_t clicks);
//
//@简介：按钮初始化
//
void App_Button_Init(void)
{
	Button_InitTypeDef Button_InitStrucutre ={0};
	Button_InitStrucutre.GPIOx=GPIOA;
	Button_InitStrucutre.GPIO_Pin=GPIO_Pin_11;
	My_Button_Init(&userKey ,&Button_InitStrucutre);
	
	My_Button_SetClickCb(&userKey ,OnUserKey_Clicked);


}

void App_Button_Proc(void)
{
	My_Button_Proc(&userKey);

}


//声明一个变量记录当前电机的活动状态
static uint8_t pwm_on = 0; //0表示电机休眠，1表示电机活动
//
//@简介：按钮点击的回调函数
//
static void OnUserKey_Clicked(uint8_t clicks)
{
	if(clicks == 1 )
	{
		//当点击这个平衡车控制按钮的时候，先对整个平衡车进行复位
		App_Control_Reset();
		//翻转电机
		if(pwm_on == 0)
		{
			pwm_on=1;
		}
		else
		{
			pwm_on= 0;
		}
		App_Motor_Cmd(pwm_on);//把开关状态传给电机Motor
		
	}
	
}
