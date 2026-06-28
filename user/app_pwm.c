#include "app_pwm.h"
#include "math.h"
static void STBY_Pin_Init(void);//对TB6612的STBY进行初始化接口

//左右电机分别初始化，把它们的初始化封装为一个函数
static void Motor_L_Init(void);//左电机初始化
static void Motor_R_Init(void);//右电机初始化


//
//@简介：对TB6612进行初始化
//

void App_PWM_Init(void)
{
	STBY_Pin_Init();//STBY引脚的初始化
	Motor_L_Init();//对左电机初始化
	Motor_R_Init();//对右电机初始化
}

//
//@简介：控制TB6612进入休眠或者活动状态
//@参数：on 0 休眠状态--向STBY写0低电压  on 非零 活动状态--向STBY写非零
void App_PWM_Cmd(uint8_t on)
{
	if(on==0)
	{
		GPIO_ResetBits(GPIOA,GPIO_Pin_1);
	}
	else
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_1);
	
	}



}


//
//@简介：设置左电机的占空比
//@参数：Duty，其取值范围为-100.0f-100.0f，表示占空比，负数表示反转
void App_PWM_Set_L(float Duty)
{
	float sign;//符号，正数 - +1 负数 - -1
	if(Duty >=0)	sign=+1;
	else  sign = -1;
	
	Duty = fabsf(Duty);//对Duty取绝对值,fabsf是C语言中取绝对值的函数
	
	if(sign >=0)//sign>0,说明要正转
	{
		//给AIN1写1高电压，AIN2写0低电压
		GPIO_ResetBits(GPIOA,GPIO_Pin_9);
		GPIO_SetBits(GPIOA,GPIO_Pin_10);
	}
	else
	{
		//给AIN1写0高电压，AIN2写1低电压
		GPIO_SetBits(GPIOA,GPIO_Pin_9);
		GPIO_ResetBits(GPIOA,GPIO_Pin_10);  //使得左右两电机驱动的轮胎同向
	}
	
	uint16_t ccr = Duty / 100.0f *  1000 ;//由占空比，反推出ccr的值，1000为计数周期
	
	TIM_SetCompare1(TIM1,ccr);//新的编程接口，SetCompare1就是设置通道1的ccr寄存器的值
}

//
//@简介：设置右电机的占空比
//@参数：Duty，其取值范围为-100.0f-100.0f，表示占空比，负数表示反转
void App_PWM_Set_R(float Duty)
{
	float sign;//符号，正数 - +1 负数 - -1
	if(Duty >=0)	sign=+1;
	else  sign = -1;
	
	Duty = fabsf(Duty);//对Duty取绝对值,fabsf是C语言中取绝对值的函数
	
	if(sign >=0)//sign>0,说明要正转
	{
		//给BIN1写1高电压，BIN2写0低电压
		GPIO_SetBits(GPIOB,GPIO_Pin_5);
		GPIO_ResetBits(GPIOB,GPIO_Pin_7);
	}
	else
	{
		//给BIN1写0高电压，BIN2写1低电压
		GPIO_SetBits(GPIOB,GPIO_Pin_7);
		GPIO_ResetBits(GPIOB,GPIO_Pin_5);
	}
	
	uint16_t ccr = Duty / 100.0f *  1000 ;//由占空比，反推出ccr的值，1000为计数周期
	
	TIM_SetCompare1(TIM4,ccr);//新的编程接口，SetCompare1就是设置通道1的ccr寄存器的值
		
}

//
//@简介：初始化STBY引脚
//PA1-OUT-PP
static void STBY_Pin_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure ={0};
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
}

//
//@简介：左电机初始化
//AIN1-PA9 OUT-PP  AIN2-PA10 OUT-PP 
static void Motor_L_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure ={0};
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9 |GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	//初始化左电机定时器，左电机使用定时器TIM1_CH1,PA8引脚
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure ={0};
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Prescaler= 72-1;//预分频取得越小越好，直接不分频，这样就可以使得PWM信号周期越大，频率越高，可以等效为直流
	TIM_TimeBaseInitStructure.TIM_Period=1000-1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStructure);
	
	
	//配置输出比较
	TIM_OCInitTypeDef TIM_OCInitStructure ={0};
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Reset;//PWM信号输出停止后，定时器TIM1保持低电平
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;//开关1
	TIM_OCInitStructure.TIM_Pulse=0; //CCR寄存器，用来调节PWM的占空比；设置比较寄存器CCR的值，先预先设置为0，后续在其他接口修改
	TIM_OC1Init(TIM1,&TIM_OCInitStructure);
	
	//配置MOE的开关，这是控制输出比较的总开关，一定不要忘记开//开关2
	TIM_CtrlPWMOutputs(TIM1,ENABLE);
	
	//闭合定时器的总开关，使之生效//开关3
	TIM_Cmd(TIM1,ENABLE);

}

//
//@简介：右电机初始化
//BIN1-PB5 OUT-PP  BIN2-PB7 OUT-PP 
static void Motor_R_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure ={0};
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5 |GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	//初始化右电机定时器，右电机使用定时器TIM4_CH1,PB6引脚
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
		
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure ={0};
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Prescaler= 72-1;
	TIM_TimeBaseInitStructure.TIM_Period=1000-1;
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);
	
	TIM_OCInitTypeDef TIM_OCInitStructure ={0};
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse=0; //设置比较寄存器CCR的值，先预先设置为0，后续在其他接口修改
	TIM_OC1Init(TIM4,&TIM_OCInitStructure);
	
	//配置MOE的开关，这是控制输出比较的总开关，一定不要忘记开
	TIM_CtrlPWMOutputs(TIM4,ENABLE);
	
	//闭合定时器的总开关，使之生效
	TIM_Cmd(TIM4,ENABLE);
}
