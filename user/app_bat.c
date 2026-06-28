#include "stm32f10x.h"                  // Device header
#include "app_bat.h"
#include "delay.h"
//
//@电池模块：任务1：根据电阻分压测得当前电池的电压，并通过串口usart发送给计算机
//任务2：根据当前电池的电压，计算出电池剩余的电量，并根据电池的电量控制3个LED灯的状态
//

static volatile float vbat = 0.0f;//声明电源电压vbat为一个全局变量，且尽在当前文件夹中调用
static uint32_t  lastTime =0;//声明一个变量记录上次LED切换的时间
static uint8_t stage =0;//声明一个变量记录LED的亮灭状态，0灭1亮
//为了使代码更加清晰，分别对多个外设进行初始化，这样更加清晰
static void bat_LED_Init(void);
static void TIM2_TRGO_Init(void);
static void bat_ADC_Init(void);

void TIM2_TRGO_Init(void)
{
	//1.配置定时器，使用TIM2的从模式控制器的TRGO模式
	
	//1.1首先配置时基单元
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Prescaler=71;//分频器预分频  分频周期1Mhz，分辨率是1μs
	TIM_TimeBaseInitStructure.TIM_Period=9999;//计数器计数周期 计数周期是10000，此时周期是10e-6*10000=10e-2s=10ms
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;//重复计数次数，实际上TIM2无此功能
		
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
	
		
	//1.2配置从模式控制器为TRGO主机模式
	TIM_SelectOutputTrigger(TIM2 ,TIM_TRGOSource_Update);
		
	//1.3闭合定时器开关，使之生效
	TIM_Cmd(TIM2,ENABLE);


}

void bat_ADC_Init(void)
{
	//初始化ADC的I/O引脚，这里ADC1用到的是PB0
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure ={0};
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;//设置为模拟输入
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	//2.初始化ADC
	
	//2.1配置ADC的时钟，并设置好ADC的分频
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); //将ADC的分频系数设置为6分频	
	
	//初始化ADC
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	
	ADC_InitTypeDef ADC_InitStructure={0};
	//这里配置的是常规序列
	ADC_InitStructure.ADC_Mode=ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode=DISABLE;//多个通道才使用扫描模式，这个是全局的，即针对常规序列，又针对注入序列
	ADC_InitStructure.ADC_ContinuousConvMode=DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;//配置常规序列的触发方式，随便选一个
	ADC_InitStructure.ADC_DataAlign=ADC_DataAlign_Right;//对齐方式选择右对齐
	ADC_InitStructure.ADC_NbrOfChannel=1;//设置常规序列的个数，随便填一个1，代表单通道
	
	ADC_Init(ADC1,&ADC_InitStructure);
	
	//设置注入序列的参数
	ADC_ExternalTrigInjectedConvConfig(ADC1,ADC_ExternalTrigInjecConv_T2_TRGO); //配置注入序列的触发方式为TIM2_TRGO
	ADC_ExternalTrigInjectedConvCmd(ADC1,ENABLE);//闭合注入序列的外部触发开关
	ADC_InjectedChannelConfig(ADC1,ADC_Channel_8,1 ,ADC_SampleTime_7Cycles5); //通道8的相关信息写入注入序列的第一行，采样时间
	
	//配置JEOC标志位触发中断
	ADC_ITConfig(ADC1 ,ADC_IT_JEOC, ENABLE);
	
	//配置中断
	NVIC_InitTypeDef NVIC_InitStructure ={0};
	NVIC_InitStructure.NVIC_IRQChannel=ADC1_2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;
	NVIC_Init(&NVIC_InitStructure);
	
	ADC_Cmd(ADC1,ENABLE);
	
}


void bat_LED_Init(void)
{
	//初始化3个LED灯的引脚
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure ={0};
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4 |GPIO_Pin_5| GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
}
//
//@简介：对电池电压检测模块进行初始化
//
void App_Bat_Init(void)
{
	TIM2_TRGO_Init();
	bat_ADC_Init();
}

//
//@简介：电池电压监控模块的进程函数根据电池剩余电量对LED进行控制
//进程函数中无法使用delay延时函数，需要自己利用GetTick()编程实现
void App_Bat_Proc(void)
{
	bat_LED_Init();
	if(vbat>7.9f)//此时电池满电，需要点亮3颗LED
	{
		GPIO_WriteBit(GPIOA,GPIO_Pin_4 |GPIO_Pin_5|GPIO_Pin_6 , Bit_SET);
	}
	else if(vbat >7.4f)//此时电池大约75%的电，点亮两颗LED
	{
		GPIO_WriteBit(GPIOA,GPIO_Pin_4 |GPIO_Pin_5 , Bit_SET);
		GPIO_WriteBit(GPIOA,GPIO_Pin_6,Bit_RESET);
	}
	else if(vbat >7.0f)//此时电池大约50%的电，点亮一颗LED
	{
		GPIO_WriteBit(GPIOA,GPIO_Pin_4 , Bit_SET);
		GPIO_WriteBit(GPIOA,GPIO_Pin_5,Bit_RESET);
		GPIO_WriteBit(GPIOA,GPIO_Pin_6,Bit_RESET);
	}
	else if(vbat >6.5f)//此时电池大约25%的电，都熄灭
	{
		GPIO_WriteBit(GPIOA,GPIO_Pin_4,Bit_RESET);
		GPIO_WriteBit(GPIOA,GPIO_Pin_5,Bit_RESET);
		GPIO_WriteBit(GPIOA,GPIO_Pin_6,Bit_RESET);
	}
	else //此时电池几乎没电，让PA4的led闪烁
	{
		uint32_t now = GetTick();//先获取当前的时间
		if(stage == 0 && now - lastTime >100)
		{
			lastTime =now;
			GPIO_WriteBit(GPIOA,GPIO_Pin_4 |GPIO_Pin_5|GPIO_Pin_6,Bit_SET);
			stage =1;
		
		}
		else if(stage == 1 && now - lastTime >100)
		{
			lastTime =now;
			GPIO_WriteBit(GPIOA,GPIO_Pin_4 |GPIO_Pin_5|GPIO_Pin_6,Bit_RESET);
			stage =0;
		
		}
		//也可以使用switch语句
//		if(now -lastTime >100)
//		{
//			switch(stage)
//			{
//				case 0: 
//				{
//				GPIO_WriteBit(GPIOA,GPIO_Pin_4, Bit_SET);//当前熄灭
//				stage =1;
//				break;
//				}
//				case 1: 
//				{
//				GPIO_WriteBit(GPIOA,GPIO_Pin_4, Bit_RESET);//当前熄灭
//				stage =0;
//				break;
//				}
//			}
//		lastTime = now;
//		}
		
	}



}


//
//@简介：获取电池的当前电压的，单位是伏特
//
float App_Bat_Get(void)
{
	return vbat;
}

//
//*ADC1和ADC2的中断响应函数
//
void ADC1_2_IRQHandler(void)
{
	if(ADC_GetFlagStatus(ADC1,ADC_FLAG_JEOC)==1)
	{
		ADC_ClearFlag(ADC1,ADC_FLAG_JEOC);//先对标志位清零
		
		uint16_t jdr1 = ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_1);//读取寄存器JDR1的值即写入注入序列第一行的值
		vbat = (jdr1 / 4095.0f * 3.3)*(8.4/3.3)	; // 4095是12位逐次逼近型adc的采样深度 4095.0f 这里的.0f不可舍弃，因为如果是整数相除，会舍弃小数点后的部分
	}


}
