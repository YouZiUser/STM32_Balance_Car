#include "stm32f10x.h"                  // Device header
#include "app_encoder.h"
#include "delay.h"
#include "math.h"

//自定义最大值函数max_T()
float max_T(float  a, float b)
{
    return a > b ? a : b;
}


//正转encoder为正，反转encoder为负

//左右两个电机，左电机编码器encoder_L 右电机encoder_R
static volatile int64_t encoder_L = 0;//定义电机当前的角度，为有符号的64位整型
//static 使得encoder这个变量仅在这个文件内部进行使用
//volatile 表示encoder是个容易丢失的关键字，encoder会在中断和常规程序中修改，加上volatile防止encoder丢失

//右电机编码器encoder_R
static volatile int64_t encoder_R = 0;

//定义T法测速的计数direction
static volatile int16_t direction_L = 1;//左电机旋转方向，正转+1  反转-1
static volatile int16_t direction_R = 1;//右电机旋转方向，正转+1  反转-1

//左电机编码器发生变化的时间，单位是微秒
static volatile uint64_t t0_l = 0 , t1_l = 0;
//右电机编码器发生变化的时间，单位是微秒
static volatile uint64_t t0_r = 0 , t1_r = 0;

//维护方向direction的值
//检测到轮胎旋转方向由正向变为反向  direction = -2
//检测到轮胎旋转方向由负向变为正向  direction = 2


//对于编码器A、B两相，需要分别配置单片机的引脚为输入上拉模式
//捕获边沿使用EXTI外部中断

//左编码器初始化A相--PB14  B相--PB15 
static void Encoder_L_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef GPIO_initStructure = {0};
	GPIO_initStructure.GPIO_Pin=GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_initStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_Init(GPIOB,&GPIO_initStructure);
	
	//左编码器EXTI的初始化
	//配置EXTI的线14监控PB14
	//需要打开AFIO，将前面的复用器配置为GPIOB
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource14);
	
	//对于A相而言，需要捕获上下双边沿 先配置EXTI的参数，再配置中断  
	EXTI_InitTypeDef   EXTI_InitStructure = {0};
	EXTI_InitStructure.EXTI_Line=EXTI_Line14;
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising_Falling;
	EXTI_Init(&EXTI_InitStructure);
	
	//开启EXTI中断
	NVIC_InitTypeDef   NVIC_InitStructure = {0};
	NVIC_InitStructure.NVIC_IRQChannel=EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
	
	NVIC_Init(&NVIC_InitStructure);
}

//右编码器初始化A相--PB3  B相--PB4
static void Encoder_R_Init(void)
{
	//单片机刚上电时，PB3/PB4并不是当作普通的IO引脚使用
	//需要使用AFIO进行重映射
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO ,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef GPIO_initStructure = {0};
	GPIO_initStructure.GPIO_Pin=GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_initStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_Init(GPIOB,&GPIO_initStructure);
	//右编码器EXTI的初始化
	//配置EXTI的线3监控PB3
	//需要打开AFIO，将前面的复用器配置为GPIOB
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//其实上面已经开过了，无所谓
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource3);
	
	//对于A相而言，需要捕获上下双边沿 先配置EXTI的参数，再配置中断  
	EXTI_InitTypeDef   EXTI_InitStructure = {0};
	EXTI_InitStructure.EXTI_Line=EXTI_Line3;
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising_Falling;
	EXTI_Init(&EXTI_InitStructure);
	
	//开启EXTI中断
	NVIC_InitTypeDef   NVIC_InitStructure = {0};
	NVIC_InitStructure.NVIC_IRQChannel=EXTI3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
	
	NVIC_Init(&NVIC_InitStructure);

}

//
//@简介：对编码器模块进行初始化
//
//左编码器A相--PB14  B相--PB15
//右编码器A相--PB3   B相--PB4
void App_Encoder_Init(void)
{
	Encoder_L_Init();
	Encoder_R_Init();
}


//         中断响应函数逻辑  encoder角度
//			 A相上升沿       A相下降沿
//B=低		encoder 1++（顺）encoder 1--（逆）
//B=高		encoder 1--（逆）encoder 1++（顺）
//注意，应当保证

//
//@简介：读取左编码器的当前位置 --->修改为读取当前左轮胎的角度，单位是度°
//
float App_Encoder_GetPos_L(void)
{
	float encoder_Lround=encoder_L / 22.0f  *(1500.0f/30613.0f) *360.0f;
	return encoder_Lround;//22.0f是磁极走过的两倍圈数（因为捕获的是上下）
	//转速比：30613/1500  轮胎转一圈是360度   encoder每+1，相当于转了(encoder_L / 22.0f*(1500.0f/30613.0f))圈
}


// 
//@简介：获取当前左轮胎的旋转角速度 omega的值 单位是 rad每秒
//
float App_Encoder_GetSpeed_L(void)
{
	//解决由于中断导致的毛刺问题
	__disable_irq();//关闭单片机的总中断
	
	//拷贝中断可能改变的三个变量
	int8_t direction_copy = direction_L;
	uint64_t t0_copy = t0_l;
	uint64_t t1_copy = t1_l;
	//再次开启中断
	__enable_irq();//中断开关
	
	uint16_t now = GetUs();
	//C语言中没有内置的max函数，需要自己定义
	if(direction_copy == +2 || direction_copy == -2) return 0.0f;
	float T_l = max_T((t0_copy - t1_copy), (now - t0_copy))*1.0e-6f;//t0和t1都是微秒级的时间，需要转换为秒
	float Speed_L=((direction_copy /T_l) / 22.0f) *(1500.0f/30613.0f) * 6.2831853f;
	return Speed_L;
}



//
//@简介：EXTI14的中断响应函数，左侧编码器EXTI中断响应函数
//对应左侧编码器的A相
void EXTI15_10_IRQHandler(void)
{
	//EXTI14的中断不是单独的中断，而是与线10-15共用一个中断
	//需要判断中断是由于EXTI14触发的
	if(EXTI_GetFlagStatus(EXTI_Line14) == SET)//标志位使用SET，IO口使用Bit_SET
	{
		//清空EXTI14的标志位
		EXTI_ClearFlag(EXTI_Line14);
		
		//将上次的t0赋给这次的t1
		t1_l = t0_l;
		
		//触发了标志位，在此处更新时间t0_l
		t0_l = GetUs();
		
		//声明一个变量保存A相当前电压
		uint8_t a = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14);
		//声明一个变量保存B相当前电压
		uint8_t b = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15);
		//通过比较A、B两相的电压来确定
		if((a == Bit_SET && b == Bit_RESET) || (a == Bit_RESET && b == Bit_SET) )//if表示当前轮胎是反转的
		{
			encoder_L--;//此时反转
			
			if(direction_L > 0)//表示在此之前轮胎是正转的，而当前是反转的，说明轮胎转向发生了改变
			{
				direction_L = -2;
			
			}
			else
			{
				direction_L = -1;
			}
		}
		else//其他两种情况轮胎正转，合并起来
		{
			encoder_L++;//此时正转
			
			if(direction_L < 0)
			{
				direction_L = +2;
			}
			else
			{
				direction_L = +1;
			}				
			
		}

	}

	
	
}
//
//@简介：读取右编码器的当前位置 --->修改为读取当前右轮胎的角度，单位是度°
//
float App_Encoder_GetPos_R(void)
{
	float encoder_Rround =encoder_R / 22.0f  *(1500.0f/30613.0f) *360.0f;
	return encoder_Rround;
}



//
//@简介：获取当前右轮胎的旋转角速度 omega的值 单位是rad每秒
//
float App_Encoder_GetSpeed_R(void)
{
	//解决由于中断导致的毛刺问题
	__disable_irq();//关闭单片机的总中断
	
	//拷贝中断可能改变的三个变量
	int8_t direction_copy = direction_R;
	uint64_t t0_copy = t0_r;
	uint64_t t1_copy = t1_r;
	//再次开启中断
	__enable_irq();//中断开关
	
	uint16_t now = GetUs();
	if(direction_copy == +2 || direction_copy == -2) return 0.0f;
	float T_r = max_T((t0_copy - t1_copy), (now - t0_copy))*1.0e-6f;//t0和t1都是微秒级的时间，需要转换为秒
	float Speed_R=((direction_copy /T_r) / 22.0f) *(1500.0f/30613.0f) *6.2831853f;
	return Speed_R;
}


//
//@简介：EXTI3的中断响应函数，右侧编码器EXTI中断响应函数
//对应右侧编码器的A相
void EXTI3_IRQHandler(void)
{
	//EXTI_Line线3只有一路信号能够触发标志位
	//首先对EXTI的那个标志位清零
	EXTI_ClearFlag(EXTI_Line3);
	
	//将上次的t0赋给这次的t1
	t1_r = t0_r;
		
	//触发了标志位，在此处更新时间t0_r
	t0_r = GetUs();
	
	
	//声明一个变量保存A相当前电压
	uint8_t a = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3);
	//声明一个变量保存B相当前电压
	uint8_t b = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4);
	//通过比较A、B两相的电压来确定
	if((a == Bit_SET && b == Bit_RESET) || (a == Bit_RESET && b == Bit_SET) )//这两种情况轮胎正转，合并起来
		{
			encoder_R++;//此时正转
			if(direction_R < 0)
			{
				direction_R = +2;			
			}
			else
			{
				direction_R = +1;
			}			
		}
	else//其他两种情况轮胎反转，合并起来
		{
			encoder_R--;//此时反转
			if(direction_R > 0)
			{
				direction_R = -2;
			
			}
			else
			{
				direction_R = -1;
			}
			
		}

}
