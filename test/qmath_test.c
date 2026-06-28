#include "math.h"
#include "app_usart2.h"
#include "delay.h"

//
//@简介：对基本运算的运算素的进行测试
//整数的+-*/
//浮点数的+-*/
//三角函数
void Qmath_Test(void)
{
	App_USART2_Init();
	
	while(1)
	{
		uint8_t a = 1;
		uint8_t b = 2;
//		//整数加法
//		int32_t t1 = GetTick();
//		for(uint32_t i=0; i <1000000;i++)
//		{
//			uint8_t c= a+b;
//		
//		}
//		uint32_t t2 = GetTick();
//		
//		float T1 =(t2-t1)/1000000.0f * 1000.0f;
		
//		//整数减法
//		uint32_t t3 = GetTick();
//		for(uint32_t i=0; i <1000000;i++)
//		{
//			int8_t d= a-b;
//		
//		}
//		uint32_t t4 = GetTick();
//		
//		float T2 =(t4-t3)/1000000.0f * 1000.0f;
		
//		//整数乘/除法
//		int32_t t3 = GetTick();
//		for(uint32_t i=0; i <1000000;i++)
//		{
//			float e= a/b;
//		
//		}
//		uint32_t t4 = GetTick();
//		
//		float T2 =(t4-t3)/1000000.0f * 1000.0f;
		
		
		//整数乘/除法
		int32_t t3 = GetTick();
		for(uint32_t i=0; i <1000000;i++)
		{
			float e= atan2f(a,b);
		
		}
		int32_t t4 = GetTick();
		
		float T2 =(t4-t3);
		My_USART_Printf(USART2,"%f\n",T2);
		Delay(500);
	
	}


}

