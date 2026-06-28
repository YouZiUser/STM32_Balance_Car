#include "bat_test.h"
#include "app_usart2.h"
#include "app_bat.h"
#include "delay.h"
//
//@简介：电池电压监控模块测试程序
//     通过串口2把电压发送给电脑
//		使用vofa显示曲线	
//测试程序的实现就按照类似与main方法的形式来写
void Bat_Test(void)
{
	App_Bat_Init();//初始化电压监控模块
	App_USART2_Init();//初始化串口
	while(1)
	{
		//每间隔10ms显示一个数据点
		float volt = App_Bat_Get();//调用编程接口App_Bat_Get()获取电压值
		My_USART_Printf(USART2,"%.3f/n",volt);
		Delay(10);
		
	
	
	}



}
