#include "mpu6050_test.h"
#include "app_usart2.h"
#include "usart.h"
#include "app_mpu6050.h"
#include "delay.h"
#include "task.h"

void MPU6050_Test(void)
{
	
	App_MPU6050_Init();
	
	App_USART2_Init();
	
	while(1)
	{
		App_MPU6050_Update();//先更新
		float ax =App_MPU6050_GetAx();
		float ay =App_MPU6050_GetAy();
		float az =App_MPU6050_GetAz();
		
		float gx =App_MPU6050_GetGx();
		float gy =App_MPU6050_GetGy();
		float gz =App_MPU6050_GetGz();
		
		float temperature=App_MPU6050_GetTemperature();
		
		//通过串口发送给计算机验证一波
		My_USART_Printf(USART2,"%f,%f,%f,%f,%f,%f,%f\n",ax,ay,az,gx,gy,gz,temperature);
		Delay(500);
		
	}

}


static void USART2_Proc(void);//USART2发送数据的进程函数

//
//@简介：用来测试欧拉角，并使用vofa进行展示
//
void MPU6050_EularAngleTest(void)
{
	App_MPU6050_Init();
	
	App_USART2_Init();
	
	while(1)
	{
		App_MPU6050_Proc();
		USART2_Proc();
	
	
	
	}




}

//以后要学会自己写进程函数
//@简介：USART2发送的数据的进程函数
//每间隔10ms，就通过单片机把欧拉角发送给电脑
static void USART2_Proc()
{
	PERIODIC(10)
	float ax =App_MPU6050_GetAx();
	float ay =App_MPU6050_GetAy();
	float az =App_MPU6050_GetAz();
		
	float gx =App_MPU6050_GetGx();
	float gy =App_MPU6050_GetGy();
	float gz =App_MPU6050_GetGz();
		
	float temperature=App_MPU6050_GetTemperature();
	
	float yaw = App_MPU6050_GetYaw();
	float pitch = App_MPU6050_GetPitch();
	float roll = App_MPU6050_GetRoll();
	
	My_USART_Printf(USART2,"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",ax,ay,az,gx,gy,gz,temperature,yaw,pitch,roll);
	
}
