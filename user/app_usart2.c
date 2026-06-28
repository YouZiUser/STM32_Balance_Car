#include "stm32f10x.h"                  // Device header
#include "app_usart2.h"

//
//@简介：对串口进行初始化，串口2作为平衡车的调试接口
//
void App_USART2_Init(void)
{
	//1.对IO口进行初始化，这里不进行重映射
	//TX对应PA2，RX对应PA3
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure ={0};
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_Init(GPIOA , &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//2.对USART2进行配置
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 , ENABLE);
	USART_InitTypeDef USART_InitStructure = {0};
	USART_InitStructure.USART_BaudRate=921600 ;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode=USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_Init(USART2,&USART_InitStructure);
	
	USART_Cmd(USART2,ENABLE);



}
