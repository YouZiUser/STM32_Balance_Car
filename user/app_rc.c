#include "app_rc.h"
#include "string.h"
#include "stdio.h"
#include "app_control.h"

#define CMD_MAX_LEN 64//定义指令的最长长度为64
//用指令的最长长度定义下方数组的长度，保证指令不丢失
static char intBuf[CMD_MAX_LEN];//专门用于中断程序的缓冲区
static char transBuf[CMD_MAX_LEN];//在中断程序和进程函数之间转运数据的缓冲区
static char procBuf[CMD_MAX_LEN];//专门用于进程函数的缓冲区

//自定义标志位 lineReceivedFlag 表示一行字符串接收完成的标志位
static volatile uint8_t lineReceivedFlag = 0;

static uint16_t intBufCursor = 0;//intBuf的游标，指向下一个空白位置
//
//@简介：对遥控器模块进行初始化
//
void App_RC_Init(void)
{
	//1.对USART3进行初始化
	//无需重映射，直接使用PB10-->TX  PB11-->RX
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure = {0};
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
				
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	USART_InitTypeDef USART_InitStructure = {0};
	USART_InitStructure.USART_BaudRate = 9600;//蓝牙芯片的波特率固定为9600，收发双方波特率应当固定一致
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits =USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_Init(USART3,&USART_InitStructure);
	
	//闭合USART的总开关
	USART_Cmd(USART3,ENABLE);
	//2.配置USART3的中断
	USART_ITConfig(USART3, USART_IT_RXNE , ENABLE);//中断标志位选择RXNE
	NVIC_InitTypeDef NVIC_InitStructure = {0};
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

//
//@简介：遥控器的进程函数，需要在main的while循环中不断被执行
//
void App_RC_Proc(void)
{
	if(lineReceivedFlag == 1)
	{
		strcpy(procBuf,transBuf);
		lineReceivedFlag = 0;//清除自定义标志位
		//上述部分完成对字符串的接收
		
		//对字符串解析 举一反三 学习字符串类型指令解析的思路
		
		//指令格式：move turnSpeed moveSpeed\n
		
		//@简介：比较两个字符串的前n个字符是否相等
		//使用编程接口strnacasecmp  在string.h中
		if(strncasecmp(procBuf, "move ", 5) == 0)//strnacasecmp作用是比较procBuf的前5个字符是否与"move "相等，相等为0，不相等则非零
		{
			//处理move指令
			//
			//@简介：编程接口sscanf读取格式化字符串中的数据 包含在stdio.h中
			//@参数：str - 被读取的字符串
			//@参数：format - 字符串的格式
			//@参数：..， - 可变参数
			//@返回：成功解析的参数个数  在此处的指令中，需要turnspeed和movespeed这两个参数解析成功，才能执行程序
			int turnSpeed , moveSpeed;//定义两个整形，转向和移速
			if(sscanf(procBuf, "move %d %d", &turnSpeed,&moveSpeed)==2)// \n 已经被解析为\0，不用再写
			{
				//对字符串执行
				App_Control_SetMoveSpeed(-moveSpeed * 0.01f * 0.7f);//moveSpeed摇杆向上为-100，向下为+100 再转为每刻度对应的速度 即可控制平衡车车速
				App_Control_SetTurnSpeed(turnSpeed * 0.01f * 15.0f );//turnSpeed摇杆向右为+100，向左为-100 再转为每刻度对应的转向速度 即可控制平衡车转向
			}									
		}		
	}
	
}

//
//@简介：串口3的中断响应函数
//
void  USART3_IRQHandler(void)
{
	if(USART_GetFlagStatus(USART3,USART_FLAG_RXNE) == SET)
	{
		uint8_t data = USART_ReceiveData(USART3);//如果是RXNE触发的中断，把这个接收到的字符存储起来
		
		if(data != '\n')//判断这个字符是否为\n，如果是，则代表指令传输结束了，不是就写入intBuf[]
		{
			intBuf[intBufCursor++] = data;
			//intBuf[intBufCursor] = data;
			//intBufCursor ++;
		}
		if(data == '\n')
		{
			intBuf[intBufCursor] = '\0';
			intBufCursor = 0;//此时把游标移到intBuf[]最前面
			strcpy(transBuf,intBuf);//此函数是用来复制字符串的，把intBuf的值赋给transBuf,此函数在string.h中
			lineReceivedFlag = 1;//给自定义标志位 置1
		}
	}
	USART_ClearFlag(USART3, USART_FLAG_RXNE);
}
