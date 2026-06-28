#ifndef APP_USART2_H
#define APP_USART2_H

#include "stm32f10x.h"                  // Device header
//先引用单片机的头文件
#include "usart.h"


//在.h文件中，写外部能够调用函数的声明
//函数的具体在.c中实现
void App_USART2_Init(void);



//为什么宏定义可以防止重复重复引用
//C语言中学习
#endif  //宏定义的作用是防止头文件被重复引用
