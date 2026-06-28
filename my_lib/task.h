#include "delay.h"

#define PERIODIC(T) \
static uint32_t nxt = 0; \
if(GetTick() < nxt) return; \
nxt += (T);
//将非阻塞式延时封装为一个宏，在进程函数需要延时的时候，直接调用即可


#define PERIODIC_START(NAME, T) \
static uint32_t NAME##_nxt = 0; \
if(GetTick() >= NAME##_nxt) {\
NAME##_nxt += (T);

#define PERIODIC_END }
