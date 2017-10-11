#include "led.h"

void delay_ms_nointer(uint32_t ms)
{
	uint32_t temp;
	SysTick->LOAD = 9010*ms;
	SysTick->VAL=0x00;  // 清空计数器
	SysTick->CTRL=0x01;  //使能
	do
	{
		temp=SysTick->CTRL;
	}while( (temp&0x01)&&( !(temp&(1<<16)) ) );  //等待时间到达
	SysTick->CTRL=0x00;  //关闭计数器
	SysTick->VAL=0x00;  //清空计数器
	
}



