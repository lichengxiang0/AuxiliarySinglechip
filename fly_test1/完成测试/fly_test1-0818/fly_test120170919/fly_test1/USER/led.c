#include "led.h"

void delay_ms_nointer(uint32_t ms)
{
	uint32_t temp;
	SysTick->LOAD = 9010*ms;
	SysTick->VAL=0x00;  // ��ռ�����
	SysTick->CTRL=0x01;  //ʹ��
	do
	{
		temp=SysTick->CTRL;
	}while( (temp&0x01)&&( !(temp&(1<<16)) ) );  //�ȴ�ʱ�䵽��
	SysTick->CTRL=0x00;  //�رռ�����
	SysTick->VAL=0x00;  //��ռ�����
	
}



