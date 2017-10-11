#include "mynop.h"

void Delay_nop_2us()
{	
	int j;
	for(j=0;j<11;j++)
	{
		__NOP();__NOP();__NOP();__NOP();
	}
}

void Delay_nop_4us()
{	
	int j;
	for(j=0;j<24;j++)
	{
		__NOP();__NOP();__NOP();__NOP();
	}
}

void Delay_nop_1us()
{	
	int j;
	for(j=0;j<5;j++)
	{
		__NOP();__NOP();__NOP();__NOP();
	}
}



