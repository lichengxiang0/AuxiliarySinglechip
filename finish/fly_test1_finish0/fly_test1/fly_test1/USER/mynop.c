#include "mynop.h"

/* nop—”≥Ÿ1Œ¢√Ó */
void nop_delay_1us()
{
	uint8_t i = 0;
	
	for(i=0; i<6 ; i++)
	{
	__NOP();
	}
}

/* nop—”≥Ÿ2Œ¢√Ó */
void nop_delay_2us()
{
	uint8_t i = 0;
	
	for(i=0; i<13 ; i++)
	{
	__NOP();
	}
}



