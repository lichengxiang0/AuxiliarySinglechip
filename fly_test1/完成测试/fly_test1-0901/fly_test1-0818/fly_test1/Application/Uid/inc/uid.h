#ifndef	_UID_H
#define _UID_H
#include "stm32f1xx_hal.h"

//#define		UID_STORAGE_ADDRESS 	((uint32_t)0x0800FC00)		//(ADDR_FLASH_PAGE_63)
#define		UID_STORAGE_ADDRESS 	((uint32_t)0x0801FC00)		//(ADDR_FLASH_PAGE_127)

void	Run_Uid_Algorithm(void);
int8_t	Get_Uid_Judge_Result(void);




#endif
