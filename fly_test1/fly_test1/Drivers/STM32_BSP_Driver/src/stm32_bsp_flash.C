/**
  ******************************************************************************
  * File Name          : stm32_bsp_flash.c
  * Description        : flash的写入和读取
	******************************************************************************
  *
  * COPYRIGHT(c) 2016 aibird
  *
  ******************************************************************************
	*
	* Author:
	*
	******************************************************************************
	*/
	
#include "stm32f1xx_hal.h"
#include "stm32_bsp_flash.h"


//pagesize是1kb   0x400

void FlashProgram(uint32_t addr,void *buffer,int len)
{
		uint32_t  PAGEError = 0;
    uint32_t Address ;
    uint32_t Dwordlen ;
    uint32_t Wordlen ;
    uint32_t HalfWordlen ;


    uint64_t *pdword;
    uint32_t *pword;
    uint16_t *phalfword;
    uint32_t step;
		static FLASH_EraseInitTypeDef EraseInitStruct;
    //diag_printf("start %d \n",OSTimeGet());
    /* Unlock the Flash to enable the flash control register access *************/
    //ResetWDg(2000);
    HAL_FLASH_Unlock();

    /* Erase the user Flash area
      (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

		
		
    /* Get the number of the start and end sectors */
		EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
		EraseInitStruct.PageAddress = addr;
		if(len>1000)
		{
			EraseInitStruct.NbPages     = 1+(len/4) / FLASH_PAGE_SIZE;		
		}
		else
		{
			EraseInitStruct.NbPages     = 1;
		}

		if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
		{
		
		}
		HAL_FLASH_Lock();//擦完了就锁起来
    /* Program the user Flash area word by byte , halword , word , double word **********/
    Address = addr;
    Dwordlen =(len / 8)+1;//64位
    Wordlen = (len /4) +1;//
    HalfWordlen = (len/2)+1;


#if 1
    Dwordlen =0;//64位
    Wordlen = 0;
    HalfWordlen = (len/2)+(len%2);

#endif
		
		
    step = 0 ;
    pdword = (uint64_t *)buffer;
		HAL_FLASH_Unlock();
    while( step < Dwordlen )
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, *pdword) == HAL_OK)
        {		
            Address = Address + 8;
            ++pdword;
            ++step;
        }
        else
        {

        }
    }
		
		

    step = 0;
    pword = (uint32_t *)pdword;
    while( step < Wordlen )
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, *pword) == HAL_OK)
        {
            Address = Address + 4;
            ++pword;
            ++step;
        }
        else
        {
            /* Error occurred while writing data in Flash memory.
               User can add here some code to deal with this error */
            //while (1)
            {
                //diag_printf("em flash program error \n");
            }
        }
    }

    step = 0;
    phalfword = (uint16_t *)pword;
    while( step < HalfWordlen)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, Address, *phalfword) == HAL_OK)
        {
            Address = Address + 2;
            ++phalfword;
            ++step;
        }
        else
        {

        }
    }
		HAL_FLASH_Lock();
}




void FlashRead(uint32_t addr,void *buffer,uint16_t len)
{
//	buffer = 	(void *)addr;
	uint16_t i=0;
	for(i=0;i<len;i++)
	{
		*(uint8_t *)((uint32_t)buffer + i) = *(uint8_t *)(addr + i);
	}
}


















