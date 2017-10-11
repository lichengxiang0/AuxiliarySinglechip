/**
  ******************************************************************************
  * File Name          : system_control.h
  * Description        : 系统控制文件
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef	_SYSTEM_CONTROL_H
#define	_SYSTEM_CONTROL_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
	 	 
	
	 
/* Exported function ---------------------------------------------------------*/
void	System_Init(void);
void	System_Run(int8_t	uc_uid_result_p);

#ifdef __cplusplus
}
#endif


#endif


