/**
  ******************************************************************************
  * File Name          : task_common_functions.h
  * Description        : 这个文件主要是用来放置task层共用的函数
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
#ifndef	_TASK_COMMON_FUNCTIONS_H
#define	_TASK_COMMON_FUNCTIONS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"


/* Exported functions ------------------------------------------------------------*/
void	Bond_Int16_t(int16_t *sw_data_p,	int16_t sw_data_min_p,	int16_t sw_data_max_p);
void	BondFloat(float *data,	float data_min,	float data_max);
void	BondTriangle(float *angle_p);
	 
FlagStatus	Average_Filter_Float(float* p_f_data_in_p,	float* p_f_data_out_p,uint8_t	*uc_flt_cycle_index,uint8_t	uc_flt_data_num);
FlagStatus	Average_Filter_Uint16(uint16_t* p_uw_data_in_p,	uint16_t* p_uw_data_out_p,uint8_t	*uc_flt_cycle_index,uint8_t	uc_flt_data_num);
int8_t			Get_Float_Sign(float	f_float_data_p);
	 
#ifdef __cplusplus
}
#endif


#endif

