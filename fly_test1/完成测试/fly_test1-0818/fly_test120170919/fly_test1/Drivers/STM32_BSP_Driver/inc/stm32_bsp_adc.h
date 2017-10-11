/**
  ******************************************************************************
  * File Name          : stm32_bsp_adc.h
  * Description        : ADC相关函数：ADC的启动和DMA的相关配置
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
#ifndef	_STM32_BSP_ADC_H
#define	_STM32_BSP_ADC_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
	 
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32_bsp_peripheral_init_export.h"

	 
/* Exported defines --------------------------------------------------------*/
#define		ADC_CHANL_NUM					( (uint8_t)2 )
	 
#define PHASE_A_CURRENT_ADC_DATA_INDEX    ((uint8_t)0)
#define PHASE_B_CURRENT_ADC_DATA_INDEX    ((uint8_t)1)	 
#define BUS_VOTAGE_ADC_DATA_INDEX					((uint8_t)2)
	 
/* Exported variables --------------------------------------------------------*/	 
extern	volatile	uint16_t		ADC1_RegularConvertedValueTab[ADC_CHANL_NUM];
extern	volatile	FlagStatus	en_adc1_conv_cplt_flag;	 
	 
/* Exported functions --------------------------------------------------------*/
void	ADC_Start(void);	 
void	Start_Adc_Conv_Period(uint8_t uc_adc_conv_period_ms_p);	  

#ifdef __cplusplus
}
#endif


#endif

