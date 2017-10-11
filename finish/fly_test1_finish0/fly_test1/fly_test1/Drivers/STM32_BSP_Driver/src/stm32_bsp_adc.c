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

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32_bsp_adc.h"
#include "stm32_bsp_timer.h"

/* External variables --------------------------------------------------------*/	
volatile	uint16_t		ADC1_RegularConvertedValueTab[ADC_CHANL_NUM];
volatile	FlagStatus	en_adc1_conv_cplt_flag = RESET;  //ADC采样完成标志


/* External functions --------------------------------------------------------*/	
void	ADC_Start(void)
{
	HAL_ADCEx_Calibration_Start(&hadc1);
	
	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)ADC1_RegularConvertedValueTab,ADC_CHANL_NUM);
//	__HAL_DMA_DISABLE_IT(hadc.DMA_Handle,DMA_IT_HT);
}
/**
	* @brief：void	Start_Adc_Conv_Period(uint8_t uc_adc_conv_period_ms_p)
	* @note:	周期性的启动ADC采样
	*/
void	Start_Adc_Conv_Period(uint8_t uc_adc_conv_period_ms_p)
{
	static	User_Timer_Typedef	st_adc_conv_period_timer = USER_TIMER_INIT_VALUE;
	
	Start_User_Timer(&st_adc_conv_period_timer);
	Update_User_Timer_Cnt(&st_adc_conv_period_timer);
	if(st_adc_conv_period_timer.ul_timer_cnt > uc_adc_conv_period_ms_p)
	{
		HAL_ADC_Start(&hadc1);
	}
}
