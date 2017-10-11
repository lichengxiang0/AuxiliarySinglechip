/**
  ******************************************************************************
  * File Name          :stm32_bsp_init.h
  * Description        :系统配置文件
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
#ifndef	_STM32_BSP_INIT_H
#define	_STM32_BSP_INIT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
	 

//#define	AS5600_I2C_HANDLE			(hi2c2)
//	 
//#define GYRO_I2C_HANDLE				(hi2c1)	
//#define ACC_I2C_HANDLE				(hi2c1)	
	 
//extern	ADC_HandleTypeDef hadc1;
//extern	DMA_HandleTypeDef hdma_adc1;
//	 
//extern	I2C_HandleTypeDef hi2c1;
//extern	I2C_HandleTypeDef hi2c2;

extern	TIM_HandleTypeDef htim1;
extern	TIM_HandleTypeDef htim2;
extern	TIM_HandleTypeDef htim3;
	 
//extern	UART_HandleTypeDef huart2;


#ifdef __cplusplus
}
#endif


#endif





















