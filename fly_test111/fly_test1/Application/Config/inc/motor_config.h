/**
  ******************************************************************************
  * File Name          :motor_config.h
  * Description        :电机驱动的配置文件
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
#ifndef	_MOTOR_CONFIG_H
#define	_MOTOR_CONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32_bsp_flash.h"

/* Exported typedef ---------------------------------------------------------*/
typedef struct 
{  
	uint16_t							motor_driver_cal_flag;
	int16_t								zero_electrical_angle;
	int8_t								electrical_angle_direction;
	int8_t								torque_direction;
	
	uint8_t								uc_xor_verify_value;
  uint16_t              uw_motor_frame_zero_pos;
}Motor_Config_Data_Struct_Typedef;


/* Exported variables ---------------------------------------------------------*/
extern	Motor_Config_Data_Struct_Typedef	st_motor_config_data;


/* Exported function ---------------------------------------------------------*/
void Get_Motor_Config_Data(void);
void Save_Motor_Config_Data(void);


#ifdef __cplusplus
}
#endif


#endif


