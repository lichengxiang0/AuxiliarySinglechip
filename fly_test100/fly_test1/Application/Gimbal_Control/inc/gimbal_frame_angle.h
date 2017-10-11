/**
  ******************************************************************************
  * File Name          :gimbal_frame_angle.c
  * Description        :¶ÁÈ¡
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
#ifndef	_GIMBAL_FRAME_ANGLE_H
#define	_GIMBAL_FRAME_ANGLE_H


#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f1xx_hal.h"


	 
/* Exported variables ------------------------------------------------------------*/
extern	int16_t		motor_x_pos_data,					motor_y_pos_data,					motor_z_pos_data;	 
//extern	int16_t		motor_x_zero_pos_data ,		motor_y_zero_pos_data ,		motor_z_zero_pos_data ; 				//µÚ¶þÌ×
extern	float			motor_frame_angle_rad[3];



/* Exported functions ------------------------------------------------------------*/
	
void Get_Motor_Frame_Angle(uint8_t	*uc_recive_motor_machine_angle_p);


#ifdef __cplusplus
}
#endif	

#endif
