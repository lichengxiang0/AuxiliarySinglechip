/**
  ******************************************************************************
  * File Name          :gimbal_spd_ctrl_lib.c
  * Description        :动态调整速度环PID参数
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
#ifndef	_GIMBAL_SPD_CTRL_LIB_H
#define	_GIMBAL_SPD_CTRL_LIB_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "pid_regulator.h"


extern	void	Cal_Motor_Spd_Pid_Para(PID_Struct_Typedef *st_motor_y_spd_pid_struct_p, PID_Struct_Typedef *st_motor_z_spd_pid_struct_p,float ** coordinate_transform_matrix_p);
	 


#ifdef __cplusplus
}
#endif	 
	 
#endif	

