/**
  ******************************************************************************
  * File Name          : task_gimbal_spd_ctrl.h
  * Description        : 这个文件主要实现1->云台速度环控制(陀螺) 2->电机重合判断
												及处理 3->缓降功能实现
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
#ifndef	_TASK_GIMBAL_SPD_CTRL_H
#define	_TASK_GIMBAL_SPD_CTRL_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Exported privates ---------------------------------------------------------*/
extern	FlagStatus	en_motor_y_and_motor_x_have_same_dir_flag;
extern  short motor_x_spd_pid_torque_out;
extern  short motor_y_spd_pid_torque_out;
extern  short motor_z_spd_pid_torque_out; 

	 
/* Exported function ---------------------------------------------------------*/	 
void	Task_Gimbal_Spd_Pid_Loop(void);






#ifdef __cplusplus
}
#endif	 
	 
#endif	

