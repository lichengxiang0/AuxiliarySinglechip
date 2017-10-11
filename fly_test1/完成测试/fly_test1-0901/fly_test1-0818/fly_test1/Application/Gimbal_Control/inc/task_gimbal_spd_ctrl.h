/**
  ******************************************************************************
  * File Name          : task_gimbal_spd_ctrl.h
  * Description        : ����ļ���Ҫʵ��1->��̨�ٶȻ�����(����) 2->����غ��ж�
												������ 3->��������ʵ��
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

