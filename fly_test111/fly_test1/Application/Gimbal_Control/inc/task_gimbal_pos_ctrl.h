/**
  ******************************************************************************
  * File Name          : task_gimbal_pos_ctrl.h
  * Description        : 这个文件主要实现1->云台位置环控制 2->随动电机判断
													3->启动柔顺
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
#ifndef	_TASK_GIMBAL_POS_CTRL_H
#define	_TASK_GIMBAL_POS_CTRL_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "gimbal_follow_ctrl.h"

/* Exported variables ------------------------------------------------------------*/
extern	volatile	FlagStatus	en_task_gimbal_pos_pid_250Hz_loop_flag;
	 
extern	Follow_Motor_Typedef		en_yaw_follow_motor;
extern	Follow_Motor_Typedef		en_pitch_follow_motor;
	
extern	float										f_yaw_follow_spd_ratio,	f_pitch_follow_spd_ratio; 

//test	 
extern	FlagStatus							en_earth_x_pos_pid_enable_flag,				en_earth_z_pos_pid_enable_flag;
extern	FlagStatus							en_pitch_follow_pos_pid_enable_flag,	en_yaw_follow_pos_pid_enable_flag;
extern	FlagStatus							en_tripus_mode_flag; 
	 
extern	FlagStatus							en_pitch_pos_pid_enable_flag;
	 
extern  float	f_earth_pos_pid_out[3];

/* Exported functions ------------------------------------------------------------*/
void	Task_Gimbal_Pos_Pid_Loop(void);






#ifdef __cplusplus
 extern "C" {
#endif

#endif	
