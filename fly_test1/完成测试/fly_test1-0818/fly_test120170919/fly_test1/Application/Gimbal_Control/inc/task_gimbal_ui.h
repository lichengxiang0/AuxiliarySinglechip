/**
  ******************************************************************************
  * File Name          :task_gimbal_ui.c
  * Description        :处理用户界面的操作,1->状态切换 2->状态切换时执行的初始化
												3->用户速度控制及限制
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
#ifndef	_TASK_GIMBAL_UI_H
#define	_TASK_GIMBAL_UI_H


#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f1xx_hal.h"


/* Exported typedef ------------------------------------------------------------*/
typedef	enum{	SLEEP=0,	NORMAL,	SLEEP_TO_NORMAL,	NORMAL_TO_SLEEP}Gimbal_Power_Mode_Typedef;
typedef enum{	HORIZONTAL=0,	VERTICAL}Camera_Viewer_Mode_Typedef;
typedef enum{	NONE_FOLLOW=0,	Z_FOLLOW,	X_Z_FOLLOW,	TRACKER,	LOCK,	Z_FOLLOW_TO_NONE_FOLLOW,
							X_Z_FOLLOW_TO_NONE_FOLLOW,	X_Z_FOLLOW_TO_Z_FOLLOW}Gimbal_Work_Mode;



/* Exported variables ------------------------------------------------------------*/
extern	Gimbal_Power_Mode_Typedef 	en_gimbal_power_mode ;
extern	Camera_Viewer_Mode_Typedef	en_camera_viewer_mode ;
extern	Gimbal_Work_Mode						en_gimbal_work_mode ;

extern	float				f_earth_y_pos_pid_ref_offset_in_horizontal_mode,
										f_earth_y_pos_pid_ref_offset_in_vertical_mode;

extern	FlagStatus	en_earth_y_pos_pid_ref_offset_in_horizontal_mode_cal_flag,
										en_earth_y_pos_pid_ref_offset_in_vertical_mode_cal_flag;

extern	uint8_t		uc_receive_ui_mode_data,	uc_receive_ui_spd_data[2];

extern	float			f_pitch_user_ctrl_spd_ratio,	f_yaw_user_ctrl_spd_ratio;

/* Exported functions ------------------------------------------------------------*/
void	Task_Procese_Ui(void);

#ifdef __cplusplus
}
#endif	

#endif
