/**
  ******************************************************************************
  * File Name          : task_motor_control.h
  * Description        : 初始化电流偏置，AD，PWM_TIMER，进行FOC运算
												 根据模式和电机位置选择电机力矩参考输入源(gyro_spd_out/motor_spd_out/motor_pos->motor_spd_out)
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
#ifndef	_TASK_MOTOR_CONTROL_H
#define	_TASK_MOTOR_CONTROL_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
	 
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "MC_FOC_Drive.h"

/* Exported typedef ------------------------------------------------------------*/
typedef enum{FRAME_MOTOR_Y=1,	FRAME_MOTOR_Z,	FRAME_MOTOR_X}Frame_Motor_Id_Typedef;	
typedef	enum{FRAME_MOTOR_CTRL_TORQUE = 1,	FRAME_MOTOR_CTRL_SPD,	FRAME_MOTOR_CTRL_POS}Frame_Motor_Ctrl_Mode_Typedef;

/* Exported variables ------------------------------------------------------------*/
extern	Frame_Motor_Id_Typedef 				en_frame_motor_id;

extern	SystStatus_t 									st_mc_state;

extern	Curr_Components								Stat_Curr_q_d_ref_src1,	Stat_Curr_q_d_ref_src2;
extern	int8_t												sc_torque_direction;

extern	Frame_Motor_Ctrl_Mode_Typedef	en_frame_motor_ctrl_mode;	

/* Exported functions ------------------------------------------------------------*/
void	Task_Motor_Ctrl_Init(void);
void	Task_Get_Motor_Pos_Info(void);
void	Task_Update_Motor_Torque_Ref(void);

#ifdef __cplusplus
}
#endif


#endif
