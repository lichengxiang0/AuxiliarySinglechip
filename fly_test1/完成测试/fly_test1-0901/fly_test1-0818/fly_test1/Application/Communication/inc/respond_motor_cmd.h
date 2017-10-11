/**
  ******************************************************************************
  * File Name          : respond_motor_cmd.h
  * Description        : 这个文件主要实现电机相关命令的实现
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
#ifndef	_RESPOND_MOTOR_CMD_H
#define	_RESPOND_MOTOR_CMD_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
/* Exported variables --------------------------------------------------------*/	
extern	FlagStatus	en_motor_enter_debug_mode_flag;
	 
	 
/* Exported functions --------------------------------------------------------*/	
void	Motor_Process_Send_Pos_Ctrl_Cmd(uint8_t	*cmd_data_array_p);
void	Motor_Process_Set_Torque_Ref_Cmd(uint16_t cmd_data_array_p);	 
	 
void	Process_Motor_Enter_Debug_Mode_Cmd(uint8_t *cmd_data_array_p);
void	Process_Motor_Set_Zero_Elec_Angle_Cmd(uint8_t *cmd_data_array_p);
void	Process_Motor_Save_Zero_Elec_Angle_Cmd(uint8_t *cmd_data_array_p);
void	Process_Motor_Set_Torque_Cmd(uint8_t *cmd_data_array_p);
void	Process_Motor_Set_Torque_Dir_Cmd(uint8_t *cmd_data_array_p);
void	Process_Motor_Set_Elec_Angle_Dir_Cmd(uint8_t *cmd_data_array_p);
void	Process_Motor_Query_Zero_Elec_Angle_Cmd(uint8_t *cmd_data_array_p); 
void	Process_Motor_Query_Config_Info_Cmd(uint8_t *cmd_data_array_p);
	
void 	Process_Motor_Set_Zero_Frame_Angle_Cmd(uint8_t *cmd_data_array_p);	 
	 

#ifdef __cplusplus
}
#endif

#endif








