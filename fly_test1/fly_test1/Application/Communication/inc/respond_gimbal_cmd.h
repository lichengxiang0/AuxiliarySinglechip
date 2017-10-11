/**
  ******************************************************************************
  * File Name          :respond_gimbal_cmd.h
  * Description        :处理1->上位机校准命令 2->离线校准 3->升级 4->其他
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
#ifndef	_RESPOND_GIMBAL_CMD_H
#define	_RESPOND_GIMBAL_CMD_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
	 

/* Exported variables --------------------------------------------------------*/	 
extern	FlagStatus	en_gimbal_enter_debug_mode_flag;
	 
/*记录姿态板上校准项目完成情况*/
/*0000 0000:bit7=0 bit0=gyro bit1:acc_scale bit2:acc_水平 bit3：整机零位*/
extern	uint8_t			uc_gimbal_cal_ok_item;


/* Exported functions --------------------------------------------------------*/	
//void	Gimbal_Process_Send_Pos_Ctrl_Cmd(uint8_t *cmd_data_array_p);
//void	Gimbal_Process_Set_Torque_Ref_Cmd(uint8_t *cmd_data_array_p); 


void	Process_Gimbal_Enter_Debug_Mode_Cmd(uint8_t *cmd_data_array_p);
void 	Process_Gimbal_Set_Zero_Frame_Angle_Cmd(uint8_t *cmd_data_array_p);
void 	Process_Gimbal_Cal_Gyro_Cmd(uint8_t *cmd_data_array_p);
void 	Process_Gimbal_Cal_Acc_Cmd(uint8_t *cmd_data_array_p);
void	Process_Query_Ahrs_Cal_Status_Cmd(uint8_t *cmd_data_array_p);
void	Process_Save_Roll_Ref_Cmd(uint8_t *cmd_data_array_p);
void	Process_Set_Spd_Ratio_Cmd(uint8_t *cmd_data_array_p);

//void	Process_Ahrs_Query_Config_Info_Cmd(uint8_t *cmd_data_array_p);

//void	Task_Gimbal_Cal_Loop(void);



#ifdef __cplusplus
}
#endif	 
	 
#endif	

