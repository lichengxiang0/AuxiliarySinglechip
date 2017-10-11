/**
  ******************************************************************************
  * File Name          :respond_gimbal_cmd.h
  * Description        :����1->��λ��У׼���� 2->����У׼ 3->���� 4->����
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
	 
/*��¼��̬����У׼��Ŀ������*/
/*0000 0000:bit7=0 bit0=gyro bit1:acc_scale bit2:acc_ˮƽ bit3��������λ*/
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

