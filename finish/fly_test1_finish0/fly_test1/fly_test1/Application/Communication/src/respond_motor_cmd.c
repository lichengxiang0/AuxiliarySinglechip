/**
  ******************************************************************************
  * File Name          : respond_motor_cmd.h
  * Description        : ����ļ���Ҫʵ�ֵ����������ʵ��
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


/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

#include "respond_motor_cmd.h"
#include "protocol.h"


#include "MC_Motor_Pos.h"
#include "task_motor_control.h"

//#include "motor_config.h"

/* External variables ------------------------------------------------------------*/
FlagStatus	en_motor_enter_debug_mode_flag = RESET;


/* External function --------------------------------------------------------*/
/**
  * @brief 	void	Motor_Process_Send_Pos_Ctrl_Cmd(uint8_t *cmd_data_array_p)
	* @note	 	����������еĿ�ܽ�
  */
void	Motor_Process_Send_Pos_Ctrl_Cmd(uint8_t	*cmd_data_array_p)
{
	/*���½Ǵ�ֵ*/
	cmd_data_array_p[en_frame_motor_id*2] = (uint8_t)(st_motor_pos_info.sc_motor_frame_angle>>8);
	cmd_data_array_p[en_frame_motor_id*2 + 1] = (uint8_t)(st_motor_pos_info.sc_motor_frame_angle);
}


/**
  * @brief 	void	Motor_Process_Set_Torque_Ref_Cmd(uint8_t *cmd_data_array_p)
	* @note	 	����������е����غ͵������ģʽ��Ϣ
  */
void	Motor_Process_Set_Torque_Ref_Cmd(uint16_t	cmd_data_array_p)
{
	uint8_t	uc_frame_motor_ctrl_mode_data_p;
	
	/*�����������*/
	//Stat_Curr_q_d_ref_src1.qI_Component1 = cmd_data_array_p;
	Stat_Curr_q_d_ref_src1.qI_Component1 = cmd_data_array_p;
	//((int16_t)(int8_t)cmd_data_array_p[en_frame_motor_id*2]<<8)|((uint8_t)cmd_data_array_p[en_frame_motor_id*2+1]);
	Stat_Curr_q_d_ref_src1.qI_Component2 = 0;
	
	/*����������Ʒ�ʽ*/
	uc_frame_motor_ctrl_mode_data_p = 0x00;
	//( cmd_data_array_p[9]>>((en_frame_motor_id-1)*2) )&0x03;
	
	if(0x00 == uc_frame_motor_ctrl_mode_data_p)
	{
		en_frame_motor_ctrl_mode = FRAME_MOTOR_CTRL_TORQUE;
	}
	else if(0x01 == uc_frame_motor_ctrl_mode_data_p)
	{
		en_frame_motor_ctrl_mode = FRAME_MOTOR_CTRL_SPD;
	}
	else if(0x10 == uc_frame_motor_ctrl_mode_data_p)
	{
		en_frame_motor_ctrl_mode = FRAME_MOTOR_CTRL_POS;
	}
}



/**
  * @brief 	void	Process_Motor_Enter_Debug_Mode_Cmd(uint8_t *cmd_data_array_p)
	* @note	 	�˺������������������debugģʽ
  */
void	Process_Motor_Enter_Debug_Mode_Cmd(uint8_t *cmd_data_array_p)
{
	if(cmd_data_array_p[2] == 0x01)
	{
		en_motor_enter_debug_mode_flag = SET;   //�������ģʽ
	}
	else if(cmd_data_array_p[2] == 0x02)
	{
		en_motor_enter_debug_mode_flag = RESET;     //����������תģʽ
	}
}



/**
  * @brief 	void	Process_Motor_Set_Zero_Elec_Angle_Cmd(uint8_t *cmd_data_array_p)
	* @note	 	�˺������趨������ڶ���λģʽ
  */
void	Process_Motor_Set_Zero_Elec_Angle_Cmd(uint8_t *cmd_data_array_p)
{
	if( (cmd_data_array_p[2] == 0x04)||(cmd_data_array_p[2] == en_frame_motor_id) )
	{
		st_mc_state = START;	
		Stat_Curr_q_d_ref.qI_Component1 = 0;
		Stat_Curr_q_d_ref.qI_Component2 = ((int16_t)(int8_t)cmd_data_array_p[3] << 8) + cmd_data_array_p[4];
	}
}


/**
  * @brief 	void	Process_Motor_Save_Zero_Elec_Angle_Cmd(uint8_t *cmd_data_array_p)
	* @note	 	�˺������趨���������λ
  */
void	Process_Motor_Save_Zero_Elec_Angle_Cmd(uint8_t *cmd_data_array_p)
{
	if( (cmd_data_array_p[2] == 0x04)||(cmd_data_array_p[2] == en_frame_motor_id) )
	{
		st_motor_pos_info.uw_motor_elec_zero_pos = st_motor_pos_info.uw_motor_machine_pos;
		Stat_Curr_q_d_ref.qI_Component1 = 0;
		Stat_Curr_q_d_ref.qI_Component2 = 0;
		
		/*������λֵ*/
		//Save_Motor_Config_Data();
		
		st_mc_state = RUN;
	}
}



/**
  * @brief 	void	Process_Motor_Set_Torque_Cmd(uint8_t *cmd_data_array_p)
	* @note	 	�˺���������������������
  */
void	Process_Motor_Set_Torque_Cmd(uint8_t *cmd_data_array_p)
{
	if( (cmd_data_array_p[2] == 0x04)||(cmd_data_array_p[2] == en_frame_motor_id) )
	{
		Stat_Curr_q_d_ref.qI_Component1 = sc_torque_direction*( ( ( (int16_t)(int8_t)cmd_data_array_p[3] ) <<8)|cmd_data_array_p[4]);
		Stat_Curr_q_d_ref.qI_Component2 = sc_torque_direction*0; 
	}

}


/**
  * @brief 	void	Process_Motor_Set_Torque_Dir_Cmd(uint8_t *cmd_data_array_p)
	* @note	 	�˺����������������ط�������
  */
void	Process_Motor_Set_Torque_Dir_Cmd(uint8_t *cmd_data_array_p)
{
	if( (cmd_data_array_p[2] == 0x04)||(cmd_data_array_p[2] == en_frame_motor_id) )
	{
		sc_torque_direction = -sc_torque_direction;
		Stat_Curr_q_d_ref.qI_Component1 = -Stat_Curr_q_d_ref.qI_Component1;
		Stat_Curr_q_d_ref.qI_Component2 = -Stat_Curr_q_d_ref.qI_Component2; 

		//Save_Motor_Config_Data();//������λֵ
	}
}


/**
  * @brief 	void	Process_Motor_Set_Elec_Angle_Dir_Cmd(uint8_t *cmd_data_array_p)
	* @note	 	�˺���������������Ƕȷ�������
  */
void	Process_Motor_Set_Elec_Angle_Dir_Cmd(uint8_t *cmd_data_array_p)
{
	if( (cmd_data_array_p[2] == 0x04)||(cmd_data_array_p[2] == en_frame_motor_id) )
	{
		st_motor_pos_info.sc_motor_elec_angle_dir = -st_motor_pos_info.sc_motor_elec_angle_dir;
		//Save_Motor_Config_Data();		//������λֵ
	}
}
/**
  * @brief 	void	Process_Motor_Query_Zero_Elec_Angle_Cmd(uint8_t *cmd_data_array_p)
	* @note	 	�˺���������������Ƕ���λ��ѯ
  */
void	Process_Motor_Query_Zero_Elec_Angle_Cmd(uint8_t *cmd_data_array_p)
{
	uint8_t	uc_fdb_cmd_data_array_p[5];	
	
	if( (cmd_data_array_p[2] == 0x04)||(cmd_data_array_p[2] == en_frame_motor_id) )
	{
		uc_fdb_cmd_data_array_p[0] = 0x54;
		uc_fdb_cmd_data_array_p[1] = 0x63;
		uc_fdb_cmd_data_array_p[2] = (uint8_t)en_frame_motor_id;
		uc_fdb_cmd_data_array_p[3] = (uint8_t)(st_motor_pos_info.uw_motor_elec_zero_pos>>8);
		uc_fdb_cmd_data_array_p[4] = (uint8_t)st_motor_pos_info.uw_motor_elec_zero_pos;
	}
}


/**
  * @brief 	void	Process_Motor_Set_Zero_Frame_Angle_Cmd(uint8_t *cmd_data_array_p)
	* @note	 	�˺����������������������λ��У׼
  */
void Process_Motor_Set_Zero_Frame_Angle_Cmd(uint8_t *cmd_data_array_p)
{
	st_motor_pos_info.uw_motor_frame_zero_pos = st_motor_pos_info.uw_motor_machine_pos;
	
	//Save_Motor_Config_Data();  //������λֵ
}


/**
  * @brief 	void	Process_Motor_Query_Config_Info_Cmd(uint8_t *cmd_data_array_p)
	* @note	 	�˺���������Ӧ��ѯ������Ϣ
  */
void	Process_Motor_Query_Config_Info_Cmd(uint8_t *cmd_data_array_p)
{
	uint8_t  uc_fdb_cmd_data_array_p[17]={0};
	
	/*�����������Ϣ��ѯ*/
	if(en_frame_motor_id == cmd_data_array_p[2])
	{
		uc_fdb_cmd_data_array_p[0] = (uint8_t)(CMD_RELAY_QUERY_CONFIG_INFO>>8);
		uc_fdb_cmd_data_array_p[1] = (uint8_t)CMD_RELAY_QUERY_CONFIG_INFO;
		uc_fdb_cmd_data_array_p[2] = cmd_data_array_p[2];
		uc_fdb_cmd_data_array_p[3] = cmd_data_array_p[3];
		
		switch (cmd_data_array_p[3])
		{
//			case 0x00:	/*У׼��־*/			
//				uc_fdb_cmd_data_array_p[4] = (uint8_t)(st_motor_config_data.motor_driver_cal_flag>>8);	
//				uc_fdb_cmd_data_array_p[5] = (uint8_t)(st_motor_config_data.motor_driver_cal_flag);	
//				break;
//			
//			case 0x01:	/*����λ*/
//				uc_fdb_cmd_data_array_p[4] = (uint8_t)(st_motor_config_data.zero_electrical_angle>>8);
//				uc_fdb_cmd_data_array_p[5] = (uint8_t)(st_motor_config_data.zero_electrical_angle);
//				break;
//			
//			case 0x02:	/*��е��λ*/
//				uc_fdb_cmd_data_array_p[4] = (uint8_t)(st_motor_config_data.uw_motor_frame_zero_pos>>8);
//				uc_fdb_cmd_data_array_p[5] = (uint8_t)(st_motor_config_data.uw_motor_frame_zero_pos);
//				break;
			
			default:
				break;
		}
		
		//�������
		Pack_And_Send_Cmd(FEEDBACK_CMD_TXBUF,uc_fdb_cmd_data_array_p,MODULE_ADD_BYTE);
	}

}
