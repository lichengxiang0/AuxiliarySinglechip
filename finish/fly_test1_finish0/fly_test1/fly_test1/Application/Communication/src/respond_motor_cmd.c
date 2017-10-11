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
	* @note	 	电机更新下行的框架角
  */
void	Motor_Process_Send_Pos_Ctrl_Cmd(uint8_t	*cmd_data_array_p)
{
	/*更新角传值*/
	cmd_data_array_p[en_frame_motor_id*2] = (uint8_t)(st_motor_pos_info.sc_motor_frame_angle>>8);
	cmd_data_array_p[en_frame_motor_id*2 + 1] = (uint8_t)(st_motor_pos_info.sc_motor_frame_angle);
}


/**
  * @brief 	void	Motor_Process_Set_Torque_Ref_Cmd(uint8_t *cmd_data_array_p)
	* @note	 	电机处理上行的力矩和电机控制模式信息
  */
void	Motor_Process_Set_Torque_Ref_Cmd(uint16_t	cmd_data_array_p)
{
	uint8_t	uc_frame_motor_ctrl_mode_data_p;
	
	/*解析电机力矩*/
	//Stat_Curr_q_d_ref_src1.qI_Component1 = cmd_data_array_p;
	Stat_Curr_q_d_ref_src1.qI_Component1 = cmd_data_array_p;
	//((int16_t)(int8_t)cmd_data_array_p[en_frame_motor_id*2]<<8)|((uint8_t)cmd_data_array_p[en_frame_motor_id*2+1]);
	Stat_Curr_q_d_ref_src1.qI_Component2 = 0;
	
	/*解析电机控制方式*/
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
	* @note	 	此函数用来处理电机板进入debug模式
  */
void	Process_Motor_Enter_Debug_Mode_Cmd(uint8_t *cmd_data_array_p)
{
	if(cmd_data_array_p[2] == 0x01)
	{
		en_motor_enter_debug_mode_flag = SET;   //进入调试模式
	}
	else if(cmd_data_array_p[2] == 0x02)
	{
		en_motor_enter_debug_mode_flag = RESET;     //进入正常运转模式
	}
}



/**
  * @brief 	void	Process_Motor_Set_Zero_Elec_Angle_Cmd(uint8_t *cmd_data_array_p)
	* @note	 	此函数用设定电机处于定零位模式
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
	* @note	 	此函数用设定电机保存零位
  */
void	Process_Motor_Save_Zero_Elec_Angle_Cmd(uint8_t *cmd_data_array_p)
{
	if( (cmd_data_array_p[2] == 0x04)||(cmd_data_array_p[2] == en_frame_motor_id) )
	{
		st_motor_pos_info.uw_motor_elec_zero_pos = st_motor_pos_info.uw_motor_machine_pos;
		Stat_Curr_q_d_ref.qI_Component1 = 0;
		Stat_Curr_q_d_ref.qI_Component2 = 0;
		
		/*保存零位值*/
		//Save_Motor_Config_Data();
		
		st_mc_state = RUN;
	}
}



/**
  * @brief 	void	Process_Motor_Set_Torque_Cmd(uint8_t *cmd_data_array_p)
	* @note	 	此函数用来处理电机力矩设置
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
	* @note	 	此函数用来处理电机力矩方向设置
  */
void	Process_Motor_Set_Torque_Dir_Cmd(uint8_t *cmd_data_array_p)
{
	if( (cmd_data_array_p[2] == 0x04)||(cmd_data_array_p[2] == en_frame_motor_id) )
	{
		sc_torque_direction = -sc_torque_direction;
		Stat_Curr_q_d_ref.qI_Component1 = -Stat_Curr_q_d_ref.qI_Component1;
		Stat_Curr_q_d_ref.qI_Component2 = -Stat_Curr_q_d_ref.qI_Component2; 

		//Save_Motor_Config_Data();//保存零位值
	}
}


/**
  * @brief 	void	Process_Motor_Set_Elec_Angle_Dir_Cmd(uint8_t *cmd_data_array_p)
	* @note	 	此函数用来处理电机电角度方向设置
  */
void	Process_Motor_Set_Elec_Angle_Dir_Cmd(uint8_t *cmd_data_array_p)
{
	if( (cmd_data_array_p[2] == 0x04)||(cmd_data_array_p[2] == en_frame_motor_id) )
	{
		st_motor_pos_info.sc_motor_elec_angle_dir = -st_motor_pos_info.sc_motor_elec_angle_dir;
		//Save_Motor_Config_Data();		//保存零位值
	}
}
/**
  * @brief 	void	Process_Motor_Query_Zero_Elec_Angle_Cmd(uint8_t *cmd_data_array_p)
	* @note	 	此函数用来处理电机电角度零位查询
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
	* @note	 	此函数用来处理电机板对整机零位的校准
  */
void Process_Motor_Set_Zero_Frame_Angle_Cmd(uint8_t *cmd_data_array_p)
{
	st_motor_pos_info.uw_motor_frame_zero_pos = st_motor_pos_info.uw_motor_machine_pos;
	
	//Save_Motor_Config_Data();  //保存零位值
}


/**
  * @brief 	void	Process_Motor_Query_Config_Info_Cmd(uint8_t *cmd_data_array_p)
	* @note	 	此函数用来响应查询配置信息
  */
void	Process_Motor_Query_Config_Info_Cmd(uint8_t *cmd_data_array_p)
{
	uint8_t  uc_fdb_cmd_data_array_p[17]={0};
	
	/*电机板配置信息查询*/
	if(en_frame_motor_id == cmd_data_array_p[2])
	{
		uc_fdb_cmd_data_array_p[0] = (uint8_t)(CMD_RELAY_QUERY_CONFIG_INFO>>8);
		uc_fdb_cmd_data_array_p[1] = (uint8_t)CMD_RELAY_QUERY_CONFIG_INFO;
		uc_fdb_cmd_data_array_p[2] = cmd_data_array_p[2];
		uc_fdb_cmd_data_array_p[3] = cmd_data_array_p[3];
		
		switch (cmd_data_array_p[3])
		{
//			case 0x00:	/*校准标志*/			
//				uc_fdb_cmd_data_array_p[4] = (uint8_t)(st_motor_config_data.motor_driver_cal_flag>>8);	
//				uc_fdb_cmd_data_array_p[5] = (uint8_t)(st_motor_config_data.motor_driver_cal_flag);	
//				break;
//			
//			case 0x01:	/*电零位*/
//				uc_fdb_cmd_data_array_p[4] = (uint8_t)(st_motor_config_data.zero_electrical_angle>>8);
//				uc_fdb_cmd_data_array_p[5] = (uint8_t)(st_motor_config_data.zero_electrical_angle);
//				break;
//			
//			case 0x02:	/*机械零位*/
//				uc_fdb_cmd_data_array_p[4] = (uint8_t)(st_motor_config_data.uw_motor_frame_zero_pos>>8);
//				uc_fdb_cmd_data_array_p[5] = (uint8_t)(st_motor_config_data.uw_motor_frame_zero_pos);
//				break;
			
			default:
				break;
		}
		
		//打包发送
		Pack_And_Send_Cmd(FEEDBACK_CMD_TXBUF,uc_fdb_cmd_data_array_p,MODULE_ADD_BYTE);
	}

}
