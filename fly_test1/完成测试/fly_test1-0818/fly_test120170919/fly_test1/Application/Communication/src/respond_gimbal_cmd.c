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


/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "respond_gimbal_cmd.h"

#include "gimbal_ahrs.h"
#include "gimbal_frame_angle.h"

#include "task_get_attitude.h"
#include "task_common_functions.h"

#include "protocol.h"

#include "stm32_bsp_uart.h"
#include <math.h>


/* private define ------------------------------------------------------------*/
#define	GIMBAL_CAL_ITEM_ERROR										((uint8_t)0x80)
#define	GIMBAL_CAL_ITEM_GYRO_OK									((uint8_t)0x01)
#define	GIMBAL_CAL_ITEM_ACC_SCALE_OK						((uint8_t)0x02)	
#define	GIMBAL_CAL_ITEM_ACC_OK									((uint8_t)0x04)	
#define	GIMBAL_CAL_ITEM_ZERO_FRAME_ANGEL_OK			((uint8_t)0x08)	

#define	GIMBAL_CAL_LOOP_PERIOD_MS								((uint8_t)5)
#define	GIMBAL_DEBUG_FEEDBACK_PERIOD_MS					((uint8_t)40)

/* private variables ------------------------------------------------------------*/
uint8_t 		uc_gimbal_cal_acc_drift_step = 0xFF;

FlagStatus	gimbal_acc_x_max_cal_ok_flag = RESET,		gimbal_acc_x_min_cal_ok_flag	= RESET;	
FlagStatus	gimbal_acc_y_max_cal_ok_flag = RESET,		gimbal_acc_y_min_cal_ok_flag	= RESET;
FlagStatus	gimbal_acc_z_max_cal_ok_flag = RESET,		gimbal_acc_z_min_cal_ok_flag	= RESET;

FlagStatus	en_gimbal_cal_gyro_drift_flag=RESET;
FlagStatus	en_gimbal_cal_acc_drift_flag=RESET;


/* External variables ------------------------------------------------------------*/
FlagStatus	en_gimbal_enter_debug_mode_flag = RESET;  


/*记录姿态板上校准项目完成情况*/
/*0000 0000:bit7=0 bit0=gyro bit1:acc_scale bit2:acc_水平 bit3：整机零位*/
uint8_t			uc_gimbal_cal_ok_item = 0x00;

/* Exported function --------------------------------------------------------*/
void Feedback_Debug_Information(void);




/* External function --------------------------------------------------------*/
/**
  * @brief 	void	Gimbal_Process_Send_Pos_Ctrl_Cmd(uint8_t *cmd_data_array_p)
	* @note	 	姿态板处理下行指令
  */
void	Gimbal_Process_Send_Pos_Ctrl_Cmd(uint8_t *cmd_data_array_p)
{
	/*更新角传值*/
	for(uint8_t i=0;i<6;i++)
	{
		uc_recive_motor_frame_angle[i] = cmd_data_array_p[2+i];
	}
	
	/*更新用户模式设定*/
//	uc_receive_ui_mode_data = cmd_data_array_p[8]; 	
//	
//	
//	if(0x03 == ((0x3C&uc_receive_ui_mode_data) >> 2))
//	{	
//		/*更新跟踪误差*/
//		uc_receive_track_err_data[0] = cmd_data_array_p[11];
//		uc_receive_track_err_data[1] = cmd_data_array_p[12];
//		uc_receive_track_err_data[2] = cmd_data_array_p[13];
//		uc_receive_track_err_data[3] = cmd_data_array_p[14];
//		
//		sw_track_err_x_test_data = ( ( (int16_t)(int8_t)uc_receive_track_err_data[0]<<8 )|uc_receive_track_err_data[1] );  //test
//		sw_track_err_y_test_data = ( ( (int16_t)(int8_t)uc_receive_track_err_data[2]<<8 )|uc_receive_track_err_data[3] );  //test
//	}
//	else if(0x04 == ((0x3C&uc_receive_ui_mode_data) >> 2))
//	{
//		/*更新电机位置环参考值*/
//		uc_receive_frame_angle_ref_data[0] = cmd_data_array_p[11];
//		uc_receive_frame_angle_ref_data[1] = cmd_data_array_p[12];
//		uc_receive_frame_angle_ref_data[2] = cmd_data_array_p[13];
//		uc_receive_frame_angle_ref_data[3] = cmd_data_array_p[14];
//		uc_receive_frame_angle_ref_data[4] = cmd_data_array_p[15];
//		uc_receive_frame_angle_ref_data[5] = cmd_data_array_p[16];		
//	}
//	else
//	{
//		/*更新速度值*/
//		uc_receive_ui_spd_data[0] = cmd_data_array_p[12];
//		uc_receive_ui_spd_data[1] = cmd_data_array_p[14];
//	}
	
	
//	/*更新用户速度值*/
//	uc_receive_ui_spd_data[0] = cmd_data_array_p[8];
//	uc_receive_ui_spd_data[1] = cmd_data_array_p[10];
//	
//	/*更新用户模式设定*/
//	uc_receive_ui_mode_data = cmd_data_array_p[12]; 
//	
//	/*收脱靶量信息*/
//	for(uint8_t j =0;j<4;j++)
//	{
//		uc_receive_track_err_data[j] = cmd_data_array_p[13+j];
//	}
}


/**
  * @brief 	void	Gimbal_Send_Torque_Ref_Cmd(uint8_t *cmd_data_array_p)
	* @note	 	姿态板更新上行指令
  */
//void	Gimbal_Process_Set_Torque_Ref_Cmd(uint8_t *cmd_data_array_p)
//{
//	/*更新力矩值*/
//	cmd_data_array_p[2] = (uint8_t)(motor_y_spd_pid_out>>8);
//	cmd_data_array_p[3] = (uint8_t)(motor_y_spd_pid_out);
//	cmd_data_array_p[4] = (uint8_t)(motor_z_spd_pid_out>>8);
//	cmd_data_array_p[5] = (uint8_t)(motor_z_spd_pid_out);
//	cmd_data_array_p[6] = (uint8_t)(motor_x_spd_pid_out>>8);
//	cmd_data_array_p[7] = (uint8_t)(motor_x_spd_pid_out);
//		

//	/*用户当前云台模式*/
//	/*更新power_mode反馈*/
////	cmd_data_array_p[8] &= 0xFC;				//bit1:bit0=**** **00
////	cmd_data_array_p[8] |= en_gimbal_power_mode;
//	switch(en_gimbal_power_mode)
//	{
//		case SLEEP:
//			cmd_data_array_p[8] &= 0xFC;	//bit1:bit0=**** **00
//			break;
//		case NORMAL:
//			cmd_data_array_p[8] &= 0xFC;	//bit1:bit0=**** **01
//			cmd_data_array_p[8] |= 0x01;
//			break;
//		case SLEEP_TO_NORMAL:
//			cmd_data_array_p[8] &= 0xFC;	//bit1:bit0=**** **02
//			cmd_data_array_p[8] |= 0x02;
//			break;
//		case NORMAL_TO_SLEEP:
//			cmd_data_array_p[8] &= 0xFC;	//bit1:bit0=**** **03
//			cmd_data_array_p[8] |= 0x03;
//			break;
//		
//		default:
//			break;
//	}

////			cmd_data_array_p[8] &= 0xFC;	//bit1:bit0=**** **01
////			cmd_data_array_p[8] |= 0x01;
//	
//	/*更新work_mode反馈*/
////	cmd_data_array_p[8] &= 0xC3;  //bit5:bit2 **00 00**
////	cmd_data_array_p[8] |= en_gimbal_work_mode<<2;
//	switch(en_gimbal_work_mode)
//	{
//		case NONE_FOLLOW:
//			cmd_data_array_p[8] &= 0xC3;  //bit5:bit2 **00 00**
//			break;
//		
//		case Z_FOLLOW:
//		case Z_FOLLOW_TO_NONE_FOLLOW:
//			cmd_data_array_p[8] &= 0xC3;  //bit5:bit2 **00 01**
//			cmd_data_array_p[8] |= 0x04;  
//			break;
//		
//		case X_Z_FOLLOW:
//		case X_Z_FOLLOW_TO_NONE_FOLLOW:
//		case X_Z_FOLLOW_TO_Z_FOLLOW:
//			cmd_data_array_p[8] &= 0xC3;  //bit5:bit2 **00 10**
//			cmd_data_array_p[8] |= 0x08;  
//			break;
//		
//		case TRACKER:
//			cmd_data_array_p[8] &= 0xC3;  //bit5:bit2 **00 11**
//			cmd_data_array_p[8] |= 0x0C;  
//			break;
//		
//		case LOCK:
//			cmd_data_array_p[8] &= 0xC3;  //bit5:bit2 **01 00**
//			cmd_data_array_p[8] |= 0x10;  
//			break;
//		
//		default:
//			break;
//	}
//		
//	/*更新取景模式反馈*/
////	cmd_data_array_p[8] &= 0xBF;  //bit6 *0** ****	
////	cmd_data_array_p[8] |= en_camera_viewer_mode<<6;
//	switch(en_camera_viewer_mode)
//	{
//		case HORIZONTAL:
//			cmd_data_array_p[8] &= 0xBF;	//bit6 *0** ****	
//			break;
//		case VERTICAL:
//			cmd_data_array_p[8] |= 0x40;	//bit6 *0** ****	
//			break;
//		default:
//			break;
//	}
//	
//	
//	/*更新电机工作模式*/
//	if( (SLEEP == en_gimbal_power_mode)||(NORMAL_TO_SLEEP == en_gimbal_power_mode) )
//	{
//		cmd_data_array_p[9] &= 0x00;
//	}
//	else
//	{
//		if(SET == en_motor_y_and_motor_x_have_same_dir_flag)
//		{
//			cmd_data_array_p[9] |= 0x01;
//			cmd_data_array_p[9] &= 0xFD;
//		}
//		else
//		{
//			cmd_data_array_p[9] &= 0xFC;
//		}
//	}
//	
//}
/*----------------------------debug命令处理---------------------------*/
/**
  * @brief 	void	Process_Gimbal_Enter_Debug_Mode_Cmd(uint8_t *cmd_data_array_p)
	* @note	 	此函数用来处理姿态板进入debug模式
  */
void	Process_Gimbal_Enter_Debug_Mode_Cmd(uint8_t *cmd_data_array_p)
{
	if(cmd_data_array_p[2] == 0x01)
	{
		en_gimbal_enter_debug_mode_flag = SET;
	}
	else if(cmd_data_array_p[2] == 0x02)
	{
		en_gimbal_enter_debug_mode_flag = RESET;   //退出debug模式时进行保存数据
	}
}


/**
  * @brief 	void	Process_Gimbal_Set_Zero_Frame_Angle_Cmd(uint8_t *cmd_data_array_p)
	* @note	 	此函数用来处理姿态板对整机零位的校准
  */
void Process_Gimbal_Set_Zero_Frame_Angle_Cmd(uint8_t *cmd_data_array_p)
{
	uint8_t	uc_fdb_cmd_data_array_p[3];
	
	uc_fdb_cmd_data_array_p[0] = 0x58;
	uc_fdb_cmd_data_array_p[1] = 0x10;
	Pack_And_Send_Cmd(FEEDBACK_CMD_TXBUF,uc_fdb_cmd_data_array_p,MODULE_ADD_BYTE);
}


/**
  * @brief 	void	Process_Gimbal_Cal_Gyro_Cmd(uint8_t *cmd_data_array_p)
	* @note	 	此函数用来处理陀螺校准命令
  */	
void Process_Gimbal_Cal_Gyro_Cmd(uint8_t *cmd_data_array_p)
{	
	en_gimbal_cal_gyro_drift_flag = SET;
	uc_gimbal_cal_ok_item = uc_gimbal_cal_ok_item&(~GIMBAL_CAL_ITEM_GYRO_OK);
}


/**
  * @brief 	void	Process_Gimbal_Cal_Gyro_Cmd(uint8_t *cmd_data_array_p)
	* @note	 	此函数用来处理陀螺校准命令
  */	
void Process_Gimbal_Cal_Acc_Cmd(uint8_t *cmd_data_array_p)
{	
	uint8_t	uc_fdb_cmd_data_array_p[3];
	
	//读取加速度校准步骤*/
	uc_gimbal_cal_acc_drift_step = cmd_data_array_p[2];
	
	if(uc_gimbal_cal_acc_drift_step > 0x08)
	{
		
	}
	else if(uc_gimbal_cal_acc_drift_step == 0x01)
	{
		uc_gimbal_cal_ok_item = uc_gimbal_cal_ok_item&(~(GIMBAL_CAL_ITEM_ACC_SCALE_OK|GIMBAL_CAL_ITEM_ACC_OK));
		//开始校准命令反馈
		uc_fdb_cmd_data_array_p[0] = 0x56;
		uc_fdb_cmd_data_array_p[1] = 0x11;
		uc_fdb_cmd_data_array_p[2] = 0x01;
		Pack_And_Send_Cmd(FEEDBACK_CMD_TXBUF,uc_fdb_cmd_data_array_p,MODULE_ADD_BYTE);
	}
	else if(uc_gimbal_cal_acc_drift_step == 0x08)
	{
		//结束校准命令反馈		
//		Save_Gimbal_Config_Data();
		uc_fdb_cmd_data_array_p[0] = 0x56;
		uc_fdb_cmd_data_array_p[1] = 0x11;
		uc_fdb_cmd_data_array_p[2] = 0x08;
		Pack_And_Send_Cmd(FEEDBACK_CMD_TXBUF,uc_fdb_cmd_data_array_p,MODULE_ADD_BYTE);
	}
	else
	{
		en_gimbal_cal_acc_drift_flag = SET;
	}

}



/**
  * @brief 	void	Process_Query_Ahrs_Cal_Status_Cmd(void)
	* @note	 	反馈校准状态
  */
void	Process_Query_Ahrs_Cal_Status_Cmd(uint8_t *cmd_data_array_p)
{
	uint8_t	uc_fdb_cmd_data_array_p[18];
	
	uc_fdb_cmd_data_array_p[0] = 0x56;
	uc_fdb_cmd_data_array_p[1] = 0x5C;
	uc_fdb_cmd_data_array_p[2]=(uint8_t)(uc_gimbal_cal_ok_item);
		
//	uc_fdb_cmd_data_array_p[2]=(uint8_t)((int16_t)(gimbal_acc_x_max_cal_ok_flag));  //gimbal_x
//	uc_fdb_cmd_data_array_p[3]=(uint8_t)((int16_t)(gimbal_acc_x_min_cal_ok_flag));	//gimbal_x
//	uc_fdb_cmd_data_array_p[4]=(uint8_t)((int16_t)(gimbal_acc_y_max_cal_ok_flag));	//gimbal_y
//	uc_fdb_cmd_data_array_p[5]=(uint8_t)((int16_t)(gimbal_acc_y_min_cal_ok_flag));	//gimbal_y
//	uc_fdb_cmd_data_array_p[6]=(uint8_t)((int16_t)(gimbal_acc_z_max_cal_ok_flag));	//gimbal_z
//	uc_fdb_cmd_data_array_p[7]=(uint8_t)((int16_t)(gimbal_acc_z_min_cal_ok_flag));	//gimbal_z
//	
//	uc_fdb_cmd_data_array_p[8] =(uint8_t)(((int16_t)(f_gimbal_gyro_data_array[0]*100))>>8);
//	uc_fdb_cmd_data_array_p[9] =(uint8_t)((int16_t)(f_gimbal_gyro_data_array[0]*100));	
//	uc_fdb_cmd_data_array_p[10]=(uint8_t)(((int16_t)(f_gimbal_gyro_data_array[1]*100))>>8);
//	uc_fdb_cmd_data_array_p[11]=(uint8_t)((int16_t)(f_gimbal_gyro_data_array[1]*100));	
//	uc_fdb_cmd_data_array_p[12]=(uint8_t)(((int16_t)(f_gimbal_gyro_data_array[2]*100))>>8);	
//	uc_fdb_cmd_data_array_p[13]=(uint8_t)((int16_t)(f_gimbal_gyro_data_array[2]*100));
	
	Pack_And_Send_Cmd(FEEDBACK_CMD_TXBUF,uc_fdb_cmd_data_array_p,MODULE_ADD_BYTE);
}	





/**
  * @brief 	void	Task_Gimbal_Cal_Loop(void)
	* @note	 	陀螺和加速度校准Loop
  */
void	Task_Gimbal_Cal_Loop(void)
{
	static	uint8_t	uc_gimbal_gyro_flt_cycle_index = 0;
	static	uint8_t	uc_gimbal_acc_flt_cycle_index = 0;
	
	static	float		gimbal_acc_x_max,gimbal_acc_x_min,gimbal_acc_y_max,gimbal_acc_y_min,gimbal_acc_z_max,gimbal_acc_z_min;
	static	float		gimbal_acc_y_at_x_max,	gimbal_acc_z_at_x_max,	gimbal_acc_y_at_x_min,	gimbal_acc_z_at_x_min;
	static	float		gimbal_acc_x_at_y_max,	gimbal_acc_z_at_y_max,	gimbal_acc_x_at_y_min,	gimbal_acc_z_at_y_min;
	static	float		gimbal_acc_x_at_z_max,	gimbal_acc_y_at_z_max,	gimbal_acc_x_at_z_min,	gimbal_acc_y_at_z_min;
	
	float		f_sensor_acc_mean_data_array_p[3];
	
	/*gyro_offset/acc_offset/acc_scale备份数据，校准正确保存后，更新否则保存前一次的数据*/
	static	float					f_sensor_gyro_offset_data_array_backup[3],	f_sensor_gyro_flt_data_array_backup[3];
	static	FlagStatus		en_judege_sensor_gyro_cal_right_flag = RESET;
	
	float		f_sensor_acc_offset_data_array_backup[3];
	float		f_sensor_acc_scale_data_array_backup[3];
		
	uint8_t	uc_fdb_cmd_data_array_p[3];
	
	static	User_Timer_Typedef	st_this_function_period_timer_p = USER_TIMER_INIT_VALUE;
	
	Start_User_Timer(&st_this_function_period_timer_p);
	Update_User_Timer_Cnt(&st_this_function_period_timer_p);
	if(st_this_function_period_timer_p.ul_timer_cnt > GIMBAL_CAL_LOOP_PERIOD_MS)   //5ms校准一次
	{	
		Reset_User_Timer(&st_this_function_period_timer_p);
	
		/*---------------------陀螺校准过程---------------------------------*/
		if(SET == en_gimbal_cal_gyro_drift_flag)
		{
			if(SET == Average_Filter_Float(f_sensor_gyro_raw_data_array,f_sensor_gyro_offset_data_array_backup,&uc_gimbal_gyro_flt_cycle_index,100))
			{	
				en_gimbal_cal_gyro_drift_flag = RESET;
		
				/*校准是否通过的标志*/
				if(	(fabs(f_sensor_gyro_offset_data_array_backup[0]) < 20)&&
						(fabs(f_sensor_gyro_offset_data_array_backup[1]) < 20)&&
						(fabs(f_sensor_gyro_offset_data_array_backup[2]) < 20)&&
						(fabs(f_sensor_gyro_raw_data_array[0] - f_sensor_gyro_offset_data_array_backup[0]) < 1)&&
						(fabs(f_sensor_gyro_raw_data_array[1] - f_sensor_gyro_offset_data_array_backup[1]) < 1)&&
						(fabs(f_sensor_gyro_raw_data_array[2] - f_sensor_gyro_offset_data_array_backup[2]) < 1)	)
				{
					f_sensor_gyro_offset_data_array[0] = -f_sensor_gyro_offset_data_array_backup[0];
					f_sensor_gyro_offset_data_array[1] = -f_sensor_gyro_offset_data_array_backup[1];
					f_sensor_gyro_offset_data_array[2] = -f_sensor_gyro_offset_data_array_backup[2];
					
					uc_gimbal_cal_ok_item = uc_gimbal_cal_ok_item|GIMBAL_CAL_ITEM_GYRO_OK;
					//Save_Gimbal_Config_Data();
				}
				else
				{
					uc_gimbal_cal_ok_item = uc_gimbal_cal_ok_item&(~GIMBAL_CAL_ITEM_GYRO_OK);
				}
				
				
				uc_fdb_cmd_data_array_p[0] = 0x57;
				uc_fdb_cmd_data_array_p[1] = 0x20;
				Pack_And_Send_Cmd(FEEDBACK_CMD_TXBUF,uc_fdb_cmd_data_array_p,MODULE_ADD_BYTE);
			}
		}
		
		/*----------------------加速度校准过程---------------------------------*/
		if(SET == en_gimbal_cal_acc_drift_flag)
		{
			switch(uc_gimbal_cal_acc_drift_step)
			{
				case 0x02:  //gimbal_x轴=sensor_z
						if(SET == Average_Filter_Float(f_sensor_acc_raw_data_array,f_sensor_acc_mean_data_array_p,&uc_gimbal_acc_flt_cycle_index,100))
						{				
							gimbal_acc_x_max = f_sensor_acc_mean_data_array_p[GIMBAL_ACC_X_DATA_INDEX];
							gimbal_acc_y_at_x_max = f_sensor_acc_mean_data_array_p[GIMBAL_ACC_Y_DATA_INDEX];
							gimbal_acc_z_at_x_max = f_sensor_acc_mean_data_array_p[GIMBAL_ACC_Z_DATA_INDEX];
							
							//反馈x校准完成
							uc_fdb_cmd_data_array_p[0] = 0x56;
							uc_fdb_cmd_data_array_p[1] = 0x11;
							uc_fdb_cmd_data_array_p[2] = 0x02;
							Pack_And_Send_Cmd(FEEDBACK_CMD_TXBUF,uc_fdb_cmd_data_array_p,MODULE_ADD_BYTE);
							
							/*当前步校准完成*/
							en_gimbal_cal_acc_drift_flag = RESET;
						}
						break;
				case 0x03://gimbal_x轴=sensor_z
						if(SET == Average_Filter_Float(f_sensor_acc_raw_data_array,f_sensor_acc_mean_data_array_p,&uc_gimbal_acc_flt_cycle_index,100))
						{
							gimbal_acc_x_min = f_sensor_acc_mean_data_array_p[GIMBAL_ACC_X_DATA_INDEX];
							gimbal_acc_y_at_x_min = f_sensor_acc_mean_data_array_p[GIMBAL_ACC_Y_DATA_INDEX];
							gimbal_acc_z_at_x_min = f_sensor_acc_mean_data_array_p[GIMBAL_ACC_Z_DATA_INDEX];
							
							//反馈x校准完成
							uc_fdb_cmd_data_array_p[0] = 0x56;
							uc_fdb_cmd_data_array_p[1] = 0x11;
							uc_fdb_cmd_data_array_p[2] = 0x03;
							Pack_And_Send_Cmd(FEEDBACK_CMD_TXBUF,uc_fdb_cmd_data_array_p,MODULE_ADD_BYTE);
							
							/*当前步校准完成*/
							en_gimbal_cal_acc_drift_flag = RESET;
						}
						break;
				case 0x04://gimbal_y轴=sensor_x
						if(SET == Average_Filter_Float(f_sensor_acc_raw_data_array,f_sensor_acc_mean_data_array_p,&uc_gimbal_acc_flt_cycle_index,100))
						{
							gimbal_acc_y_max = f_sensor_acc_mean_data_array_p[GIMBAL_ACC_Y_DATA_INDEX];
							gimbal_acc_x_at_y_max = f_sensor_acc_mean_data_array_p[GIMBAL_ACC_X_DATA_INDEX];
							gimbal_acc_z_at_y_max = f_sensor_acc_mean_data_array_p[GIMBAL_ACC_Z_DATA_INDEX];				
							
							//反馈x校准完成
							uc_fdb_cmd_data_array_p[0] = 0x56;
							uc_fdb_cmd_data_array_p[1] = 0x11;
							uc_fdb_cmd_data_array_p[2] = 0x04;
							Pack_And_Send_Cmd(FEEDBACK_CMD_TXBUF,uc_fdb_cmd_data_array_p,MODULE_ADD_BYTE);
							
							/*当前步校准完成*/
							en_gimbal_cal_acc_drift_flag = RESET;
						}
						break;
				case 0x05://gimbal_y轴=sensor_x
						if(SET == Average_Filter_Float(f_sensor_acc_raw_data_array,f_sensor_acc_mean_data_array_p,&uc_gimbal_acc_flt_cycle_index,100))
						{
							gimbal_acc_y_min = f_sensor_acc_mean_data_array_p[GIMBAL_ACC_Y_DATA_INDEX];
							gimbal_acc_x_at_y_min = f_sensor_acc_mean_data_array_p[GIMBAL_ACC_X_DATA_INDEX];
							gimbal_acc_z_at_y_min = f_sensor_acc_mean_data_array_p[GIMBAL_ACC_Z_DATA_INDEX];
							
							
							//反馈x校准完成
							uc_fdb_cmd_data_array_p[0] = 0x56;
							uc_fdb_cmd_data_array_p[1] = 0x11;
							uc_fdb_cmd_data_array_p[2] = 0x05;
							Pack_And_Send_Cmd(FEEDBACK_CMD_TXBUF,uc_fdb_cmd_data_array_p,MODULE_ADD_BYTE);
							
							/*当前步校准完成*/
							en_gimbal_cal_acc_drift_flag = RESET;
						}
						break;
				case 0x06://gimbal_z轴=sensor_y
						if(SET == Average_Filter_Float(f_sensor_acc_raw_data_array,f_sensor_acc_mean_data_array_p,&uc_gimbal_acc_flt_cycle_index,100))
						{
							gimbal_acc_z_max = f_sensor_acc_mean_data_array_p[GIMBAL_ACC_Z_DATA_INDEX];
							gimbal_acc_x_at_z_max = f_sensor_acc_mean_data_array_p[GIMBAL_ACC_X_DATA_INDEX];
							gimbal_acc_y_at_z_max = f_sensor_acc_mean_data_array_p[GIMBAL_ACC_Y_DATA_INDEX];
							
							//反馈x校准完成
							uc_fdb_cmd_data_array_p[0] = 0x56;
							uc_fdb_cmd_data_array_p[1] = 0x11;
							uc_fdb_cmd_data_array_p[2] = 0x06;
							Pack_And_Send_Cmd(FEEDBACK_CMD_TXBUF,uc_fdb_cmd_data_array_p,MODULE_ADD_BYTE);
							
							/*当前步校准完成*/
							en_gimbal_cal_acc_drift_flag = RESET;
						}
						break;
				case 0x07://gimbal_z轴=sensor_y
						if(SET == Average_Filter_Float(f_sensor_acc_raw_data_array,f_sensor_acc_mean_data_array_p,&uc_gimbal_acc_flt_cycle_index,100))
						{
							gimbal_acc_z_min = f_sensor_acc_mean_data_array_p[GIMBAL_ACC_Z_DATA_INDEX];
							gimbal_acc_x_at_z_min = f_sensor_acc_mean_data_array_p[GIMBAL_ACC_X_DATA_INDEX];
							gimbal_acc_y_at_z_min = f_sensor_acc_mean_data_array_p[GIMBAL_ACC_Y_DATA_INDEX];
							
							//计算加速度偏移
							f_sensor_acc_offset_data_array_backup[GIMBAL_ACC_X_DATA_INDEX] = -(gimbal_acc_x_max + gimbal_acc_x_min)/2;
							f_sensor_acc_offset_data_array_backup[GIMBAL_ACC_Y_DATA_INDEX] = -(gimbal_acc_y_max + gimbal_acc_y_min)/2;
							f_sensor_acc_offset_data_array_backup[GIMBAL_ACC_Z_DATA_INDEX] = -(gimbal_acc_z_max + gimbal_acc_z_min)/2;
							
							//计算加速度灵敏度，并将scale值限定在0.8～1.2的范围内
							//加速度偏置需在±0.3g之内
							if(	(fabs(f_sensor_acc_offset_data_array_backup[GIMBAL_ACC_X_DATA_INDEX]) < 0.3)&&
									(fabs(f_sensor_acc_offset_data_array_backup[GIMBAL_ACC_Y_DATA_INDEX]) < 0.3)&&
									(fabs(f_sensor_acc_offset_data_array_backup[GIMBAL_ACC_Z_DATA_INDEX]) < 0.3)&&
									(fabs(gimbal_acc_x_max + f_sensor_acc_offset_data_array_backup[GIMBAL_ACC_X_DATA_INDEX]) > 0.8)&&(fabs(gimbal_acc_x_max + f_sensor_acc_offset_data_array_backup[GIMBAL_ACC_X_DATA_INDEX]) < 1.2)&&
									(fabs(gimbal_acc_y_max + f_sensor_acc_offset_data_array_backup[GIMBAL_ACC_Y_DATA_INDEX]) > 0.8)&&(fabs(gimbal_acc_y_max + f_sensor_acc_offset_data_array_backup[GIMBAL_ACC_Y_DATA_INDEX]) < 1.2)&&
									(fabs(gimbal_acc_z_max + f_sensor_acc_offset_data_array_backup[GIMBAL_ACC_Z_DATA_INDEX]) > 0.8)&&(fabs(gimbal_acc_z_max + f_sensor_acc_offset_data_array_backup[GIMBAL_ACC_Z_DATA_INDEX]) < 1.2)
								)
							{
								f_sensor_acc_scale_data_array_backup[GIMBAL_ACC_X_DATA_INDEX] = 1.f/fabs(gimbal_acc_x_max + f_sensor_acc_offset_data_array_backup[GIMBAL_ACC_X_DATA_INDEX]);
								f_sensor_acc_scale_data_array_backup[GIMBAL_ACC_Y_DATA_INDEX] = 1.f/fabs(gimbal_acc_y_max + f_sensor_acc_offset_data_array_backup[GIMBAL_ACC_Y_DATA_INDEX]);
								f_sensor_acc_scale_data_array_backup[GIMBAL_ACC_Z_DATA_INDEX] = 1.f/fabs(gimbal_acc_z_max + f_sensor_acc_offset_data_array_backup[GIMBAL_ACC_Z_DATA_INDEX]);
								
								f_sensor_acc_scale_data_array[0] = f_sensor_acc_scale_data_array_backup[0];
								f_sensor_acc_scale_data_array[1] = f_sensor_acc_scale_data_array_backup[1];
								f_sensor_acc_scale_data_array[2] = f_sensor_acc_scale_data_array_backup[2];
								
								uc_gimbal_cal_ok_item = uc_gimbal_cal_ok_item|GIMBAL_CAL_ITEM_ACC_SCALE_OK;
							}
							
							
							/*在scale是正确的后，才能继续判断*/
							if(uc_gimbal_cal_ok_item&GIMBAL_CAL_ITEM_ACC_SCALE_OK)
							{
								//判断x_max校准时是否水平
								gimbal_acc_y_at_x_max = (gimbal_acc_y_at_x_max + f_sensor_acc_offset_data_array_backup[GIMBAL_ACC_Y_DATA_INDEX])*f_sensor_acc_scale_data_array[GIMBAL_ACC_Y_DATA_INDEX];
								gimbal_acc_z_at_x_max = (gimbal_acc_z_at_x_max + f_sensor_acc_offset_data_array_backup[GIMBAL_ACC_Z_DATA_INDEX])*f_sensor_acc_scale_data_array[GIMBAL_ACC_Z_DATA_INDEX];
								
								if( (fabs(gimbal_acc_y_at_x_max) < 0.1)&&(fabs(gimbal_acc_z_at_x_max) < 0.1) )
								{
									gimbal_acc_x_max_cal_ok_flag = SET;
								}
								else
								{
									gimbal_acc_x_max_cal_ok_flag = RESET;
								}
								
								//判断x_min校准时是否水平
								gimbal_acc_y_at_x_min = (gimbal_acc_y_at_x_min + f_sensor_acc_offset_data_array_backup[GIMBAL_ACC_Y_DATA_INDEX])*f_sensor_acc_scale_data_array[GIMBAL_ACC_Y_DATA_INDEX];
								gimbal_acc_z_at_x_min = (gimbal_acc_z_at_x_min + f_sensor_acc_offset_data_array_backup[GIMBAL_ACC_Z_DATA_INDEX])*f_sensor_acc_scale_data_array[GIMBAL_ACC_Z_DATA_INDEX];
								
								if( (fabs(gimbal_acc_y_at_x_min) < 0.1)&&(fabs(gimbal_acc_z_at_x_min) < 0.1) )
								{
									gimbal_acc_x_min_cal_ok_flag = SET;
								}
								else
								{
									gimbal_acc_x_min_cal_ok_flag = RESET;
								}
								
								
								//判断y_max校准时是否水平
								gimbal_acc_x_at_y_max = (gimbal_acc_x_at_y_max + f_sensor_acc_offset_data_array_backup[GIMBAL_ACC_X_DATA_INDEX])*f_sensor_acc_scale_data_array[GIMBAL_ACC_X_DATA_INDEX];
								gimbal_acc_z_at_y_max = (gimbal_acc_z_at_y_max + f_sensor_acc_offset_data_array_backup[GIMBAL_ACC_Z_DATA_INDEX])*f_sensor_acc_scale_data_array[GIMBAL_ACC_Z_DATA_INDEX];
								
								if( (fabs(gimbal_acc_x_at_y_max) < 0.1)&&(fabs(gimbal_acc_z_at_y_max) < 0.1) )
								{
									gimbal_acc_y_max_cal_ok_flag = SET;
								}
								else
								{
									gimbal_acc_y_max_cal_ok_flag = RESET;
								}
								
								//判断y_min校准时是否水平
								gimbal_acc_x_at_y_min = (gimbal_acc_x_at_y_min + f_sensor_acc_offset_data_array_backup[GIMBAL_ACC_X_DATA_INDEX])*f_sensor_acc_scale_data_array[GIMBAL_ACC_X_DATA_INDEX];
								gimbal_acc_z_at_y_min = (gimbal_acc_z_at_y_min + f_sensor_acc_offset_data_array_backup[GIMBAL_ACC_Z_DATA_INDEX])*f_sensor_acc_scale_data_array[GIMBAL_ACC_Z_DATA_INDEX];
								
								if( (fabs(gimbal_acc_x_at_y_min) < 0.1)&&(fabs(gimbal_acc_z_at_y_min) < 0.1) )
								{
									gimbal_acc_y_min_cal_ok_flag = SET;
								}
								else
								{
									gimbal_acc_y_min_cal_ok_flag = RESET;
								}
							
							
								//判断z_max校准时是否水平
								gimbal_acc_x_at_z_max = (gimbal_acc_x_at_z_max + f_sensor_acc_offset_data_array_backup[GIMBAL_ACC_X_DATA_INDEX])*f_sensor_acc_scale_data_array[GIMBAL_ACC_X_DATA_INDEX];
								gimbal_acc_y_at_z_max = (gimbal_acc_y_at_z_max + f_sensor_acc_offset_data_array_backup[GIMBAL_ACC_Y_DATA_INDEX])*f_sensor_acc_scale_data_array[GIMBAL_ACC_Y_DATA_INDEX];
								
								if( (fabs(gimbal_acc_x_at_z_max) < 0.1)&&(fabs(gimbal_acc_y_at_z_max) < 0.1) )
								{
									gimbal_acc_z_max_cal_ok_flag = SET;
								}
								else
								{
									gimbal_acc_z_max_cal_ok_flag = RESET;
								}
							
								//判断z_min校准时是否水平
								gimbal_acc_x_at_z_min = (gimbal_acc_x_at_z_min + f_sensor_acc_offset_data_array_backup[GIMBAL_ACC_X_DATA_INDEX])*f_sensor_acc_scale_data_array[GIMBAL_ACC_X_DATA_INDEX];
								gimbal_acc_y_at_z_min = (gimbal_acc_y_at_z_min + f_sensor_acc_offset_data_array_backup[GIMBAL_ACC_Y_DATA_INDEX])*f_sensor_acc_scale_data_array[GIMBAL_ACC_Y_DATA_INDEX];
								
								if( (fabs(gimbal_acc_x_at_z_min) < 0.1)&&(fabs(gimbal_acc_y_at_z_min) < 0.1) )
								{
									gimbal_acc_z_min_cal_ok_flag = SET;
								}
								else
								{
									gimbal_acc_z_min_cal_ok_flag = RESET;
								}
													
								/*判断加速度校准是否正确*/
								if(	(SET == gimbal_acc_x_max_cal_ok_flag)&&(SET == gimbal_acc_x_min_cal_ok_flag)&&
										(SET == gimbal_acc_y_max_cal_ok_flag)&&(SET == gimbal_acc_y_min_cal_ok_flag)&&
										(SET == gimbal_acc_z_max_cal_ok_flag)&&(SET == gimbal_acc_z_min_cal_ok_flag) )
								{
									uc_gimbal_cal_ok_item = uc_gimbal_cal_ok_item|GIMBAL_CAL_ITEM_ACC_OK;
									
									f_sensor_acc_offset_data_array[0] = f_sensor_acc_offset_data_array_backup[0];		
									f_sensor_acc_offset_data_array[1] = f_sensor_acc_offset_data_array_backup[1];
									f_sensor_acc_offset_data_array[2] = f_sensor_acc_offset_data_array_backup[2];
								}
							}
							
							
							/*校准通过,保存数据*/
							if(uc_gimbal_cal_ok_item|GIMBAL_CAL_ITEM_ACC_OK|GIMBAL_CAL_ITEM_ACC_SCALE_OK)
							{
									 ;//Save_Gimbal_Config_Data();
							}	
							
							//反馈x校准完成
							uc_fdb_cmd_data_array_p[0] = 0x56;
							uc_fdb_cmd_data_array_p[1] = 0x11;
							uc_fdb_cmd_data_array_p[2] = 0x07;
							Pack_And_Send_Cmd(FEEDBACK_CMD_TXBUF,uc_fdb_cmd_data_array_p,MODULE_ADD_BYTE);
							
							/*当前步校准完成*/
							en_gimbal_cal_acc_drift_flag = RESET;						
						}
						break;
				
			}
		}
	}
	
	/*调试模式下不断反馈状态信息*/
	if(SET == en_gimbal_enter_debug_mode_flag)
	{
		Feedback_Debug_Information();
	}
		
}






/**
  * @brief 	void Feedback_Debug_Information(void)
	* @note	 	调试状态下反馈调试信息
  */
void Feedback_Debug_Information(void)
{
	uint8_t	uc_fdb_cmd_data_array_p[18];
	static	FlagStatus	Feedback_Odd_Flag = SET;
	
	static	User_Timer_Typedef	st_this_function_period_timer_p = USER_TIMER_INIT_VALUE;
	Start_User_Timer(&st_this_function_period_timer_p);
	Update_User_Timer_Cnt(&st_this_function_period_timer_p);
	if(st_this_function_period_timer_p.ul_timer_cnt > GIMBAL_DEBUG_FEEDBACK_PERIOD_MS)  
	{	
		Reset_User_Timer(&st_this_function_period_timer_p);
		if(SET == Feedback_Odd_Flag)
		{
			uc_fdb_cmd_data_array_p[0] = 0x56;
			uc_fdb_cmd_data_array_p[1] = 0x2C;
			/*校准过程种传递原始数据，校准完成后传递校准后数据*/
			if( (uc_gimbal_cal_acc_drift_step >= 0x02)&&(uc_gimbal_cal_acc_drift_step <= 0x07) )
			{
				uc_fdb_cmd_data_array_p[2]=(uint8_t)(((int16_t)(f_sensor_acc_raw_data_array[GIMBAL_ACC_X_DATA_INDEX]*100))>>8);
				uc_fdb_cmd_data_array_p[3]=(uint8_t)((int16_t)(f_sensor_acc_raw_data_array[GIMBAL_ACC_X_DATA_INDEX]*100));	
				uc_fdb_cmd_data_array_p[4]=(uint8_t)(((int16_t)(f_sensor_acc_raw_data_array[GIMBAL_ACC_Y_DATA_INDEX]*100))>>8);
				uc_fdb_cmd_data_array_p[5]=(uint8_t)((int16_t)(f_sensor_acc_raw_data_array[GIMBAL_ACC_Y_DATA_INDEX]*100));	
				uc_fdb_cmd_data_array_p[6]=(uint8_t)(((int16_t)(f_sensor_acc_raw_data_array[GIMBAL_ACC_Z_DATA_INDEX]*100))>>8);	
				uc_fdb_cmd_data_array_p[7]=(uint8_t)((int16_t)(f_sensor_acc_raw_data_array[GIMBAL_ACC_Z_DATA_INDEX]*100));
			}
			else
			{
				uc_fdb_cmd_data_array_p[2]=(uint8_t)(((int16_t)(f_gimbal_acc_data_array[0]*100))>>8);
				uc_fdb_cmd_data_array_p[3]=(uint8_t)((int16_t)(f_gimbal_acc_data_array[0]*100));	
				uc_fdb_cmd_data_array_p[4]=(uint8_t)(((int16_t)(f_gimbal_acc_data_array[1]*100))>>8);
				uc_fdb_cmd_data_array_p[5]=(uint8_t)((int16_t)(f_gimbal_acc_data_array[1]*100));	
				uc_fdb_cmd_data_array_p[6]=(uint8_t)(((int16_t)(f_gimbal_acc_data_array[2]*100))>>8);	
				uc_fdb_cmd_data_array_p[7]=(uint8_t)((int16_t)(f_gimbal_acc_data_array[2]*100));
			}
			
			
			if(SET == en_gimbal_cal_gyro_drift_flag)
			{
				uc_fdb_cmd_data_array_p[8]=(uint8_t)(((int16_t)(f_sensor_gyro_raw_data_array[GIMBAL_GYRO_X_DATA_INDEX]*100))>>8);
				uc_fdb_cmd_data_array_p[9]=(uint8_t)((int16_t)(f_sensor_gyro_raw_data_array[GIMBAL_GYRO_X_DATA_INDEX]*100));	
				uc_fdb_cmd_data_array_p[10]=(uint8_t)(((int16_t)(f_sensor_gyro_raw_data_array[GIMBAL_GYRO_Y_DATA_INDEX]*100))>>8);
				uc_fdb_cmd_data_array_p[11]=(uint8_t)((int16_t)(f_sensor_gyro_raw_data_array[GIMBAL_GYRO_Y_DATA_INDEX]*100));	
				uc_fdb_cmd_data_array_p[12]=(uint8_t)(((int16_t)(f_sensor_gyro_raw_data_array[GIMBAL_GYRO_Z_DATA_INDEX]*100))>>8);	
				uc_fdb_cmd_data_array_p[13]=(uint8_t)((int16_t)(f_sensor_gyro_raw_data_array[GIMBAL_GYRO_Z_DATA_INDEX]*100));
			}
			else
			{
				uc_fdb_cmd_data_array_p[8]=(uint8_t)(((int16_t)(f_gimbal_gyro_data_array[0]*100))>>8);
				uc_fdb_cmd_data_array_p[9]=(uint8_t)((int16_t)(f_gimbal_gyro_data_array[0]*100));	
				uc_fdb_cmd_data_array_p[10]=(uint8_t)(((int16_t)(f_gimbal_gyro_data_array[1]*100))>>8);
				uc_fdb_cmd_data_array_p[11]=(uint8_t)((int16_t)(f_gimbal_gyro_data_array[1]*100));	
				uc_fdb_cmd_data_array_p[12]=(uint8_t)(((int16_t)(f_gimbal_gyro_data_array[2]*100))>>8);	
				uc_fdb_cmd_data_array_p[13]=(uint8_t)((int16_t)(f_gimbal_gyro_data_array[2]*100));
			}
			
			Pack_And_Send_Cmd(FEEDBACK_CMD_TXBUF,uc_fdb_cmd_data_array_p,MODULE_ADD_BYTE);
		}
		else
		{		
			Get_Motor_Frame_Angle(uc_recive_motor_frame_angle);
			
			uc_fdb_cmd_data_array_p[0] = 0x56;
			uc_fdb_cmd_data_array_p[1] = 0x3A;
			uc_fdb_cmd_data_array_p[2]=(uint8_t)(((int16_t)(motor_x_pos_data))>>8);
			uc_fdb_cmd_data_array_p[3]=(uint8_t)((int16_t)(motor_x_pos_data));	
			uc_fdb_cmd_data_array_p[4]=(uint8_t)(((int16_t)(motor_y_pos_data))>>8);
			uc_fdb_cmd_data_array_p[5]=(uint8_t)((int16_t)(motor_y_pos_data));	
			uc_fdb_cmd_data_array_p[6]=(uint8_t)(((int16_t)(motor_z_pos_data))>>8);	
			uc_fdb_cmd_data_array_p[7]=(uint8_t)((int16_t)(motor_z_pos_data));
			
			uc_fdb_cmd_data_array_p[8]=(uint8_t)(0x00);
			uc_fdb_cmd_data_array_p[9]=(uint8_t)(0x00);	
			uc_fdb_cmd_data_array_p[10]=(uint8_t)(0x00);
			uc_fdb_cmd_data_array_p[11]=(uint8_t)(0x00);	
			Pack_And_Send_Cmd(FEEDBACK_CMD_TXBUF,uc_fdb_cmd_data_array_p,MODULE_ADD_BYTE);	
		}
		
		Feedback_Odd_Flag = !Feedback_Odd_Flag;
	}
}



/**
  * @brief 	void	Process_Save_Roll_Ref_Cmd(uint8_t *cmd_data_array_p)
	* @note	 	保存横滚角参考值
  */
//void	Process_Save_Roll_Ref_Cmd(uint8_t *cmd_data_array_p)
//{
//	uint8_t	uc_fdb_cmd_data_array_p[2];
//	
//	if(HORIZONTAL == en_camera_viewer_mode)
//	{
//		if( (f_euler_array[1]>-20)&&(f_euler_array[1]<20) )
//		{
//			f_earth_y_pos_pid_ref_offset_in_horizontal_mode = f_euler_array[1];			
//		}
//		else
//		{
//			f_earth_y_pos_pid_ref_offset_in_horizontal_mode = 0;
//		}
//		en_earth_y_pos_pid_ref_offset_in_horizontal_mode_cal_flag = SET;
//	}
//	else if(VERTICAL == en_camera_viewer_mode)
//	{
//		if( (f_euler_array[1]>70)&&(f_euler_array[1]<110) )
//		{
//			f_earth_y_pos_pid_ref_offset_in_vertical_mode = f_euler_array[1];
//		}
//		else
//		{
//			f_earth_y_pos_pid_ref_offset_in_vertical_mode = 90;
//		}
//		en_earth_y_pos_pid_ref_offset_in_vertical_mode_cal_flag = SET;
//	}
//	
//	Save_Gimbal_Config_Data();
//	
//	uc_fdb_cmd_data_array_p[0] = 0x56;
//	uc_fdb_cmd_data_array_p[1] = 0x80;
//	
//	Pack_And_Send_Cmd(FEEDBACK_CMD_TXBUF,uc_fdb_cmd_data_array_p,MODULE_ADD_BYTE);
//}




//
/**
  * @brief 	void	Process_Set_Spd_Ratio_Cmd(uint8_t *cmd_data_array_p)
	* @note	 	保存横滚角参考值
  */
/**
  * @brief 	void	Process_Ahrs_Query_Config_Info_Cmd(uint8_t *cmd_data_array_p)
	* @note	 	此函数用来响应查询配置信息
  */
