/**
  ******************************************************************************
  * File Name          :gimbal_config.h
  * Description        :姿态板部分的配置文件
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
#include "gimbal_config.h"	

#include "gimbal_ahrs.h"
#include "gimbal_spd_ctrl.h"
#include "gimbal_attitude_ctrl.h"
#include "gimbal_follow_ctrl.h"

#include "task_gimbal_pos_ctrl.h"
#include "respond_gimbal_cmd.h"
#include "task_gimbal_ui.h"

#include "protocol.h"
#include "stm32_bsp_uart.h"
#include "stm32_bsp_peripheral_init_export.h"

#include <math.h>


/* Private define ------------------------------------------------------------*/
#define		GIMABL_CONFIG_DATA_ADDRESS			(ADDR_FLASH_PAGE_125)


/* External variables ---------------------------------------------------------*/
Gimbal_Config_Data_Struct_Typedef			st_gimbal_config_data;



/* External function ---------------------------------------------------------*/
/**
  * @brief 	void Get_Gimbal_Config_Data(void)
	* @note	 	读取云台配置数据
  */
void Get_Gimbal_Config_Data(void)
{
	uint8_t 	uc_check_sum_p = 0;
	uint32_t	ul_st_config_data_address_p;
	
	FlashRead( GIMABL_CONFIG_DATA_ADDRESS,(&st_gimbal_config_data), sizeof(st_gimbal_config_data) );
	
	if(st_gimbal_config_data.ahrs_cal_flag == 0x55AA)
	{	
		if(	(fabs(st_gimbal_config_data.gyro_x_offset) < 20)&&
				(fabs(st_gimbal_config_data.gyro_y_offset) < 20)&&
				(fabs(st_gimbal_config_data.gyro_z_offset) < 20)	)
		{
			f_sensor_gyro_offset_data_array[GIMBAL_GYRO_X_DATA_INDEX] = st_gimbal_config_data.gyro_x_offset;
			f_sensor_gyro_offset_data_array[GIMBAL_GYRO_Y_DATA_INDEX] = st_gimbal_config_data.gyro_y_offset;
			f_sensor_gyro_offset_data_array[GIMBAL_GYRO_Z_DATA_INDEX] = st_gimbal_config_data.gyro_z_offset;
		}
		
		if(	(fabs(st_gimbal_config_data.acc_x_offset) < 0.5)&&
				(fabs(st_gimbal_config_data.acc_y_offset) < 0.5)&&
				(fabs(st_gimbal_config_data.acc_z_offset) < 0.5)	)			
		{
			f_sensor_acc_offset_data_array[GIMBAL_ACC_X_DATA_INDEX] = st_gimbal_config_data.acc_x_offset;
			f_sensor_acc_offset_data_array[GIMBAL_ACC_Y_DATA_INDEX] = st_gimbal_config_data.acc_y_offset;
			f_sensor_acc_offset_data_array[GIMBAL_ACC_Z_DATA_INDEX] = st_gimbal_config_data.acc_z_offset;
		}
		
		
		if(	(st_gimbal_config_data.acc_x_scale > 0.8)&&(st_gimbal_config_data.acc_x_scale < 1.2)&&
				(st_gimbal_config_data.acc_y_scale > 0.8)&&(st_gimbal_config_data.acc_y_scale < 1.2)&&
				(st_gimbal_config_data.acc_y_scale > 0.8)&&(st_gimbal_config_data.acc_y_scale < 1.2) )
		{
			f_sensor_acc_scale_data_array[GIMBAL_ACC_X_DATA_INDEX] = st_gimbal_config_data.acc_x_scale;
			f_sensor_acc_scale_data_array[GIMBAL_ACC_Y_DATA_INDEX] = st_gimbal_config_data.acc_y_scale;
			f_sensor_acc_scale_data_array[GIMBAL_ACC_Z_DATA_INDEX] = st_gimbal_config_data.acc_z_scale;
		}
		
		
		en_earth_y_pos_pid_ref_offset_in_horizontal_mode_cal_flag = st_gimbal_config_data.earth_y_pos_pid_ref_offset_in_horizontal_mode_cal_flag;
		en_earth_y_pos_pid_ref_offset_in_vertical_mode_cal_flag = st_gimbal_config_data.earth_y_pos_pid_ref_offset_in_vertical_mode_cal_flag;
		
		
//读取FLASH里面的欧拉角偏置	
		if(SET == en_earth_y_pos_pid_ref_offset_in_horizontal_mode_cal_flag)
		{
			if( (st_gimbal_config_data.earth_y_pos_pid_ref_offset_in_horizontal_mode > -20)&&(st_gimbal_config_data.earth_y_pos_pid_ref_offset_in_horizontal_mode < 20) )
			{
				f_earth_y_pos_pid_ref_offset_in_horizontal_mode = st_gimbal_config_data.earth_y_pos_pid_ref_offset_in_horizontal_mode;
			}
		}
		
		if(SET == en_earth_y_pos_pid_ref_offset_in_vertical_mode_cal_flag)
		{
			if( (st_gimbal_config_data.earth_y_pos_pid_ref_offset_in_vertical_mode > 70)&&(st_gimbal_config_data.earth_y_pos_pid_ref_offset_in_vertical_mode < 110) )
			{
				f_earth_y_pos_pid_ref_offset_in_vertical_mode		=	st_gimbal_config_data.earth_y_pos_pid_ref_offset_in_vertical_mode;
			}		
		}
	
		/*cal check_sum*/
		ul_st_config_data_address_p = (uint32_t)&st_gimbal_config_data;
		for(uint16_t i=0;i<sizeof(st_gimbal_config_data);i++)
		{
			uc_check_sum_p ^= *(uint8_t *)ul_st_config_data_address_p++;
		}
		
		st_gimbal_config_data.uc_xor_verify_value = uc_check_sum_p;
		
		
		/*读取gimbal校准状态*/
		if( (st_gimbal_config_data.uc_gimbal_cal_ok_item_cfg&0x80) == 0x00)
		{
			uc_gimbal_cal_ok_item = st_gimbal_config_data.uc_gimbal_cal_ok_item_cfg;
		}
		
		/*读取用户控制速度系数*/
		if( (st_gimbal_config_data.f_yaw_user_ctrl_spd_ratio_cfg >= 1.0f)&&(st_gimbal_config_data.f_yaw_user_ctrl_spd_ratio_cfg <= 5.0f) )
		{
			f_yaw_user_ctrl_spd_ratio = st_gimbal_config_data.f_yaw_user_ctrl_spd_ratio_cfg;
		}
		
		if( (st_gimbal_config_data.f_pitch_user_ctrl_spd_ratio_cfg >= 1.0f)&&(st_gimbal_config_data.f_pitch_user_ctrl_spd_ratio_cfg <= 5.0f) )
		{
			f_pitch_user_ctrl_spd_ratio = st_gimbal_config_data.f_pitch_user_ctrl_spd_ratio_cfg;
		}
		
		/*读取随动速度系数*/
		if( (st_gimbal_config_data.f_yaw_follow_spd_ratio_cfg >= 5.0f)&&(st_gimbal_config_data.f_yaw_follow_spd_ratio_cfg <= 10.0f) )
		{
			f_yaw_follow_spd_ratio = st_gimbal_config_data.f_yaw_follow_spd_ratio_cfg;
		}
		
		if( (st_gimbal_config_data.f_pitch_follow_spd_ratio_cfg >= 5.0f)&&(st_gimbal_config_data.f_pitch_follow_spd_ratio_cfg <= 10.0f) )
		{
			f_pitch_follow_spd_ratio = st_gimbal_config_data.f_pitch_follow_spd_ratio_cfg;
		}			
	}
}


/**
  * @brief 	void Save_Gimbal_Config_Data(void)
	* @note	 	保存云台配置数据
  */
void Save_Gimbal_Config_Data(void)
{
	uint8_t 	uc_check_sum_p = 0;
	uint32_t	ul_st_config_data_address_p;
	
	/*因为此函数执行周期很长，导致长时间无法处理接收到的串口数据，导致接收缓冲区溢出*/
	__HAL_UART_DISABLE_IT(SET_CMD_RXBUF->ptUartHandel,UART_IT_RXNE);  						//test
	
	st_gimbal_config_data.ahrs_cal_flag = 0x55AA;
	
	st_gimbal_config_data.gyro_x_offset = f_sensor_gyro_offset_data_array[GIMBAL_GYRO_X_DATA_INDEX];
	st_gimbal_config_data.gyro_y_offset = f_sensor_gyro_offset_data_array[GIMBAL_GYRO_Y_DATA_INDEX];
	st_gimbal_config_data.gyro_z_offset = f_sensor_gyro_offset_data_array[GIMBAL_GYRO_Z_DATA_INDEX];
	
	st_gimbal_config_data.acc_x_offset = f_sensor_acc_offset_data_array[GIMBAL_ACC_X_DATA_INDEX];
	st_gimbal_config_data.acc_y_offset = f_sensor_acc_offset_data_array[GIMBAL_ACC_Y_DATA_INDEX];
	st_gimbal_config_data.acc_z_offset = f_sensor_acc_offset_data_array[GIMBAL_ACC_Z_DATA_INDEX];
	
	st_gimbal_config_data.acc_x_scale = f_sensor_acc_scale_data_array[GIMBAL_ACC_X_DATA_INDEX];
	st_gimbal_config_data.acc_y_scale = f_sensor_acc_scale_data_array[GIMBAL_ACC_Y_DATA_INDEX];
	st_gimbal_config_data.acc_z_scale = f_sensor_acc_scale_data_array[GIMBAL_ACC_Z_DATA_INDEX];
	
	st_gimbal_config_data.earth_y_pos_pid_ref_offset_in_horizontal_mode = f_earth_y_pos_pid_ref_offset_in_horizontal_mode;
	st_gimbal_config_data.earth_y_pos_pid_ref_offset_in_horizontal_mode_cal_flag = en_earth_y_pos_pid_ref_offset_in_horizontal_mode_cal_flag;
	
	st_gimbal_config_data.earth_y_pos_pid_ref_offset_in_vertical_mode = f_earth_y_pos_pid_ref_offset_in_vertical_mode;
	st_gimbal_config_data.earth_y_pos_pid_ref_offset_in_vertical_mode_cal_flag = en_earth_y_pos_pid_ref_offset_in_vertical_mode_cal_flag;
	
	st_gimbal_config_data.st_motor_x_spd_pid = st_motor_x_spd_pid_struct;
	st_gimbal_config_data.st_motor_y_spd_pid = st_motor_y_spd_pid_struct;
	st_gimbal_config_data.st_motor_z_spd_pid = st_motor_z_spd_pid_struct;
	
	st_gimbal_config_data.st_earth_x_pos_pid = st_earth_x_pos_pid_struct;
	st_gimbal_config_data.st_earth_y_pos_pid = st_earth_y_pos_pid_struct;
	st_gimbal_config_data.st_earth_z_pos_pid = st_earth_z_pos_pid_struct;
	
	st_gimbal_config_data.st_earth_z_follow_pos_pid = st_earth_z_follow_pos_pid_struct;	
	
//	st_gimbal_config_data.st_temp_gyro_offset_polynomial_fit = st_temp_gyro_offset_polyfit_struct;
	
	ul_st_config_data_address_p = (uint32_t)&st_gimbal_config_data;
	for(uint16_t i=0;i<sizeof(st_gimbal_config_data);i++)
	{
		uc_check_sum_p ^= *(uint8_t *)ul_st_config_data_address_p++;
	}
	uc_check_sum_p ^= st_gimbal_config_data.uc_xor_verify_value;	
	st_gimbal_config_data.uc_xor_verify_value = uc_check_sum_p;
	
	/*存储gimbal校准状态*/
	st_gimbal_config_data.uc_gimbal_cal_ok_item_cfg = uc_gimbal_cal_ok_item;
	
	/*存储user_ctrl_spd_ratio*/
	st_gimbal_config_data.f_yaw_user_ctrl_spd_ratio_cfg = f_yaw_user_ctrl_spd_ratio;
	st_gimbal_config_data.f_pitch_user_ctrl_spd_ratio_cfg = f_pitch_user_ctrl_spd_ratio;
	
	/*存储follow_spd_ratio*/
	st_gimbal_config_data.f_yaw_follow_spd_ratio_cfg = f_yaw_follow_spd_ratio;
	st_gimbal_config_data.f_pitch_follow_spd_ratio_cfg = f_pitch_follow_spd_ratio;

	FlashProgram( GIMABL_CONFIG_DATA_ADDRESS,(&st_gimbal_config_data), sizeof(st_gimbal_config_data) );
	
	__HAL_UART_ENABLE_IT(SET_CMD_RXBUF->ptUartHandel,UART_IT_RXNE);  //test
}



