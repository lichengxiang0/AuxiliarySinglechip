/**
  ******************************************************************************
  * File Name          :motor_config.h
  * Description        :电机驱动的配置文件
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
#include "motor_config.h"

#include "task_motor_control.h"

/* Private define ------------------------------------------------------------*/
#define		MOTOR_CONFIG_DATA_ADDRESS			(ADDR_FLASH_PAGE_124)


/* External variables ------------------------------------------------------------*/
Motor_Config_Data_Struct_Typedef	st_motor_config_data = {	0x0000,
																														0x0000,
																														0x01,
																														0x01,
																														0x00,
																														0x0000
																													};

/* External function ---------------------------------------------------------*/
/**
  * @brief 	void Get_Motor_Config_Data(void)
	* @note	 	获取电机配置数据
  */
void Get_Motor_Config_Data(void)
{
	uint8_t 	uc_check_sum_p = 0;
	uint32_t	ul_st_config_data_address_p;
	
	FlashRead( MOTOR_CONFIG_DATA_ADDRESS,(&st_motor_config_data), sizeof(st_motor_config_data) );
	
	if(0x55AA == st_motor_config_data.motor_driver_cal_flag)
	{	
		st_motor_pos_info.uw_motor_elec_zero_pos = st_motor_config_data.zero_electrical_angle;
		st_motor_pos_info.sc_motor_elec_angle_dir = st_motor_config_data.electrical_angle_direction;
		sc_torque_direction = st_motor_config_data.torque_direction;
		st_motor_pos_info.uw_motor_frame_zero_pos = st_motor_config_data.uw_motor_frame_zero_pos;
		/*cal check_sum*/
		ul_st_config_data_address_p = (uint32_t)&st_motor_config_data;
		for(uint8_t i=0;i<sizeof(st_motor_config_data);i++)
		{
			uc_check_sum_p ^= *(uint8_t *)ul_st_config_data_address_p++;
		}
		st_motor_config_data.uc_xor_verify_value = uc_check_sum_p;
	}
}


/**
  * @brief 	void Save_Motor_Config_Data(void)
	* @note	 	保存电机配置数据
  */
void Save_Motor_Config_Data(void)
{
	uint8_t 	uc_check_sum_p = 0;
	uint32_t	ul_st_config_data_address_p;
	
	st_motor_config_data.motor_driver_cal_flag = 0x55AA;
	st_motor_config_data.zero_electrical_angle = st_motor_pos_info.uw_motor_elec_zero_pos;
	st_motor_config_data.electrical_angle_direction = st_motor_pos_info.sc_motor_elec_angle_dir;
	st_motor_config_data.torque_direction = sc_torque_direction;
	
	st_motor_config_data.uw_motor_frame_zero_pos = st_motor_pos_info.uw_motor_frame_zero_pos;
	//cal check_sum
	ul_st_config_data_address_p = (uint32_t)&st_motor_config_data;
	for(uint8_t i=0;i<sizeof(st_motor_config_data);i++)
	{
		uc_check_sum_p ^= *(uint8_t *)ul_st_config_data_address_p++;
	}
	uc_check_sum_p ^= st_motor_config_data.uc_xor_verify_value;
	
	st_motor_config_data.uc_xor_verify_value = uc_check_sum_p;
	FlashProgram( MOTOR_CONFIG_DATA_ADDRESS,(&st_motor_config_data), sizeof(st_motor_config_data) );
}

