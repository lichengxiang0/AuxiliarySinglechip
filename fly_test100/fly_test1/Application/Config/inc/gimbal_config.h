/**
  ******************************************************************************
  * File Name          :gimbal_config.h
  * Description        :��̬�岿�ֵ������ļ�
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
#ifndef	_GIMBAL_CONFIG_H
#define	_GIMBAL_CONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32_bsp_flash.h"
	 
#include "pid_regulator.h" 

/* Exported typedef ---------------------------------------------------------*/	 
typedef struct
{
	float 				temp1_gyro_data[4];  //0:temp 1:x_off 2:y_off 3:z_off
	float 				temp2_gyro_data[4];
	float 				temp3_gyro_data[4];
	
	float 				temp_gyro_x_polynomial_coefficient[3];  //����������
	float 				temp_gyro_y_polynomial_coefficient[3];  //����������
	float 				temp_gyro_z_polynomial_coefficient[3];  //����������
	
	uint8_t 			temp_gyro_data_cnt;
	FlagStatus		temp_gyro_polynomial_ok_flag;
	
}Temp_Gyro_Offset_Polynomial_Fit_Typedef;



typedef struct   //�汾���������У������¼ӵı���Ҫ�ŵ�����沢���ӱ�־λ
{ 	
	uint16_t							ahrs_cal_flag;
	
	uint16_t							motor_x_zero_pos_data;
	uint16_t							motor_y_zero_pos_data;
	uint16_t							motor_z_zero_pos_data;
	
	float									gyro_x_offset;
	float     						gyro_y_offset;
	float     						gyro_z_offset;
	
	float     						acc_x_offset;
	float     						acc_y_offset;
	float     						acc_z_offset;
	
	float									acc_x_scale;
	float									acc_y_scale;
	float									acc_z_scale;
	
	//20160226
	PID_Struct_Typedef		st_motor_x_spd_pid;
	PID_Struct_Typedef		st_motor_y_spd_pid;
	PID_Struct_Typedef		st_motor_z_spd_pid;
	
	PID_Struct_Typedef		st_earth_x_pos_pid;
	PID_Struct_Typedef		st_earth_y_pos_pid;
	PID_Struct_Typedef		st_earth_z_pos_pid;
	
	PID_Struct_Typedef		st_earth_z_follow_pos_pid;
	
	Temp_Gyro_Offset_Polynomial_Fit_Typedef	st_temp_gyro_offset_polynomial_fit;
	
	//���ӽǶ�ƫ��	
	float                 earth_y_pos_pid_ref_offset_in_horizontal_mode;
	FlagStatus						earth_y_pos_pid_ref_offset_in_horizontal_mode_cal_flag;
	
	float                 earth_y_pos_pid_ref_offset_in_vertical_mode;
	FlagStatus						earth_y_pos_pid_ref_offset_in_vertical_mode_cal_flag;

	uint8_t								uc_xor_verify_value;
	
	/*2016-05-18�����ٶ�������Ϣ�ͱ���У׼״̬*/
	uint8_t								uc_gimbal_cal_ok_item_cfg;
	
	float									f_yaw_follow_spd_ratio_cfg;
	float									f_pitch_follow_spd_ratio_cfg;
	
	float									f_yaw_user_ctrl_spd_ratio_cfg;
	float									f_pitch_user_ctrl_spd_ratio_cfg;
	
}Gimbal_Config_Data_Struct_Typedef;


/* Exported variables ---------------------------------------------------------*/
extern	Gimbal_Config_Data_Struct_Typedef			st_gimbal_config_data;


/* Exported function ---------------------------------------------------------*/
void Get_Gimbal_Config_Data(void);
void Save_Gimbal_Config_Data(void);

#ifdef __cplusplus
}
#endif


#endif











