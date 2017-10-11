/**
  ******************************************************************************
  * File Name          :gimbal_track_ctrl.c
  * Description        :云台跟踪控制
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
#include "gimbal_track_ctrl.h"

#include "pid_regulator.h"
	
	
/* Exported variables ------------------------------------------------------------*/
/* External variables ------------------------------------------------------------*/
PID_Struct_Typedef	st_phone_x_track_pid_struct,			st_phone_y_track_pid_struct;
int16_t							sw_phone_x_track_pid_ref = 0,			sw_phone_y_track_pid_ref = 0;
int16_t							sw_phone_x_track_pid_fdb,					sw_phone_y_track_pid_fdb;
int16_t							sw_phone_x_track_pid_out,					sw_phone_y_track_pid_out;

uint8_t							uc_receive_track_err_data[4];
int16_t							sw_track_err_x_test_data,			sw_track_err_y_test_data;			//test
	
	
/* External functions ------------------------------------------------------------*/
/**
	* @brief：void Phone_X_Track_Pid_Updata(void)
	* @note： 手机长边跟踪位置环PID更新
	*/
void Phone_X_Track_Pid_Updata(void)
{
	sw_phone_x_track_pid_fdb = ( ( (int16_t)(int8_t)uc_receive_track_err_data[0]<<8 )|uc_receive_track_err_data[1] );  //test
//	sw_phone_x_track_pid_fdb = ( ( (uint16_t)uc_receive_track_err_data[0]<<8 )|uc_receive_track_err_data[1] ) - 240;  //test
	sw_phone_x_track_pid_out = PI_Regulator(sw_phone_x_track_pid_ref,sw_phone_x_track_pid_fdb,&st_phone_x_track_pid_struct);
}		
	
	
	
/**
	* @brief：void Phone_Y_Track_Pid_Updata(void)
	* @note： 手机长边跟踪位置环PID更新
	*/
void Phone_Y_Track_Pid_Updata(void)
{
	sw_phone_y_track_pid_fdb = ( ( (int16_t)(int8_t)uc_receive_track_err_data[2]<<8 )|uc_receive_track_err_data[3] );			//test
//	sw_phone_y_track_pid_fdb = ( ( (uint16_t)uc_receive_track_err_data[2]<<8 )|uc_receive_track_err_data[3] )  - 160;  //test
	sw_phone_y_track_pid_out = PI_Regulator(sw_phone_y_track_pid_ref,sw_phone_y_track_pid_fdb,&st_phone_y_track_pid_struct);
}



		