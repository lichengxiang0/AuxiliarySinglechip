/**
  ******************************************************************************
  * File Name          :motor_pos_ctrl.c
  * Description        :电机位置环控制
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
#include "motor_pos_ctrl.h"

/* Private define ------------------------------------------------------------*/
#define	DEG_TO_RADIAN		((float)0.01744)
#define	RADIAN_TO_DEG		((float)57.3)



/* Exported variables ------------------------------------------------------------*/
PID_Struct_Typedef	st_motor_x_pos_pid_struct,			st_motor_y_pos_pid_struct,				st_motor_z_pos_pid_struct;
int16_t							motor_x_pos_pid_ref,						motor_y_pos_pid_ref,							motor_z_pos_pid_ref;
int16_t							motor_x_pos_pid_fdb,						motor_y_pos_pid_fdb,							motor_z_pos_pid_fdb;
int16_t							motor_x_pos_pid_out,						motor_y_pos_pid_out,							motor_z_pos_pid_out;

uint8_t							uc_receive_frame_angle_ref_data[6];

/* External functions ------------------------------------------------------------*/
/**
	* @brief：void Motor_X_Pos_Pid_Updata(float*	motor_frame_angle_rad_p)
	* @note： X轴位置环更新
	*	@param：float*	motor_frame_angle_rad_p电机框架角单位rad
	*/
void Motor_X_Pos_Pid_Updata(float*	motor_frame_angle_rad_p)
{
	motor_x_pos_pid_ref = ( ( (int16_t)(int8_t)uc_receive_frame_angle_ref_data[0]<<8 )|uc_receive_frame_angle_ref_data[1] );
	
	motor_x_pos_pid_fdb = (int16_t)(motor_frame_angle_rad_p[0]*RADIAN_TO_DEG*80);
	motor_x_pos_pid_out = PI_Regulator(motor_x_pos_pid_ref,motor_x_pos_pid_fdb,&st_motor_x_pos_pid_struct);
}	

/**
	* @brief：void Motor_Y_Pos_Pid_Updata(float*	motor_frame_angle_rad_p)
	* @note： Y轴位置环更新
	*	@param：float*	motor_frame_angle_rad_p电机框架角单位rad
	*/
void Motor_Y_Pos_Pid_Updata(float*	motor_frame_angle_rad_p)
{
	motor_y_pos_pid_ref = ( ( (int16_t)(int8_t)uc_receive_frame_angle_ref_data[2]<<8 )|uc_receive_frame_angle_ref_data[3] );
	
	motor_y_pos_pid_fdb = (int16_t)(motor_frame_angle_rad_p[1]*RADIAN_TO_DEG*80);
	motor_y_pos_pid_out = PI_Regulator(motor_y_pos_pid_ref,motor_y_pos_pid_fdb,&st_motor_y_pos_pid_struct);
}




/**
	* @brief：void Motor_Z_Pos_Pid_Updata(float*	motor_frame_angle_rad_p)
	* @note： Z轴位置环更新
	*	@param：float*	motor_frame_angle_rad_p电机框架角单位rad
	*/
void Motor_Z_Pos_Pid_Updata(float*	motor_frame_angle_rad_p)
{
	motor_z_pos_pid_ref = ( ( (int16_t)(int8_t)uc_receive_frame_angle_ref_data[4]<<8 )|uc_receive_frame_angle_ref_data[5] );
	motor_z_pos_pid_fdb = (int16_t)(motor_frame_angle_rad_p[2]*RADIAN_TO_DEG*80);
	motor_z_pos_pid_out = PI_Regulator(motor_z_pos_pid_ref,motor_z_pos_pid_fdb,&st_motor_z_pos_pid_struct);
}





	







