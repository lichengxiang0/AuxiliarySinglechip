/**
  ******************************************************************************
  * File Name          :gimbal_frame_angle.c
  * Description        :读取框架角度
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
	
#include "stm32f1xx_hal.h"
#include "gimbal_frame_angle.h"

/* Private variables ------------------------------------------------------------*/
#define		PI		(3.14159)


/* External variables ------------------------------------------------------------*/
int16_t		motor_x_pos_data,					motor_y_pos_data,					motor_z_pos_data;  /*上传显示用*/
//int16_t		motor_x_zero_pos_data = 0x820,		motor_y_zero_pos_data = 0x976,		motor_z_zero_pos_data = 0x600 ; 					//第二套
float			motor_frame_angle_rad[3];  														//xyz

/* External functions ------------------------------------------------------------*/
void Get_Motor_Frame_Angle(uint8_t	*uc_recive_motor_frame_angle_p)
{	
	motor_x_pos_data = -( ( (int16_t)(int8_t)uc_recive_motor_frame_angle_p[4]<<8 )|uc_recive_motor_frame_angle_p[5] );
	motor_y_pos_data = -( ( (int16_t)(int8_t)uc_recive_motor_frame_angle_p[0]<<8 )|uc_recive_motor_frame_angle_p[1] );
	motor_z_pos_data = -( ( (int16_t)(int8_t)uc_recive_motor_frame_angle_p[2]<<8 )|uc_recive_motor_frame_angle_p[3] );
	
	
	motor_frame_angle_rad[0] = ( (float)motor_x_pos_data*PI )/2047.0;
	motor_frame_angle_rad[1] = ( (float)motor_y_pos_data*PI )/2047.0;
	motor_frame_angle_rad[2] = ( (float)motor_z_pos_data*PI )/2047.0;
}
