/**
  ******************************************************************************
  * File Name          :gimbal_spd_ctrl.c
  * Description        :云台电机速度环控制
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
#ifndef	_GIMBAL_SPD_CTRL_H
#define	_GIMBAL_SPD_CTRL_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
//#include "pid_regulator.h"
	 
/* Exported define ------------------------------------------------------------*/
//------------------------------default value for motor x y z spd pid argument -----------------------------// 
/* default values for motor x spd pid control loop */
#define MOTOR_X_SPEED_PID_REFERENCE   	  ((int16_t)0)//100//1500
#define MOTOR_X_SPEED_PID_KP_DEFAULT      ((int16_t)0)//950)//250//3000//800//1500//2000//400//300
#define MOTOR_X_SPEED_PID_KI_DEFAULT      ((int16_t)0)//100)//25//300//100//200//300//40//100
#define MOTOR_X_SPEED_PID_KD_DEFAULT      ((int16_t)0)//0000

/* motor x spd pid parameter dividers*/
#define MOTOR_X_SPEED_PID_KPDIV ((uint16_t)(100))
#define MOTOR_X_SPEED_PID_KIDIV ((uint16_t)(100))
#define MOTOR_X_SPEED_PID_KDDIV ((uint16_t)(16))

/* default values for motor y spd pid control loop */
#define MOTOR_Y_SPEED_PID_REFERENCE   	  ((int16_t)0)//100//1500
#define MOTOR_Y_SPEED_PID_KP_DEFAULT      ((int16_t)0)//250)//2000)//4000//2000
#define MOTOR_Y_SPEED_PID_KI_DEFAULT      ((int16_t)0)//50)//150)//200//80
#define MOTOR_Y_SPEED_PID_KD_DEFAULT      ((int16_t)0)//0000

/* motor y spd pid parameter dividers*/
#define MOTOR_Y_SPEED_PID_KPDIV ((uint16_t)(100))
#define MOTOR_Y_SPEED_PID_KIDIV ((uint16_t)(100))
#define MOTOR_Y_SPEED_PID_KDDIV ((uint16_t)(16))

/* default values for motor z spd pid control loop */
#define MOTOR_Z_SPEED_PID_REFERENCE   	  ((int16_t)0)//100//1500
#define MOTOR_Z_SPEED_PID_KP_DEFAULT      ((int16_t)850)//550)//4000//2000
#define MOTOR_Z_SPEED_PID_KI_DEFAULT      ((int16_t)320)//280)//200//80
#define MOTOR_Z_SPEED_PID_KD_DEFAULT      ((int16_t)0)//0000

/* motor z spd pid parameter dividers*/
#define MOTOR_Z_SPEED_PID_KPDIV ((uint16_t)(100))
#define MOTOR_Z_SPEED_PID_KIDIV ((uint16_t)(100))
#define MOTOR_Z_SPEED_PID_KDDIV ((uint16_t)(16))


/* Exported variables ------------------------------------------------------------*/
//extern	PID_Struct_Typedef	st_motor_x_spd_pid_struct,			st_motor_y_spd_pid_struct,				st_motor_z_spd_pid_struct;
extern	int16_t							motor_x_spd_pid_ref,						motor_y_spd_pid_ref,							motor_z_spd_pid_ref;
extern	int16_t							motor_x_spd_pid_fdb,						motor_y_spd_pid_fdb,							motor_z_spd_pid_fdb;
extern	int16_t							motor_x_spd_pid_out,						motor_y_spd_pid_out,							motor_z_spd_pid_out;


/* Exported functions ------------------------------------------------------------*/
void	Gimbal_Spd_To_Motor_Spd(float	*gimbal_gyro_data_p,	float *motor_gyro_data_p,	float ** coordinate_transform_matrix_p);
void	Earth_Spd_To_Motor_Spd(float	*earth_gyro_data_p,	float *motor_gyro_data_p,	float ** coordinate_transform_matrix_p);
void	Motor_Spd_Pid_Updata(float *gimbal_gyro_data_p,	float ** coordinate_transform_matrix_p);

#ifdef __cplusplus
}
#endif	 
	 
#endif	
