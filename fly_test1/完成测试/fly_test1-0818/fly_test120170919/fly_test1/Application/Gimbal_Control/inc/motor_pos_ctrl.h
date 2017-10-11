/**
  ******************************************************************************
  * File Name          :motor_pos_ctrl.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef	_MOTOR_POS_CTRL_H
#define	_MOTOR_POS_CTRL_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
//#include "pid_regulator.h"
#include "motor_pos_ctrl.h"
	 
/* Exported define ------------------------------------------------------------*/
//------------------------------default value for motor x y z pos pid argument -----------------------------// 
/* default values for motor x pos pid control loop */
/* default values for earth x pos pid control loop */
#define MOTOR_X_POS_PID_REFERENCE   		(int16_t)0//0//1500
#define MOTOR_X_POS_PID_KP_DEFAULT      (int16_t)1500//5000//4000//1800//2000//8000//2000//25
#define MOTOR_X_POS_PID_KI_DEFAULT      (int16_t)0//100//200//500//500//200//100
#define MOTOR_X_POS_PID_KD_DEFAULT      (int16_t)0//0

/* earth x pos pid parameter dividers*/
#define MOTOR_X_POS_PID_KPDIV ((uint16_t)(1000))
#define MOTOR_X_POS_PID_KIDIV ((uint16_t)(100))
#define MOTOR_X_POS_PID_KDDIV ((uint16_t)(16))


/* default values for earth y pos pid control loop */
#define MOTOR_Y_POS_PID_REFERENCE   		 (int16_t)0//0//1500
#define MOTOR_Y_POS_PID_KP_DEFAULT      (int16_t)1000//5000//4000//4000//4000//1800//2000//8000//2000//25
#define MOTOR_Y_POS_PID_KI_DEFAULT      (int16_t)0//5000//4000//400//200//500//500//200//100
#define MOTOR_Y_POS_PID_KD_DEFAULT      (int16_t)0//0

/* earth y pos pid parameter dividers*/
#define MOTOR_Y_POS_PID_KPDIV ((uint16_t)(1000))		//10
#define MOTOR_Y_POS_PID_KIDIV ((uint16_t)(1000))		//100
#define MOTOR_Y_POS_PID_KDDIV ((uint16_t)(16))

/* default values for earth z pos pid control loop */
#define MOTOR_Z_POS_PID_REFERENCE   		(int16_t)0//0//1500
#define MOTOR_Z_POS_PID_KP_DEFAULT      (int16_t)1000//1000//4000//1800//2000//8000//2000//25
#define MOTOR_Z_POS_PID_KI_DEFAULT     (int16_t)0//200//500//500//200//100
#define MOTOR_Z_POS_PID_KD_DEFAULT      (int16_t)0//0

/* earth z pos pid parameter dividers*/
#define MOTOR_Z_POS_PID_KPDIV ((uint16_t)(1000))
#define MOTOR_Z_POS_PID_KIDIV ((uint16_t)(1000))
#define MOTOR_Z_POS_PID_KDDIV ((uint16_t)(16))


/* Exported variables ------------------------------------------------------------*/
extern	PID_Struct_Typedef	st_motor_x_pos_pid_struct,			st_motor_y_pos_pid_struct,				st_motor_z_pos_pid_struct;
extern	int16_t							motor_x_pos_pid_ref,						motor_y_pos_pid_ref,							motor_z_pos_pid_ref;
extern	int16_t							motor_x_pos_pid_fdb,						motor_y_pos_pid_fdb,							motor_z_pos_pid_fdb;
extern	int16_t							motor_x_pos_pid_out,						motor_y_pos_pid_out,							motor_z_pos_pid_out;

extern	uint8_t							uc_receive_frame_angle_ref_data[6];


void Motor_X_Pos_Pid_Updata(float*	motor_frame_angle_rad_p);
void Motor_Y_Pos_Pid_Updata(float*	motor_frame_angle_rad_p);
void Motor_Z_Pos_Pid_Updata(float*	motor_frame_angle_rad_p);

#ifdef __cplusplus
}
#endif	 
	 
#endif	
