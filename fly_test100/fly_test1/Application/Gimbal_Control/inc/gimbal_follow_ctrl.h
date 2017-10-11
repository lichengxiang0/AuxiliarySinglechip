/**
  ******************************************************************************
  * File Name          :gimbal_attitude_ctrl.c
  * Description        :云台姿态控制
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
#ifndef	_GIMBAL_FOLLOW_CTRL_H
#define	_GIMBAL_FOLLOW_CTRL_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "pid_regulator.h"
	 

/* Exported typedef ------------------------------------------------------------*/	 
typedef	enum{FOLLOW_MOTOR_X = 0,	FOLLOW_MOTOR_Y,	FOLLOW_MOTOR_Z} Follow_Motor_Typedef;	 
	 
/* Exported define ------------------------------------------------------------*/	 
//------------------------------default value for earth z follow pos pid argument -----------------------------// 
/* default values for earth x follow	pos pid control loop */
#define EARTH_X_FOLLOW_POS_PID_REFERENCE   			((int16_t)0)//0//1500
#define EARTH_X_FOLLOW_POS_PID_KP_DEFAULT      	((int16_t)150)//1000//4000//1800//2000//8000//2000//25
#define EARTH_X_FOLLOW_POS_PID_KI_DEFAULT     	((int16_t)0)//200//500//500//200//100
#define EARTH_X_FOLLOW_POS_PID_KD_DEFAULT      	((int16_t)0)//0

/* earth x pos pid parameter dividers*/
#define EARTH_X_FOLLOW_POS_PID_KPDIV ((uint16_t)(1000))
#define EARTH_X_FOLLOW_POS_PID_KIDIV ((uint16_t)(1000))
#define EARTH_X_FOLLOW_POS_PID_KDDIV ((uint16_t)(16))

/* default values for earth z follow	pos pid control loop */
#define EARTH_Z_FOLLOW_POS_PID_REFERENCE   		(int16_t)0//0//1500
//20160225
#define EARTH_Z_FOLLOW_POS_PID_KP_DEFAULT      (int16_t)150//1000//4000//1800//2000//8000//2000//25//150
#define EARTH_Z_FOLLOW_POS_PID_KI_DEFAULT     (int16_t)0//200//500//500//200//100
#define EARTH_Z_FOLLOW_POS_PID_KD_DEFAULT      (int16_t)0//0

/* earth z pos pid parameter dividers*/
#define EARTH_Z_FOLLOW_POS_PID_KPDIV ((uint16_t)(1000))
#define EARTH_Z_FOLLOW_POS_PID_KIDIV ((uint16_t)(1000))
#define EARTH_Z_FOLLOW_POS_PID_KDDIV ((uint16_t)(16)) 




/* Exported variables ------------------------------------------------------------*/
extern	int16_t		sw_pitch_follow_pos_pid_ref,	sw_yaw_follow_pos_pid_ref; 


extern	PID_Struct_Typedef	st_earth_x_follow_pos_pid_struct,	st_earth_z_follow_pos_pid_struct;
extern	int16_t	earth_x_follow_pos_pid_ref,			earth_z_follow_pos_pid_ref;
extern	int16_t	earth_x_follow_pos_pid_fdb,			earth_z_follow_pos_pid_fdb;
extern	int16_t	earth_x_follow_pos_pid_out,			earth_z_follow_pos_pid_out;



/* Exported functions ------------------------------------------------------------*/

/**
	* @brief：void Cal_Pitch_Follow_Pos_Pid_Ref_Range(float	f_pitch_follow_pos_pid_ref_min,	float	f_pitch_follow_pos_pid_ref_max)
	* @note： 在Pitch_Follow电机为Z电机时需要对pitch_follow的参考进行限制
	*	@param：f_pitch_follow_pos_pid_ref_min：下限
	*	@param：f_pitch_follow_pos_pid_ref_min：上限
	*/
void Cal_Pitch_Follow_Pos_Pid_Ref_Range(float	*euler_p,	float	*motor_angle_p, float	*f_pitch_follow_pos_pid_ref_min_p,	float	*f_pitch_follow_pos_pid_ref_max_p);
void Pitch_Follow_Pos_Pid_Updata(float	*euler_p,	float	*motor_angle_p,	Follow_Motor_Typedef en_pitch_follow_motor_p,	float ** coordinate_transform_matrix_p);
void Yaw_Follow_Pos_Pid_Updata(float	*euler_p, 	float	*motor_angle,	Follow_Motor_Typedef en_yaw_follow_motor_p,	float ** coordinate_transform_matrix_p);



#ifdef __cplusplus
}
#endif	 
	 
#endif		 



