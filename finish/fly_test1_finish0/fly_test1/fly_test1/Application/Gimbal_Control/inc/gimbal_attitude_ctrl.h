/**
  ******************************************************************************
  * File Name          :gimbal_attitude_ctrl.c
  * Description        :ÔÆÌ¨×ËÌ¬¿ØÖÆ
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
#ifndef	_GIMBAL_ATTITUDE_CTRL_H
#define	_GIMBAL_ATTITUDE_CTRL_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "pid_regulator.h"

	 
/* Exported define ------------------------------------------------------------*/
//------------------------------default value for earth x y z pos pid argument -----------------------------// 
/* default values for earth x pos pid control loop */
#define EARTH_X_POS_PID_REFERENCE   		(int16_t)0 //7100//1500
#define EARTH_X_POS_PID_KP_DEFAULT      (int16_t)0//400//5000//4000//1800//2000//8000//2000//25
#define EARTH_X_POS_PID_KI_DEFAULT      (int16_t)0//100//200//500//500//200//100
#define EARTH_X_POS_PID_KD_DEFAULT      (int16_t)0//0

/* earth x pos pid parameter dividers*/
#define EARTH_X_POS_PID_KPDIV ((uint16_t)(1000))
#define EARTH_X_POS_PID_KIDIV ((uint16_t)(100))
#define EARTH_X_POS_PID_KDDIV ((uint16_t)(16))


/* default values for earth y pos pid control loop */
#define EARTH_Y_POS_PID_REFERENCE   		 (int16_t)0//0//1500
#define EARTH_Y_POS_PID_KP_DEFAULT      (int16_t)0//150//1000//5000//4000//4000//4000//1800//2000//8000//2000//25
#define EARTH_Y_POS_PID_KI_DEFAULT      (int16_t)0//5000//4000//400//200//500//500//200//100
#define EARTH_Y_POS_PID_KD_DEFAULT      (int16_t)0//0

/* earth y pos pid parameter dividers*/
#define EARTH_Y_POS_PID_KPDIV ((uint16_t)(1000))		//10
#define EARTH_Y_POS_PID_KIDIV ((uint16_t)(1000))		//100
#define EARTH_Y_POS_PID_KDDIV ((uint16_t)(16))

/* default values for earth z pos pid control loop */
#define EARTH_Z_POS_PID_REFERENCE   		(int16_t)0//0//1500
#define EARTH_Z_POS_PID_KP_DEFAULT      (int16_t)1000//1000//4000//1800//2000//8000//2000//25
#define EARTH_Z_POS_PID_KI_DEFAULT     (int16_t)0//200//500//500//200//100
#define EARTH_Z_POS_PID_KD_DEFAULT      (int16_t)0//0

/* earth z pos pid parameter dividers*/
#define EARTH_Z_POS_PID_KPDIV ((uint16_t)(1000))
#define EARTH_Z_POS_PID_KIDIV ((uint16_t)(1000))
#define EARTH_Z_POS_PID_KDDIV ((uint16_t)(16))



/* Exported variables ------------------------------------------------------------*/
extern	PID_Struct_Typedef	st_earth_x_pos_pid_struct,st_earth_y_pos_pid_struct,st_earth_z_pos_pid_struct;
extern	int16_t	earth_x_pos_pid_ref,			earth_y_pos_pid_ref,			earth_z_pos_pid_ref;
extern	int16_t	earth_x_pos_pid_fdb,			earth_y_pos_pid_fdb,			earth_z_pos_pid_fdb;
extern	int16_t	earth_x_pos_pid_out,			earth_y_pos_pid_out,			earth_z_pos_pid_out;



/* Exported functions ------------------------------------------------------------*/
void Earth_X_Pos_Pid_Updata(float	euler_pitch_p);
void Earth_Y_Pos_Pid_Updata(float	euler_roll_p);
void Earth_Z_Pos_Pid_Updata(float	euler_yaw_p);	 
	

#ifdef __cplusplus
}
#endif	 
	 
#endif	
