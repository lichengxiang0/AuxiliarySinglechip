/**
  ******************************************************************************
  * File Name          :gimbal_track_ctrl.c
  * Description        :ÔÆÌ¨¸ú×Ù¿ØÖÆ
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
#ifndef	_GIMBAL_TRACK_CTRL_H
#define	_GIMBAL_TRACK_CTRL_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
//#include "pid_regulator.h"

/* Exported define ------------------------------------------------------------*/
//------------------------------default value for earth x y z pos pid argument -----------------------------// 
/* default values for earth x pos pid control loop */
#define PHONE_X_TRACK_PID_REFERENCE   		(int16_t)0//0//1500
#define PHONE_X_TRACK_PID_KP_DEFAULT      (int16_t)-200//5000//4000//1800//2000//8000//2000//25
#define PHONE_X_TRACK_PID_KI_DEFAULT      (int16_t)0//100//200//500//500//200//100
#define PHONE_X_TRACK_PID_KD_DEFAULT      (int16_t)0//0

/* earth x pos pid parameter dividers*/
#define PHONE_X_TRACK_PID_KPDIV ((uint16_t)(1000))
#define PHONE_X_TRACK_PID_KIDIV ((uint16_t)(1000))
#define PHONE_X_TRACK_PID_KDDIV ((uint16_t)(16))
	 
	 
	 
/* default values for earth y pos pid control loop */
#define PHONE_Y_TRACK_PID_REFERENCE   		 	(int16_t)0//0//1500
#define PHONE_Y_TRACK_PID_KP_DEFAULT      	(int16_t)200//5000//4000//4000//4000//1800//2000//8000//2000//25
#define PHONE_Y_TRACK_PID_KI_DEFAULT      	(int16_t)0//5000//4000//400//200//500//500//200//100
#define PHONE_Y_TRACK_PID_KD_DEFAULT      	(int16_t)0//0

/* earth y pos pid parameter dividers*/
#define PHONE_Y_TRACK_PID_KPDIV ((uint16_t)(1000))		//10
#define PHONE_Y_TRACK_PID_KIDIV ((uint16_t)(1000))		//100
#define PHONE_Y_TRACK_PID_KDDIV ((uint16_t)(16))	 
	 
	 

/* Exported variables ---------------------------------------------------------*/	 
//extern	PID_Struct_Typedef	st_phone_x_track_pid_struct,			st_phone_y_track_pid_struct;
extern	int16_t							sw_phone_x_track_pid_ref,					sw_phone_y_track_pid_ref;
extern	int16_t							sw_phone_x_track_pid_fdb,					sw_phone_y_track_pid_fdb;
extern	int16_t							sw_phone_x_track_pid_out,					sw_phone_y_track_pid_out;

extern	uint8_t							uc_receive_track_err_data[4];
extern	int16_t							sw_track_err_x_test_data,			sw_track_err_y_test_data;	  //test
	 
/* Exported functions ---------------------------------------------------------*/
void Phone_X_Track_Pid_Updata(void);
void Phone_Y_Track_Pid_Updata(void);	 

#ifdef __cplusplus
}
#endif	 
	 
#endif	





