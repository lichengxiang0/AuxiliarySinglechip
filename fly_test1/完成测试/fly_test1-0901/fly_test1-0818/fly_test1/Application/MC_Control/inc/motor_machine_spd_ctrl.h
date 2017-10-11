/**
  ******************************************************************************
  * File Name          : motor_machine_spd_ctrl.h
  * Description        : 电机机械速度环控制
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
#ifndef	_MOTOR_MACHINE_SPD_CTRL_H
#define	_MOTOR_MACHINE_SPD_CTRL_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
	 
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "pid_regulator.h"

/* Exported defines ------------------------------------------------------------*/	 
#define MOTOR_MACHINE_SPD_PID_KP_DEFAULT  ((int16_t)100)
#define MOTOR_MACHINE_SPD_PID_KI_DEFAULT  ((int16_t)150)
#define MOTOR_MACHINE_SPD_PID_KD_DEFAULT  ((int16_t)0)


/* default values for Torque Flux(TF) PID 的放大倍数 */
#define MOTOR_MACHINE_SPD_PID_KPDIV ((uint16_t)(512))			
#define MOTOR_MACHINE_SPD_PID_KIDIV ((uint16_t)(4096))		
#define MOTOR_MACHINE_SPD_PID_KDDIV ((uint16_t)(16))		

	 
/* Exported variables ------------------------------------------------------------*/	 
extern	PID_Struct_Typedef	st_motor_machine_spd_pid_struct;
extern	int16_t							sc_motor_machine_spd_pid_ref;
extern	int16_t							sc_motor_machine_spd_pid_fdb;
extern	int16_t							sc_motor_machine_spd_pid_out;


/* Exported functions ------------------------------------------------------------*/
void	Motor_Machine_Spd_Pid_Init(void);
int16_t	Get_Motor_Machine_Spd(uint16_t	uw_motor_now_machine_pos_p);
void	Motor_Machine_Spd_Pid_Update(int16_t	sc_motor_machine_spd_p);


#ifdef __cplusplus
}
#endif

#endif
