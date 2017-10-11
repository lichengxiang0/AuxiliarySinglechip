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



/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "motor_machine_spd_ctrl.h"

#include "stm32_bsp_timer.h"

/* Private defines ------------------------------------------------------------*/
#define	MOTOR_MACHINE_SPD_FLT_BUF_LENGTH			((uint8_t)5)


/* External variables ------------------------------------------------------------*/
PID_Struct_Typedef	st_motor_machine_spd_pid_struct;
int16_t							sc_motor_machine_spd_pid_ref = 0;
int16_t							sc_motor_machine_spd_pid_fdb;
int16_t							sc_motor_machine_spd_pid_out;


/* External functions ------------------------------------------------------------*/
/**
	* @brief：void	Motor_Machine_Spd_Pid_Init(void)
	* @note:	motor_machine_spd_pid初始化
	*/
void	Motor_Machine_Spd_Pid_Init(void)
{
	st_motor_machine_spd_pid_struct.hKp_Gain    = MOTOR_MACHINE_SPD_PID_KP_DEFAULT;  //250
  st_motor_machine_spd_pid_struct.hKp_Divisor = MOTOR_MACHINE_SPD_PID_KPDIV;       //100

  st_motor_machine_spd_pid_struct.hKi_Gain = MOTOR_MACHINE_SPD_PID_KI_DEFAULT;    //20
  st_motor_machine_spd_pid_struct.hKi_Divisor = MOTOR_MACHINE_SPD_PID_KIDIV;      //100
  
  st_motor_machine_spd_pid_struct.hKd_Gain = MOTOR_MACHINE_SPD_PID_KD_DEFAULT;    //0
  st_motor_machine_spd_pid_struct.hKd_Divisor = MOTOR_MACHINE_SPD_PID_KDDIV;      //16
  st_motor_machine_spd_pid_struct.wPreviousError = 0;
  
	
	st_motor_machine_spd_pid_struct.hLower_Threshold_Integral = -1000;
	st_motor_machine_spd_pid_struct.hUpper_Threshold_Integral = 1000;
  st_motor_machine_spd_pid_struct.wLower_Limit_Integral = -900 * MOTOR_MACHINE_SPD_PID_KIDIV;
  st_motor_machine_spd_pid_struct.wUpper_Limit_Integral = 900 * MOTOR_MACHINE_SPD_PID_KIDIV;
  st_motor_machine_spd_pid_struct.hLower_Limit_Output= -900;   //Lower Limit for Output limitation
  st_motor_machine_spd_pid_struct.hUpper_Limit_Output= 900;   //Upper Limit for Output limitation
	st_motor_machine_spd_pid_struct.wIntegral = 0;
}


/**
	* @brief：int16_t	Get_Motor_Machine_Spd(uint16_t	uw_motor_now_machine_pos_p)
	* @note:	计算电机机械速度，利用位置进行微分
	*/

int16_t	Get_Motor_Machine_Spd(uint16_t	uw_motor_now_machine_pos_p)
{
	static	int16_t		motor_machine_spd_flt_buf[MOTOR_MACHINE_SPD_FLT_BUF_LENGTH];
	static	uint16_t	motor_machine_spd_estimate_cnt = 0;
	static	uint16_t	uw_motor_last_machine_pos_p;
	
	float		motor_machine_spd = 0,	motor_machine_spd_sum = 0;
	static	int16_t	sw_motor_machine_spd_avr_p;
	
	/*速度更新周期*/
	static	uint32_t  ul_get_motor_machine_spd_last_micros;   
	float		f_get_motor_machine_spd_period;         
	
	f_get_motor_machine_spd_period = Get_Function_Period(&ul_get_motor_machine_spd_last_micros); 
	

	if( (uw_motor_now_machine_pos_p - uw_motor_last_machine_pos_p) > 2048 )
	{
		motor_machine_spd = ( (float)(4096 - (uw_motor_now_machine_pos_p - uw_motor_last_machine_pos_p) )/(4096*f_get_motor_machine_spd_period)*360*10 );  
	}
	else if( (uw_motor_now_machine_pos_p - uw_motor_last_machine_pos_p) < -2048 )
	{
		motor_machine_spd = ( (float)(-4096 - (uw_motor_now_machine_pos_p - uw_motor_last_machine_pos_p) )/(4096*f_get_motor_machine_spd_period)*360*10 ); 
	}
	else
	{
		motor_machine_spd = ( (float)(uw_motor_now_machine_pos_p - uw_motor_last_machine_pos_p)/(4096*f_get_motor_machine_spd_period)*360*10 );
	}
	uw_motor_last_machine_pos_p = uw_motor_now_machine_pos_p;
	
	
	/*往缓冲数组中循环写入速度数据*/
	motor_machine_spd_flt_buf[motor_machine_spd_estimate_cnt] = motor_machine_spd;
	motor_machine_spd_estimate_cnt++;
	
	if(motor_machine_spd_estimate_cnt >= MOTOR_MACHINE_SPD_FLT_BUF_LENGTH)  
	{
		motor_machine_spd_estimate_cnt = 0;
	}
	
	/*计算缓冲数组中速度的均值*/
	for(uint8_t i=0;i<MOTOR_MACHINE_SPD_FLT_BUF_LENGTH;i++) 
	{
		motor_machine_spd_sum +=  motor_machine_spd_flt_buf[i];
	}
	sw_motor_machine_spd_avr_p = motor_machine_spd_sum/MOTOR_MACHINE_SPD_FLT_BUF_LENGTH; 
 	motor_machine_spd_sum = 0;
	
	return(sw_motor_machine_spd_avr_p);
}


/**
	* @brief：void	Motor_Machine_Spd_Pid_Update(int16_t	uw_motor_now_machine_pos_p)
	* @note:	更新电机机械速度环PID
	*/
void	Motor_Machine_Spd_Pid_Update(int16_t	sc_motor_machine_spd_p)
{
	sc_motor_machine_spd_pid_fdb = sc_motor_machine_spd_p;
	sc_motor_machine_spd_pid_out = PI_Regulator(sc_motor_machine_spd_pid_ref,sc_motor_machine_spd_pid_fdb,&st_motor_machine_spd_pid_struct);
}
