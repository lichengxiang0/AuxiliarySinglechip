/**
  ******************************************************************************
  * File Name          : task_gimbal_spd_ctrl.c
  * Description        : 这个文件主要实现1->云台速度环控制(陀螺) 2->电机重合判断
												及处理 3->缓降功能实现
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
#include "task_gimbal_spd_ctrl.h"

#include "task_gimbal_ui.h"
#include "task_common_functions.h"

#include "gimbal_ahrs.h"
#include "gimbal_coordinate_transform.h"
#include "gimbal_spd_ctrl.h"
#include "gimbal_frame_angle.h"

#include "board_gimbal_cmd_process.h"

#include "stm32_bsp_timer.h"

/* Private defines ------------------------------------------------------------*/
#define	DEG_TO_RADIAN		((float)0.01744)
#define	RADIAN_TO_DEG		((float)57.3)
	
#define	GIMBAL_SPD_PID_LOOP_PERIOD_MS					((uint8_t)1)
	
/* Private variables ------------------------------------------------------------*/
/*退出重合模式时，对motor_y的速度和力矩进行限制用*/
int16_t	motor_y_spd_ref_limit,						motor_y_spd_ref_limit_inc = 3;
int16_t	motor_y_torque_ref_limit,					motor_y_torque_ref_limit_inc = 5;

int16_t motor_x_torque_ref_limit,					motor_x_torque_ref_limit_inc = 5;

int16_t	motor_torque_ref_limit = 400,			motor_torque_ref_limit_inc = 10;

short motor_x_spd_pid_torque_out,motor_y_spd_pid_torque_out,motor_z_spd_pid_torque_out;

/* External variables ------------------------------------------------------------*/
/*motor_x和motor_y重合标志*/
FlagStatus	en_motor_y_and_motor_x_have_same_dir_flag = RESET;

/* Private functions ------------------------------------------------------------*/
/**
	* @brief：void	Porcess_X_Y_Coincide(void)
	* @note： motor_x和motor_y重合时处理，重合标志处理和重合退出时特殊处理
	*/
void	Porcess_X_Y_Coincide(void)
{
	//判断X轴和Y轴是否重合
	if(  ( (motor_frame_angle_rad[2]*RADIAN_TO_DEG > -110)&&(motor_frame_angle_rad[2]*RADIAN_TO_DEG < -70) )||( (motor_frame_angle_rad[2]*RADIAN_TO_DEG > 70)&&(motor_frame_angle_rad[2]*RADIAN_TO_DEG < 110) )  ) 
	{
		if(en_motor_y_and_motor_x_have_same_dir_flag != SET)
		{
			en_motor_y_and_motor_x_have_same_dir_flag = SET;
		}
	}
	else
	{
		if(en_motor_y_and_motor_x_have_same_dir_flag != RESET)
		{
			en_motor_y_and_motor_x_have_same_dir_flag = RESET;
			
			motor_y_torque_ref_limit = 0;
			motor_y_spd_ref_limit = 0;
		}
	}	
	
	/*----------退出重合模式后，motor_y速度环参考和力矩环的输出渐渐恢复，避免抖动--------*/
	/*motor_y电机速度环参考递增*/
	motor_y_spd_ref_limit += motor_y_spd_ref_limit_inc;
	Bond_Int16_t(&motor_y_spd_ref_limit,-1200,1200);

	/*motor_y电机力矩环参考递增*/
	motor_y_torque_ref_limit += motor_y_torque_ref_limit_inc;
	Bond_Int16_t(&motor_y_torque_ref_limit,-900,900);
}



/* External functions ------------------------------------------------------------*/
/**
	* @brief：void	Task_Gimbal_Spd_Pid_1000HZ_Loop(void)
	* @note： 电机速度环任务(陀螺速度环)
	*/

//float		f_update_period;//test

void	Task_Gimbal_Spd_Pid_Loop(void)
{	
	static	User_Timer_Typedef	st_this_function_period_timer_p = USER_TIMER_INIT_VALUE;
	static	User_Timer_Typedef	st_torque_ref_dec_period_timer_p = USER_TIMER_INIT_VALUE;
	
	/*计算ahrs_update的更新周期*/
//	static	uint32_t		ul_update_sys_micros;											//test	
//	f_update_period = Get_Function_Period(&ul_update_sys_micros);			//test	

	Start_User_Timer(&st_this_function_period_timer_p);
	Update_User_Timer_Cnt(&st_this_function_period_timer_p);
	
	if(st_this_function_period_timer_p.ul_timer_cnt > GIMBAL_SPD_PID_LOOP_PERIOD_MS)    //执行周期1ms
	{	
//		Reset_User_Timer(&st_this_function_period_timer_p);
	
		/*处理motor_x和motor_y重合问题*/
		//Porcess_X_Y_Coincide();

		/*限制motor_y速度*/
		//Bond_Int16_t(&motor_y_spd_pid_ref,-motor_y_spd_ref_limit,motor_y_spd_ref_limit);
		
		/*motor_spd_pid_updata*/
		//Motor_Spd_Pid_Updata(f_sensor_gyro_raw_data_array,coordinate_transform_matrix);
		
		Motor_Spd_Pid_Updata(f_sensor_gyro_flt_data_array,coordinate_transform_matrix);
		
		/*限制motor_y力矩*/
		//Bond_Int16_t(&motor_y_spd_pid_out, -motor_y_torque_ref_limit, motor_y_torque_ref_limit);
		/*对缓降和SLEEP的处理*/
		switch(en_gimbal_power_mode)
		{
			case SLEEP:							//SLEEP:		
					/*复位缓降初始状态*/
					Stop_User_Timer(&st_torque_ref_dec_period_timer_p);
					motor_torque_ref_limit = 400;
			
//					motor_x_spd_pid_out = 0;
//					motor_y_spd_pid_out = 0;
//					motor_z_spd_pid_out = 0;		
				break;
			
			case SLEEP_TO_NORMAL:
				
			case NORMAL:
				/*复位缓降初始状态*/
				Stop_User_Timer(&st_torque_ref_dec_period_timer_p);
				motor_torque_ref_limit = 400;
				break;
			
			case	NORMAL_TO_SLEEP:				
				/*--------------NORMAL_TO_SLEEP模式下的缓降功-----------*/	
				Start_User_Timer(&st_torque_ref_dec_period_timer_p);
				Update_User_Timer_Cnt(&st_torque_ref_dec_period_timer_p);
				if(st_torque_ref_dec_period_timer_p.ul_timer_cnt > 100) 												//100ms执行一次
				{	
					Reset_User_Timer(&st_torque_ref_dec_period_timer_p);
					motor_torque_ref_limit -= motor_torque_ref_limit_inc;
				}
				
				/*normal_to_sleep结束，注意motor_torque_ref_limit的初始化，否则第一次会跳过normal_to_Sleep过程*/
				if(motor_torque_ref_limit < 0)
				{
					en_gimbal_power_mode = SLEEP;			
				}
				
				Bond_Int16_t(&motor_x_spd_pid_out,-motor_torque_ref_limit,motor_torque_ref_limit);
				Bond_Int16_t(&motor_y_spd_pid_out,-motor_torque_ref_limit,motor_torque_ref_limit);
				Bond_Int16_t(&motor_z_spd_pid_out,-motor_torque_ref_limit,motor_torque_ref_limit);
				break;
				
			default:
				break;				
		}	
		/*配置力矩方向*/
		motor_x_spd_pid_torque_out	= motor_x_spd_pid_out;
		
		motor_y_spd_pid_torque_out	= motor_y_spd_pid_out;
		
		motor_z_spd_pid_torque_out	= motor_z_spd_pid_out;
		
		/*给电机驱动发送指令*/
		Board_Gimbal_Send_Torque_Ref();
	}
}
	








