/**
  ******************************************************************************
  * File Name          : task_gimbal_spd_ctrl.c
  * Description        : ����ļ���Ҫʵ��1->��̨�ٶȻ�����(����) 2->����غ��ж�
												������ 3->��������ʵ��
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
/*�˳��غ�ģʽʱ����motor_y���ٶȺ����ؽ���������*/
int16_t	motor_y_spd_ref_limit,						motor_y_spd_ref_limit_inc = 3;
int16_t	motor_y_torque_ref_limit,					motor_y_torque_ref_limit_inc = 5;

int16_t motor_x_torque_ref_limit,					motor_x_torque_ref_limit_inc = 5;

int16_t	motor_torque_ref_limit = 400,			motor_torque_ref_limit_inc = 10;

short motor_x_spd_pid_torque_out,motor_y_spd_pid_torque_out,motor_z_spd_pid_torque_out;

/* External variables ------------------------------------------------------------*/
/*motor_x��motor_y�غϱ�־*/
FlagStatus	en_motor_y_and_motor_x_have_same_dir_flag = RESET;

/* Private functions ------------------------------------------------------------*/
/**
	* @brief��void	Porcess_X_Y_Coincide(void)
	* @note�� motor_x��motor_y�غ�ʱ�����غϱ�־������غ��˳�ʱ���⴦��
	*/
void	Porcess_X_Y_Coincide(void)
{
	//�ж�X���Y���Ƿ��غ�
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
	
	/*----------�˳��غ�ģʽ��motor_y�ٶȻ��ο������ػ�����������ָ������ⶶ��--------*/
	/*motor_y����ٶȻ��ο�����*/
	motor_y_spd_ref_limit += motor_y_spd_ref_limit_inc;
	Bond_Int16_t(&motor_y_spd_ref_limit,-1200,1200);

	/*motor_y������ػ��ο�����*/
	motor_y_torque_ref_limit += motor_y_torque_ref_limit_inc;
	Bond_Int16_t(&motor_y_torque_ref_limit,-900,900);
}



/* External functions ------------------------------------------------------------*/
/**
	* @brief��void	Task_Gimbal_Spd_Pid_1000HZ_Loop(void)
	* @note�� ����ٶȻ�����(�����ٶȻ�)
	*/

//float		f_update_period;//test

void	Task_Gimbal_Spd_Pid_Loop(void)
{	
	static	User_Timer_Typedef	st_this_function_period_timer_p = USER_TIMER_INIT_VALUE;
	static	User_Timer_Typedef	st_torque_ref_dec_period_timer_p = USER_TIMER_INIT_VALUE;
	
	/*����ahrs_update�ĸ�������*/
//	static	uint32_t		ul_update_sys_micros;											//test	
//	f_update_period = Get_Function_Period(&ul_update_sys_micros);			//test	

	Start_User_Timer(&st_this_function_period_timer_p);
	Update_User_Timer_Cnt(&st_this_function_period_timer_p);
	
	if(st_this_function_period_timer_p.ul_timer_cnt > GIMBAL_SPD_PID_LOOP_PERIOD_MS)    //ִ������1ms
	{	
//		Reset_User_Timer(&st_this_function_period_timer_p);
	
		/*����motor_x��motor_y�غ�����*/
		//Porcess_X_Y_Coincide();

		/*����motor_y�ٶ�*/
		//Bond_Int16_t(&motor_y_spd_pid_ref,-motor_y_spd_ref_limit,motor_y_spd_ref_limit);
		
		/*motor_spd_pid_updata*/
		//Motor_Spd_Pid_Updata(f_sensor_gyro_raw_data_array,coordinate_transform_matrix);
		
		Motor_Spd_Pid_Updata(f_sensor_gyro_flt_data_array,coordinate_transform_matrix);
		
		/*����motor_y����*/
		//Bond_Int16_t(&motor_y_spd_pid_out, -motor_y_torque_ref_limit, motor_y_torque_ref_limit);
		/*�Ի�����SLEEP�Ĵ���*/
		switch(en_gimbal_power_mode)
		{
			case SLEEP:							//SLEEP:		
					/*��λ������ʼ״̬*/
					Stop_User_Timer(&st_torque_ref_dec_period_timer_p);
					motor_torque_ref_limit = 400;
			
//					motor_x_spd_pid_out = 0;
//					motor_y_spd_pid_out = 0;
//					motor_z_spd_pid_out = 0;		
				break;
			
			case SLEEP_TO_NORMAL:
				
			case NORMAL:
				/*��λ������ʼ״̬*/
				Stop_User_Timer(&st_torque_ref_dec_period_timer_p);
				motor_torque_ref_limit = 400;
				break;
			
			case	NORMAL_TO_SLEEP:				
				/*--------------NORMAL_TO_SLEEPģʽ�µĻ�����-----------*/	
				Start_User_Timer(&st_torque_ref_dec_period_timer_p);
				Update_User_Timer_Cnt(&st_torque_ref_dec_period_timer_p);
				if(st_torque_ref_dec_period_timer_p.ul_timer_cnt > 100) 												//100msִ��һ��
				{	
					Reset_User_Timer(&st_torque_ref_dec_period_timer_p);
					motor_torque_ref_limit -= motor_torque_ref_limit_inc;
				}
				
				/*normal_to_sleep������ע��motor_torque_ref_limit�ĳ�ʼ���������һ�λ�����normal_to_Sleep����*/
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
		/*�������ط���*/
		motor_x_spd_pid_torque_out	= motor_x_spd_pid_out;
		
		motor_y_spd_pid_torque_out	= motor_y_spd_pid_out;
		
		motor_z_spd_pid_torque_out	= motor_z_spd_pid_out;
		
		/*�������������ָ��*/
		Board_Gimbal_Send_Torque_Ref();
	}
}
	








