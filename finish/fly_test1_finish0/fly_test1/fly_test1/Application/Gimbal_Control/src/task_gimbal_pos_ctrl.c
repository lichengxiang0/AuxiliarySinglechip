/**
  ******************************************************************************
  * File Name          : task_gimbal_pos_ctrl.h
  * Description        : 这个文件主要实现1->云台位置环控制 2->随动电机判断
													3->启动柔顺
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
#include "task_gimbal_pos_ctrl.h"

#include "task_get_attitude.h"
#include "task_gimbal_ui.h"
#include "task_common_functions.h"

#include "gimbal_ahrs.h"
#include "gimbal_coordinate_transform.h"
#include "gimbal_spd_ctrl.h"
#include "gimbal_attitude_ctrl.h"
#include "gimbal_follow_ctrl.h"
#include "gimbal_frame_angle.h"
#include "gimbal_track_ctrl.h"
#include "motor_pos_ctrl.h"

#include "stm32_bsp_timer.h"

#include <math.h>

/* Private defines ------------------------------------------------------------*/
#define		GIMABL_POS_PID_LOOP_PERIOD_MS					((uint8_t)10)

/* Private variables ------------------------------------------------------------*/
float	f_earth_pos_pid_out[3];

/* External variables ------------------------------------------------------------*/
volatile	FlagStatus	en_task_gimbal_pos_pid_250Hz_loop_flag = RESET;

Follow_Motor_Typedef		en_yaw_follow_motor = FOLLOW_MOTOR_Y;
Follow_Motor_Typedef		en_pitch_follow_motor = FOLLOW_MOTOR_X;

float										f_yaw_follow_spd_ratio = 5,	f_pitch_follow_spd_ratio = 5;

FlagStatus							en_tripus_mode_flag = RESET;
FlagStatus							en_pitch_pos_pid_enable_flag = SET; 		//test

/* Private functions ------------------------------------------------------------*/
/**					test
	* @brief：void	Judge_Tripus_Mode(float	*motor_angle)
	* @note： 锁定模式时判断是否处于三脚架模式
	*	@param：float	*motor_angle：框架角
	*/
void	Judge_Tripus_Mode(float	*motor_angle)
{
	static	User_Timer_Typedef		st_judge_tripus_mode_timer_p = 	USER_TIMER_INIT_VALUE;
	static	float									f_start_yaw_motor_angle = 0,	f_now_yaw_motor_angle = 0;
	static	uint16_t							uw_tirpus_mode_cnt;
	
	Start_User_Timer(&st_judge_tripus_mode_timer_p);
	Update_User_Timer_Cnt(&st_judge_tripus_mode_timer_p);
	
	if(st_judge_tripus_mode_timer_p.ul_timer_cnt < 20)
	{
		f_start_yaw_motor_angle = motor_angle[en_yaw_follow_motor]*57.3;
	}
	else if(st_judge_tripus_mode_timer_p.ul_timer_cnt > 120)
	{
		f_now_yaw_motor_angle = motor_angle[en_yaw_follow_motor]*57.3;
		
		/*100ms内电机角度误差小于0.5则认为是三角脚模式(手柄静止不动)*/
		if(fabs(f_now_yaw_motor_angle - f_start_yaw_motor_angle) < 0.5)
		{
			uw_tirpus_mode_cnt++;
			if(uw_tirpus_mode_cnt > 250)
			{
				uw_tirpus_mode_cnt = 250;
			}
		}
		else
		{
			uw_tirpus_mode_cnt = 0;
		}
		
		Reset_User_Timer(&st_judge_tripus_mode_timer_p);
	}
	
	if(uw_tirpus_mode_cnt > 2)   //10S
	{
		en_tripus_mode_flag = SET;
	}
	else
	{
		en_tripus_mode_flag = RESET;
	}
}


/**
	* @brief：void	Cal_Pitch_Follow_Motor(float ** coordinate_transform_matrix_p)
	* @note： 计算PITCH弱随动的电机
	*	@param：float ** coordinate_transform_matrix_p：坐标旋转矩阵
	*/
void	Cal_Pitch_Follow_Motor(float ** coordinate_transform_matrix_p)
{
//	float	earth_x_to_motor_x,	earth_x_to_motor_y,	earth_x_to_motor_z;
//	
//	earth_x_to_motor_x = coordinate_transform_matrix_p[3][0];
//	earth_x_to_motor_y = coordinate_transform_matrix_p[3][3];	
//	earth_x_to_motor_z = coordinate_transform_matrix_p[3][6];
	
	if(X_RIGHT == en_gimbal_coordinate)
	{
		en_pitch_follow_motor = FOLLOW_MOTOR_X;
	}
	else
	{
		en_pitch_follow_motor = FOLLOW_MOTOR_Z;
	}	
}


/**
	* @brief：void	Cal_Yaw_Follow_Motor(float ** coordinate_transform_matrix_p)
	* @note： 计算方位弱随动的电机
	*	@param：float ** coordinate_transform_matrix_p：坐标旋转矩阵
	*/
void	Cal_Yaw_Follow_Motor(float ** coordinate_transform_matrix_p)
{
	float	earth_z_to_motor_x,	earth_z_to_motor_y,	earth_z_to_motor_z;
	
	static	Coordinate_Typedef en_gimbal_last_coordinate;
	
	earth_z_to_motor_x = coordinate_transform_matrix_p[3][2];
	earth_z_to_motor_y = coordinate_transform_matrix_p[3][5];	
	earth_z_to_motor_z = coordinate_transform_matrix_p[3][8];
	
	if( (X_RIGHT == en_gimbal_coordinate)&&(X_FRONT == en_gimbal_last_coordinate) )
	{
		en_yaw_follow_motor = FOLLOW_MOTOR_Z;
	}
	
	if(X_RIGHT == en_gimbal_coordinate)
	{	
		if( (earth_z_to_motor_y < -0.707)||(earth_z_to_motor_y > 0.707) )
		{
			en_yaw_follow_motor = FOLLOW_MOTOR_Y;
		}
		else if( (earth_z_to_motor_y > -0.5)&&(earth_z_to_motor_y < 0.5) )
		{
			en_yaw_follow_motor = FOLLOW_MOTOR_Z;
		}
	}
	
	if(X_FRONT == en_gimbal_coordinate)
	{
		en_yaw_follow_motor = FOLLOW_MOTOR_X;
	}
	
	en_gimbal_last_coordinate = en_gimbal_coordinate;
}


/**
	* @brief：void	Cal_Follow_Pos_Pid_Para(void)
	* @note： 处理随动位置环PID参数
	*/
void	Cal_Follow_Pos_Pid_Para(void)
{
		uint16_t	uw_pitch_follow_pos_pid_error,		uw_yaw_follow_pos_pid_err;
	/*根据误差调整Z_Follow_Pos_Pid的Kp值
		原理如下
		Kp范围：50-250   误差范围：0-60*80->0-4800
		|Kp
		|    
		| 
		|_ _ _ _ _ _err
		求Kp和Err之间的线性关系式
	*/
	
	/*pitch_follow随动速度*/
	uw_pitch_follow_pos_pid_error = fabs(earth_x_follow_pos_pid_ref - earth_x_follow_pos_pid_fdb);
	
	if(uw_pitch_follow_pos_pid_error > 4000)
	{
		uw_pitch_follow_pos_pid_error = 4000;   		//60*80  
	}
	/*f_pitch_follow_spd_ratio=[5-10]  [200-400]*/
	st_earth_x_follow_pos_pid_struct.hKp_Gain = 50 + (uw_pitch_follow_pos_pid_error/100.f)*f_pitch_follow_spd_ratio; //test
	
	
	/*yaw_follow随动速度*/
	uw_yaw_follow_pos_pid_err = fabs(earth_z_follow_pos_pid_ref - earth_z_follow_pos_pid_fdb);
	
	if(uw_yaw_follow_pos_pid_err > 4000)
	{
		uw_yaw_follow_pos_pid_err = 4000;   				//60*80  
	}
	/*f_yaw_follow_spd_ratio=[5-10]  [200-400]*/
	st_earth_z_follow_pos_pid_struct.hKp_Gain = 50 + (uw_yaw_follow_pos_pid_err/100.f)*f_yaw_follow_spd_ratio; //test
//	st_earth_z_follow_pos_pid_struct.hKp_Gain = 500;
}

/**
	* @brief：void	Process_Gimbal_Pos_Ctrl(void)
	* @note： 处理不同工作模式下，各位置环的更新和输出
	*/
void	Process_Gimbal_Pos_Ctrl(void)
{
	static	User_Timer_Typedef	z_follow_ok_wait_timer = USER_TIMER_INIT_VALUE;
	
	switch(en_gimbal_work_mode)
	{
		case NONE_FOLLOW:	
			//Judge_Tripus_Mode(motor_frame_angle_rad);				//判断是否是三脚架模式test
		
			if(RESET == en_tripus_mode_flag)
			{
				//Earth_X_Pos_Pid_Updata(f_euler_array[0]);
				Earth_Y_Pos_Pid_Updata(f_euler_array[1]);			
				//Earth_Z_Pos_Pid_Updata(f_euler_array[2]);
			
				//f_earth_pos_pid_out[0] = earth_x_pos_pid_out;
				f_earth_pos_pid_out[1] = earth_y_pos_pid_out;
				//f_earth_pos_pid_out[2] = earth_z_pos_pid_out;		
			}
			else
			{
				Yaw_Follow_Pos_Pid_Updata(f_euler_array,	motor_frame_angle_rad,	en_yaw_follow_motor,	coordinate_transform_matrix);		
			
				Earth_X_Pos_Pid_Updata(f_euler_array[0]);
				//Earth_Y_Pos_Pid_Updata(f_euler_array[1]);			
			
				f_earth_pos_pid_out[0] = earth_x_pos_pid_out;
				//f_earth_pos_pid_out[1] = earth_y_pos_pid_out;
				//f_earth_pos_pid_out[2] = earth_z_follow_pos_pid_out;	
			}
			break;
		
		case Z_FOLLOW:
			//Yaw弱随动PID更新
			Yaw_Follow_Pos_Pid_Updata(f_euler_array,	motor_frame_angle_rad,	en_yaw_follow_motor,	coordinate_transform_matrix);		
			
			Earth_X_Pos_Pid_Updata(f_euler_array[0]);
			Earth_Y_Pos_Pid_Updata(f_euler_array[1]);			
		
			f_earth_pos_pid_out[0] = earth_x_pos_pid_out;
			f_earth_pos_pid_out[1] = earth_y_pos_pid_out;
			f_earth_pos_pid_out[2] = earth_z_follow_pos_pid_out;	
			break;
		
		case Z_FOLLOW_TO_NONE_FOLLOW:
			Yaw_Follow_Pos_Pid_Updata(f_euler_array,	motor_frame_angle_rad,	en_yaw_follow_motor,	coordinate_transform_matrix);		
			
			Earth_X_Pos_Pid_Updata(f_euler_array[0]);
			Earth_Y_Pos_Pid_Updata(f_euler_array[1]);			
		
			f_earth_pos_pid_out[0] = earth_x_pos_pid_out;
			f_earth_pos_pid_out[1] = earth_y_pos_pid_out;
			f_earth_pos_pid_out[2] = earth_z_follow_pos_pid_out;	
			
			Start_User_Timer(&z_follow_ok_wait_timer);
			Update_User_Timer_Cnt(&z_follow_ok_wait_timer);
		
			if( (fabs(earth_z_follow_pos_pid_ref - earth_z_follow_pos_pid_fdb)<10*80)||
					(z_follow_ok_wait_timer.ul_timer_cnt > 5000) )  									//5ms
			{
				en_gimbal_work_mode = NONE_FOLLOW;
				Stop_User_Timer(&z_follow_ok_wait_timer);
			}
			break;
		
		case X_Z_FOLLOW:
			//Yaw_Follow_Pos_Pid_Updata(f_euler_array,	motor_frame_angle_rad,	en_yaw_follow_motor,	coordinate_transform_matrix);		
			
			Pitch_Follow_Pos_Pid_Updata(f_euler_array,	motor_frame_angle_rad,	en_pitch_follow_motor,	coordinate_transform_matrix);	
		
			//Earth_Y_Pos_Pid_Updata(f_euler_array[1]);			
			
			f_earth_pos_pid_out[0] = earth_x_follow_pos_pid_out;
			//f_earth_pos_pid_out[1] = earth_y_pos_pid_out;
			//f_earth_pos_pid_out[2] = earth_z_follow_pos_pid_out;
			break;
		
		case X_Z_FOLLOW_TO_NONE_FOLLOW:
			Yaw_Follow_Pos_Pid_Updata(f_euler_array,	motor_frame_angle_rad,	en_yaw_follow_motor,	coordinate_transform_matrix);		
			
			Pitch_Follow_Pos_Pid_Updata(f_euler_array,	motor_frame_angle_rad,	en_pitch_follow_motor,	coordinate_transform_matrix);	
		
			Earth_Y_Pos_Pid_Updata(f_euler_array[1]);			
			
			f_earth_pos_pid_out[0] = earth_x_follow_pos_pid_out;
			f_earth_pos_pid_out[1] = earth_y_pos_pid_out;
			f_earth_pos_pid_out[2] = earth_z_follow_pos_pid_out;
			
			Start_User_Timer(&z_follow_ok_wait_timer);
			Update_User_Timer_Cnt(&z_follow_ok_wait_timer);
		
			if( (fabs(earth_z_follow_pos_pid_ref - earth_z_follow_pos_pid_fdb)<10*80)||
					(z_follow_ok_wait_timer.ul_timer_cnt > 5000) )
			{
				en_gimbal_work_mode = NONE_FOLLOW;
				Stop_User_Timer(&z_follow_ok_wait_timer);
			}
			break;
		
		case X_Z_FOLLOW_TO_Z_FOLLOW:
			Yaw_Follow_Pos_Pid_Updata(f_euler_array,	motor_frame_angle_rad,	en_yaw_follow_motor,	coordinate_transform_matrix);		
			
			Pitch_Follow_Pos_Pid_Updata(f_euler_array,	motor_frame_angle_rad,	en_pitch_follow_motor,	coordinate_transform_matrix);	
		
			Earth_Y_Pos_Pid_Updata(f_euler_array[1]);			
			
			f_earth_pos_pid_out[0] = earth_x_follow_pos_pid_out;
			f_earth_pos_pid_out[1] = earth_y_pos_pid_out;
			f_earth_pos_pid_out[2] = earth_z_follow_pos_pid_out;

			en_gimbal_work_mode = Z_FOLLOW;
			break;
		
		case TRACKER:				
			Phone_X_Track_Pid_Updata();
			Phone_Y_Track_Pid_Updata();	
		
			Earth_Y_Pos_Pid_Updata(f_euler_array[1]);			
			
			if(HORIZONTAL == en_camera_viewer_mode)
			{
				f_earth_pos_pid_out[0] = sw_phone_y_track_pid_out;  //pitch
				f_earth_pos_pid_out[1] = earth_y_pos_pid_out;				//roll
				f_earth_pos_pid_out[2] = sw_phone_x_track_pid_out;  //yaw
			}
			else if(VERTICAL == en_camera_viewer_mode)
			{
				f_earth_pos_pid_out[0] = sw_phone_x_track_pid_out;  //pitch
				f_earth_pos_pid_out[1] = earth_y_pos_pid_out;				//roll
				f_earth_pos_pid_out[2] = sw_phone_y_track_pid_out;  //yaw
			}		
				
			break;
			
		case LOCK:        												//位置锁定模式
			Motor_X_Pos_Pid_Updata(motor_frame_angle_rad);        //X轴
			//Motor_Y_Pos_Pid_Updata(motor_frame_angle_rad);
			//Motor_Z_Pos_Pid_Updata(motor_frame_angle_rad);
			break;
		
		default:
			break;
	}
	
	
}


/* External functions ------------------------------------------------------------*/
/**
	* @brief：void	Task_Gimbal_Pos_Pid_250Hz_Loop(void)
	* @note： 位置环循环
	*/
void	Task_Gimbal_Pos_Pid_Loop(void)
{
	float	f_motor_spd_pid_ref_p[3];
	
	uint16_t	uw_pos_pid_limit_inc = 10;
	
	static	User_Timer_Typedef	st_this_function_period_timer_p = USER_TIMER_INIT_VALUE;
	Start_User_Timer(&st_this_function_period_timer_p);
	Update_User_Timer_Cnt(&st_this_function_period_timer_p);
	
	if(st_this_function_period_timer_p.ul_timer_cnt > GIMABL_POS_PID_LOOP_PERIOD_MS)   //10ms更新周期
	{	
		Reset_User_Timer(&st_this_function_period_timer_p);
		
		/*计算随动轴*/
		//Cal_Yaw_Follow_Motor(coordinate_transform_matrix);			//得到YAW轴的跟随电机
		
		//Cal_Pitch_Follow_Motor(coordinate_transform_matrix);		//根据坐标得到PICTH跟随电机 
		
		//Cal_Follow_Pos_Pid_Para();  															//计算随动位置环PID的KP
	
		switch(en_gimbal_power_mode)
		{
			case NORMAL:
				st_earth_x_pos_pid_struct.hLower_Limit_Output = -300;			//-IQMAX;   //Lower Limit for Output limitation
				st_earth_x_pos_pid_struct.hUpper_Limit_Output = 300;				//IQMAX;   	//Upper Limit for Output limitation
				
				st_earth_y_pos_pid_struct.hLower_Limit_Output = -300;			//-IQMAX;   //Lower Limit for Output limitation
				st_earth_y_pos_pid_struct.hUpper_Limit_Output = 300;				//IQMAX;   	//Upper Limit for Output limitation
				
				st_earth_z_pos_pid_struct.hLower_Limit_Output = -300;			//-IQMAX;   //Lower Limit for Output limitation
				st_earth_z_pos_pid_struct.hUpper_Limit_Output = 300;				//IQMAX;   	//Upper Limit for Output limitation
				
				st_earth_x_follow_pos_pid_struct.hLower_Limit_Output= -300;	//-IQMAX;   //Lower Limit for Output limitation
				st_earth_x_follow_pos_pid_struct.hUpper_Limit_Output= 300;	//IQMAX;   	//Upper Limit for Output limitation
				
				st_earth_x_follow_pos_pid_struct.hLower_Limit_Output= -300;	//-IQMAX;   //Lower Limit for Output limitation
				st_earth_x_follow_pos_pid_struct.hUpper_Limit_Output= 300;	//IQMAX;   	//Upper Limit for Output limitation			
				
				f_earth_pos_pid_out[0] = 0;
				f_earth_pos_pid_out[1] = 0;
				f_earth_pos_pid_out[2] = 0;
				break;
			
			case SLEEP_TO_NORMAL:
				st_earth_x_pos_pid_struct.hLower_Limit_Output -= uw_pos_pid_limit_inc;//-IQMAX;   //Lower Limit for Output limitation
				st_earth_x_pos_pid_struct.hUpper_Limit_Output += uw_pos_pid_limit_inc;//IQMAX;   //Upper Limit for Output limitation
			
				st_earth_y_pos_pid_struct.hLower_Limit_Output -= uw_pos_pid_limit_inc;//-IQMAX;   //Lower Limit for Output limitation
				st_earth_y_pos_pid_struct.hUpper_Limit_Output += uw_pos_pid_limit_inc;//IQMAX;   //Upper Limit for Output limitation
				
				st_earth_z_pos_pid_struct.hLower_Limit_Output -= uw_pos_pid_limit_inc;//-IQMAX;   //Lower Limit for Output limitation
				st_earth_z_pos_pid_struct.hUpper_Limit_Output += uw_pos_pid_limit_inc;//IQMAX;   //Upper Limit for Output limitation
				
				st_earth_x_follow_pos_pid_struct.hLower_Limit_Output -= uw_pos_pid_limit_inc;//-IQMAX;   //Lower Limit for Output limitation
				st_earth_x_follow_pos_pid_struct.hUpper_Limit_Output += uw_pos_pid_limit_inc;//IQMAX;   //Upper Limit for Output limitation
				
				st_earth_z_follow_pos_pid_struct.hLower_Limit_Output -= uw_pos_pid_limit_inc;//-IQMAX;   //Lower Limit for Output limitation
				st_earth_z_follow_pos_pid_struct.hUpper_Limit_Output += uw_pos_pid_limit_inc;//IQMAX;   //Upper Limit for Output limitation
				
				Bond_Int16_t(&(st_earth_x_pos_pid_struct.hUpper_Limit_Output),-2000,2000);
				Bond_Int16_t(&(st_earth_x_pos_pid_struct.hLower_Limit_Output),-2000,2000);
				
				Bond_Int16_t(&(st_earth_y_pos_pid_struct.hUpper_Limit_Output),-1200,1200);
				Bond_Int16_t(&(st_earth_y_pos_pid_struct.hLower_Limit_Output),-1200,1200);
				
				Bond_Int16_t(&(st_earth_z_pos_pid_struct.hUpper_Limit_Output),-1200,1200);
				Bond_Int16_t(&(st_earth_z_pos_pid_struct.hLower_Limit_Output),-1200,1200);
				
				Bond_Int16_t(&(st_earth_x_follow_pos_pid_struct.hUpper_Limit_Output),-1500,1500);
				Bond_Int16_t(&(st_earth_x_follow_pos_pid_struct.hLower_Limit_Output),-1500,1500);
				
				Bond_Int16_t(&(st_earth_z_follow_pos_pid_struct.hUpper_Limit_Output),-1500,1500);
				Bond_Int16_t(&(st_earth_z_follow_pos_pid_struct.hLower_Limit_Output),-1500,1500);	
				
				/*更新PID*/
				Process_Gimbal_Pos_Ctrl();
				
				/*判断启动是否完成*/
				if( fabs(earth_y_pos_pid_ref - earth_y_pos_pid_fdb) < 10*80 )
				{
					en_gimbal_power_mode = NORMAL;
					
					st_earth_x_pos_pid_struct.hLower_Limit_Output= -2000;		//-IQMAX;   //Lower Limit for Output limitation
					st_earth_x_pos_pid_struct.hUpper_Limit_Output= 2000;		//IQMAX;   //Upper Limit for Output limitation
					
					st_earth_y_pos_pid_struct.hLower_Limit_Output= -1200;//-IQMAX;   //Lower Limit for Output limitation
					st_earth_y_pos_pid_struct.hUpper_Limit_Output= 1200;//IQMAX;   //Upper Limit for Output limitation
					
					st_earth_z_pos_pid_struct.hLower_Limit_Output= -1200;//-IQMAX;   //Lower Limit for Output limitation
					st_earth_z_pos_pid_struct.hUpper_Limit_Output= 1200;//IQMAX;   //Upper Limit for Output limitation
					
					st_earth_x_follow_pos_pid_struct.hLower_Limit_Output= -1500;//-IQMAX;   //Lower Limit for Output limitation
					st_earth_x_follow_pos_pid_struct.hUpper_Limit_Output= 1500;//IQMAX;   //Upper Limit for Output limitation
					
					st_earth_z_follow_pos_pid_struct.hLower_Limit_Output= -1500;//-IQMAX;   //Lower Limit for Output limitation
					st_earth_z_follow_pos_pid_struct.hUpper_Limit_Output= 1500;//IQMAX;   //Upper Limit for Output limitation
				}			
				break;
			
			case SLEEP:
				Process_Gimbal_Pos_Ctrl();          												//位置环实时更新10ms
				break;
			
			case NORMAL_TO_SLEEP:		
				f_earth_pos_pid_out[0] = 0;
				f_earth_pos_pid_out[1] = 0;
				f_earth_pos_pid_out[2] = 0;
				break;
		}
			
		if(RESET == en_pitch_pos_pid_enable_flag)
		{
			f_earth_pos_pid_out[0] = 0;
		}
		
		if( ( NORMAL == en_gimbal_power_mode)&&( LOCK == en_gimbal_work_mode) )						//NORMAL  LOCK
		{
			motor_x_spd_pid_ref = motor_x_pos_pid_out;
			motor_y_spd_pid_ref = motor_y_pos_pid_out;
			motor_z_spd_pid_ref = motor_z_pos_pid_out;
		}
		else
		{
			;
//			Earth_Spd_To_Motor_Spd(f_earth_pos_pid_out,f_motor_spd_pid_ref_p,coordinate_transform_matrix);
//			
//			motor_x_spd_pid_ref = f_motor_spd_pid_ref_p[0];
//			motor_y_spd_pid_ref = f_motor_spd_pid_ref_p[1];
//			motor_z_spd_pid_ref = f_motor_spd_pid_ref_p[2];
		}
	}
}






