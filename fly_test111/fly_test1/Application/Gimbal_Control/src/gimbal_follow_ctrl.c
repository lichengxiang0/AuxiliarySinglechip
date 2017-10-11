/**
  ******************************************************************************
  * File Name          :gimbal_follow_ctrl.c
  * Description        :��̨���涯����
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
#include "stm32f1xx_hal.h"
#include "gimbal_follow_ctrl.h"
#include <math.h>

/* Private define ------------------------------------------------------------*/
#define	DEG_TO_RADIAN		((float)0.01744)
#define	RADIAN_TO_DEG		((float)57.3)
	

/* Private variables ------------------------------------------------------------*/
float								earth_x_follow_pos_pid_out_kp = 0.9,			earth_z_follow_pos_pid_out_kp = 0.9;


/* External variables ------------------------------------------------------------*/
int16_t		sw_pitch_follow_pos_pid_ref=0,	sw_yaw_follow_pos_pid_ref=0; //���涯��

PID_Struct_Typedef	st_earth_x_follow_pos_pid_struct,	st_earth_z_follow_pos_pid_struct;
int16_t	earth_x_follow_pos_pid_ref,			earth_z_follow_pos_pid_ref;
int16_t	earth_x_follow_pos_pid_fdb,			earth_z_follow_pos_pid_fdb;
int16_t	earth_x_follow_pos_pid_out,			earth_z_follow_pos_pid_out;



/* Private functions ------------------------------------------------------------*/
/**
	* @brief��void	Cal_Yaw_Follow_Ref(float	*euler_p, 	float	*motor_angle,	Follow_Motor_Typedef en_yaw_follow_motor_p,	float ** coordinate_transform_matrix_p)
	* @note�� �������涯�Ĳο�
	*	@param��
	*/
void	Cal_Yaw_Follow_Ref(float	*euler_p, 	float	*motor_angle,	Follow_Motor_Typedef en_yaw_follow_motor_p,	float ** coordinate_transform_matrix_p)
{
	float		motor_to_earth_matrix_p[9];
	
	static	float		yaw_error_q0,	yaw_error_q1,		yaw_error_q2,		yaw_error_q3;
	float		euler_yaw_ref_inc;
	
	motor_to_earth_matrix_p[0] = coordinate_transform_matrix_p[5][0];
	motor_to_earth_matrix_p[3] = coordinate_transform_matrix_p[5][1];
	motor_to_earth_matrix_p[6] = coordinate_transform_matrix_p[5][2];
	
	motor_to_earth_matrix_p[1] = coordinate_transform_matrix_p[5][3];
	motor_to_earth_matrix_p[4] = coordinate_transform_matrix_p[5][4];
	motor_to_earth_matrix_p[7] = coordinate_transform_matrix_p[5][5];
	
	motor_to_earth_matrix_p[2] = coordinate_transform_matrix_p[5][6];
	motor_to_earth_matrix_p[5] = coordinate_transform_matrix_p[5][7];
	motor_to_earth_matrix_p[8] = coordinate_transform_matrix_p[5][8];
	
	
	if(FOLLOW_MOTOR_X == en_yaw_follow_motor_p) 					//x��
	{
		yaw_error_q0 = cos( (sw_yaw_follow_pos_pid_ref*DEG_TO_RADIAN-motor_angle[0])/2);
		yaw_error_q1 = sin( (sw_yaw_follow_pos_pid_ref*DEG_TO_RADIAN-motor_angle[0])/2)*motor_to_earth_matrix_p[0];
		yaw_error_q2 = sin( (sw_yaw_follow_pos_pid_ref*DEG_TO_RADIAN-motor_angle[0])/2)*motor_to_earth_matrix_p[3];
		yaw_error_q3 = sin( (sw_yaw_follow_pos_pid_ref*DEG_TO_RADIAN-motor_angle[0])/2)*motor_to_earth_matrix_p[6];		
	}
	else if(FOLLOW_MOTOR_Y == en_yaw_follow_motor_p) 			//y��
	{
		yaw_error_q0 = cos( (sw_yaw_follow_pos_pid_ref*DEG_TO_RADIAN-motor_angle[1])/2);
		yaw_error_q1 = sin( (sw_yaw_follow_pos_pid_ref*DEG_TO_RADIAN-motor_angle[1])/2)*motor_to_earth_matrix_p[1];
		yaw_error_q2 = sin( (sw_yaw_follow_pos_pid_ref*DEG_TO_RADIAN-motor_angle[1])/2)*motor_to_earth_matrix_p[4];
		yaw_error_q3 = sin( (sw_yaw_follow_pos_pid_ref*DEG_TO_RADIAN-motor_angle[1])/2)*motor_to_earth_matrix_p[7];		
	}
	else if(FOLLOW_MOTOR_Z == en_yaw_follow_motor_p) 			//z��
	{
		yaw_error_q0 = cos( (sw_yaw_follow_pos_pid_ref*DEG_TO_RADIAN-motor_angle[2])/2);
		yaw_error_q1 = sin( (sw_yaw_follow_pos_pid_ref*DEG_TO_RADIAN-motor_angle[2])/2)*motor_to_earth_matrix_p[2];
		yaw_error_q2 = sin( (sw_yaw_follow_pos_pid_ref*DEG_TO_RADIAN-motor_angle[2])/2)*motor_to_earth_matrix_p[5];
		yaw_error_q3 = sin( (sw_yaw_follow_pos_pid_ref*DEG_TO_RADIAN-motor_angle[2])/2)*motor_to_earth_matrix_p[8];		
	}
	
	
	euler_yaw_ref_inc = atan2f(2*(yaw_error_q1*yaw_error_q2 + yaw_error_q0*yaw_error_q3),( 1 - 2*(yaw_error_q2*yaw_error_q2 + yaw_error_q3*yaw_error_q3)) )*57.3;
	earth_z_follow_pos_pid_ref = (int16_t)((euler_yaw_ref_inc + euler_p[2])*80);
	
	//��ֵ����,�����ڡ�180��ķ�Χ��
	if(earth_z_follow_pos_pid_ref > 180*80)
	{
		earth_z_follow_pos_pid_ref = earth_z_follow_pos_pid_ref - 360*80;
	}
	else if(earth_z_follow_pos_pid_ref < -180*80)
	{
		earth_z_follow_pos_pid_ref = earth_z_follow_pos_pid_ref + 360*80;
	}
	
}





/**
	* @brief��void Cal_Pitch_Follow_Ref(float	*euler_p,	float	*motor_angle_p,	Follow_Motor_Typedef en_pitch_follow_motor_p)
	* @note�� �������涯�Ĳο�
	*	@param��
	*/
void Cal_Pitch_Follow_Ref(float	*euler_p,	float	*motor_angle_p,	Follow_Motor_Typedef en_pitch_follow_motor_p)
{	
	float pitch_follow_motor_pos_pid_err;
	
	if(FOLLOW_MOTOR_X == en_pitch_follow_motor_p)//����pitch������
	{
		pitch_follow_motor_pos_pid_err = sw_pitch_follow_pos_pid_ref - motor_angle_p[0]*57.3;
		
//		if(pitch_follow_motor_pos_pid_err > 180)
//		{
//			pitch_follow_motor_pos_pid_err = pitch_follow_motor_pos_pid_err - 360;
//		}
//		else if(pitch_follow_motor_pos_pid_err < -180)
//		{
//			pitch_follow_motor_pos_pid_err = pitch_follow_motor_pos_pid_err + 360;
//		}
		
		earth_x_follow_pos_pid_ref = (euler_p[0] + pitch_follow_motor_pos_pid_err )*80;
	}
	else if(FOLLOW_MOTOR_Y == en_pitch_follow_motor_p)
	{
		pitch_follow_motor_pos_pid_err = sw_pitch_follow_pos_pid_ref - motor_angle_p[1]*57.3;
//		if(pitch_follow_motor_pos_pid_err > 180)
//		{
//			pitch_follow_motor_pos_pid_err = pitch_follow_motor_pos_pid_err - 360;
//		}
//		else if(pitch_follow_motor_pos_pid_err < -180)
//		{
//			pitch_follow_motor_pos_pid_err = pitch_follow_motor_pos_pid_err + 360;
//		}
		
		earth_x_follow_pos_pid_ref = (euler_p[0] + pitch_follow_motor_pos_pid_err )*80;
	}
	else if(FOLLOW_MOTOR_Z == en_pitch_follow_motor_p) //����pitch������
	{
		pitch_follow_motor_pos_pid_err = sw_pitch_follow_pos_pid_ref - motor_angle_p[2]*57.3;
//		if(pitch_follow_motor_pos_pid_err > 180)
//		{
//			pitch_follow_motor_pos_pid_err = pitch_follow_motor_pos_pid_err - 360;
//		}
//		else if(pitch_follow_motor_pos_pid_err < -180)
//		{
//			pitch_follow_motor_pos_pid_err = pitch_follow_motor_pos_pid_err + 360;
//		}	
		earth_x_follow_pos_pid_ref = (euler_p[0] + pitch_follow_motor_pos_pid_err )*80;
	}
	
	
	//��ֵ����,�����ڡ�180��ķ�Χ��
	if(earth_x_follow_pos_pid_ref > 180*80)
	{
		earth_x_follow_pos_pid_ref = earth_x_follow_pos_pid_ref - 360*80;
	}
	else if(earth_x_follow_pos_pid_ref < -180*80)
	{
		earth_x_follow_pos_pid_ref = earth_x_follow_pos_pid_ref + 360*80;
	}
}




/* External functions ------------------------------------------------------------*/
/**
	* @brief��void Cal_Pitch_Follow_Pos_Pid_Ref_Range(float	f_pitch_follow_pos_pid_ref_min,	float	f_pitch_follow_pos_pid_ref_max)
	* @note�� ��Pitch_Follow���ΪZ���ʱ��Ҫ��pitch_follow�Ĳο���������
	*	@param��f_pitch_follow_pos_pid_ref_min������
	*	@param��f_pitch_follow_pos_pid_ref_min������
	*/
void Cal_Pitch_Follow_Pos_Pid_Ref_Range(float	*euler_p,	float	*motor_angle_p, float	*f_pitch_follow_pos_pid_ref_min_p,	float	*f_pitch_follow_pos_pid_ref_max_p)
{	
	*f_pitch_follow_pos_pid_ref_min_p = -30 + motor_angle_p[2]*57.3 - euler_p[0];
	*f_pitch_follow_pos_pid_ref_max_p = 30 + motor_angle_p[2]*57.3 - euler_p[0];
}


/**
	* @brief��void Pitch_Follow_Pos_Pid_Updata(float	*euler_p,	float	*motor_angle_p,	Follow_Motor_Typedef en_pitch_follow_motor_p,	float ** coordinate_transform_matrix_p)
	* @note�� Pitch���涯PID����
	*	@param��
	*/
void Pitch_Follow_Pos_Pid_Updata(float	*euler_p,	float	*motor_angle_p,	Follow_Motor_Typedef en_pitch_follow_motor_p,	float ** coordinate_transform_matrix_p)
{

	Cal_Pitch_Follow_Ref(euler_p,motor_angle_p,	en_pitch_follow_motor_p);
	
	earth_x_follow_pos_pid_fdb = (int16_t)(euler_p[0]*80);
	
	//Ѱ�����·��
	if( (earth_x_follow_pos_pid_ref - earth_x_follow_pos_pid_fdb) > 180*80 )
	{
		earth_x_follow_pos_pid_fdb += 360*80;
	}
	else if( (earth_x_follow_pos_pid_ref - earth_x_follow_pos_pid_fdb) < -180*80 )
	{
		earth_x_follow_pos_pid_fdb -= 360*80;
	}
		
	earth_x_follow_pos_pid_out = earth_x_follow_pos_pid_out*earth_x_follow_pos_pid_out_kp + 
																(1 - earth_x_follow_pos_pid_out_kp)*PI_Regulator(earth_x_follow_pos_pid_ref,earth_x_follow_pos_pid_fdb,&st_earth_x_follow_pos_pid_struct);
}




/**
	* @brief��void Yaw_Follow_Pos_Pid_Updata(float	*euler_p, 	float	*motor_angle,	Follow_Motor_Typedef en_yaw_follow_motor_p,	float ** coordinate_transform_matrix_p)
	* @note�� Yaw���涯PID����
	*	@param��
	*/
void Yaw_Follow_Pos_Pid_Updata(float	*euler_p, 	float	*motor_angle,	Follow_Motor_Typedef en_yaw_follow_motor_p,	float ** coordinate_transform_matrix_p)
{
	Cal_Yaw_Follow_Ref(euler_p,motor_angle,en_yaw_follow_motor_p,coordinate_transform_matrix_p);
	
	earth_z_follow_pos_pid_fdb = (int16_t)(euler_p[2]*80);
	
	//Ѱ�����·��
	if( (earth_z_follow_pos_pid_ref - earth_z_follow_pos_pid_fdb) > 180*80 )
	{
		earth_z_follow_pos_pid_fdb += 360*80;
	}
	else if( (earth_z_follow_pos_pid_ref - earth_z_follow_pos_pid_fdb) < -180*80 )
	{
		earth_z_follow_pos_pid_fdb -= 360*80;
	}
	
	earth_z_follow_pos_pid_out = earth_z_follow_pos_pid_out*earth_z_follow_pos_pid_out_kp + 
															(1 - earth_z_follow_pos_pid_out_kp)*PI_Regulator(earth_z_follow_pos_pid_ref,earth_z_follow_pos_pid_fdb,&st_earth_z_follow_pos_pid_struct);
}





