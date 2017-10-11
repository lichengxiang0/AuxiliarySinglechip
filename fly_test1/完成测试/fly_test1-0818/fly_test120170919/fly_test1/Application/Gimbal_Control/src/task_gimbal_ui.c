/**
  ******************************************************************************
  * File Name          :task_gimbal_ui.c
  * Description        :�����û�����Ĳ���,1->״̬�л� 2->״̬�л�ʱִ�еĳ�ʼ��
												3->�û��ٶȿ��Ƽ�����
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
#include "task_gimbal_ui.h"

#include "gimbal_attitude_ctrl.h"
#include "gimbal_follow_ctrl.h"
#include "gimbal_frame_angle.h"
#include "gimbal_ahrs.h"
#include "gimbal_coordinate_transform.h"
#include "gimbal_spd_ctrl.h"

#include "task_common_functions.h"
#include "task_gimbal_pos_ctrl.h"

#include "stm32_bsp_timer.h"

#include <math.h>

/* Private defines ------------------------------------------------------------*/
#define	DEG_TO_RADIAN		((float)0.01744)
#define	RADIAN_TO_DEG		((float)57.3)

#define	UI_UPDATA_PERIOD_MS		((uint8_t)5)  /*5ms*/

/*�����е��λ��ض��壬����������λ*/
#define	MOTOR_X_FRAME_ANGLE_MIN		((int16_t)(-90))
#define	MOTOR_X_FRAME_ANGLE_MAX		((int16_t)(40))

#define	MOTOR_Y_FRAME_ANGLE_MIN		((int16_t)(-130))
#define	MOTOR_Y_FRAME_ANGLE_MAX		((int16_t)(130))
#define	MOTOR_Z_FRAME_ANGLE_MIN		((int16_t)(-30))
#define	MOTOR_Z_FRAME_ANGLE_MAX		((int16_t)(30))

#define	MANUAL_ADJ_REV_ANGLE			((int16_t)5)  //ҡ�˿��ƺ��ֶ��Ƕ�  ��λλ��<��>�����е��λ<��Ԥ���Ƕ�>
#define	AUTO_ADJ_REV_ANGLE				((int16_t)2)	//����λ  ��������е��λ���Զ��ص��Ƕȣ�����ʼ���ٻ�е��λ����

/* Private variables ------------------------------------------------------------*/

float			f_earth_x_pos_pid_ref_shadow = 0,	f_earth_x_pos_pid_ref_shadow_min = 0,	f_earth_x_pos_pid_ref_shadow_max = 0;
float			f_earth_y_pos_pid_ref_shadow = 0;
float			f_earth_z_pos_pid_ref_shadow = 0, f_earth_z_pos_pid_ref_shadow_min = 0,	f_earth_z_pos_pid_ref_shadow_max = 0;

float			f_pitch_follow_pos_pid_ref_shadow = 0,	f_pitch_follow_pos_pid_ref_shadow_min = 0,	f_pitch_follow_pos_pid_ref_shadow_max = 0;
float			f_yaw_follow_pos_pid_ref_shadow = 0,		f_yaw_follow_pos_pid_ref_shadow_min = 0,		f_yaw_follow_pos_pid_ref_shadow_max = 0;

/*�ֶ�ת�������־*/
FlagStatus	en_manual_turn_motor_x_flag = RESET,	en_manual_turn_motor_y_flag = RESET,	en_manual_turn_motor_z_flag = RESET;




/* External variables ------------------------------------------------------------*/
Gimbal_Power_Mode_Typedef 	en_gimbal_power_mode = SLEEP;					//SLEEP_TO_NORMAL;
Camera_Viewer_Mode_Typedef	en_camera_viewer_mode = HORIZONTAL;   //Ĭ��ˮƽλ��

Gimbal_Work_Mode						en_gimbal_work_mode = NONE_FOLLOW;
//Gimbal_Work_Mode						en_gimbal_work_mode = Z_FOLLOW;

float				f_earth_y_pos_pid_ref_offset_in_horizontal_mode = 0,	
						f_earth_y_pos_pid_ref_offset_in_vertical_mode = 90;

FlagStatus	en_earth_y_pos_pid_ref_offset_in_horizontal_mode_cal_flag = RESET,
						en_earth_y_pos_pid_ref_offset_in_vertical_mode_cal_flag = RESET;	

uint8_t			uc_receive_ui_mode_data,		uc_receive_ui_spd_data[2];

float				f_pitch_user_ctrl_spd_ratio = 1.5,	f_yaw_user_ctrl_spd_ratio = 1.5;

/* Private functions ------------------------------------------------------------*/
//void	Update_Pid_Ref(void)
//{
//	
//}
/**
	* @brief��void	Process_Gimbal_Power_Mode_State_Change(uint8_t *uc_ui_data_p)
	* @note:	����Gimbal_Power_Mode��״̬���л�
	*	@param��uint8_t *uc_ui_data_p������״̬��Ϣ���� 
	*/
void	Process_Gimbal_Power_Mode_State_Change(uint8_t *uc_ui_data_p)
{
	/*����normal����ʽen_gimbal_power_mode��״̬�л�*/
	if(0x01 == (*uc_ui_data_p&0x03))
	{	
		switch(en_gimbal_power_mode)
		{
			case SLEEP:	
			case NORMAL_TO_SLEEP:
				en_gimbal_power_mode = SLEEP_TO_NORMAL;
				break;
			
			case SLEEP_TO_NORMAL:
			case NORMAL:
				break;
			
			default:
				break;						
		}
	}
	else if(0x00 == (*uc_ui_data_p&0x03))
	{
		switch(en_gimbal_power_mode)
		{
			case SLEEP:	
			case NORMAL_TO_SLEEP:
				break;
			
			case SLEEP_TO_NORMAL:
			case NORMAL:
				en_gimbal_power_mode = NORMAL_TO_SLEEP;
				break;
			
			default:
				break;						
		}
	}
}


/**
	* @brief��void	Process_Camera_Viewer_Mode_State_Change(uint8_t *uc_ui_data_p)
	* @note:	����Camera_Viewer_Mode��״̬���л�
	*	@param��uint8_t *uc_ui_data_p������״̬��Ϣ���� 
	*/
void	Process_Camera_Viewer_Mode_State_Change(uint8_t *uc_ui_data_p)
{
	if(0x40 == (*uc_ui_data_p&0x40))
	{
		switch(en_camera_viewer_mode)
		{
			case	HORIZONTAL:
				en_camera_viewer_mode = VERTICAL;	
				break;
			
			case VERTICAL:
				break;
			
			default:
				break;
		}
	}
	else if(0x00 == (*uc_ui_data_p&0x40))
	{
		switch(en_camera_viewer_mode)
		{
			case	HORIZONTAL:				
				break;
			
			case VERTICAL:
				en_camera_viewer_mode = HORIZONTAL;
				break;
			
			default:
				break;
		}
	}
}



/**
	* @brief��void	Process_Gimbal_Work_Mode_State_Change(uint8_t *uc_ui_data_p)
	* @note:	����Gimbal_Work_Mode��״̬���л�
	*	@param��uint8_t *uc_ui_data_p������״̬��Ϣ���� 
	*/
void	Process_Gimbal_Work_Mode_State_Change(uint8_t *uc_ui_data_p)
{
	if(0x00 == ((*uc_ui_data_p>>2)&0x0F)) //NONE_FOLLOW
	{
		switch(en_gimbal_work_mode) 
		{
			case NONE_FOLLOW:	
				
			case TRACKER:
			case LOCK:
				en_gimbal_work_mode = NONE_FOLLOW;
				break;
			
			case Z_FOLLOW:
			case Z_FOLLOW_TO_NONE_FOLLOW:
				/*��Ҫ�ȴ�YAW�涯��λ�ú����л���NONE_FOLLOW*/
				en_gimbal_work_mode = Z_FOLLOW_TO_NONE_FOLLOW;  //�ȴ�Z_FOLLOW����
				
				break;
			
			case X_Z_FOLLOW:
			case X_Z_FOLLOW_TO_NONE_FOLLOW:
			case X_Z_FOLLOW_TO_Z_FOLLOW:	
				/*��Ҫ�ȴ�YAW��PITCH�涯��λ�ú����л���NONE_FOLLOW*/
				en_gimbal_work_mode = X_Z_FOLLOW_TO_NONE_FOLLOW;  //�ȴ�X_Z_FOLLOW����
				break;
			
			default:
				break;
		}
	}
	else if(0x01 == ((*uc_ui_data_p>>2)&0x0F))  //Z_FOLLOW
	{
		
		switch(en_gimbal_work_mode) 
		{					
			case NONE_FOLLOW:
				
			case Z_FOLLOW:	
			case Z_FOLLOW_TO_NONE_FOLLOW:
				
			case TRACKER:
			case LOCK:
				en_gimbal_work_mode = Z_FOLLOW;
				break;
			
			case X_Z_FOLLOW:
			case X_Z_FOLLOW_TO_Z_FOLLOW:
			case X_Z_FOLLOW_TO_NONE_FOLLOW:
				/*��Ҫ�ȴ�PTICH�涯��λ�ú����л���Z_FOLLOW*/
				en_gimbal_work_mode = X_Z_FOLLOW_TO_Z_FOLLOW;
				break;
			
			default:
				break;
		}
	}
	else if(0x02 == ((*uc_ui_data_p>>2)&0x0F)) //X_Z_FOLLOW
	{
		switch(en_gimbal_work_mode) 
		{
			case NONE_FOLLOW:
			
			case Z_FOLLOW:
			case Z_FOLLOW_TO_NONE_FOLLOW:	
			
			case X_Z_FOLLOW:
			case X_Z_FOLLOW_TO_NONE_FOLLOW:
			case X_Z_FOLLOW_TO_Z_FOLLOW:
				
			case TRACKER:
			case LOCK:
				en_gimbal_work_mode = X_Z_FOLLOW;
				break;
				
			default:
				break;
		}
	}
	else	if(0x03 == ((*uc_ui_data_p>>2)&0x0F)) 
	{
		en_gimbal_work_mode = TRACKER;
	}
	else if(0x04 == ((*uc_ui_data_p>>2)&0x0F))
	{
		en_gimbal_work_mode = LOCK;
	}
}



/**
	* @brief��void	Process_Motor_Limit_Event(float	*euler_p, float	*motor_angle,	float ** coordinate_transform_matrix_p)
	* @note:	�Զ�����λ�û��ο�ֵ�����⵽������е��λ
	*	@param��float	*euler_p, float	*motor_angle,	float ** coordinate_transform_matrix_p
	*/
void	Process_Motor_Limit_Event(float	*euler_p, float	*motor_angle,	float ** coordinate_transform_matrix_p)
{
	if(HORIZONTAL == en_camera_viewer_mode)
	{
		/*motor_x��λ�û��ο�������*/
//		BondFloat(&f_pitch_follow_pos_pid_ref_shadow, MOTOR_X_FRAME_ANGLE_MIN, MOTOR_X_FRAME_ANGLE_MAX);	
		if(f_pitch_follow_pos_pid_ref_shadow < MOTOR_X_FRAME_ANGLE_MIN)
		{
			f_pitch_follow_pos_pid_ref_shadow = MOTOR_X_FRAME_ANGLE_MIN + AUTO_ADJ_REV_ANGLE;
		}
		else if(f_pitch_follow_pos_pid_ref_shadow > MOTOR_X_FRAME_ANGLE_MAX)
		{
			f_pitch_follow_pos_pid_ref_shadow = MOTOR_X_FRAME_ANGLE_MAX - AUTO_ADJ_REV_ANGLE;
		}
		
		if(motor_angle[0]*RADIAN_TO_DEG < MOTOR_X_FRAME_ANGLE_MIN)
		{
			f_earth_x_pos_pid_ref_shadow = euler_p[0] + (MOTOR_X_FRAME_ANGLE_MIN + AUTO_ADJ_REV_ANGLE - motor_angle[0]*RADIAN_TO_DEG)*Get_Float_Sign(coordinate_transform_matrix_p[3][0]);
		}
		else if(motor_angle[0]*RADIAN_TO_DEG > MOTOR_X_FRAME_ANGLE_MAX)
		{
			f_earth_x_pos_pid_ref_shadow = euler_p[0] + (MOTOR_X_FRAME_ANGLE_MAX - AUTO_ADJ_REV_ANGLE - motor_angle[0]*RADIAN_TO_DEG)*Get_Float_Sign(coordinate_transform_matrix_p[3][0]);
		}
		
		/*motor_z��λ�û��ο�������->motor_z���涯λ��Ϊ0���ɵ���,����Ƕȿɵ���*/
		if(FOLLOW_MOTOR_Z == en_yaw_follow_motor)
		{
			if(motor_angle[2]*RADIAN_TO_DEG < MOTOR_Z_FRAME_ANGLE_MIN)
			{
				f_earth_z_pos_pid_ref_shadow = euler_p[2] + (MOTOR_Z_FRAME_ANGLE_MIN + AUTO_ADJ_REV_ANGLE - motor_angle[2]*RADIAN_TO_DEG)*Get_Float_Sign(coordinate_transform_matrix_p[3][8]);
			}
			else if(motor_angle[2]*RADIAN_TO_DEG > MOTOR_Z_FRAME_ANGLE_MAX)
			{
				f_earth_z_pos_pid_ref_shadow = euler_p[2] + (MOTOR_Z_FRAME_ANGLE_MAX - AUTO_ADJ_REV_ANGLE - motor_angle[2]*RADIAN_TO_DEG)*Get_Float_Sign(coordinate_transform_matrix_p[3][8]);
			}
		}	
		else/*motor_y��λ�û��ο�������->motor_y���涯λ�ÿɵ���,����Ƕȿɵ���*/
		{
//			BondFloat(&f_yaw_follow_pos_pid_ref_shadow, MOTOR_Y_FRAME_ANGLE_MIN, MOTOR_Y_FRAME_ANGLE_MAX);	
			if(f_yaw_follow_pos_pid_ref_shadow < MOTOR_Y_FRAME_ANGLE_MIN)
			{
				f_yaw_follow_pos_pid_ref_shadow = MOTOR_Y_FRAME_ANGLE_MIN + AUTO_ADJ_REV_ANGLE;
			}
			else if(f_yaw_follow_pos_pid_ref_shadow > MOTOR_Y_FRAME_ANGLE_MAX)
			{
				f_yaw_follow_pos_pid_ref_shadow = MOTOR_Y_FRAME_ANGLE_MAX - AUTO_ADJ_REV_ANGLE;
			}
			
			
			if(motor_angle[1]*RADIAN_TO_DEG < MOTOR_Y_FRAME_ANGLE_MIN)
			{
				f_earth_z_pos_pid_ref_shadow = euler_p[2] + (MOTOR_Y_FRAME_ANGLE_MIN + AUTO_ADJ_REV_ANGLE - motor_angle[1]*RADIAN_TO_DEG)*Get_Float_Sign(coordinate_transform_matrix_p[3][5]);
				
			}
			else if(motor_angle[1]*RADIAN_TO_DEG > MOTOR_Y_FRAME_ANGLE_MAX)
			{
				f_earth_z_pos_pid_ref_shadow = euler_p[2] + (MOTOR_Y_FRAME_ANGLE_MAX - AUTO_ADJ_REV_ANGLE - motor_angle[1]*RADIAN_TO_DEG)*Get_Float_Sign(coordinate_transform_matrix_p[3][5]);
			}
		}
	}
	else  //����ģʽ�£�����λ����
	{
		/*motor_x��λ�û��ο�������*/
//		BondFloat(&f_yaw_follow_pos_pid_ref_shadow, MOTOR_X_FRAME_ANGLE_MIN, MOTOR_X_FRAME_ANGLE_MAX);
		if(f_yaw_follow_pos_pid_ref_shadow < MOTOR_X_FRAME_ANGLE_MIN)
		{
			f_yaw_follow_pos_pid_ref_shadow = MOTOR_X_FRAME_ANGLE_MIN + AUTO_ADJ_REV_ANGLE;
		}
		else if(f_yaw_follow_pos_pid_ref_shadow > MOTOR_X_FRAME_ANGLE_MAX)
		{
			f_yaw_follow_pos_pid_ref_shadow = MOTOR_X_FRAME_ANGLE_MAX - AUTO_ADJ_REV_ANGLE;
		}
		
		
		if(motor_angle[0]*RADIAN_TO_DEG < MOTOR_X_FRAME_ANGLE_MIN)
		{
			f_earth_z_pos_pid_ref_shadow = euler_p[2] + (MOTOR_X_FRAME_ANGLE_MIN + AUTO_ADJ_REV_ANGLE - motor_angle[0]*RADIAN_TO_DEG)*Get_Float_Sign(coordinate_transform_matrix_p[3][2]);
		}
		else if(motor_angle[0]*RADIAN_TO_DEG > MOTOR_X_FRAME_ANGLE_MAX)
		{
			f_earth_z_pos_pid_ref_shadow = euler_p[2] + (MOTOR_X_FRAME_ANGLE_MAX - AUTO_ADJ_REV_ANGLE - motor_angle[0]*RADIAN_TO_DEG)*Get_Float_Sign(coordinate_transform_matrix_p[3][2]);
		}
		
		/*motor_z��λ�û��ο�������*/
		BondFloat(&f_earth_x_pos_pid_ref_shadow, -30, 30);
		if(euler_p[0] < -30)
		{
			f_pitch_follow_pos_pid_ref_shadow = motor_angle[2]*RADIAN_TO_DEG - (euler_p[0] - (-30 + AUTO_ADJ_REV_ANGLE) )*Get_Float_Sign(coordinate_transform_matrix_p[3][6]);
		}
		else if(euler_p[0] > 30)
		{
			f_pitch_follow_pos_pid_ref_shadow = motor_angle[2]*RADIAN_TO_DEG - (euler_p[0] - ( 30 - AUTO_ADJ_REV_ANGLE) )*Get_Float_Sign(coordinate_transform_matrix_p[3][6]);
		}
	}
}


/**
	* @brief��void	Process_State_Change_Action(void)
	* @note�� ����״̬�л�ʱ�������ƻ�·�ĳ�ʼ������
	*/
void	Process_State_Change_Action(void)
{
	static	Camera_Viewer_Mode_Typedef 	en_last_camera_viewer_mode = HORIZONTAL;
	static	Gimbal_Work_Mode						en_last_gimbal_work_mode = Z_FOLLOW;
	
	switch(en_gimbal_power_mode)
	{
		case SLEEP:
			/*SLEEPģʽʱ�����й���ģʽ�����ڳ�ʼ��״̬*/
			f_earth_x_pos_pid_ref_shadow = 0;
			f_earth_z_pos_pid_ref_shadow = 0;
		
			f_pitch_follow_pos_pid_ref_shadow = 0;
			f_yaw_follow_pos_pid_ref_shadow = 0;
			if(VERTICAL == en_camera_viewer_mode)
			{
				f_earth_y_pos_pid_ref_shadow = f_earth_y_pos_pid_ref_offset_in_vertical_mode;
			}
			else if(HORIZONTAL == en_camera_viewer_mode)
			{
				f_earth_y_pos_pid_ref_shadow = f_earth_y_pos_pid_ref_offset_in_horizontal_mode;
			}
			break;
		case SLEEP_TO_NORMAL:
		case NORMAL:
		case NORMAL_TO_SLEEP:
			break;
		
		default:
			break;
	}

	/*camera_viewer_mode�л�ʱ�����ƻ���ʼ��*/
	/*�������л�ʱ������ģʽ�л�ΪZ_FOLLOW,�Һ��Թ���ģʽ�ı���л�*/
	switch(en_last_camera_viewer_mode)
	{
		case	HORIZONTAL:
			switch(en_camera_viewer_mode)
			{
				case VERTICAL:
					/*�������л�ʱ�����������м�״̬����״̬�л�������Ŀ��ֵ�ı䣬ֱ�ӳ�ʼ��*/
					if(X_Z_FOLLOW_TO_Z_FOLLOW == en_gimbal_work_mode)
					{
						en_gimbal_work_mode = Z_FOLLOW;
					}
					if( (X_Z_FOLLOW_TO_NONE_FOLLOW == en_gimbal_work_mode)||(Z_FOLLOW_TO_NONE_FOLLOW == en_gimbal_work_mode ) )
					{
						en_gimbal_work_mode = NONE_FOLLOW;
					}
					en_last_gimbal_work_mode = en_gimbal_work_mode;
					
					/*���ݺ������л����ģʽ��ʼ��������ֻ����Z_Follow�ĳ�ʼ��*/
					f_earth_x_pos_pid_ref_shadow = 0;  //test
					f_earth_y_pos_pid_ref_shadow = f_earth_y_pos_pid_ref_offset_in_vertical_mode;
					
					break;
				
				case HORIZONTAL:
				default:
					break;
			}
			break;
				
		case VERTICAL:
			switch(en_camera_viewer_mode)
			{							
				case HORIZONTAL:
					/*�������л�ʱ�����������м�״̬����״̬�л�������Ŀ��ֵ�ı䣬ֱ�ӳ�ʼ��*/
					if(X_Z_FOLLOW_TO_Z_FOLLOW == en_gimbal_work_mode)
					{
						en_gimbal_work_mode = Z_FOLLOW;
					}
					if( (X_Z_FOLLOW_TO_NONE_FOLLOW == en_gimbal_work_mode)||(Z_FOLLOW_TO_NONE_FOLLOW == en_gimbal_work_mode ) )
					{
						en_gimbal_work_mode = NONE_FOLLOW;
					}
					en_last_gimbal_work_mode = en_gimbal_work_mode;
					
					
					/*���ݺ������л����ģʽ��ʼ��������ֻ����Z_Follow�ĳ�ʼ��*/
					f_earth_x_pos_pid_ref_shadow = 0;  																			//test
					f_earth_y_pos_pid_ref_shadow = f_earth_y_pos_pid_ref_offset_in_horizontal_mode;

					break;
				
				case VERTICAL:
				default:
					break;
			}			
			break;
				
		default:
			break;
	}		
	en_last_camera_viewer_mode = en_camera_viewer_mode;
	
	
	/*gimbal_work_mode�л�ʱ�����ƻ���ʼ��*/
	switch(en_last_gimbal_work_mode)
	{
		case	NONE_FOLLOW:
			switch(en_gimbal_work_mode)
			{
				case NONE_FOLLOW:
				case Z_FOLLOW:
				case Z_FOLLOW_TO_NONE_FOLLOW:
				break;
				
				case	X_Z_FOLLOW:
				case	X_Z_FOLLOW_TO_NONE_FOLLOW:
				case	X_Z_FOLLOW_TO_Z_FOLLOW:	
					if(FOLLOW_MOTOR_X == en_pitch_follow_motor)
					{
						f_pitch_follow_pos_pid_ref_shadow = motor_frame_angle_rad[0]*RADIAN_TO_DEG;
					}
					else if(FOLLOW_MOTOR_Z == en_pitch_follow_motor)
					{
						f_pitch_follow_pos_pid_ref_shadow = motor_frame_angle_rad[2]*RADIAN_TO_DEG;
//						Cal_Pitch_Follow_Pos_Pid_Ref_Range(f_euler_array,motor_frame_angle_rad,&f_pitch_follow_pos_pid_ref_shadow_min,&f_pitch_follow_pos_pid_ref_shadow_max);
//						BondFloat(&f_pitch_follow_pos_pid_ref_shadow,f_pitch_follow_pos_pid_ref_shadow_min,f_pitch_follow_pos_pid_ref_shadow_max);
					}
				break;
					
				case TRACKER:
				case LOCK:
				break;
				
				default:
				break;
			}
		break;
		
		case	Z_FOLLOW:
		case	Z_FOLLOW_TO_NONE_FOLLOW:
			switch(en_gimbal_work_mode)
			{
				case NONE_FOLLOW:
					f_earth_z_pos_pid_ref_shadow = f_euler_array[2];
				break;
				
				case Z_FOLLOW:
				case Z_FOLLOW_TO_NONE_FOLLOW:
				break;
				
				case	X_Z_FOLLOW:
				case	X_Z_FOLLOW_TO_NONE_FOLLOW:
				case	X_Z_FOLLOW_TO_Z_FOLLOW:	
					if(FOLLOW_MOTOR_X == en_pitch_follow_motor)
					{
						f_pitch_follow_pos_pid_ref_shadow = motor_frame_angle_rad[0]*RADIAN_TO_DEG;
					}
					else if(FOLLOW_MOTOR_Z == en_pitch_follow_motor)
					{
						f_pitch_follow_pos_pid_ref_shadow = motor_frame_angle_rad[2]*RADIAN_TO_DEG;
//						Cal_Pitch_Follow_Pos_Pid_Ref_Range(f_euler_array,motor_frame_angle_rad,&f_pitch_follow_pos_pid_ref_shadow_min,&f_pitch_follow_pos_pid_ref_shadow_max);
//						BondFloat(&f_pitch_follow_pos_pid_ref_shadow,f_pitch_follow_pos_pid_ref_shadow_min,f_pitch_follow_pos_pid_ref_shadow_max);
					}
				break;
			
				default:
				break;
			}
		break;
		
		case	X_Z_FOLLOW:
		case	X_Z_FOLLOW_TO_NONE_FOLLOW:
		case	X_Z_FOLLOW_TO_Z_FOLLOW:		
			switch(en_gimbal_work_mode)
			{
				case NONE_FOLLOW:
					/*��ʼ��yaw�ǲο�*/
					f_earth_z_pos_pid_ref_shadow = f_euler_array[2];
					
					/*��ʼ��pitch�ǲο������޷�*/
					f_earth_x_pos_pid_ref_shadow = f_euler_array[0];
//					if(HORIZONTAL == en_camera_viewer_mode)
//					{
//						BondFloat(&f_earth_x_pos_pid_ref_shadow,-60,60);
//					}
//					else if(VERTICAL == en_camera_viewer_mode)
//					{
//						BondFloat(&f_earth_x_pos_pid_ref_shadow,-30,30);
//					}
				break;
							
				case Z_FOLLOW:
				case Z_FOLLOW_TO_NONE_FOLLOW:					
					/*��ʼ��pitch�ǲο������޷�*/
					f_earth_x_pos_pid_ref_shadow = f_euler_array[0];
//					if(HORIZONTAL == en_camera_viewer_mode)
//					{
//						BondFloat(&f_earth_x_pos_pid_ref_shadow,-60,60);
//					}
//					else if(VERTICAL == en_camera_viewer_mode)
//					{
//						BondFloat(&f_earth_x_pos_pid_ref_shadow,-30,30);
//					}
				break;
				
				case	X_Z_FOLLOW:
				case	X_Z_FOLLOW_TO_NONE_FOLLOW:
				case	X_Z_FOLLOW_TO_Z_FOLLOW:						
				break;
						
				default:
				break;
			}
		break;
			
		case	TRACKER:
		case 	LOCK:
			switch(en_gimbal_work_mode)
			{
				case NONE_FOLLOW:
					/*��ʼ��yaw�ǲο�*/
					f_earth_z_pos_pid_ref_shadow = f_euler_array[2];
					
					/*��ʼ��pitch�ǲο������޷�*/
					f_earth_x_pos_pid_ref_shadow = f_euler_array[0];
//					if(HORIZONTAL == en_camera_viewer_mode)
//					{
//						BondFloat(&f_earth_x_pos_pid_ref_shadow,-60,60);
//					}
//					else if(VERTICAL == en_camera_viewer_mode)
//					{
//						BondFloat(&f_earth_x_pos_pid_ref_shadow,-30,30);
//					}
				break;
							
				case Z_FOLLOW:
				case Z_FOLLOW_TO_NONE_FOLLOW:					
					/*��ʼ��pitch�ǲο������޷�*/
					f_earth_x_pos_pid_ref_shadow = f_euler_array[0];
//					if(HORIZONTAL == en_camera_viewer_mode)
//					{
//						BondFloat(&f_earth_x_pos_pid_ref_shadow,-60,60);
//					}
//					else if(VERTICAL == en_camera_viewer_mode)
//					{
//						BondFloat(&f_earth_x_pos_pid_ref_shadow,-30,30);
//					}
				break;
				
				case	X_Z_FOLLOW:
				case	X_Z_FOLLOW_TO_NONE_FOLLOW:
				case	X_Z_FOLLOW_TO_Z_FOLLOW:	
					if(FOLLOW_MOTOR_X == en_pitch_follow_motor)
					{
						f_pitch_follow_pos_pid_ref_shadow = motor_frame_angle_rad[0]*RADIAN_TO_DEG;
					}
					else if(FOLLOW_MOTOR_Z == en_pitch_follow_motor)
					{
						f_pitch_follow_pos_pid_ref_shadow = motor_frame_angle_rad[2]*RADIAN_TO_DEG;
//						Cal_Pitch_Follow_Pos_Pid_Ref_Range(f_euler_array,motor_frame_angle_rad,&f_pitch_follow_pos_pid_ref_shadow_min,&f_pitch_follow_pos_pid_ref_shadow_max);
//						BondFloat(&f_pitch_follow_pos_pid_ref_shadow,f_pitch_follow_pos_pid_ref_shadow_min,f_pitch_follow_pos_pid_ref_shadow_max);
					}
				break;
				
				case TRACKER:
				case LOCK:
				break;
				
				default:
					break;
			}
		break;
		
		default:
		break;	
	}
	en_last_gimbal_work_mode = en_gimbal_work_mode;
}

void	Process_User_Spd_Ctrl(uint8_t	*uc_ui_spd_data_p)
{
	static	FlagStatus	en_this_funtion_first_run_flag = SET;
	static	FlagStatus	en_last_tripus_mode_flag = RESET;
	
	/*f_user_x_spd:joystickˮƽ��Ӧ�ٶȣ�f_user_y_spd��joystick��ֱ��Ӧ�ٶ�*/
	float			f_user_x_spd,	f_user_y_spd;  
	
	float	f_user_x_spd_on_earth[3],	f_user_y_spd_on_earth[3];
	float	f_user_x_spd_on_motor[3],	f_user_y_spd_on_motor[3];
	
	static	uint32_t	ul_last_user_spd_ctrl_micros;
	float		f_user_spd_ctrl_period;
	
	if(SET == en_this_funtion_first_run_flag)
	{	
		f_earth_x_pos_pid_ref_shadow = 0;
		f_earth_y_pos_pid_ref_shadow = f_earth_y_pos_pid_ref_offset_in_horizontal_mode;
		f_earth_z_pos_pid_ref_shadow = 0;
		
		f_pitch_follow_pos_pid_ref_shadow = 0;
		f_yaw_follow_pos_pid_ref_shadow = 0;
		
		earth_x_pos_pid_ref = f_earth_x_pos_pid_ref_shadow*80;
		earth_y_pos_pid_ref = f_earth_y_pos_pid_ref_shadow*80;
		earth_z_pos_pid_ref = f_earth_z_pos_pid_ref_shadow*80;
		
		sw_pitch_follow_pos_pid_ref = f_pitch_follow_pos_pid_ref_shadow*RADIAN_TO_DEG;
		sw_yaw_follow_pos_pid_ref = f_yaw_follow_pos_pid_ref_shadow*RADIAN_TO_DEG;
		
		en_this_funtion_first_run_flag = RESET;
		return;
	}
	else
	{
		f_user_x_spd = ((int8_t)uc_ui_spd_data_p[0]/3)*10;		//�˵�����1��2��λ��ֻ��Ӧ��3��λ��������  //test
		//f_user_y_spd = ((int8_t)uc_ui_spd_data_p[1]/3)*10;    //test
		 
//		f_user_y_spd_on_earth[0] = f_user_y_spd;
//		f_user_y_spd_on_earth[1] = 0;
//		f_user_y_spd_on_earth[2] = 0;
		
//		switch(en_gimbal_work_mode)
//		{	
//			/*pitch �� yaw�ɿ�*/
//			case NONE_FOLLOW:
				f_user_x_spd_on_earth[0] = f_user_x_spd;
				f_user_x_spd_on_earth[1] = 0;
				f_user_x_spd_on_earth[2] = 0;
//				break;
//			
//			/*pitch �� roll�ɿ�*/
//			case Z_FOLLOW:
//			case Z_FOLLOW_TO_NONE_FOLLOW:					
//				f_user_x_spd_on_earth[0] = 0;
//				f_user_x_spd_on_earth[1] = f_user_x_spd;
//				f_user_x_spd_on_earth[2] = 0;
//				break;
//			
//			/*pitch�ɿ�*/
//			case	X_Z_FOLLOW:
//			case	X_Z_FOLLOW_TO_NONE_FOLLOW:
//			case	X_Z_FOLLOW_TO_Z_FOLLOW:		
//				f_user_x_spd_on_earth[0] = 0;
//				f_user_x_spd_on_earth[1] = 0;
//				f_user_x_spd_on_earth[2] = 0;
//				break;
//			
//			default:
//				break;
//		}
		
		Earth_Spd_To_Motor_Spd(f_user_x_spd_on_earth,f_user_x_spd_on_motor,coordinate_transform_matrix);
		
		//Earth_Spd_To_Motor_Spd(f_user_y_spd_on_earth,f_user_y_spd_on_motor,coordinate_transform_matrix);
		
		/*-----------------����ģʽʱ���ڼ���λ�ô����û��ٶȵĴ���-----------------*/
		if(HORIZONTAL == en_camera_viewer_mode)
		{
			/*HORIZONTALʱf_user_y_spdֻ��motor_x��Ӱ��*/
			/*motor_x����λ�Ǻ������ٿ����ٶ�*/
			if(		( (motor_frame_angle_rad[0]*RADIAN_TO_DEG > (MOTOR_X_FRAME_ANGLE_MAX - MANUAL_ADJ_REV_ANGLE))&&(f_user_x_spd_on_motor[0] > 0) )||
						( (motor_frame_angle_rad[0]*RADIAN_TO_DEG < (MOTOR_X_FRAME_ANGLE_MIN + MANUAL_ADJ_REV_ANGLE))&&(f_user_x_spd_on_motor[0] < 0) )
				)
			{
				f_user_x_spd = 0;
			}
			
			/*HORIZONTALʱf_user_x_spdֻ��motor_y/motor_z��Ӱ��*/
			/*motor_y/motor_z����λ�Ǻ������ٿ����ٶ�*/
//			if(		( (motor_frame_angle_rad[1]*RADIAN_TO_DEG > (MOTOR_Y_FRAME_ANGLE_MAX - MANUAL_ADJ_REV_ANGLE))&&(f_user_x_spd_on_motor[1] > 0) )||
//						( (motor_frame_angle_rad[1]*RADIAN_TO_DEG < (MOTOR_Y_FRAME_ANGLE_MIN + MANUAL_ADJ_REV_ANGLE))&&(f_user_x_spd_on_motor[1] < 0) )||
//						( (motor_frame_angle_rad[2]*RADIAN_TO_DEG > (MOTOR_Z_FRAME_ANGLE_MAX - MANUAL_ADJ_REV_ANGLE))&&(f_user_x_spd_on_motor[2] > 0) )||
//						( (motor_frame_angle_rad[2]*RADIAN_TO_DEG < (MOTOR_Z_FRAME_ANGLE_MIN + MANUAL_ADJ_REV_ANGLE))&&(f_user_x_spd_on_motor[2] < 0) )
//				)
//			{
//				f_user_x_spd = 0;
//			}
		}
//		else 
//		if(VERTICAL == en_camera_viewer_mode)
//		{
//			switch(en_gimbal_work_mode)
//			{
//				case	NONE_FOLLOW:
//					/*f_user_x_spdֻӰ��motor_x������yaw*/
//					if(		( (motor_frame_angle_rad[0]*RADIAN_TO_DEG > (MOTOR_X_FRAME_ANGLE_MAX - MANUAL_ADJ_REV_ANGLE))&&(f_user_x_spd_on_motor[0] > 0) )||
//								( (motor_frame_angle_rad[0]*RADIAN_TO_DEG < (MOTOR_X_FRAME_ANGLE_MIN + MANUAL_ADJ_REV_ANGLE))&&(f_user_x_spd_on_motor[0] < 0) )
//						)
//					{
//						f_user_x_spd = 0;
//					}
//					/*f_user_y_spdֻӰ��motor_y/motor_z,����pitch*/
//					if(		( (motor_frame_angle_rad[1]*RADIAN_TO_DEG > (MOTOR_Y_FRAME_ANGLE_MAX - MANUAL_ADJ_REV_ANGLE))&&(f_user_y_spd_on_motor[1] > 0) )||
//								( (motor_frame_angle_rad[1]*RADIAN_TO_DEG < (MOTOR_Y_FRAME_ANGLE_MIN + MANUAL_ADJ_REV_ANGLE))&&(f_user_y_spd_on_motor[1] < 0) )||
//								( (motor_frame_angle_rad[2]*RADIAN_TO_DEG > (MOTOR_Z_FRAME_ANGLE_MAX - MANUAL_ADJ_REV_ANGLE))&&(f_user_y_spd_on_motor[2] > 0) )||
//								( (motor_frame_angle_rad[2]*RADIAN_TO_DEG < (MOTOR_Z_FRAME_ANGLE_MIN + MANUAL_ADJ_REV_ANGLE))&&(f_user_y_spd_on_motor[2] < 0) )
//						)
//					{
//						f_user_y_spd = 0;
//					}
//					break;
////					
////				case 	Z_FOLLOW:
////				case 	X_Z_FOLLOW:
////					/*f_user_y_spdֻӰ��motor_z������pitch*/
////					if(		( (motor_frame_angle_rad[2]*RADIAN_TO_DEG > (MOTOR_Z_FRAME_ANGLE_MAX - MANUAL_ADJ_REV_ANGLE))&&(f_user_y_spd_on_motor[2] > 0) )||
////								( (motor_frame_angle_rad[2]*RADIAN_TO_DEG < (MOTOR_Z_FRAME_ANGLE_MIN + MANUAL_ADJ_REV_ANGLE))&&(f_user_y_spd_on_motor[2] < 0) )
////						)
////					{
////						f_user_y_spd = 0;
////					}
////					/*f_user_x_spdֻӰ��motor_y,���ں��*/
////					if(		( (motor_frame_angle_rad[1]*RADIAN_TO_DEG > (MOTOR_Y_FRAME_ANGLE_MAX - MANUAL_ADJ_REV_ANGLE))&&(f_user_x_spd_on_motor[1] > 0) )||
////								( (motor_frame_angle_rad[1]*RADIAN_TO_DEG < (MOTOR_Y_FRAME_ANGLE_MIN + MANUAL_ADJ_REV_ANGLE))&&(f_user_x_spd_on_motor[1] < 0) )
////						)
////					{
////						f_user_x_spd = 0;
////					}
////					break;
////					
//				default:
//					break;
//			}			
//		}
			
		
		/*���ֻ�����µ�λ�òο�*/
		f_user_spd_ctrl_period = Get_Function_Period(&ul_last_user_spd_ctrl_micros);
		
		switch(en_gimbal_work_mode)
		{
		
			/*pitch �� yaw�ɿ�*/
			case NONE_FOLLOW:		
				f_earth_x_pos_pid_ref_shadow += f_user_x_spd*f_user_spd_ctrl_period*f_pitch_user_ctrl_spd_ratio;   //test
				//f_earth_z_pos_pid_ref_shadow += f_user_x_spd*f_user_spd_ctrl_period*f_yaw_user_ctrl_spd_ratio;		//test
				break;
			
//			/*pitch �� roll�ɿ�*/
//			case Z_FOLLOW:
//			case Z_FOLLOW_TO_NONE_FOLLOW:					
//				f_earth_x_pos_pid_ref_shadow += f_user_y_spd*f_user_spd_ctrl_period*f_pitch_user_ctrl_spd_ratio;		//test
//				/*��֤ҡ������ʱ���������ת*/
//				f_earth_y_pos_pid_ref_shadow += -f_user_x_spd*f_user_spd_ctrl_period;								
//				break;
//			
//			/*pitch_follow�ɿ�*/
//			case	X_Z_FOLLOW:
//			case	X_Z_FOLLOW_TO_NONE_FOLLOW:
//			case	X_Z_FOLLOW_TO_Z_FOLLOW:		
//			f_pitch_follow_pos_pid_ref_shadow += f_user_y_spd*f_user_spd_ctrl_period*f_pitch_user_ctrl_spd_ratio;		//test
////				/*����ģʽ�¶�pitch_follow�ο������޷�*/
////				if(FOLLOW_MOTOR_Z == en_pitch_follow_motor)
////				{
////					Cal_Pitch_Follow_Pos_Pid_Ref_Range(f_euler_array,motor_frame_angle_rad,&f_pitch_follow_pos_pid_ref_shadow_min,&f_pitch_follow_pos_pid_ref_shadow_max);
////					BondFloat(&f_pitch_follow_pos_pid_ref_shadow,f_pitch_follow_pos_pid_ref_shadow_min,f_pitch_follow_pos_pid_ref_shadow_max);
////				}
//				break;
			
			default:
				break;
		}
		
		/*�ֶ��趨λ�û��ο�ֵ����*/
//		if( (HORIZONTAL == en_camera_viewer_mode)&&(SLEEP == en_gimbal_power_mode) )  //NORMAL
//		{
//			if(SET == en_manual_turn_motor_x_flag)
//			{
//				f_earth_x_pos_pid_ref_shadow = f_euler_array[0];
//				f_pitch_follow_pos_pid_ref_shadow = motor_frame_angle_rad[0]*RADIAN_TO_DEG;				
//			}

////			if( (SET == en_manual_turn_motor_y_flag)&&(FOLLOW_MOTOR_Y == en_yaw_follow_motor) )
////			{
////				f_earth_z_pos_pid_ref_shadow = f_euler_array[2];
////				f_yaw_follow_pos_pid_ref_shadow = motor_frame_angle_rad[1]*RADIAN_TO_DEG;
////			}
////			
////			/*yaw�涯�������ʱ��λ�òο�Ϊ0*/
////			if(FOLLOW_MOTOR_Z == en_yaw_follow_motor)
////			{
////				f_yaw_follow_pos_pid_ref_shadow = 0;
////			}
//		}
		
		
		/*����λ����*/
//		Process_Motor_Limit_Event(f_euler_array, motor_frame_angle_rad, coordinate_transform_matrix);
//			
//		
//		/*����������*/
//		if( (RESET == en_last_tripus_mode_flag)&&(SET == en_tripus_mode_flag) )
//		{
//			f_yaw_follow_pos_pid_ref_shadow = motor_frame_angle_rad[en_yaw_follow_motor]*RADIAN_TO_DEG;
//		}
//		else if( SET == en_tripus_mode_flag )
//		{
//			f_earth_z_pos_pid_ref_shadow = f_euler_array[2];
//		}
//		en_last_tripus_mode_flag = en_tripus_mode_flag;
		
		
		
		earth_x_pos_pid_ref = f_earth_x_pos_pid_ref_shadow*80;
//		earth_y_pos_pid_ref = f_earth_y_pos_pid_ref_shadow*80;
//		earth_z_pos_pid_ref = f_earth_z_pos_pid_ref_shadow*80;
		sw_pitch_follow_pos_pid_ref = f_pitch_follow_pos_pid_ref_shadow;  //����λΪ��
		sw_yaw_follow_pos_pid_ref = f_yaw_follow_pos_pid_ref_shadow;			//����λΪ��
	}
}



/**
	* @brief��void	Process_Manual_Turn_Motor_Event(void)
	* @note�� �ֶ�ת�����ʱ�䴦���ñ�־λ�������ж��Ƿ���Ҫ�ֶ�����λ�û��ο�
	*	@note��	�ֶ��趨�Ƕ�û����λֵ����Ҫ����Process_User_Spd_Ctrl()֮ǰ
	*/
//uint16_t		error_thr_test = 100,		error_thr_test1 = 20;				//test
void	Process_Manual_Turn_Motor_Event(void)
{	
	static	User_Timer_Typedef		st_motor_spd_pid_error_watch_window_timer_p = USER_TIMER_INIT_VALUE,
																st_motor_x_spd_pid_error_timer_p = USER_TIMER_INIT_VALUE,
																st_motor_y_spd_pid_error_timer_p = USER_TIMER_INIT_VALUE,
																st_motor_z_spd_pid_error_timer_p = USER_TIMER_INIT_VALUE,
																st_motor_x_spd_pid_normal_timer_p = USER_TIMER_INIT_VALUE,
																st_motor_y_spd_pid_normal_timer_p = USER_TIMER_INIT_VALUE,
																st_motor_z_spd_pid_normal_timer_p = USER_TIMER_INIT_VALUE;
	/*--------------------------manual_turn_motor_x_event----------------------------------*/
	Start_User_Timer(&st_motor_spd_pid_error_watch_window_timer_p);
	
	Start_User_Timer(&st_motor_x_spd_pid_error_timer_p);
	Start_User_Timer(&st_motor_y_spd_pid_error_timer_p);
	Start_User_Timer(&st_motor_z_spd_pid_error_timer_p);
	
	Start_User_Timer(&st_motor_x_spd_pid_normal_timer_p);
	Start_User_Timer(&st_motor_y_spd_pid_normal_timer_p);
	Start_User_Timer(&st_motor_z_spd_pid_normal_timer_p);
	
	Update_User_Timer_Cnt(&st_motor_spd_pid_error_watch_window_timer_p);
	Update_User_Timer_Cnt(&st_motor_x_spd_pid_error_timer_p);
	Update_User_Timer_Cnt(&st_motor_y_spd_pid_error_timer_p);
	Update_User_Timer_Cnt(&st_motor_z_spd_pid_error_timer_p);
	
	Update_User_Timer_Cnt(&st_motor_x_spd_pid_normal_timer_p);
	Update_User_Timer_Cnt(&st_motor_y_spd_pid_normal_timer_p);
	Update_User_Timer_Cnt(&st_motor_z_spd_pid_normal_timer_p);
	
	if(st_motor_spd_pid_error_watch_window_timer_p.ul_timer_cnt < 300)
	{
		/*--------------------------manual_turn_motor_x_event----------------------------------*/	
		if( fabs(motor_x_spd_pid_ref - motor_x_spd_pid_fdb) > 100)
		{
			Resume_User_Timer(&st_motor_x_spd_pid_error_timer_p);	
		}
		else
		{
			Pause_User_Timer(&st_motor_x_spd_pid_error_timer_p);
		}
		if(st_motor_x_spd_pid_error_timer_p.ul_timer_cnt > 200)
		{
			en_manual_turn_motor_x_flag = SET;	
		}
		
		if( fabs(motor_x_spd_pid_ref - motor_x_spd_pid_fdb) <= 20)
		{
			Resume_User_Timer(&st_motor_x_spd_pid_normal_timer_p);	
		}
		else
		{
			Pause_User_Timer(&st_motor_x_spd_pid_normal_timer_p);
		}
		if(st_motor_x_spd_pid_normal_timer_p.ul_timer_cnt > 200)
		{
			en_manual_turn_motor_x_flag = RESET;	
		}	
		
		
		/*--------------------------manual_turn_motor_y_event----------------------------------*/	
		if( fabs(motor_y_spd_pid_ref - motor_y_spd_pid_fdb) > 100)
		{
			Resume_User_Timer(&st_motor_y_spd_pid_error_timer_p);	
		}
		else
		{
			Pause_User_Timer(&st_motor_y_spd_pid_error_timer_p);
		}
		if(st_motor_y_spd_pid_error_timer_p.ul_timer_cnt > 200)
		{
			en_manual_turn_motor_y_flag = SET;	
		}
			
		if( fabs(motor_y_spd_pid_ref - motor_y_spd_pid_fdb) <= 20)
		{
			Resume_User_Timer(&st_motor_y_spd_pid_normal_timer_p);	
		}
		else
		{
			Pause_User_Timer(&st_motor_y_spd_pid_normal_timer_p);
		}
		if(st_motor_y_spd_pid_normal_timer_p.ul_timer_cnt > 200)
		{
			en_manual_turn_motor_y_flag = RESET;	
		}
		
		
		/*--------------------------manual_turn_motor_z_event----------------------------------*/	
		if( fabs(motor_z_spd_pid_ref - motor_z_spd_pid_fdb) > 100)
		{
			Resume_User_Timer(&st_motor_z_spd_pid_error_timer_p);	
		}
		else
		{
			Pause_User_Timer(&st_motor_z_spd_pid_error_timer_p);
		}
		if(st_motor_z_spd_pid_error_timer_p.ul_timer_cnt > 200)
		{
			en_manual_turn_motor_z_flag = SET;	
		}
			
		if( fabs(motor_z_spd_pid_ref - motor_z_spd_pid_fdb) <= 20)
		{
			Resume_User_Timer(&st_motor_z_spd_pid_normal_timer_p);	
		}
		else
		{
			Pause_User_Timer(&st_motor_z_spd_pid_normal_timer_p);
		}
		if(st_motor_z_spd_pid_normal_timer_p.ul_timer_cnt > 200)
		{
			en_manual_turn_motor_z_flag = RESET;	
		}	
	}
	else
	{	
		Reset_User_Timer(&st_motor_spd_pid_error_watch_window_timer_p);
		
		Reset_User_Timer(&st_motor_x_spd_pid_error_timer_p);
		Reset_User_Timer(&st_motor_y_spd_pid_error_timer_p);
		Reset_User_Timer(&st_motor_z_spd_pid_error_timer_p);
		
		Reset_User_Timer(&st_motor_x_spd_pid_normal_timer_p);
		Reset_User_Timer(&st_motor_y_spd_pid_normal_timer_p);
		Reset_User_Timer(&st_motor_z_spd_pid_normal_timer_p);
	}
}


/* External functions ------------------------------------------------------------*/
/**
	* @brief��void Earth_X_Pos_Pid_Updata(float	euler_pitch_p)
	* @note�� X��λ�û�����
	*	@param��float	euler_pitch_p��euler_pitch
	*/
void	Task_Procese_Ui(void)
{
	static	uint8_t uc_ui_last_data_p;
	uint8_t	uc_ui_new_data_p,	uc_ui_new_xor_last_data_p;
	
	static	User_Timer_Typedef	st_this_function_period_timer_p = USER_TIMER_INIT_VALUE;
	
	Start_User_Timer(&st_this_function_period_timer_p);
	
	Update_User_Timer_Cnt(&st_this_function_period_timer_p);
	
	if(st_this_function_period_timer_p.ul_timer_cnt > UI_UPDATA_PERIOD_MS)   //5ms
	{	
		Reset_User_Timer(&st_this_function_period_timer_p);
		
		uc_ui_new_data_p = uc_receive_ui_mode_data;
		
		/*if new user action is coming*/
		uc_ui_new_xor_last_data_p = (uc_ui_new_data_p^uc_ui_last_data_p);
		uc_ui_last_data_p = uc_ui_new_data_p;
			
		uc_ui_new_xor_last_data_p = 0xFF; 			//test
		
		if(uc_ui_new_xor_last_data_p)
		{
			/*����en_gimbal_power_modeģʽ�л�*/
			if(0x00 != (uc_ui_new_xor_last_data_p&0x03) )
			{				
				Process_Gimbal_Power_Mode_State_Change(&uc_receive_ui_mode_data);			
			}
			
			/*����en_camera_viewer_modeģʽ�л�*/
			if(0x00 != (uc_ui_new_xor_last_data_p&0x40))
			{
				Process_Camera_Viewer_Mode_State_Change(&uc_receive_ui_mode_data);
			}
			
			/*����en_gimbal_work_modeģʽ�л�*/
			if(0x00 != (uc_ui_new_xor_last_data_p&0x3C))
			{
				Process_Gimbal_Work_Mode_State_Change(&uc_receive_ui_mode_data);
			}
		}
		
		/*����ģʽ�л��󣬸����ƻ�·�ο��ĳ�ʼ��*/
		Process_State_Change_Action();
		
		/*�ֶ�ת������¼�����*/
		Process_Manual_Turn_Motor_Event();
		
		/*�����û��ٶȿ���*/
		Process_User_Spd_Ctrl(uc_receive_ui_spd_data);
	}
}





