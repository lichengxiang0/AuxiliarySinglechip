/**
  ******************************************************************************
  * File Name          :gimbal_attitude_ctrl.c
  * Description        :��̨��̬����
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
#include "gimbal_attitude_ctrl.h"

/* External variables ------------------------------------------------------------*/
PID_Struct_Typedef	st_earth_x_pos_pid_struct,st_earth_y_pos_pid_struct,st_earth_z_pos_pid_struct;
int16_t	earth_x_pos_pid_ref,			earth_y_pos_pid_ref,			earth_z_pos_pid_ref;
int16_t	earth_x_pos_pid_fdb,			earth_y_pos_pid_fdb,			earth_z_pos_pid_fdb;
int16_t	earth_x_pos_pid_out,			earth_y_pos_pid_out,			earth_z_pos_pid_out;

/* Private functions ------------------------------------------------------------*/
/**
	* @brief���������·��
	* @note
	*	@param  float	ref��fdb���ο��ͷ���
	*/
float	Cal_Short_Route(float	ref,	float fdb)
{
	if( (ref - fdb) > 180*80 )
	{
		fdb += 360*80;
	}
	else if( (ref - fdb) < -180*80 )
	{
		fdb -= 360*80;
	}
	
	return(fdb);
}




/* External functions ------------------------------------------------------------*/
/**
	* @brief��void Earth_X_Pos_Pid_Updata(float	euler_pitch_p)
	* @note�� X��λ�û�����
	*	@param��float	euler_pitch_p��euler_pitch	������
	*/
void Earth_X_Pos_Pid_Updata(float	euler_pitch_p)
{
	earth_x_pos_pid_fdb = (int16_t)(euler_pitch_p*80);
	
	/*�������·��*/
	earth_x_pos_pid_fdb = Cal_Short_Route(earth_x_pos_pid_ref,earth_x_pos_pid_fdb);
	
	earth_x_pos_pid_out = PI_Regulator(earth_x_pos_pid_ref,earth_x_pos_pid_fdb,&st_earth_x_pos_pid_struct);
}	



/**
	* @brief��void Earth_Y_Pos_Pid_Updata(float	euler_pitch_p)
	* @note�� Y��λ�û�����
	*	@param��float	euler_roll_p�������
	*/
void Earth_Y_Pos_Pid_Updata(float	euler_roll_p)
{
	earth_y_pos_pid_fdb = (int16_t)(euler_roll_p*80);
	
	/*�������·��*/
	earth_y_pos_pid_fdb = Cal_Short_Route(earth_y_pos_pid_ref,earth_y_pos_pid_fdb);
	
	earth_y_pos_pid_out = PI_Regulator(earth_y_pos_pid_ref,earth_y_pos_pid_fdb,&st_earth_y_pos_pid_struct);
}




/**
	* @brief��void Earth_Z_Pos_Pid_Updata(float	euler_pitch_p)
	* @note�� Z��λ�û�����
	*	@param��float	euler_yaw_p����λ��
	*/
void Earth_Z_Pos_Pid_Updata(float	euler_yaw_p)
{
	earth_z_pos_pid_fdb = (int16_t)(euler_yaw_p*80);
	
		/*�������·��*/
	earth_z_pos_pid_fdb = Cal_Short_Route(earth_z_pos_pid_ref,earth_z_pos_pid_fdb);
	

	earth_z_pos_pid_out = PI_Regulator(earth_z_pos_pid_ref,earth_z_pos_pid_fdb,&st_earth_z_pos_pid_struct);
}





