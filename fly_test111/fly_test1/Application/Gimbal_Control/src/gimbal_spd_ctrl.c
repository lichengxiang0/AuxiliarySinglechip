/**
  ******************************************************************************
  * File Name          :gimbal_spd_ctrl.c
  * Description        :��̨����ٶȻ�����
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
#include "gimbal_spd_ctrl.h"
#include "gimbal_cal_spd_pid_para.h"
#include "task_gimbal_pos_ctrl.h"

/* Exported variables ------------------------------------------------------------*/
PID_Struct_Typedef	st_motor_x_spd_pid_struct,			st_motor_y_spd_pid_struct,				st_motor_z_spd_pid_struct;
int16_t							motor_x_spd_pid_ref,						motor_y_spd_pid_ref,							motor_z_spd_pid_ref;
int16_t							motor_x_spd_pid_fdb,						motor_y_spd_pid_fdb,							motor_z_spd_pid_fdb;
int16_t							motor_x_spd_pid_out,						motor_y_spd_pid_out,							motor_z_spd_pid_out;

/**
  * @brief 	void	Gimbal_Spd_To_Motor_Spd(	float	*gimbal_gyro_data_p,	float *motor_gyro_data_p,	float ** coordinate_transform_matrix_p)
	* @note	 	�������ٶ�(���Կռ�)
	*	@param  float	*gimbal_gyro_data_p����̨�ٶ�
	*	@param  float	*motor_gyro_data_p������ٶ�
	*	@param  float ** coordinate_transform_matrix_p�� ��ǰ����ϵ
  */
void	Gimbal_Spd_To_Motor_Spd(float	*gimbal_gyro_data_p,	float *motor_gyro_data_p,	float ** coordinate_transform_matrix_p)
{
	
	motor_gyro_data_p[0] = gimbal_gyro_data_p[0]*coordinate_transform_matrix_p[1][0] + gimbal_gyro_data_p[1]*coordinate_transform_matrix_p[1][1] + gimbal_gyro_data_p[2]*coordinate_transform_matrix_p[1][2];
	motor_gyro_data_p[1] = gimbal_gyro_data_p[0]*coordinate_transform_matrix_p[1][3] + gimbal_gyro_data_p[1]*coordinate_transform_matrix_p[1][4] + gimbal_gyro_data_p[2]*coordinate_transform_matrix_p[1][5];
	motor_gyro_data_p[2] = gimbal_gyro_data_p[0]*coordinate_transform_matrix_p[1][6] + gimbal_gyro_data_p[1]*coordinate_transform_matrix_p[1][7] + gimbal_gyro_data_p[2]*coordinate_transform_matrix_p[1][8];
}


/**
  * @brief 	void	Earth_Spd_To_Motor_Spd(	float	*gimbal_gyro_data_p,	float *motor_gyro_data_p,	float ** coordinate_transform_matrix_p)
	* @note	 	�������ٶ�(���Կռ�)
	*	@param  float	*earth_gyro_data_p���������ϵ�ٶ�
	*	@param  float	*motor_gyro_data_p������ٶ�
	*	@param  float ** coordinate_transform_matrix_p�� ��ǰ����ϵ
*/
void	Earth_Spd_To_Motor_Spd(float	*earth_gyro_data_p,	float *motor_gyro_data_p,	float ** coordinate_transform_matrix_p)
{
	float	gimbal_gyro_data_p[3];
	
	gimbal_gyro_data_p[0] = coordinate_transform_matrix_p[2][0]*earth_gyro_data_p[0] + coordinate_transform_matrix_p[2][1]*earth_gyro_data_p[1] + coordinate_transform_matrix_p[2][2]*earth_gyro_data_p[2];
	gimbal_gyro_data_p[1] = coordinate_transform_matrix_p[2][3]*earth_gyro_data_p[0] + coordinate_transform_matrix_p[2][4]*earth_gyro_data_p[1] + coordinate_transform_matrix_p[2][5]*earth_gyro_data_p[2];
	gimbal_gyro_data_p[2] = coordinate_transform_matrix_p[2][6]*earth_gyro_data_p[0] + coordinate_transform_matrix_p[2][7]*earth_gyro_data_p[1] + coordinate_transform_matrix_p[2][8]*earth_gyro_data_p[2];
	
	Gimbal_Spd_To_Motor_Spd(gimbal_gyro_data_p,motor_gyro_data_p,coordinate_transform_matrix_p);
}
/*
  * @brief 	void	Motor_Spd_Pid_Updata(float *gimbal_gyro_data_p,	float ** coordinate_transform_matrix_p)
	* @note	 	����ٶȻ�����
	*	@param  float *gimbal_gyro_data_p����̨�ٶ�
	*	@param  float ** coordinate_transform_matrix_p�� ��ǰ����ϵ
*/
//test
float	motor_x_torque_coefficient = 1,		motor_y_torque_coefficient = 0.5,		motor_z_torque_coefficient = 1; //��������������y.z����0.4

void	Motor_Spd_Pid_Updata(float *gimbal_gyro_data_p,	float ** coordinate_transform_matrix_p)
{
	float	f_motor_gyro_data_p[3];
	
	//��ȡ����ٶȷ���
	Gimbal_Spd_To_Motor_Spd(gimbal_gyro_data_p,f_motor_gyro_data_p,coordinate_transform_matrix_p); 
	
	//��̬�������ٶ�PID����	
	//Cal_Motor_Spd_Pid_Para(&st_motor_y_spd_pid_struct,&st_motor_z_spd_pid_struct,coordinate_transform_matrix_p);
	
	//motor_x_spd_pid_fdb = (int16_t)(gimbal_gyro_data_p[0]*10);
	
	//motor_x_spd_pid_ref = (int16_t)(f_earth_pos_pid_out[0]*10);
	
	//motor_x_spd_pid_out = PI_Regulator(motor_x_spd_pid_ref,motor_x_spd_pid_fdb,&st_motor_x_spd_pid_struct)/10;
	
	//motor_x_spd_pid_out = motor_x_spd_pid_out*(1 - motor_x_torque_coefficient) - 
	//											 motor_x_torque_coefficient*PI_Regulator(motor_x_spd_pid_ref,motor_x_spd_pid_fdb,&st_motor_x_spd_pid_struct)/10;
	
	motor_y_spd_pid_fdb = (int16_t)(f_motor_gyro_data_p[1]*10);	
	
	motor_y_spd_pid_ref = (int16_t)(f_earth_pos_pid_out[1]*10);
	
	motor_y_spd_pid_out = motor_y_spd_pid_out*(1 - motor_y_torque_coefficient) + 
														 motor_y_torque_coefficient*PI_Regulator(motor_y_spd_pid_ref,motor_y_spd_pid_fdb,&st_motor_y_spd_pid_struct)/10;		
	
//	motor_z_spd_pid_fdb = (int16_t)(gimbal_gyro_data_p[0]*10);
//	motor_z_spd_pid_ref = (int16_t)(f_earth_pos_pid_out[0]*10);
//	
//	motor_z_spd_pid_out = motor_z_spd_pid_out*(1 - motor_z_torque_coefficient) - 
//														 motor_z_torque_coefficient*PI_Regulator(motor_z_spd_pid_ref,motor_z_spd_pid_fdb,&st_motor_z_spd_pid_struct)/10;
}
