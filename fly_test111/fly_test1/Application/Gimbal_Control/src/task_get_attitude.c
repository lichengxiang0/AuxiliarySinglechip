/**
  ******************************************************************************
  * File Name          : task_get_attitude.c
  * Description        : ����ļ���Ҫʵ�֣�1->gyro/acc���������ݶ�ȡ;2->X_Right/X_Front
													����ϵ����̬;3->�����������̬��ʹ�õ�����ϵ;4->����任����
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
#include "task_get_attitude.h"

#include "fxas21002_driver.h"
#include "mma8452_driver.h"
#include "icm20602_driver.h"
#include "mpu6500_driver.h"

#include "gimbal_ahrs.h"
#include "gimbal_coordinate_transform.h"
#include "gimbal_frame_angle.h"

#include "stm32_bsp_timer.h"


#include "mpu6050_driver.h"
#include "soft_iic.h"

/* External variables ------------------------------------------------------------*/
Coordinate_Typedef	en_gimbal_coordinate = X_RIGHT;
uint8_t		uc_recive_motor_frame_angle[6]; 				//data[0-6]�����123


//extern	I2C_HandleTypeDef hi2c2;


/* Exported function --------------------------------------------------------*/
/**
  * @brief 	void	Task_Init_Imu_Sensor(void)
	* @note	 	��ʼ��IMU
	
void	Task_Init_Imu_Sensor(void)
{
//	Init_Mpu6000();
//	Init_Fxas21002(&GYRO_I2C_HANDLE);
//	Init_Mma8452(&ACC_I2C_HANDLE);
	
//	Init_Fxas21002_Soft();
//	Init_Mma8452_Soft();
}
*/



/**
  * @brief 	void	Task_Get_Imu_Sensor_Data(void)
	* @note	 	��ȡIMU����������
  */
void	Task_Get_Imu_Sensor_Data(void)
{
	float	f_icm20602_gyro_data_array_p[3], 	f_icm20602_acc_data_array_p[3];
	float	f_mma8452_acc_data_array_p[3];
	
//	Read_Mma8452_Acc_Data(&ACC_I2C_HANDLE,f_sensor_acc_raw_data_array);
//	Read_Icm20602_Data(&GYRO_I2C_HANDLE,f_sensor_gyro_raw_data_array,f_sensor_acc_raw_data_array);	
	
	Read_Icm20602_Data_Soft(f_sensor_gyro_raw_data_array,f_sensor_acc_raw_data_array);	//f_icm20602_acc_data_array_p); //�������������ݲɼ�
	Read_Mma8452_Acc_Data_Soft(f_sensor_acc_raw_data_array);        									 //������ٶ����ݲɼ�
	
	Cal_Filter_Sensor_Data();																				//���ڻ����˲�
}

/**
  * @brief 	void	Task_Get_Imu_Sensor_Data(void)
	* @note	 	��ȡIMU��̬������ϵ
  */
float		f_gimbal_ahrs_update_period;

void	Task_Get_Imu_Attitude(void)
{
	static	FlagStatus	en_first_run_this_task_flag = SET;
	
	/*����ahrs_update�ĸ�������*/
	static	uint32_t		ul_gimbal_ahrs_update_sys_micros;
	//	
	static	User_Timer_Typedef	st_ahrs_init_period_timer_p = USER_TIMER_INIT_VALUE;
	
	/*��һ��ִ�д˺��������ü��ٶȳ�ʼ����̬,��̬����*/
	if(SET == en_first_run_this_task_flag)
	{	
		en_first_run_this_task_flag = RESET;
//	Gimbal_Ahrs_Init();		
		en_ahrs_init_flag = RESET;
		return;
	}
	else
	{
		Start_User_Timer(&st_ahrs_init_period_timer_p);
		Update_User_Timer_Cnt(&st_ahrs_init_period_timer_p);
		if(st_ahrs_init_period_timer_p.ul_timer_cnt > 2000)
		{			
			/*��ʱ����д����Ҳ���Ի�����ʱ������kp=5����ʹ��Ԫ����������*/
			en_ahrs_init_flag = SET;
			Stop_User_Timer(&st_ahrs_init_period_timer_p);
		}
		//���㺯��ִ������
		f_gimbal_ahrs_update_period = Get_Function_Period(&ul_gimbal_ahrs_update_sys_micros);
		
		Gimbal_Ahrs_Update(f_gimbal_ahrs_update_period);
		
	}
				
	f_euler_array[0] = f_euler_x_right_array[0];
	f_euler_array[1] = f_euler_x_right_array[1];
	f_euler_array[2] = f_euler_x_right_array[2];
		
	f_quat_array[0] = f_quat_x_right_array[0];
	f_quat_array[1] = f_quat_x_right_array[1];
	f_quat_array[2] = f_quat_x_right_array[2];
	f_quat_array[3] = f_quat_x_right_array[3];
	
}


/**
  * @brief 	void	Task_Get_Coordinate_Transform_Matrix(void)
	* @note	 	��ȡ����ת������(EARTH��X_RIGHT����ϵ��)
  */
void	Task_Get_Coordinate_Transform_Matrix(void)
{
	/*��ȡ�����ܽǶ�Ҳ�����ڴ��ڽ����н���*/
	Get_Motor_Frame_Angle(uc_recive_motor_frame_angle);
	/*���ݵ�ǰ��Ԫ��/����ϵ/�����ܽǼ������ת����*/
	Coordinate_Transform(motor_frame_angle_rad,f_quat_array,en_gimbal_coordinate);
}


