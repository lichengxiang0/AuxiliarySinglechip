/**
  ******************************************************************************
  * File Name          : task_get_attitude.c
  * Description        : 这个文件主要实现，1->gyro/acc传感器数据读取;2->X_Right/X_Front
													坐标系下姿态;3->最终输出的姿态和使用的坐标系;4->坐标变换矩阵
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
uint8_t		uc_recive_motor_frame_angle[6]; 				//data[0-6]：电机123


//extern	I2C_HandleTypeDef hi2c2;


/* Exported function --------------------------------------------------------*/
/**
  * @brief 	void	Task_Init_Imu_Sensor(void)
	* @note	 	初始化IMU
	
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
	* @note	 	读取IMU传感器数据
  */
void	Task_Get_Imu_Sensor_Data(void)
{
	float	f_icm20602_gyro_data_array_p[3], 	f_icm20602_acc_data_array_p[3];
	float	f_mma8452_acc_data_array_p[3];
	
//	Read_Mma8452_Acc_Data(&ACC_I2C_HANDLE,f_sensor_acc_raw_data_array);
//	Read_Icm20602_Data(&GYRO_I2C_HANDLE,f_sensor_gyro_raw_data_array,f_sensor_acc_raw_data_array);	
	
	Read_Icm20602_Data_Soft(f_sensor_gyro_raw_data_array,f_sensor_acc_raw_data_array);	//f_icm20602_acc_data_array_p); //六轴陀螺仪数据采集
	Read_Mma8452_Acc_Data_Soft(f_sensor_acc_raw_data_array);        									 //三轴加速度数据采集
	
	Cal_Filter_Sensor_Data();																				//窗口滑动滤波
}

/**
  * @brief 	void	Task_Get_Imu_Sensor_Data(void)
	* @note	 	获取IMU姿态，坐标系
  */
float		f_gimbal_ahrs_update_period;

void	Task_Get_Imu_Attitude(void)
{
	static	FlagStatus	en_first_run_this_task_flag = SET;
	
	/*计算ahrs_update的更新周期*/
	static	uint32_t		ul_gimbal_ahrs_update_sys_micros;
	//	
	static	User_Timer_Typedef	st_ahrs_init_period_timer_p = USER_TIMER_INIT_VALUE;
	
	/*第一次执行此函数，利用加速度初始化姿态,姿态更新*/
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
			/*暂时这种写法，也可以画两秒时间增加kp=5，来使四元数快速收敛*/
			en_ahrs_init_flag = SET;
			Stop_User_Timer(&st_ahrs_init_period_timer_p);
		}
		//计算函数执行周期
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
	* @note	 	获取坐标转换矩阵(EARTH到X_RIGHT坐标系的)
  */
void	Task_Get_Coordinate_Transform_Matrix(void)
{
	/*获取电机框架角度也可以在串口接受中进行*/
	Get_Motor_Frame_Angle(uc_recive_motor_frame_angle);
	/*根据当前四元数/坐标系/电机框架角计算各旋转矩阵*/
	Coordinate_Transform(motor_frame_angle_rad,f_quat_array,en_gimbal_coordinate);
}


