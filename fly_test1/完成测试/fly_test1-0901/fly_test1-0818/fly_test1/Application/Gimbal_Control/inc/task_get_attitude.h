/**
  ******************************************************************************
  * File Name          : task_get_attitude.h
  * Description        : 这个文件主要实现，1->gyro/acc传感器数据读取;2->X_Right/X_Front
													坐标系下姿态and最终输出的姿态和使用的坐标系;3->坐标变换矩阵
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef	_TASK_GET_ATTITUDE_H
#define	_TASK_GET_ATTITUDE_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "gimbal_coordinate_transform.h"
	 
/* Exported variables ------------------------------------------------------------*/
extern	Coordinate_Typedef	en_gimbal_coordinate;
extern	uint8_t		uc_recive_motor_frame_angle[6];	 
	 
/* Exported functions ------------------------------------------------------------*/
void	Task_Get_Imu_Sensor_Data(void);
void	Task_Get_Imu_Attitude(void);	 
void	Task_Get_Coordinate_Transform_Matrix(void);



#ifdef __cplusplus
}
#endif


#endif

