/**
  ******************************************************************************
  * File Name          : gimbal_ahrs.h
  * Description        : Description        : This file provides code for gimbal attitude .
  * ����ϵ����
	* ����NORTH  �ϣ�SOUTH	����EAST		����WEST  �죺UP	�أ�DOWN
	* �����죺ENU		�����죺NWU
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
#ifndef	_GIMBAL_AHRS_H
#define	_GIMBAL_AHRS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
	 
/* Exported defines --------------------------------------------------------*/
//#define	GIMBAL_GYRO_X_DATA_INDEX		((uint8_t)2)
//#define	GIMBAL_GYRO_Y_DATA_INDEX		((uint8_t)0)
//#define	GIMBAL_GYRO_Z_DATA_INDEX		((uint8_t)1)	 

//#define	GIMBAL_ACC_X_DATA_INDEX			((uint8_t)2)
//#define	GIMBAL_ACC_Y_DATA_INDEX			((uint8_t)0)
//#define	GIMBAL_ACC_Z_DATA_INDEX			((uint8_t)1)	
	 
/*icm20602*/
#define	GIMBAL_GYRO_X_DATA_INDEX		((uint8_t)0)
#define	GIMBAL_GYRO_Y_DATA_INDEX		((uint8_t)1)
#define	GIMBAL_GYRO_Z_DATA_INDEX		((uint8_t)2)	 
/*mma8452 ps test*/
#define	GIMBAL_ACC_X_DATA_INDEX			((uint8_t)0)
#define	GIMBAL_ACC_Y_DATA_INDEX			((uint8_t)1)
#define	GIMBAL_ACC_Z_DATA_INDEX			((uint8_t)2)	
	 

/* Exported variables --------------------------------------------------------*/
extern	float	f_sensor_gyro_raw_data_array[3],	f_sensor_gyro_flt_data_array[3],	f_sensor_gyro_offset_data_array[3];
extern	float	f_sensor_acc_raw_data_array[3],		f_sensor_acc_flt_data_array[3],		f_sensor_acc_offset_data_array[3],	f_sensor_acc_scale_data_array[3] ;

extern	float	f_gimbal_gyro_data_array[3];		
extern	float	f_gimbal_acc_data_array[3];

extern	float	f_quat_x_right_array[4],	f_quat_x_front_array[4],	f_quat_array[4];
extern	float	f_euler_x_right_array[3],	f_euler_x_front_array[3],	f_euler_array[3];

extern	FlagStatus	en_ahrs_init_flag;
	 
	 
void	Cal_Filter_Sensor_Data(void);
void	Gimbal_Ahrs_Init(void);
void	Gimbal_Ahrs_Update(float	dt);



#ifdef __cplusplus
}
#endif


#endif
