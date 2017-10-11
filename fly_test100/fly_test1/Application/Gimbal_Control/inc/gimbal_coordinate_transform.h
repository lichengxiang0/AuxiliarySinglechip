/**
  ******************************************************************************
  * File Name          :gimbal_coordinate_transform.c
  * Description        :×ø±ê±ä»»
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
#ifndef	_GIMBAL_COORDINATE_TRANSFORM_H
#define	_GIMBAL_COORDINATE_TRANSFORM_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"


/* Exported typedef ---------------------------------------------------------*/
typedef	enum{X_RIGHT = 0, X_LEFT,	X_FRONT,	X_BACK,	X_UP,	X_DOWN}Coordinate_Typedef;



/* Exported privates ---------------------------------------------------------*/
extern	float	*coordinate_transform_matrix[6];


/* Exported function ---------------------------------------------------------*/
void	Coordinate_Transform(float	*motor_angle_p,	float	*quad_p,	Coordinate_Typedef en_coordinate_dir_p);

#ifdef __cplusplus
}
#endif	 
	 
#endif	


