#ifndef	_MC_MOTOR_POS_H
#define	_MC_MOTOR_POS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

	 
/* Exported types ------------------------------------------------------------*/
/**
  * @brief  Motor pos data_struct 
  */	 
typedef	struct
{
	uint16_t	uw_motor_machine_pos;
	/*电机电角度*/
	uint16_t	uw_motor_elec_zero_pos;
	int16_t		sw_motor_elec_angle;
	int8_t		sc_motor_elec_angle_dir;
	/*电机框架角度*/
	uint16_t	uw_motor_frame_zero_pos;
	int16_t		sc_motor_frame_angle;
}Motor_Pos_Info_Typedef;

	 
/* Exported define -----------------------------------------------------------*/	 

/* define motor_pos_sensor data length 	 
#define		MOTOR_POS_SENSOR_DATA_LENGTH_8		((uint8_t)0x08)
#define		MOTOR_POS_SENSOR_DATA_LENGTH_9		((uint8_t)0x07)	
#define		MOTOR_POS_SENSOR_DATA_LENGTH_10		((uint8_t)0x06)
#define		MOTOR_POS_SENSOR_DATA_LENGTH_11		((uint8_t)0x05)	
#define		MOTOR_POS_SENSOR_DATA_LENGTH_12		((uint8_t)0x04)
#define		MOTOR_POS_SENSOR_DATA_LENGTH_13		((uint8_t)0x03)	
#define		MOTOR_POS_SENSOR_DATA_LENGTH_14		((uint8_t)0x02)
#define		MOTOR_POS_SENSOR_DATA_LENGTH_15		((uint8_t)0x01)	
#define		MOTOR_POS_SENSOR_DATA_LENGTH_16		((uint8_t)0x00)		 
*/
	 

/* External variables --------------------------------------------------------*/

/* 电角度零位和电角度方向 */	 
//extern	uint16_t	uw_electrical_zero_angle;             
//extern	int8_t		sc_electrical_angel_direction;

///* 电机角度传感器的值 */
//extern	uint16_t	uw_motor_machanical_angle;
extern	Motor_Pos_Info_Typedef	st_motor_pos_info;

/* Exported function ---------------------------------------------------------*/
int16_t POS_GetElectricalAngle(void);
void		Cal_Motor_Frame_Angle(void);


#ifdef __cplusplus
}
#endif

#endif
