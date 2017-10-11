/**
  ******************************************************************************
  * File Name          : MC_Motor_Pos.c
  * Description        : This file provides code to get the PMSM_Motor pos and spd .
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
#include "MC_Motor_Pos.h"

/* Private define ------------------------------------------------------------*/
#define	POLE_PAIR_NUM 	((uint8_t)10)		//(俯仰电机)   //2    /* Number of motor pole pairs */

/* Private variables ---------------------------------------------------------*/


/* External variables --------------------------------------------------------*/

/*
typedef	struct
{
	uint16_t	uw_motor_machine_pos;

	uint16_t	uw_motor_elec_zero_pos;
	int16_t		sw_motor_elec_angle;
	int8_t		sc_motor_elec_angle_dir;

	uint16_t	uw_motor_frame_zero_pos;
	int16_t		sc_motor_frame_angle;
}Motor_Pos_Info_Typedef;
*/


/*en_frame_motor_id = FRAME_MOTOR_Y;*/
/*FRAME_MOTOR_Y 0x01EA 0x0995*/
/*FRAME_MOTOR_Z 0x0A56 0x05E6*/
/*FRAME_MOTOR_X 0x0285 0x03E9*/
//test
Motor_Pos_Info_Typedef	st_motor_pos_info = { 0x0000,				/*uw_motor_machine_pos*/
	
																							0x05E7,				/*uw_motor_elec_zero_pos*/
																							0x0000,				/*sw_motor_elec_angle*/
																							0x01,					/*sc_motor_elec_angle_dir*/
	
																							0x0131,				/*uw_motor_frame_zero_pos*/
																							0x0000				/*sc_motor_frame_angle*/
																						};
																									


/* Exported function --------------------------------------------------------*/
/**
  * @brief 	POS_GetElectricalAngle angle
	* @note	 	cal motor electrical angle
	* @param 	none
  */
int16_t POS_GetElectricalAngle(void)
{
		int32_t  			q24_FluxAngle1;
		
		/* cal machinichal angle */
		q24_FluxAngle1 = (int32_t)(st_motor_pos_info.uw_motor_machine_pos - st_motor_pos_info.uw_motor_elec_zero_pos)<<4;
		
		/* cal electrical angle */
		q24_FluxAngle1 = 	q24_FluxAngle1*POLE_PAIR_NUM;
		
		/* limit q24_FluxAngle1 to -32768～32768 */
    if (q24_FluxAngle1 < 0)
    {
        while (q24_FluxAngle1 < 0)
        {
            q24_FluxAngle1 += 65536;
        }
    }
    else
    {
        while (q24_FluxAngle1 >= 65536)
        {
            q24_FluxAngle1 -= 65536;
        }
    }
		
		/* return electrical angle */
		st_motor_pos_info.sw_motor_elec_angle = (int16_t)q24_FluxAngle1*(st_motor_pos_info.sc_motor_elec_angle_dir);
		
		return(st_motor_pos_info.sw_motor_elec_angle);
} 
/**
  * @brief 	void	Cal_Motor_Frame_Angle(void)
	* @note	 	计算电机的框架角
  */
void	Cal_Motor_Frame_Angle(void)
{
	if(st_motor_pos_info.uw_motor_frame_zero_pos <= 0x7FF)
	{
		if( (st_motor_pos_info.uw_motor_machine_pos > (st_motor_pos_info.uw_motor_frame_zero_pos + 0x7FF))&&(st_motor_pos_info.uw_motor_machine_pos <= 0xFFF))
		{
			st_motor_pos_info.sc_motor_frame_angle = ( st_motor_pos_info.uw_motor_machine_pos - (st_motor_pos_info.uw_motor_frame_zero_pos + 0xFFF) ) - 1;
		}
		else
		{
			st_motor_pos_info.sc_motor_frame_angle = st_motor_pos_info.uw_motor_machine_pos - st_motor_pos_info.uw_motor_frame_zero_pos ;
		}
	}
	else
	{
		if( (st_motor_pos_info.uw_motor_machine_pos >= 0 )&&(st_motor_pos_info.uw_motor_machine_pos < (st_motor_pos_info.uw_motor_frame_zero_pos - 0x800)) )
		{
			st_motor_pos_info.sc_motor_frame_angle = st_motor_pos_info.uw_motor_machine_pos + 0xFFF - st_motor_pos_info.uw_motor_frame_zero_pos + 1;
		}
		else
		{
			st_motor_pos_info.sc_motor_frame_angle = st_motor_pos_info.uw_motor_machine_pos - st_motor_pos_info.uw_motor_frame_zero_pos ;
		}
	}
}
