/**
  ******************************************************************************
  * File Name          :sys_config.h
  * Description        :系统配置文件
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
#ifndef	_SYS_CONFIG_H
#define	_SYS_CONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

	 
/* Exported typedef ---------------------------------------------------------*/
typedef struct 
{  
  uint16_t   	first_program_flag;			
	uint8_t	   	pcb_calss;
	uint8_t	   	pcb_sn;
} Firmware_Config_Data_Struct_Typedef;


typedef	enum{BOARD_ID0 = 0,	BOARD_ID1,	BOARD_ID2,	BOARD_ID3,	BOARD_ID4}Board_Id_Typedef;

/* Exported variables ------------------------------------------------------------*/
extern	Board_Id_Typedef		en_board_id;

extern	const	char uc_firmware_version_array[10];

extern	Firmware_Config_Data_Struct_Typedef	st_firmware_config_data;

/* Exported function --------------------------------------------------------*/	
void	Get_Firmware_Config_Data(void);
void	Save_Firmware_Config_Data(void);

	
#ifdef __cplusplus
}
#endif


#endif
