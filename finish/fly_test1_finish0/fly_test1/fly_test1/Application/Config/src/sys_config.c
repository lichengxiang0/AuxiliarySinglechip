/**
  ******************************************************************************
  * File Name          :sys_config.c
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



/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "sys_config.h"
#include "stm32_bsp_flash.h"


/* Private define ------------------------------------------------------------*/
#define		FIRMWARE_CONFIG_DATA_ADDRESS			(ADDR_FLASH_PAGE_126)	

/* External variables ------------------------------------------------------------*/
Board_Id_Typedef		en_board_id = BOARD_ID3;

const	char uc_firmware_version_array[10] = {"V1.0.0"};

Firmware_Config_Data_Struct_Typedef	st_firmware_config_data = {	0x0000,
																																0x02,
																																0x03,
																															};

																																																		
/* External function ------------------------------------------------------------*/																									
/**
  * @brief 	void Get_Firmware_Config_Data(void)
	* @note	 	读取固件信息
  */																									
void	Get_Firmware_Config_Data(void)
{
	FlashRead( FIRMWARE_CONFIG_DATA_ADDRESS,(&st_firmware_config_data), sizeof(st_firmware_config_data) );
	
	if(0x55AA == st_firmware_config_data.first_program_flag)
	{
		en_board_id = (Board_Id_Typedef)st_firmware_config_data.pcb_sn;
	}
}	
																									
/**
  * @brief 	void Save_Firmware_Config_Data(void)
	* @note	 	保存固件信息
  */
void	Save_Firmware_Config_Data(void)
{
	/*st_firmware_config_data中的内容除了st_firmware_config_data.first_program_flag都是只读的*/
	FlashProgram( FIRMWARE_CONFIG_DATA_ADDRESS,(&st_firmware_config_data), sizeof(st_firmware_config_data) );
}	

