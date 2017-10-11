/**
  ******************************************************************************
  * File Name          :board_motor_cmd_process.h
  * Description        :处理电机板通信协议
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
#ifndef	_BOARD_GIMBAL_CMD_PROCESS_H
#define	_BOARD_GIMBAL_CMD_PROCESS_H


#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f1xx_hal.h"
#include "stm32_bsp_uart.h" 

//extern uint8_t uc_Pack_And_Send_Buf[10];
	 
/* Exported functions ------------------------------------------------------------*/ 
void	Board_Gimbal_Cmd_Trans_Process(USART_Buf_TypeDef * ptRxBuf,uint8_t * data_array_p);		 
//void	Board_Gimbal_Send_Torque_Ref(void);	 
//void 	Board_Gimbal_Send_Euler_Angle_And_Frame_Angle(void);	 
	 
#ifdef __cplusplus
}
#endif	

#endif	 

