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
	
	
/* Includes ------------------------------------------------------------------*/	 
#include "stm32f1xx_hal.h"

//#include "sys_config.h"

//#include "gimbal_frame_angle.h"
//#include "gimbal_ahrs.h"

#include "board_gimbal_cmd_process.h"


#include "stm32_bsp_uart.h"

#include "protocol.h"
#include "system_control.h"
#include "led.h"
#include "tim.h"

//extern uint8_t uc_Pack_And_Send_Buf[10];

//#include "respond_gimbal_cmd.h"

//#include "task_gimbal_spd_ctrl.h"

/* Private functions ------------------------------------------------------------*/
/**
	* @brief：void	Board_Gimbal_Process_Firmware_Update_Cmd(uint8_t * data_array_p)	
	* @note： Board_Gimbal处理固件升级命令
	* @param：uint8_t * data_array_p：具体命令内容
	*/
//void	Board_Gimbal_Process_Firmware_Update_Cmd(uint8_t * data_array_p)
//{
//	uint8_t		uc_fdb_jump_to_iap_cmd_data[2] = {0x50,0x10};
//	/*清除0x55AA，保存*/
//	//st_firmware_config_data.first_program_flag = 0x0000;
//	//Save_Firmware_Config_Data();
//	
//	/*反馈*/
//	Pack_And_Send_Cmd(FEEDBACK_CMD_TXBUF,uc_fdb_jump_to_iap_cmd_data,MODULE_ADD_BYTE);
//	//while保证串口命令已发送完成
//	while( FEEDBACK_CMD_TXBUF->ucBufCnt );
//	while(__HAL_UART_GET_FLAG(FEEDBACK_CMD_TXBUF->ptUartHandel ,UART_FLAG_TC)==RESET);
//	
//	//软件复位
//	HAL_NVIC_SystemReset();
//}



/**
	* @brief：void	Board_Gimbal_Process_Jump_To_Iap_Cmd(uint8_t * data_array_p)	
	* @note： Board_Gimbal处理跳转到bootloader命令
* @param：USART_Buf_TypeDef * ptRxBuf:串口号
	* @param：uint8_t * data_array_p：具体命令内容
	*/
//void	Board_Gimbal_Process_Jump_To_Iap_Cmd(USART_Buf_TypeDef * ptRxBuf,uint8_t * data_array_p)
//{
//	uint8_t		uc_send_jump_to_iap_cmd_data[2] = {0x50,0x10};
//	
//	/*上位机直接发送的跳转命令，转发并跳转*/
//	if(SET_CMD_RXBUF == ptRxBuf)   
//	{
//		/*清除0x55AA，保存*/
////		st_firmware_config_data.first_program_flag = 0x0000;
////		Save_Firmware_Config_Data();
//	}
//	
//	//软件复位
//	HAL_NVIC_SystemReset();	
//}


/* External functions ------------------------------------------------------------*/
/**
	* @brief：void	Board_Gimbal_Cmd_Trans_Process(USART_Buf_TypeDef * ptRxBuf,uint8_t * data_array_p)	
	* @note： Board_3通信协议的转发和处理
	*	@param：USART_Buf_TypeDef * ptRxBuf：接收缓冲区
	* @param：uint8_t * data_array_p：具体命令内容
	*/
void	Board_Gimbal_Cmd_Trans_Process(USART_Buf_TypeDef * ptRxBuf,uint8_t * data_array_p)	
{
//	uint8_t OPenUmbrella,AirLight,AirEngineOrder,ChargeControl;
//	OPenUmbrella = UART_ReadChar(ptRxBuf,0);
//	AirLight = UART_ReadChar(ptRxBuf,5);
//	AirEngineOrder = UART_ReadChar(ptRxBuf,6);
//	ChargeControl = UART_ReadChar(ptRxBuf,8);
//	
//		switch(OPenUmbrella)  //开伞
//									{
//										case 0x01:
//													__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,1000);  //45°
////														HAL_Delay(1000);
//										
//												LED0 = 0;
//												LED1 = 1;
////													HAL_Delay(500);
//												uc_Pack_And_Send_Buf[7] = 0x01;  //开伞
//												break;
//										case 0x02:
//													__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,2000);  //135°
////														HAL_Delay(1000);
//										
//												LED0 = 1;
//												LED1 = 0;
////													HAL_Delay(500);
//												uc_Pack_And_Send_Buf[7] = 0x02;  //关伞
//												break;
//										default:
//													__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,500);  //0°
////														HAL_Delay(500);	
//												
//												LED0 = 1;
//												LED1 = 1;
//												uc_Pack_And_Send_Buf[7] = 0x0;
//												break;
//									}
//						
//									switch(AirLight)  //灯光
//									{
//										case 0x01:
//												uc_Pack_And_Send_Buf[8] = 0x01;  /* 可飞行 */
//												break;
//										case 0x02:
//												uc_Pack_And_Send_Buf[8] = 0x02;
//												break;
//										case 0x03:
//												uc_Pack_And_Send_Buf[8] = 0x03;
//												break;
//										default:
//												uc_Pack_And_Send_Buf[8] = 0x00;
//												break;
//									}
//									
//									switch(AirEngineOrder)  //油机命令
//									{
//										case 0x01:
//												uc_Pack_And_Send_Buf[3] = 0x01;
//												break;
//										case 0x02:
//												uc_Pack_And_Send_Buf[3] = 0x02;
//												break;
//										case 0x03:
//												uc_Pack_And_Send_Buf[3] = 0x10;
//												break;
//										default:
//												uc_Pack_And_Send_Buf[3] = 0x20;
//												break;
//									}
//									
//									switch(ChargeControl)  //充电控制
//									{
//										case 0x01:
//												uc_Pack_And_Send_Buf[9] = 0x01;
//												break;
//										case 0x0:
//												uc_Pack_And_Send_Buf[9] = 0x0;
//												break;
//										default:
//												break;
//										
//									}
	
	
//	uint16_t uw_cmd_word;
//	uw_cmd_word = (((uint16_t)data_array_p[0])<<8) | ((uint16_t)data_array_p[1]);  
//	
//	if( SET_CMD_RXBUF == ptRxBuf)
//	{
//		switch(uw_cmd_word)
//		{
//			
//			
//			
//			case JOYSTICK_CMD_ENTER_DEBUG_MODE:
//			case JOYSTICK_CMD_CAL_JOY:
//			case JOYSTICK_CMD_RESET_KEY_VALUE:
//			case JOYSTICK_CMD_QUERY_JOY_CAL_STATUS:
//				break;
			
			/*电机Debug部分响应+转发*/
			
//			case CMD_QUERY_CONFIG_INFO:
//				Process_Motor_Query_Config_Info_Cmd(data_array_p);
//				Process_Ahrs_Query_Config_Info_Cmd(data_array_p);
//				break;
//			
//			case CMD_JUMP_TO_IAP:
//				Board_Gimbal_Process_Jump_To_Iap_Cmd(ptRxBuf,data_array_p);
//				break;
//			
//			default:
//				break;
//		}
//	}
}



/**
  * @brief 	void	Board_Gimbal_Send_Torque_Ref(void)
	* @note	 	姿态板发送力矩数据
  */
//void	Board_Gimbal_Send_Torque_Ref(void)
//{
//	static uint8_t		uc_send_torque_ref_cmd_data[10] = {0x01,0x08};
//	
//	/*更新发送数据*/
//	//Gimbal_Process_Set_Torque_Ref_Cmd(uc_send_torque_ref_cmd_data);
//	
//	/*电机取力矩*/
//	//Motor_Process_Set_Torque_Ref_Cmd(motor_x_spd_pid_torque_out);
//	//Motor_Process_Set_Torque_Ref_Cmd(motor_y_spd_pid_torque_out);
//	//Motor_Process_Set_Torque_Ref_Cmd(uc_send_torque_ref_cmd_data);
//		
//	Pack_And_Send_Cmd(FEEDBACK_CMD_TXBUF,uc_send_torque_ref_cmd_data,MODULE_ADD_BYTE);
//}
/**
  * @brief 	void Feedback_Euler_Angle_And_Frame_Angle_Information(void)
	* @note	 	回传姿态角和电机框架角
  */
//void Board_Gimbal_Send_Euler_Angle_And_Frame_Angle(void)
//{
//	uint8_t	uc_fdb_cmd_data_array_p[18];
//	int16_t temp = 0;
//	
//	static	User_Timer_Typedef	st_this_function_period_timer_p = USER_TIMER_INIT_VALUE;
//	
//	Start_User_Timer(&st_this_function_period_timer_p);
//	Update_User_Timer_Cnt(&st_this_function_period_timer_p);
//	
//	if(st_this_function_period_timer_p.ul_timer_cnt > 200)		//200ms发送一次
//	{
//		Reset_User_Timer(&st_this_function_period_timer_p);
//		
//		uc_fdb_cmd_data_array_p[0] = 0xF8;
//		uc_fdb_cmd_data_array_p[1] = 0x0F;
//		
//		temp = motor_frame_angle_rad[0]*573;
//		uc_fdb_cmd_data_array_p[2]=(uint8_t)(((int16_t)(temp))>>8);
//		uc_fdb_cmd_data_array_p[3]=(uint8_t)((int16_t)(temp));	
//		
//		temp = motor_frame_angle_rad[1]*573;
//		uc_fdb_cmd_data_array_p[4]=(uint8_t)(((int16_t)(temp))>>8);
//		uc_fdb_cmd_data_array_p[5]=(uint8_t)((int16_t)(temp));	
//		
//		temp = motor_frame_angle_rad[2]*573;
//		uc_fdb_cmd_data_array_p[6]=(uint8_t)(((int16_t)(temp))>>8);	
//		uc_fdb_cmd_data_array_p[7]=(uint8_t)((int16_t)(temp));
//		
//		
//		
//		temp = (int16_t)f_euler_array[0]*10;
//		uc_fdb_cmd_data_array_p[8] = (uint8_t)(temp>>8);
//		uc_fdb_cmd_data_array_p[9] = (uint8_t)temp;
//		
//		temp = (int16_t)f_euler_array[1]*10;
//		uc_fdb_cmd_data_array_p[10] = (uint8_t)(temp>>8);
//		uc_fdb_cmd_data_array_p[11] = (uint8_t)temp;
//		
//		temp = (int16_t)f_euler_array[2]*10;
//		uc_fdb_cmd_data_array_p[12] = (uint8_t)(temp>>8);
//		uc_fdb_cmd_data_array_p[13] = (uint8_t)temp;
//		
//		//预留字节
//		uc_fdb_cmd_data_array_p[14] = 0x00;
//		uc_fdb_cmd_data_array_p[15] = 0x00;
//		uc_fdb_cmd_data_array_p[16] = 0x00;
//		
//		Pack_And_Send_Cmd(FEEDBACK_CMD_TXBUF,uc_fdb_cmd_data_array_p,MODULE_ADD_BYTE);
//	}
//}
