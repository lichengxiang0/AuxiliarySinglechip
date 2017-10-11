/**
  ******************************************************************************
  * File Name          : system_control.h
  * Description        : ϵͳ�����ļ�
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
#include "system_control.h"

#include "stm32_bsp_timer.h"
#include "stm32_bsp_uart.h"
#include "stm32_bsp_peripheral_init_export.h"

//#include "soft_iic.h"
//#include "icm20602_driver.h"
//#include "mma8452_driver.h"
//#include "as5600_driver.h"
//#include "soft_iic1.h"

//#include "uid.h"

//#include "task_get_attitude.h"
#include "protocol.h"
#include "board_gimbal_cmd_process.h"
#include "respond_gimbal_cmd.h"
#include "respond_motor_cmd.h"

#include "usart.h"

#include "my_adc.h"

//#include "task_motor_control.h"


uint8_t uc_Pack_And_Send_Buf[10];

/* Private define ------------------------------------------------------------*/
#define APPLICATION_ADDRESS     ((uint32_t)0x08002800)

/**
  * @brief 	void PID_Init(void)
	* @note	 	��̨����PID��ʼ��
  */

void	System_Init(void)
{

		/*����΢�뼶��ʱ������������*/
	  Statr_Microsecond_Timer();	
	
		/*���FOC���Ƴ�ʼ��*/
//		HAL_Delay(100);

//	  Task_Motor_Ctrl_Init();

//	/*-----------------------��̬��ĳ�ʼ������----------------------*/
//	/*��ȡ��̬��������Ϣ*/
//	Get_Gimbal_Config_Data();
//	
	/*��ʼ�����ݺͼ��ٶȴ�����*/
//		I2C_Configuration();
//		Init_Icm20602_Soft();
//		Init_Mma8452_Soft();
	
  	/*�����ڽ����ж�*/
		__HAL_UART_ENABLE_IT(&huart3,UART_IT_RXNE);
	
		/*������ܽ��*/
		//Run_Uid_Algorithm();
}

//float	main_while_period;  //test
//uint32_t	ul_main_while_last_micros;  //test

//uint16_t	i2c_start_cnt,	i2c_end_cnt;
//float			i2c_period;

void	System_Run(int8_t	uc_uid_result_p)
{
	if(uc_uid_result_p)
	{		
			static	User_Timer_Typedef	Pack_And_Send_timer = USER_TIMER_INIT_VALUE;
		
			/*���ڴ���*/
			Task_Anasyse_Protocol(SET_CMD_RXBUF,Board_Gimbal_Cmd_Trans_Process);
				
			/* AD�ɼ� */
			Start_ADC_Conv_Period();
		
			Start_User_Timer(&Pack_And_Send_timer);
			Update_User_Timer_Cnt(&Pack_And_Send_timer);
			if( Pack_And_Send_timer.ul_timer_cnt==1000 )  //1000ms����
			{
				Reset_User_Timer(&Pack_And_Send_timer);
			/* ���ڴ������ */
				Pack_And_Send_Cmd(FEEDBACK_CMD_TXBUF,uc_Pack_And_Send_Buf,0x01);
			}
		
			/*----------------------���������ѭ��-----------------------------*/
			/*��ȡ���λ�úͿ�ܽ���Ϣ*/
			/*������Ϣ��ADC��DMA�ж��и���*/
//			Task_Get_Motor_Pos_Info();
//			
//			/*debugʱ������������̬������زο���Ϣ*/
//			/*FOC������ADC��DMA�н���*/
//			if(RESET == en_motor_enter_debug_mode_flag)
//			{
//				/*��uid����*/
//				Task_Update_Motor_Torque_Ref();
//			}
//		
//		/*----------------------AHRS������ѭ��-----------------------------*/
//		  i2c_start_cnt = __HAL_TIM_GET_COUNTER(&htim2);
//			/*��ȡ����������*/
//			Task_Get_Imu_Sensor_Data();
//		
//			/*��̬����*/		
//			Task_Get_Imu_Attitude();
//			/*����ת������*/
//			Task_Get_Coordinate_Transform_Matrix();
		
//		/*�û���������*/
//		Task_Procese_Ui();	
//		
//		/*λ�û����� test*/
//		Task_Gimbal_Pos_Pid_Loop();    
//		
//		/*�ٶȻ�����*/
//		Task_Gimbal_Spd_Pid_Loop();	
				 
			/*����У׼*/
//			Task_Gimbal_Cal_Loop();
//			
//			/*��ʱ����ŷ���ǺͿ�ܽ�*/
//			Board_Gimbal_Send_Euler_Angle_And_Frame_Angle();
//			
//			i2c_end_cnt = __HAL_TIM_GET_COUNTER(&htim2); 
//		
//			if( i2c_end_cnt < i2c_start_cnt )
//			{ 
//				i2c_period =  ((float)(i2c_end_cnt + (0xffff - i2c_start_cnt ) ) / 1000000.0f);
//			}
//			else	
//			{
//				i2c_period =  ((float)(i2c_end_cnt - i2c_start_cnt ) / 1000000.0f);
//			}
//			
//			main_while_period = Get_Function_Period(&ul_main_while_last_micros);   //���㺯��ִ������s


	}
}
