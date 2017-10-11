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

#include "protocol.h"
#include "board_gimbal_cmd_process.h"
#include "respond_gimbal_cmd.h"
#include "respond_motor_cmd.h"
#include "tim.h"
#include "usart.h"

#include "my_adc.h"
#include "myiic.h"
#include "sys.h"
//#include "task_motor_control.h"

extern ADC_HandleTypeDef hadc1;
uint8_t uc_Pack_And_Send_Buf[11];
uint8_t myUart1_4g_module_data[25];
uint8_t AircraftModule1,AirExceptionInf,AirCustomerInf;
uint8_t AirLongitude[4],AirLatitude[4],AirHight[4],AirSatelliteTime[4];
__IO uint16_t timer_count=0;

/* Private define ------------------------------------------------------------*/
#define APPLICATION_ADDRESS     ((uint32_t)0x08002800)

/**
  * @brief 	void PID_Init(void)
	* @note	 	��̨����PID��ʼ��
  */

void	System_Init(void)
{
	
	/* ����IIC��ʱ���ߺ�������Ϊ�ߵ�ƽ   */
	IIC_SCL=1; 
	IIC_SDA=1;
//	HAL_TIM_Base_Start_IT(&htim4);
	
	/* ������ʱ��1ͨ��1������PWM����� */
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	
	/* ����ADC�ɼ� */
	HAL_ADC_Start(&hadc1);
	HAL_ADCEx_Calibration_Start(&hadc1);  
	while(HAL_ADCEx_Calibration_Start(&hadc1));	 //�ȴ�У׼����

	
	/* PWM���벶�� */
	HAL_TIM_Base_Start_IT(&htim3);  //������ʱ��
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);  //������ʱ��ͨ�����벶�񲢿����ж�
	

		/*����΢�뼶��ʱ������������*/
	  Statr_Microsecond_Timer();	
	
	
  	/*�����ڽ����ж�*/
		__HAL_UART_ENABLE_IT(&huart3,UART_IT_RXNE);
	

}


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
			if( Pack_And_Send_timer.ul_timer_cnt==100 )  //100ms����
			{
				Reset_User_Timer(&Pack_And_Send_timer);
			/* ���ڴ������ */
				Pack_And_Send_Cmd(FEEDBACK_CMD_TXBUF,uc_Pack_And_Send_Buf,0x01);
			}
		

	}
}


//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if(htim->Instance==TIM4)
//	{
//		printf("Aibirdtype=%X&Exceptinfo=%X&userphone=%X&Longitude=%X&Latitude=%X&Altitude=%X&Duration=%X",AircraftModule1,AirExceptionInf,AirCustomerInf,AirLongitude,AirLatitude,AirHight,AirSatelliteTime);
//	}
//	
//}


