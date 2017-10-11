/**
  ******************************************************************************
  * File Name          : system_control.h
  * Description        : 系统控制文件
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
	* @note	 	云台控制PID初始化
  */

void	System_Init(void)
{
	
	/* 设置IIC的时钟线和数据线为高电平   */
	IIC_SCL=1; 
	IIC_SDA=1;
//	HAL_TIM_Base_Start_IT(&htim4);
	
	/* 开启定时器1通道1，用于PWM的输出 */
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	
	/* 启动ADC采集 */
	HAL_ADC_Start(&hadc1);
	HAL_ADCEx_Calibration_Start(&hadc1);  
	while(HAL_ADCEx_Calibration_Start(&hadc1));	 //等待校准结束

	
	/* PWM输入捕获 */
	HAL_TIM_Base_Start_IT(&htim3);  //启动定时器
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);  //启动定时器通道输入捕获并开启中断
	

		/*启动微秒级定时器，用作测试*/
	  Statr_Microsecond_Timer();	
	
	
  	/*开串口接收中断*/
		__HAL_UART_ENABLE_IT(&huart3,UART_IT_RXNE);
	

}


void	System_Run(int8_t	uc_uid_result_p)
{
	if(uc_uid_result_p)
	{		
			static	User_Timer_Typedef	Pack_And_Send_timer = USER_TIMER_INIT_VALUE;
		
			/*串口处理*/
			Task_Anasyse_Protocol(SET_CMD_RXBUF,Board_Gimbal_Cmd_Trans_Process);

	
		/* AD采集 */
			Start_ADC_Conv_Period();
		
			Start_User_Timer(&Pack_And_Send_timer);
			Update_User_Timer_Cnt(&Pack_And_Send_timer);
			if( Pack_And_Send_timer.ul_timer_cnt==100 )  //100ms发送
			{
				Reset_User_Timer(&Pack_And_Send_timer);
			/* 串口打包发送 */
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


