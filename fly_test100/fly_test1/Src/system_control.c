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
	* @note	 	云台控制PID初始化
  */

void	System_Init(void)
{

		/*启动微秒级定时器，用作测试*/
	  Statr_Microsecond_Timer();	
	
		/*电机FOC控制初始化*/
//		HAL_Delay(100);

//	  Task_Motor_Ctrl_Init();

//	/*-----------------------姿态板的初始化工作----------------------*/
//	/*获取姿态板配置信息*/
//	Get_Gimbal_Config_Data();
//	
	/*初始化陀螺和加速度传感器*/
//		I2C_Configuration();
//		Init_Icm20602_Soft();
//		Init_Mma8452_Soft();
	
  	/*开串口接收中断*/
		__HAL_UART_ENABLE_IT(&huart3,UART_IT_RXNE);
	
		/*计算加密结果*/
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
		
			/*串口处理*/
			Task_Anasyse_Protocol(SET_CMD_RXBUF,Board_Gimbal_Cmd_Trans_Process);
				
			/* AD采集 */
			Start_ADC_Conv_Period();
		
			Start_User_Timer(&Pack_And_Send_timer);
			Update_User_Timer_Cnt(&Pack_And_Send_timer);
			if( Pack_And_Send_timer.ul_timer_cnt==1000 )  //1000ms发送
			{
				Reset_User_Timer(&Pack_And_Send_timer);
			/* 串口打包发送 */
				Pack_And_Send_Cmd(FEEDBACK_CMD_TXBUF,uc_Pack_And_Send_Buf,0x01);
			}
		
			/*----------------------电机驱动主循环-----------------------------*/
			/*读取电机位置和框架角信息*/
			/*电流信息在ADC的DMA中断中更新*/
//			Task_Get_Motor_Pos_Info();
//			
//			/*debug时不更新来自姿态板的力矩参考信息*/
//			/*FOC运算在ADC的DMA中进行*/
//			if(RESET == en_motor_enter_debug_mode_flag)
//			{
//				/*无uid加密*/
//				Task_Update_Motor_Torque_Ref();
//			}
//		
//		/*----------------------AHRS控制主循环-----------------------------*/
//		  i2c_start_cnt = __HAL_TIM_GET_COUNTER(&htim2);
//			/*读取传感器数据*/
//			Task_Get_Imu_Sensor_Data();
//		
//			/*姿态更新*/		
//			Task_Get_Imu_Attitude();
//			/*坐标转化处理*/
//			Task_Get_Coordinate_Transform_Matrix();
		
//		/*用户操作处理*/
//		Task_Procese_Ui();	
//		
//		/*位置环处理 test*/
//		Task_Gimbal_Pos_Pid_Loop();    
//		
//		/*速度环处理*/
//		Task_Gimbal_Spd_Pid_Loop();	
				 
			/*处理校准*/
//			Task_Gimbal_Cal_Loop();
//			
//			/*定时反馈欧拉角和框架角*/
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
//			main_while_period = Get_Function_Period(&ul_main_while_last_micros);   //计算函数执行周期s


	}
}
