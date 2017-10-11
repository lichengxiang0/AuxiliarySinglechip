/**
  ******************************************************************************
  * File Name          : task_motor_control.c
  * Description        : 初始化电流偏置，AD，PWM_TIMER，进行FOC运算
												 根据模式和电机位置选择电机力矩参考输入源(gyro_spd_out/motor_spd_out/motor_pos->motor_spd_out)
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
#include "task_motor_control.h"

#include "stm32_bsp_timer.h"
#include "stm32_bsp_adc.h"

#include "as5600_driver.h"
#include "soft_iic1.h"

#include "MC_Motor_Pos.h"
#include "MC_FOC_Drive.h"
#include "stm32f0x_svpwm_3shunt.h"
#include "motor_machine_spd_ctrl.h"

#include "gimbal_ahrs.h"


/* External variables ------------------------------------------------------------*/
Frame_Motor_Id_Typedef 		en_frame_motor_id = FRAME_MOTOR_Y;

/*定义电机状态*/
SystStatus_t 							st_mc_state = IDLE;

/*定义电机力矩参考的来源，Stat_Curr_q_d_ref_src1为电机陀螺速度环输出，Stat_Curr_q_d_ref_src1为电机机械速度环输出*/
Curr_Components						Stat_Curr_q_d_ref_src1,	Stat_Curr_q_d_ref_src2;
int8_t										sc_torque_direction = 1;

Frame_Motor_Ctrl_Mode_Typedef	en_frame_motor_ctrl_mode = FRAME_MOTOR_CTRL_TORQUE;	

/* Private functions ------------------------------------------------------------*/
/**
	* @brief：void	Set_Pwm_Chanle1_Compare(uint16_t	uw_data_p)
	* @note:	设置PWM占空比
	*/
void	Set_Pwm_Chanle1_Compare(uint16_t	uw_data_p)
{
	__HAL_TIM_SET_COMPARE(&PWM_TIMER,TIM_CHANNEL_1,uw_data_p);
}

void	Set_Pwm_Chanle2_Compare(uint16_t	uw_data_p)
{
	__HAL_TIM_SET_COMPARE(&PWM_TIMER,TIM_CHANNEL_2,uw_data_p);
}

void	Set_Pwm_Chanle3_Compare(uint16_t	uw_data_p)
{
	__HAL_TIM_SET_COMPARE(&PWM_TIMER,TIM_CHANNEL_3,uw_data_p);
}

void	Set_Pwm_Chanle4_Compare(uint16_t	uw_data_p)
{
	__HAL_TIM_SET_COMPARE(&PWM_TIMER,TIM_CHANNEL_4,uw_data_p);
}




/**
	* @brief：void	HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
	* @note:	处理ADC采样的DMA中断
	*/

/*test*/
uint32_t	ul_adc_acq_last_micros;
float			f_adc_acq_period;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{	  
  en_adc1_conv_cplt_flag = SET;
	
	/*更新电流信息*/
	st_motor_current.uw_ia = ADC1_RegularConvertedValueTab[PHASE_A_CURRENT_ADC_DATA_INDEX];
	st_motor_current.uw_ib = ADC1_RegularConvertedValueTab[PHASE_B_CURRENT_ADC_DATA_INDEX];
   
	f_adc_acq_period = Get_Function_Period(&ul_adc_acq_last_micros);		//test
	
	switch (st_mc_state)
	{
		case RUN:          
			FOC_Model();  									//PMSM torque and flux   电流环 
			break;   

		case START:        
			Motor_Fix_On_Elec_Zero_Pos();		
			break; 

		default:      
			break;
	}

}



/**
	* @brief：void	HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	* @note:	处理PWM_TIMER的updata中断，FOC在此进行处理，8KHZ更新频率
	*/
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	switch (st_mc_state)
//	{
//		case RUN:          
//							FOC_Model();  //PMSM torque and flux    
//							break;          
//		case START:        
//							Motor_Fix_On_Elec_Zero_Pos();
//							break; 
//		
//		default:      
//			break;
//	 }
//}



/* External functions ------------------------------------------------------------*/
/**
	* @brief：void	Task_Motor_Ctrl_Init(void)
	* @note:	初始化电机控制各参数和外设
	*/
void	Task_Motor_Ctrl_Init(void)
{
	/*初始化As5600*/
	Init_As5600_Soft();
	
	/*使能ADC*/
	ADC_Start();
	
	/*使能PWM_TIEMR，但是并不启动PWM的输出，启动ADC转换*/
	Start_Pwm_Tiemr();
	
	/*电流校准*/
	Disable_Pwm_Output();
	SVPWM_3ShuntCurrentReadingCalibration( (FlagStatus *)(&en_adc1_conv_cplt_flag) );
	Enable_Pwm_Output();
	
	/*电机力矩环和磁链环的PID初始化*/
	Motor_Machine_Spd_Pid_Init();
	Motor_Torque_Flux_Pid_Init();
	
	/*电机处于FOC模式*/
	st_mc_state = RUN;
	
	/*使能CK5G13*/
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
}

/**
	* @brief：void	Task_Get_Motor_Pos_Info(void)
	* @note:	读取电机位置信息
	*/
void	Task_Get_Motor_Pos_Info(void)
{
	Read_As5600_Angle_Data_Soft( &(st_motor_pos_info.uw_motor_machine_pos) );
	
	Cal_Motor_Frame_Angle();	
}

/**
	* @brief：void	Task_Update_Motor_Torque_Ref(void)
	* @note:	根据电机工作模式更新电机的力矩值
	*/
void	Task_Update_Motor_Torque_Ref(void)
{
  static	User_Timer_Typedef	st_motor_machine_spd_pid_update_timer = USER_TIMER_INIT_VALUE;
	static	User_Timer_Typedef	st_motor_machine_spd_out_cmpl_flt_k_inc_timer = USER_TIMER_INIT_VALUE;  //motor_machine_spd_out_cmpl_flt_k递增定时器
	
	static	Frame_Motor_Ctrl_Mode_Typedef	en_frame_motor_last_ctrl_mode = FRAME_MOTOR_CTRL_TORQUE;
	static	float		f_motor_machine_spd_out_cmpl_flt_k = 1;   
	
	/*启动此函数需要的定时器*/
	Start_User_Timer(&st_motor_machine_spd_pid_update_timer);
	Start_User_Timer(&st_motor_machine_spd_out_cmpl_flt_k_inc_timer);
	
	/*处理力矩源切换时的抖动，缓慢增加力矩系数*/
	Update_User_Timer_Cnt(&st_motor_machine_spd_out_cmpl_flt_k_inc_timer);
	if(en_frame_motor_ctrl_mode != en_frame_motor_last_ctrl_mode)
	{
			f_motor_machine_spd_out_cmpl_flt_k = 0;
	}
	else
	{
		if(st_motor_machine_spd_out_cmpl_flt_k_inc_timer.ul_timer_cnt > 3)
		{
			Reset_User_Timer(&st_motor_machine_spd_out_cmpl_flt_k_inc_timer);
			
			f_motor_machine_spd_out_cmpl_flt_k += 0.01;
			if(f_motor_machine_spd_out_cmpl_flt_k > 0.4)  
			{
				f_motor_machine_spd_out_cmpl_flt_k = 0.4;  
			}
		}
	}
	en_frame_motor_last_ctrl_mode = en_frame_motor_ctrl_mode;
	
	
	/*更新电机的力矩*/
	if(FRAME_MOTOR_CTRL_TORQUE == en_frame_motor_ctrl_mode)
	{
		Stat_Curr_q_d_ref.qI_Component1 = (1-f_motor_machine_spd_out_cmpl_flt_k)*Stat_Curr_q_d_ref.qI_Component1 +  
																			f_motor_machine_spd_out_cmpl_flt_k*Stat_Curr_q_d_ref_src1.qI_Component1*sc_torque_direction;
	  //Stat_Curr_q_d_ref.qI_Component1 = Stat_Curr_q_d_ref_src1.qI_Component1*sc_torque_direction;
		Stat_Curr_q_d_ref.qI_Component2 = Stat_Curr_q_d_ref_src1.qI_Component2*sc_torque_direction;
	}
	else 
	if(FRAME_MOTOR_CTRL_SPD == en_frame_motor_ctrl_mode)						//FRAME_MOTOR_CTRL_SPD
	{
		/*速度环更新*/		
		Update_User_Timer_Cnt(&st_motor_machine_spd_pid_update_timer);
		if(st_motor_machine_spd_pid_update_timer.ul_timer_cnt >= 3)    //更新周期3ms
		{			
			Reset_User_Timer(&st_motor_machine_spd_pid_update_timer);
			
			/*更新速度环PID*/
			//Motor_Machine_Spd_Pid_Update( (int16_t)(f_sensor_gyro_raw_data_array[2]*10) );   //pitch轴速度值
			
			Motor_Machine_Spd_Pid_Update(Get_Motor_Machine_Spd(st_motor_pos_info.uw_motor_machine_pos));  
				
			/*更新电机力矩源*/	
					
			Stat_Curr_q_d_ref_src2.qI_Component1 = 	(1-f_motor_machine_spd_out_cmpl_flt_k)*Stat_Curr_q_d_ref_src1.qI_Component1 + 
																							f_motor_machine_spd_out_cmpl_flt_k*sc_motor_machine_spd_pid_out;
			//Stat_Curr_q_d_ref_src2.qI_Component1 =	f_motor_machine_spd_out_cmpl_flt_k*sc_motor_machine_spd_pid_out;
			Stat_Curr_q_d_ref_src2.qI_Component2 = 0;
		}
		
		/*更新电机的力矩参考*/		
		Stat_Curr_q_d_ref.qI_Component1 = Stat_Curr_q_d_ref_src2.qI_Component1*sc_torque_direction;
		Stat_Curr_q_d_ref.qI_Component2 = Stat_Curr_q_d_ref_src2.qI_Component2*sc_torque_direction;
	}
	else  if(FRAME_MOTOR_CTRL_POS == en_frame_motor_ctrl_mode)
	{
		/*位置环更新*/
		
		/*速度环更新*/		
		Stat_Curr_q_d_ref.qI_Component1 = Stat_Curr_q_d_ref_src2.qI_Component1*sc_torque_direction;
		Stat_Curr_q_d_ref.qI_Component2 = Stat_Curr_q_d_ref_src2.qI_Component2*sc_torque_direction;
	}
}



