/**
  ******************************************************************************
  * File Name          : task_motor_control.c
  * Description        : ��ʼ������ƫ�ã�AD��PWM_TIMER������FOC����
												 ����ģʽ�͵��λ��ѡ�������زο�����Դ(gyro_spd_out/motor_spd_out/motor_pos->motor_spd_out)
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

/*������״̬*/
SystStatus_t 							st_mc_state = IDLE;

/*���������زο�����Դ��Stat_Curr_q_d_ref_src1Ϊ��������ٶȻ������Stat_Curr_q_d_ref_src1Ϊ�����е�ٶȻ����*/
Curr_Components						Stat_Curr_q_d_ref_src1,	Stat_Curr_q_d_ref_src2;
int8_t										sc_torque_direction = 1;

Frame_Motor_Ctrl_Mode_Typedef	en_frame_motor_ctrl_mode = FRAME_MOTOR_CTRL_TORQUE;	

/* Private functions ------------------------------------------------------------*/
/**
	* @brief��void	Set_Pwm_Chanle1_Compare(uint16_t	uw_data_p)
	* @note:	����PWMռ�ձ�
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
	* @brief��void	HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
	* @note:	����ADC������DMA�ж�
	*/

/*test*/
uint32_t	ul_adc_acq_last_micros;
float			f_adc_acq_period;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{	  
  en_adc1_conv_cplt_flag = SET;
	
	/*���µ�����Ϣ*/
	st_motor_current.uw_ia = ADC1_RegularConvertedValueTab[PHASE_A_CURRENT_ADC_DATA_INDEX];
	st_motor_current.uw_ib = ADC1_RegularConvertedValueTab[PHASE_B_CURRENT_ADC_DATA_INDEX];
   
	f_adc_acq_period = Get_Function_Period(&ul_adc_acq_last_micros);		//test
	
	switch (st_mc_state)
	{
		case RUN:          
			FOC_Model();  									//PMSM torque and flux   ������ 
			break;   

		case START:        
			Motor_Fix_On_Elec_Zero_Pos();		
			break; 

		default:      
			break;
	}

}



/**
	* @brief��void	HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	* @note:	����PWM_TIMER��updata�жϣ�FOC�ڴ˽��д���8KHZ����Ƶ��
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
	* @brief��void	Task_Motor_Ctrl_Init(void)
	* @note:	��ʼ��������Ƹ�����������
	*/
void	Task_Motor_Ctrl_Init(void)
{
	/*��ʼ��As5600*/
	Init_As5600_Soft();
	
	/*ʹ��ADC*/
	ADC_Start();
	
	/*ʹ��PWM_TIEMR�����ǲ�������PWM�����������ADCת��*/
	Start_Pwm_Tiemr();
	
	/*����У׼*/
	Disable_Pwm_Output();
	SVPWM_3ShuntCurrentReadingCalibration( (FlagStatus *)(&en_adc1_conv_cplt_flag) );
	Enable_Pwm_Output();
	
	/*������ػ��ʹ�������PID��ʼ��*/
	Motor_Machine_Spd_Pid_Init();
	Motor_Torque_Flux_Pid_Init();
	
	/*�������FOCģʽ*/
	st_mc_state = RUN;
	
	/*ʹ��CK5G13*/
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
}

/**
	* @brief��void	Task_Get_Motor_Pos_Info(void)
	* @note:	��ȡ���λ����Ϣ
	*/
void	Task_Get_Motor_Pos_Info(void)
{
	Read_As5600_Angle_Data_Soft( &(st_motor_pos_info.uw_motor_machine_pos) );
	
	Cal_Motor_Frame_Angle();	
}

/**
	* @brief��void	Task_Update_Motor_Torque_Ref(void)
	* @note:	���ݵ������ģʽ���µ��������ֵ
	*/
void	Task_Update_Motor_Torque_Ref(void)
{
  static	User_Timer_Typedef	st_motor_machine_spd_pid_update_timer = USER_TIMER_INIT_VALUE;
	static	User_Timer_Typedef	st_motor_machine_spd_out_cmpl_flt_k_inc_timer = USER_TIMER_INIT_VALUE;  //motor_machine_spd_out_cmpl_flt_k������ʱ��
	
	static	Frame_Motor_Ctrl_Mode_Typedef	en_frame_motor_last_ctrl_mode = FRAME_MOTOR_CTRL_TORQUE;
	static	float		f_motor_machine_spd_out_cmpl_flt_k = 1;   
	
	/*�����˺�����Ҫ�Ķ�ʱ��*/
	Start_User_Timer(&st_motor_machine_spd_pid_update_timer);
	Start_User_Timer(&st_motor_machine_spd_out_cmpl_flt_k_inc_timer);
	
	/*��������Դ�л�ʱ�Ķ�����������������ϵ��*/
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
	
	
	/*���µ��������*/
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
		/*�ٶȻ�����*/		
		Update_User_Timer_Cnt(&st_motor_machine_spd_pid_update_timer);
		if(st_motor_machine_spd_pid_update_timer.ul_timer_cnt >= 3)    //��������3ms
		{			
			Reset_User_Timer(&st_motor_machine_spd_pid_update_timer);
			
			/*�����ٶȻ�PID*/
			//Motor_Machine_Spd_Pid_Update( (int16_t)(f_sensor_gyro_raw_data_array[2]*10) );   //pitch���ٶ�ֵ
			
			Motor_Machine_Spd_Pid_Update(Get_Motor_Machine_Spd(st_motor_pos_info.uw_motor_machine_pos));  
				
			/*���µ������Դ*/	
					
			Stat_Curr_q_d_ref_src2.qI_Component1 = 	(1-f_motor_machine_spd_out_cmpl_flt_k)*Stat_Curr_q_d_ref_src1.qI_Component1 + 
																							f_motor_machine_spd_out_cmpl_flt_k*sc_motor_machine_spd_pid_out;
			//Stat_Curr_q_d_ref_src2.qI_Component1 =	f_motor_machine_spd_out_cmpl_flt_k*sc_motor_machine_spd_pid_out;
			Stat_Curr_q_d_ref_src2.qI_Component2 = 0;
		}
		
		/*���µ�������زο�*/		
		Stat_Curr_q_d_ref.qI_Component1 = Stat_Curr_q_d_ref_src2.qI_Component1*sc_torque_direction;
		Stat_Curr_q_d_ref.qI_Component2 = Stat_Curr_q_d_ref_src2.qI_Component2*sc_torque_direction;
	}
	else  if(FRAME_MOTOR_CTRL_POS == en_frame_motor_ctrl_mode)
	{
		/*λ�û�����*/
		
		/*�ٶȻ�����*/		
		Stat_Curr_q_d_ref.qI_Component1 = Stat_Curr_q_d_ref_src2.qI_Component1*sc_torque_direction;
		Stat_Curr_q_d_ref.qI_Component2 = Stat_Curr_q_d_ref_src2.qI_Component2*sc_torque_direction;
	}
}



