/**
  ******************************************************************************
  * File Name          : MC_FOC_Drive.c
  * Description        : This file provides code to run FOC .
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
#include "MC_FOC_Drive.h"

//uint_16  i2c_end_cnt1;

/* Private macro -------------------------------------------------------------*/
#define SATURATION_TO_S16(a)    if (a > S16_MAX)              \
                                {                             \
                                  a = S16_MAX;                \
                                }                             \
                                else if (a < -S16_MAX)        \
                                {                             \
                                  a = -S16_MAX;               \
                                }  


/* Private variables ---------------------------------------------------------*/
/* Electrical, magnetic and mechanical variables*/
Curr_Components Stat_Curr_a_b;              /*Stator currents Ia,Ib*/ 

Curr_Components Stat_Curr_alfa_beta;        /*Ialpha & Ibeta, Clarke's  
                                            transformations of Ia & Ib */

Curr_Components Stat_Curr_q_d;              /*Iq & Id, Parke's transformations of 
                                            Ialpha & Ibeta, */

Volt_Components Stat_Volt_a_b;              /*Stator voltages Va, Vb*/ 

Volt_Components Stat_Volt_q_d;              /*Vq & Vd, voltages on a reference
                                            frame synchronous with the rotor flux*/

Volt_Components Stat_Volt_alfa_beta;        /*Valpha & Vbeta, RevPark transformations
                                             of Vq & Vd*/

PID_Struct_Typedef 	st_motor_torque_pid;

PID_Struct_Typedef 	st_motor_flux_pid;


/* External variables ---------------------------------------------------------*/
Curr_Components Stat_Curr_q_d_ref; 	//�ٶȻ�����������û������ģ�飬ֱ����Ϊ����������ֵ��

/* External function ---------------------------------------------------------*/
/**
  * @brief 	Motor_Torque_Flux_Pid_Init angle
	* @note	 	init torque and flux pid
  */
void	Motor_Torque_Flux_Pid_Init(void)
{
	//motor torque pid argument init//
  st_motor_torque_pid.hKp_Gain    = TORQUE_PID_KP_DEFAULT;
  st_motor_torque_pid.hKp_Divisor = TF_KPDIV;  	//����

  st_motor_torque_pid.hKi_Gain = TORQUE_PID_KI_DEFAULT;
  st_motor_torque_pid.hKi_Divisor = TF_KIDIV;		//����
  
  st_motor_torque_pid.hKd_Gain = TORQUE_PID_KD_DEFAULT;
  st_motor_torque_pid.hKd_Divisor = TF_KDDIV;		//����
  st_motor_torque_pid.wPreviousError = 0;
  
	st_motor_torque_pid.hLower_Threshold_Integral = S16_MIN;
	st_motor_torque_pid.hUpper_Threshold_Integral = S16_MAX;
  st_motor_torque_pid.hLower_Limit_Output= S16_MIN;   //Lower Limit for Output limitation
  st_motor_torque_pid.hUpper_Limit_Output= S16_MAX;   //Upper Limit for Output limitation
  st_motor_torque_pid.wLower_Limit_Integral = S16_MIN * TF_KIDIV;  	//TF_KIDIV�������4096
  st_motor_torque_pid.wUpper_Limit_Integral = S16_MAX * TF_KIDIV;		//TF_KIDIV�������4096
  st_motor_torque_pid.wIntegral = 0;
	
	
	//motor flux pid argument init//
  st_motor_flux_pid.hKp_Gain    = TORQUE_PID_KP_DEFAULT;
  st_motor_flux_pid.hKp_Divisor = TF_KPDIV;    //����

  st_motor_flux_pid.hKi_Gain = TORQUE_PID_KI_DEFAULT;
  st_motor_flux_pid.hKi_Divisor = TF_KIDIV;		//����
  
  st_motor_flux_pid.hKd_Gain = TORQUE_PID_KD_DEFAULT;
  st_motor_flux_pid.hKd_Divisor = TF_KDDIV;		//����
  st_motor_flux_pid.wPreviousError = 0;
  
	st_motor_flux_pid.hLower_Threshold_Integral = S16_MIN;
	st_motor_flux_pid.hUpper_Threshold_Integral = S16_MAX;
  st_motor_flux_pid.hLower_Limit_Output= S16_MIN;   //Lower Limit for Output limitation
  st_motor_flux_pid.hUpper_Limit_Output= S16_MAX;   //Upper Limit for Output limitation
  st_motor_flux_pid.wLower_Limit_Integral = S16_MIN * TF_KIDIV;		//TF_KIDIV�������4096
  st_motor_flux_pid.wUpper_Limit_Integral = S16_MAX * TF_KIDIV;		//TF_KIDIV�������4096
  st_motor_flux_pid.wIntegral = 0;
}




/**
  * @brief 	Motor_Torque_Flux_Ctrl_Loop
	* @note	 	������ػ��ʹ��������ƣ�FOCģ��
  */
void FOC_Model(void)	  //���������������ϸ��տ�ͼ���
{  
  /**********STARTS THE VECTOR CONTROL *********************** */ 
  Stat_Curr_a_b = SVPWM_3ShuntGetPhaseCurrentValues();			//��ȡ2��ĵ���ֵ
  
  Stat_Curr_alfa_beta = Clarke(Stat_Curr_a_b);	// �õ�Valpha��Vbeta,Clark�任
  
  Stat_Curr_q_d = Park(Stat_Curr_alfa_beta,POS_GetElectricalAngle());  // Stat_Curr_q_dΪ��ǰ��Id��Iqֵ
																   // ����ֵΪ Stat_Curr_q_d_ref_ref
	
	//��ǰ������
  /*loads the Torque Regulator output reference voltage Vqs   */
  Stat_Volt_q_d.qV_Component1 = Fast_PI_Regulator(Stat_Curr_q_d_ref.qI_Component1, 
                        Stat_Curr_q_d.qI_Component1, &st_motor_torque_pid);
	
  /*loads the Flux Regulator output reference voltage Vds */ 
  Stat_Volt_q_d.qV_Component2 = Fast_PI_Regulator(Stat_Curr_q_d_ref.qI_Component2, 
                        Stat_Curr_q_d.qI_Component2, &st_motor_flux_pid);
	
	SATURATION_TO_S16(Stat_Volt_q_d.qV_Component1);
	
	SATURATION_TO_S16(Stat_Volt_q_d.qV_Component2);
	
  //circle limitation
  RevPark_Circle_Limitation(&Stat_Volt_q_d); 
  /*Performs the Reverse Park transformation,
  i.e transforms stator voltages Vqs and Vds into Valpha and Vbeta on a 
  stationary reference frame*/
  Stat_Volt_alfa_beta = Rev_Park(Stat_Volt_q_d);
  /*Valpha and Vbeta finally drive the power stage*/ 
  SVPWM_3ShuntCalcDutyCycles(Stat_Volt_alfa_beta); 								//ʵ�ʵĵ���������� 
}


/**
  * @brief 	Motor_Fix_On_Elec_Zero_Pos angle
	* @note	 	�����ת������λλ��
  */
void Motor_Fix_On_Elec_Zero_Pos(void)
{         
	Stat_Curr_a_b = SVPWM_3ShuntGetPhaseCurrentValues(); 
	Stat_Curr_alfa_beta = Clarke(Stat_Curr_a_b);
	/* ����Ƕ�����Ϊ0 */
	Stat_Curr_q_d = Park(Stat_Curr_alfa_beta, 0);
  
  /*loads the Torque Regulator output reference voltage Vqs   */
  Stat_Volt_q_d.qV_Component1 = Fast_PI_Regulator(Stat_Curr_q_d_ref.qI_Component1, 
                        Stat_Curr_q_d.qI_Component1, &st_motor_torque_pid);
	
//  HAL_GPIO_WritePin(GPIOF,GPIO_PIN_7,GPIO_PIN_RESET);
  /*loads the Flux Regulator output reference voltage Vds */ 
  Stat_Volt_q_d.qV_Component2 = Fast_PI_Regulator(Stat_Curr_q_d_ref.qI_Component2, 
                          Stat_Curr_q_d.qI_Component2, &st_motor_flux_pid);

	SATURATION_TO_S16(Stat_Volt_q_d.qV_Component1);
	SATURATION_TO_S16(Stat_Volt_q_d.qV_Component2);
	
	RevPark_Circle_Limitation(&Stat_Volt_q_d);

	/*Performs the Reverse Park transformation,
	i.e transforms stator voltages Vqs and Vds into Valpha and Vbeta on a 
	stationary reference frame*/

	Stat_Volt_alfa_beta = Rev_Park(Stat_Volt_q_d);	 //��PARK()�������ǳɶ�ʹ�ã�����thea�ǶȲ��䡣

	/*Valpha and Vbeta finally drive the power stage*/ 
	SVPWM_3ShuntCalcDutyCycles(Stat_Volt_alfa_beta);     
}

