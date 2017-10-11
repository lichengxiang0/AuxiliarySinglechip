/**
  ******************************************************************************
  * File Name          : stm32f0x_svpwm_3shunt.c
  * Description        : This file provides code to control motor current sample and pwm drive.
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
#include "stm32f0x_svpwm_3shunt.h"

/* Private define ---------------------------------------------------------*/
#define T		    		(PWM_PERIOD * 4)
#define T_SQRT3     (uint16_t)(T * SQRT_3)

#define SECTOR_1	((uint8_t)1)
#define SECTOR_2	((uint8_t)2)
#define SECTOR_3	((uint8_t)3)
#define SECTOR_4	((uint8_t)4)
#define SECTOR_5	((uint8_t)5)
#define SECTOR_6	((uint8_t)6)

/* Private variables ---------------------------------------------------------*/
/* 相电流偏移 
uint16_t	uw_motor_phase_current_offset[2]; 
*/

uint16_t	bSector;


/* External variables --------------------------------------------------------*/
/* adc转换完成标志，用来进行电流校准 
volatile	FlagStatus	en_new_current_data_ok_flag = RESET;
*/

/* 相电流 
volatile	uint16_t	uw_motor_phase_current[2];
*/


volatile	Motor_Current_Info_Typedef	st_motor_current;


/* 电机驱动用的定时器 
TIM_HandleTypeDef st_motor_driver_pwm_timer;
*/

/* Exported function --------------------------------------------------------*/
/**
  * @brief 	SVPWM_3ShuntCurrentReadingCalibration
	* @note	 	cal motor phase current offset
  */
void SVPWM_3ShuntCurrentReadingCalibration(FlagStatus *en_current_update_p)
{
  uint16_t 	bIndex;
	uint32_t	ul_phase_a_offset_sum = 0;
	uint32_t	ul_phase_b_offset_sum = 0;

  for(bIndex=0; bIndex < 10; bIndex++)
  {
    /*wait the ADC1 JEOC pending flag */	
		do
		{
		}while(SET != *en_current_update_p);
			
    ul_phase_a_offset_sum += st_motor_current.uw_ia;
    ul_phase_b_offset_sum += st_motor_current.uw_ib;
    
    /* Clear the ADC1 JEOC pending flag */
		*en_current_update_p = RESET;
  }

	st_motor_current.uw_ia_offset = ul_phase_a_offset_sum/10;
	st_motor_current.uw_ib_offset = ul_phase_b_offset_sum/10;
}




/**
  * @brief 	SVPWM_3ShuntGetPhaseCurrentValues
	* @note	 	cal motor phase current
  */
Curr_Components SVPWM_3ShuntGetPhaseCurrentValues(void)
{
  Curr_Components Local_Stator_Currents;
  int16_t wAux;  
	
	wAux = (int16_t)( st_motor_current.uw_ia_offset - st_motor_current.uw_ia );  
	//Saturation of Ia 
	if (wAux < S16_MIN)
	{
			Local_Stator_Currents.qI_Component1= S16_MIN;
	}  
	else  if (wAux > S16_MAX)
	{ 
			Local_Stator_Currents.qI_Component1= S16_MAX;
	}
	else
	{
			Local_Stator_Currents.qI_Component1= wAux;
	}
									
	
	wAux = (int16_t)( st_motor_current.uw_ib_offset - st_motor_current.uw_ib ); 							
	if (wAux < S16_MIN)
	{
			Local_Stator_Currents.qI_Component2= S16_MIN;
	}  
	else  if (wAux > S16_MAX)
	{ 
			Local_Stator_Currents.qI_Component2= S16_MAX;
	}
	else
	{
			Local_Stator_Currents.qI_Component2= wAux;
	}

	Local_Stator_Currents.qI_Component1 = -Local_Stator_Currents.qI_Component1;
	Local_Stator_Currents.qI_Component2 = -Local_Stator_Currents.qI_Component2;
	
	return(Local_Stator_Currents); 	
}



/**
  * @brief 	SVPWM_3ShuntCalcDutyCycles
	* @note	 	计算三路PWM波的占空比
  */

void SVPWM_3ShuntCalcDutyCycles(Volt_Components Stat_Volt_Input)
{	
	uint8_t PWM4Direction=0;
	int32_t wX, wY, wZ, wUAlpha, wUBeta;

	uint16_t  hTimePhA=0, hTimePhB=0, hTimePhC=0, hTimePhD=0;
	uint16_t  hDeltaDuty;

	wUAlpha = Stat_Volt_Input.qV_Component1 * T_SQRT3 ;
	wUBeta = -(Stat_Volt_Input.qV_Component2 * T);

	wX = wUBeta;
	wY = (wUBeta + wUAlpha)/2;
	wZ = (wUBeta - wUAlpha)/2;

	// Sector calculation from wX, wY, wZ
	if (wY<0)
	{
		if (wZ<0)
		{
			bSector = SECTOR_5;
		}
		else // wZ >= 0
			if (wX<=0)
			{
				bSector = SECTOR_4;
			}
			else // wX > 0
			{
				bSector = SECTOR_3;
			}
	}
  else // wY > 0
	{
     if (wZ>=0)
     {
       bSector = SECTOR_2;
     }
     else // wZ < 0
		 if (wX<=0)
		 {  
			 bSector = SECTOR_6;
		 }
		 else // wX > 0
		 {
			 bSector = SECTOR_1;
		 }
	}
   
   
  PWM4Direction=0;
    
  switch(bSector)
  {  
    case SECTOR_1:
                hTimePhA = (T/8) + ((((T + wX) - wZ)/2)/131072);
								hTimePhB = hTimePhA + wZ/131072;
								hTimePhC = hTimePhB - wX/131072;

//                hTimePhD = PWM_PERIOD - 1;
                // ADC Syncronization setting value 
								           
                if ((uint16_t)(PWM_PERIOD-hTimePhA) > TW_AFTER)
                {
									hTimePhD = PWM_PERIOD - 1;
                }
                else
                {
									hDeltaDuty = (uint16_t)(hTimePhA - hTimePhB);
                  
									// Definition of crossing point
                  if (hDeltaDuty > TW_DT_TR_TS) 
                  {
                      hTimePhD = hTimePhA - TW_BEFORE; // Ts before Phase A 
                  }
                  else if((uint16_t)(PWM_PERIOD-hTimePhA) >TW_DT_TN_TS_HALF)
                  {
										hTimePhD = hTimePhA + TW_AFTER; // DT + Tn after Phase A
                     
                    if (hTimePhD >= PWM_PERIOD)
                    {
                      // Trigger of ADC at Falling Edge PWM4
                      // OCR update
                      
                      //Set Polarity of CC4 Low
												PWM4Direction=1;
												
												hTimePhD = (2 * PWM_PERIOD) - hTimePhD-1;
										}
									}
//									else
//									{
//										while(1);
//									}
								}
                                                      
                break;
    case SECTOR_2:
                hTimePhA = (T/8) + ((((T + wY) - wZ)/2)/131072);
								hTimePhB = hTimePhA + wZ/131072;
								hTimePhC = hTimePhA - wY/131072;
                
//								hTimePhD = PWM_PERIOD - 1;
                // ADC Syncronization setting value
                if ((uint16_t)(PWM_PERIOD-hTimePhB) > TW_AFTER)
                {
                  hTimePhD = PWM_PERIOD - 1;
                }
                else
                {
                  hDeltaDuty = (uint16_t)(hTimePhB - hTimePhA);
                  
                  // Definition of crossing point
                  if (hDeltaDuty > TW_DT_TR_TS) 
                  {
                    hTimePhD = hTimePhB - TW_BEFORE; // Ts before Phase B 
                  }
                  else if((uint16_t)(PWM_PERIOD-hTimePhB) > TW_DT_TN_TS_HALF)
                  {
                    hTimePhD = hTimePhB + TW_AFTER; // DT + Tn after Phase B
                    
                    if (hTimePhD >= PWM_PERIOD)
                    {
                      PWM4Direction=1;
                      
                      hTimePhD = (2 * PWM_PERIOD) - hTimePhD-1;
                    }
                  }
//									else
//									{
//										while(1);
//									}
                }            	
                break;

    case SECTOR_3:
                hTimePhA = (T/8) + ((((T - wX) + wY)/2)/131072);
								hTimePhC = hTimePhA - wY/131072;
								hTimePhB = hTimePhC + wX/131072;
		
//		            hTimePhD = PWM_PERIOD - 1;
                // ADC Syncronization setting value
								
                if ((uint16_t)(PWM_PERIOD-hTimePhB) > TW_AFTER)
                {
                  hTimePhD = PWM_PERIOD - 1;
                }
                else
                {
                  hDeltaDuty = (uint16_t)(hTimePhB - hTimePhC);
                  
                  // Definition of crossing point
                  if (hDeltaDuty > TW_DT_TR_TS) 
                  {
                    hTimePhD = hTimePhB - TW_BEFORE; // Ts before Phase B 
                  }
                  else if((uint16_t)(PWM_PERIOD-hTimePhB) > TW_DT_TN_TS_HALF)
                  {
                    hTimePhD = hTimePhB + TW_AFTER; // DT + Tn after Phase B
                    
                    if (hTimePhD >= PWM_PERIOD)
                    {
                      // Trigger of ADC at Falling Edge PWM4
                      // OCR update
                      
                      //Set Polarity of CC4 Low
                      PWM4Direction=1;
                      
                      hTimePhD = (2 * PWM_PERIOD) - hTimePhD-1;
                    }
                  }
//									else
//									{
//										while(1);
//									}
                }
			
                break;
    
    case SECTOR_4:
                hTimePhA = (T/8) + ((((T + wX) - wZ)/2)/131072);
                hTimePhB = hTimePhA + wZ/131072;
                hTimePhC = hTimePhB - wX/131072;
                
//								hTimePhD = PWM_PERIOD - 1;
                // ADC Syncronization setting value
								
                if ((uint16_t)(PWM_PERIOD-hTimePhC) > TW_AFTER)
                {
                  hTimePhD = PWM_PERIOD - 1;
                }
                else
                {
                  hDeltaDuty = (uint16_t)(hTimePhC - hTimePhB);
                  
                  // Definition of crossing point
                  if (hDeltaDuty > TW_DT_TR_TS)
                  {
                    hTimePhD = hTimePhC - TW_BEFORE; // Ts before Phase C 
                  }
                  else if((uint16_t)(PWM_PERIOD-hTimePhC) > TW_DT_TN_TS_HALF)
                  {
                    hTimePhD = hTimePhC + TW_AFTER; // DT + Tn after Phase C
                    
                    if (hTimePhD >= PWM_PERIOD)
                    {
                      // Trigger of ADC at Falling Edge PWM4
                      // OCR update
                      
                      //Set Polarity of CC4 Low
                      PWM4Direction=1;
                      
                      hTimePhD = (2 * PWM_PERIOD) - hTimePhD-1;
                    }
                  }
//									else
//									{
//										while(1);
//									}
                }
			
                break;  
    
    case SECTOR_5:
                hTimePhA = (T/8) + ((((T + wY) - wZ)/2)/131072);
								hTimePhB = hTimePhA + wZ/131072;
								hTimePhC = hTimePhA - wY/131072;
                
//								hTimePhD = PWM_PERIOD - 1;
                // ADC Syncronization setting value
                if ((uint16_t)(PWM_PERIOD-hTimePhC) > TW_AFTER)
                {
                  hTimePhD = PWM_PERIOD - 1;
                }
                else
                {
                  hDeltaDuty = (uint16_t)(hTimePhC - hTimePhA);
                  
                  // Definition of crossing point
                  if (hDeltaDuty > TW_DT_TR_TS) 
                  {
                    hTimePhD = hTimePhC - TW_BEFORE; // Ts before Phase C 
                  }
                  else if((uint16_t)(PWM_PERIOD-hTimePhC) > TW_DT_TN_TS_HALF)
                  {
                    hTimePhD = hTimePhC + TW_AFTER; // DT + Tn after Phase C
                    
                    if (hTimePhD >= PWM_PERIOD)
                    {
                      // Trigger of ADC at Falling Edge PWM4
                      // OCR update
                      
                      //Set Polarity of CC4 Low
                      PWM4Direction=1;
                      
                      hTimePhD = (2 * PWM_PERIOD) - hTimePhD-1;
                    }
                  }
//									else
//									{
//										while(1);
//									}
                }
				
		break;
                
    case SECTOR_6:
                hTimePhA = (T/8) + ((((T - wX) + wY)/2)/131072);
								hTimePhC = hTimePhA - wY/131072;
								hTimePhB = hTimePhC + wX/131072;
												
								hTimePhD = PWM_PERIOD - 1;
                // ADC Syncronization setting value
								
                if ((uint16_t)(PWM_PERIOD-hTimePhA) > TW_AFTER)
                {
                  hTimePhD = PWM_PERIOD - 1;
                }
                else
                {
                  hDeltaDuty = (uint16_t)(hTimePhA - hTimePhC);
                  
                  // Definition of crossing point
                  if (hDeltaDuty > TW_DT_TR_TS) 
                  {
                    hTimePhD = hTimePhA - TW_BEFORE; // Ts before Phase A 
                  }
                  else if((uint16_t)(PWM_PERIOD-hTimePhA) > TW_DT_TN_TS_HALF)
                  {
                    hTimePhD = hTimePhA + TW_AFTER; // DT + Tn after Phase A
                    
                    if (hTimePhD >= PWM_PERIOD)
                    {
                      // Trigger of ADC at Falling Edge PWM4
                      // OCR update
                      
                      //Set Polarity of CC4 Low
                      PWM4Direction=1;
                      
                      hTimePhD = (2 * PWM_PERIOD) - hTimePhD-1;
                    }
                  }
//									else
//									{
//										while(1);
//									}
                }
				
                break;
    default:
		break;
   }
  
	 
	hTimePhD = PWM_PERIOD - TW_TEST;
  
	 
	Set_Pwm_Chanle1_Compare(hTimePhA);
	Set_Pwm_Chanle2_Compare(hTimePhB);
	Set_Pwm_Chanle3_Compare(hTimePhC);
	Set_Pwm_Chanle4_Compare(hTimePhD);	
}



/**
  * @brief 	Set_Pwm_Chanle1_Compare
	* @note	 	设置PWM通道1的占空比
  */
__weak void	Set_Pwm_Chanle1_Compare(uint16_t	uw_data_p)
{
	
}

/**
  * @brief 	Set_Pwm_Chanle2_Compare
	* @note	 	设置PWM通道2的占空比
  */
__weak void	Set_Pwm_Chanle2_Compare(uint16_t	uw_data_p)
{
	
}



/**
  * @brief 	Set_Pwm_Chanle3_Compare
	* @note	 	设置PWM通道3的占空比
  */
__weak void	Set_Pwm_Chanle3_Compare(uint16_t	uw_data_p)
{
	
}



/**
  * @brief 	Set_Pwm_Chanle4_Compare
	* @note	 	设置PWM通道4的占空比
  */
__weak void	Set_Pwm_Chanle4_Compare(uint16_t	uw_data_p)
{
	
}



