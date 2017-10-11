#ifndef	_STM32F0x_SVPWM_3SHUNT_H
#define	_STM32F0x_SVPWM_3SHUNT_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "MC_type.h"
#include "MC_const.h"
	 
/* Exported types ------------------------------------------------------------*/
/**
  * @brief  Motor Current Information 
  */	 
typedef	struct
{
	/* adc转换完成标志，用来进行电流校准 */
	FlagStatus	en_update_flag;		
	
	/* 相电流 */
	uint16_t		uw_ia;
	uint16_t		uw_ib;
	
	/* 相电流偏移 */
	uint16_t		uw_ia_offset;
	uint16_t		uw_ib_offset;
	
}Motor_Current_Info_Typedef;	
	 

/* define freq */	 
#define CKTIM	((u32)64000000uL) 	/* Silicon running at 48MHz Resolution: 1Hz */////////////////////// PWM Frequency ///////////////////////////////////

/****	 Pattern type is center aligned  ****/
#define PWM_PRSC ((u8)0)
	 
	 
#define PWM_FREQ ((u16) 16000)
	 
	 
/* define time */	 	 
#define DEADTIME_NS	((u16)100)	 //100
	
	 
//700ns,实际的采样时间为2800,两路串联采样
#define SAMPLING_TIME_NS   	2800  


#if (SAMPLING_TIME_NS == 1400)
#define SAMPLING_TIME_CK  ADC_SampleTime_1Cycles5
#elif (SAMPLING_TIME_NS == 2800)
#define SAMPLING_TIME_CK  ADC_SampleTime_7Cycles5
#elif (SAMPLING_TIME_NS == 4800)
#define SAMPLING_TIME_CK  ADC_SampleTime_13Cycles5
#warning "Sampling time is not a possible value"
#endif


#define TNOISE_NS 1550    //2.55usec
#define TRISE_NS 1550     //2.55usec


/* 计算时间到 TIMER_CNT 的转换*/                            
#define PWM_PERIOD ((u16) (CKTIM / (u32)(2 * PWM_FREQ *(PWM_PRSC+1)))) 
        
////////////////////////////// Deadtime Value /////////////////////////////////
#define DEADTIME  (u16)((unsigned long long)CKTIM/2 \
          *(unsigned long long)DEADTIME_NS/1000000000uL) 
						
#define SAMPLING_TIME (u16)(((u16)(SAMPLING_TIME_NS) * 48uL)/1000uL) 
#define TNOISE 				(u16)((((u16)(TNOISE_NS)) * 48uL)/1000uL)
#define TRISE 				(u16)((((u16)(TRISE_NS)) * 48uL)/1000uL)
#define TDEAD 				(u16)((DEADTIME_NS * 48uL)/1000uL)

#if (TNOISE_NS > TRISE_NS)
  #define MAX_TNTR_NS TNOISE_NS
#else
  #define MAX_TNTR_NS TRISE_NS
#endif

#define TW_AFTER 						((u16)(((DEADTIME_NS+MAX_TNTR_NS)*48ul)/1000ul))
#define TW_BEFORE 					(((u16)(((((u16)(SAMPLING_TIME_NS)))*48ul)/1000ul))+1)
#define TW_DT_TN_TS	 				((u16)(((DEADTIME_NS+MAX_TNTR_NS+SAMPLING_TIME_NS)*48ul)/1000ul))
#define TW_DT_TR_TS	 				((u16)(((DEADTIME_NS+TRISE_NS+SAMPLING_TIME_NS)*48ul)/1000ul))
#define TW_DT_TN_TS_HALF		((u16)(TW_DT_TN_TS/2))
#define	TW_TEST			 				((u16)10)//((u16)(((100)*48ul)/1000ul) + 1)	


/* External variables --------------------------------------------------------*/
/* adc转换完成标志，用来进行电流校准 
extern	volatile	FlagStatus	en_new_current_data_ok_flag;
*/

/* 相电流 
extern	volatile	uint16_t	uw_motor_phase_current[2];					
*/	

/* 电机电流信息 */
extern	volatile	Motor_Current_Info_Typedef	st_motor_current;	
	
/* External function --------------------------------------------------------*/
					
void SVPWM_3ShuntCurrentReadingCalibration(FlagStatus *en_current_update_p);
Curr_Components SVPWM_3ShuntGetPhaseCurrentValues(void);
void SVPWM_3ShuntCalcDutyCycles(Volt_Components Stat_Volt_Input);


void	Set_Pwm_Chanle1_Compare(uint16_t	uw_data_p);
void	Set_Pwm_Chanle2_Compare(uint16_t	uw_data_p);
void	Set_Pwm_Chanle3_Compare(uint16_t	uw_data_p);
void	Set_Pwm_Chanle4_Compare(uint16_t	uw_data_p);

#ifdef __cplusplus
}
#endif	 

#endif
