/**
  ******************************************************************************
  * File Name          : stm32_bsp_tiemr.h
  * Description        : TIMER相关函数：1->输入捕获函数的启动和捕获处理和捕获波形
													2->PWM_TIMER的启动，PWM的开启关闭，3->用户用秒表的函数实现
													4->相关PWM的宏定义
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
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _STM32_BSP_TIMER_H
#define _STM32_BSP_TIMER_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "stm32_bsp_peripheral_init_export.h"
	 
	 
/* Exported typedef --------------------------------------------------------*/
typedef enum{USER_TIMER_STOP = 0,	USER_TIMER_RUN,	USER_TIMER_PAUSE}User_Timer_State_Typedef;
typedef	struct
{
	User_Timer_State_Typedef	en_timer_state;	
	uint32_t						ul_timer_start_cnt;
	uint32_t						ul_timer_now_cnt;
	uint32_t						ul_timer_once_cnt;
	uint32_t						ul_timer_pause_cnt;
	uint32_t						ul_timer_cnt;
}User_Timer_Typedef;

/* Exported define --------------------------------------------------------*/
#define	PWM_TIMER							(htim1)

#define	USER_TIMER_INIT_VALUE	{USER_TIMER_STOP,0,0,0,0,0}

/* Exported functions --------------------------------------------------------*/
/*输入捕获TIMER*/
void	Statr_Microsecond_Timer(void);

/**/
void	Start_Pwm_Tiemr(void);
void	Enable_Pwm_Output(void);
void	Disable_Pwm_Output(void);
	
/**/
float Get_Function_Period(uint32_t	*ul_last_sys_micros);

/**/
void	Update_User_Timer_Cnt(User_Timer_Typedef *st_user_timer_p);
void	Start_User_Timer(User_Timer_Typedef *st_user_timer_p);
void	Stop_User_Timer(User_Timer_Typedef *st_user_timer_p);
void	Reset_User_Timer(User_Timer_Typedef *st_user_timer_p);

void	Pause_User_Timer(User_Timer_Typedef *st_user_timer_p);
void	Resume_User_Timer(User_Timer_Typedef *st_user_timer_p);

#ifdef __cplusplus
}
#endif

#endif
