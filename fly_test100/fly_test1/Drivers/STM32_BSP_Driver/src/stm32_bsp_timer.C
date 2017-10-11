/**
  ******************************************************************************
  * File Name          : stm32_bsp_tiemr.c
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

#include "stm32f1xx_hal.h"
#include "stm32_bsp_timer.h"


/* Private define --------------------------------------------------------*/
/*系统微秒级定时器，用来计算函数执行周期，精确到us*/
#define	MICROSECOND_TIMER		(htim2)

#define	ADC_TRIG_TIMER			(htim3)



/* External functions --------------------------------------------------------*/
/**
  * @brief 	void	Statr_Microsecond_Timer(void)
	* @note	 	启动微秒级定时器
  */
void	Statr_Microsecond_Timer(void) 
{
	HAL_TIM_Base_Start(&MICROSECOND_TIMER);
}


/*---------------PWM波输出相关函数-----------------*/
/**
  * @brief 	void	Start_Pwm_Tiemr(void)
	* @note	 	启动PWM的TIMER和AD采集(AD由PWM_TIMER的通道4触发)
  */
void	Start_Pwm_Tiemr(void)
{
	HAL_TIM_Base_Start(&PWM_TIMER);								//使能PWMtimer的updata中断
	
	HAL_TIM_Base_Start(&ADC_TRIG_TIMER);					//启动AD定时器触发采集
	
	HAL_TIM_PWM_Start(&PWM_TIMER,TIM_CHANNEL_4);  //使能PWM4的比较捕获中断
}


/**
  * @brief 	void	Enable_Pwm_Output(void)
	* @note	 	使能PWM的输出
  */
void	Enable_Pwm_Output(void)
{
	HAL_TIM_PWM_Start(&PWM_TIMER,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&PWM_TIMER,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&PWM_TIMER,TIM_CHANNEL_3);
	
	
	HAL_TIMEx_PWMN_Start(&PWM_TIMER,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&PWM_TIMER,TIM_CHANNEL_3);	  //定时器和MOE都已经使能了
	HAL_TIMEx_PWMN_Start(&PWM_TIMER,TIM_CHANNEL_1);
}


/**
  * @brief 	void	Enable_Pwm_Output(void)
	* @note	 	关闭PWM的输出，不管PWM_TIEMR的CHANNEL4(触发AD)
  */
void	Disable_Pwm_Output(void)
{
	HAL_TIM_PWM_Stop(&PWM_TIMER,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&PWM_TIMER,TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&PWM_TIMER,TIM_CHANNEL_3);
	
	HAL_TIMEx_PWMN_Stop(&PWM_TIMER,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&PWM_TIMER,TIM_CHANNEL_3);	  //定时器和MOE都已经使能了
	HAL_TIMEx_PWMN_Stop(&PWM_TIMER,TIM_CHANNEL_1);   
}



/*---------------其他时间函数-----------------*/
/**
  * @brief 	float Get_Function_Period(uint32_t	*ul_last_sys_micros)
	* @note	 	计算函数执行周期(us)
  */
float Get_Function_Period(uint32_t	*ul_last_sys_micros)
{
	uint32_t	ul_now_sys_micros;
	float			period;
	
	ul_now_sys_micros = __HAL_TIM_GET_COUNTER(&MICROSECOND_TIMER);
	
	if( ul_now_sys_micros < (*ul_last_sys_micros) )
	{ 
		period =  ((float)(ul_now_sys_micros + (0xffff - (*ul_last_sys_micros) ) ) / 1000000.0f);
	}
	else	
	{
		period =  ((float)(ul_now_sys_micros - (*ul_last_sys_micros) ) / 1000000.0f);
	}
	*ul_last_sys_micros = ul_now_sys_micros;	
	
	return(period);
}
/**
  * @brief 	void	Update_User_Timer_Cnt(User_Timer_Typedef *st_user_timer_p)
	* @note	 	更新用户TIMER(秒表)的CNT
  */
void	Update_User_Timer_Cnt(User_Timer_Typedef *st_user_timer_p)
{
	st_user_timer_p->ul_timer_once_cnt = 0;
	
	if(USER_TIMER_RUN == st_user_timer_p->en_timer_state)
	{
		st_user_timer_p->ul_timer_now_cnt = HAL_GetTick();
		if(st_user_timer_p->ul_timer_now_cnt == st_user_timer_p->ul_timer_start_cnt)
		{
			st_user_timer_p->ul_timer_once_cnt = 0;
		}
		else if(st_user_timer_p->ul_timer_now_cnt > st_user_timer_p->ul_timer_start_cnt)
		{
			st_user_timer_p->ul_timer_once_cnt = st_user_timer_p->ul_timer_now_cnt - st_user_timer_p->ul_timer_start_cnt;
		}
		else 
		{
			st_user_timer_p->ul_timer_once_cnt = st_user_timer_p->ul_timer_now_cnt + 0xFFFFFFFF - st_user_timer_p->ul_timer_start_cnt;
		}
		
		st_user_timer_p->ul_timer_cnt = st_user_timer_p->ul_timer_pause_cnt + st_user_timer_p->ul_timer_once_cnt;
	}
	else if(USER_TIMER_PAUSE == st_user_timer_p->en_timer_state)
	{
		st_user_timer_p->ul_timer_cnt = st_user_timer_p->ul_timer_pause_cnt;
	}
	else
	{
		st_user_timer_p->ul_timer_cnt = 0;
	}
}


/**
  * @brief 	void	Update_User_Timer_Cnt(User_Timer_Typedef *st_user_timer_p)
	* @note	 	启动用户TIMER(秒表)
  */
void	Start_User_Timer(User_Timer_Typedef *st_user_timer_p)
{
	if(USER_TIMER_STOP == st_user_timer_p->en_timer_state)
	{
		st_user_timer_p->en_timer_state = USER_TIMER_RUN;
		
		st_user_timer_p->ul_timer_start_cnt = HAL_GetTick();
		st_user_timer_p->ul_timer_now_cnt = HAL_GetTick();
		st_user_timer_p->ul_timer_pause_cnt = 0;
		Update_User_Timer_Cnt(st_user_timer_p);
	}
}


/**
  * @brief 	void	Stop_User_Timer(User_Timer_Typedef *st_user_timer_p)
	* @note	 	停止用户TIMER(秒表)
  */
void	Stop_User_Timer(User_Timer_Typedef *st_user_timer_p)
{
	if(USER_TIMER_STOP != st_user_timer_p->en_timer_state)
	{
		st_user_timer_p->en_timer_state = USER_TIMER_STOP;
		st_user_timer_p->ul_timer_start_cnt = HAL_GetTick();
		st_user_timer_p->ul_timer_now_cnt = HAL_GetTick();
		st_user_timer_p->ul_timer_pause_cnt = 0;
		Update_User_Timer_Cnt(st_user_timer_p);
	}
}

/**
  * @brief 	void	Reset_User_Timer(User_Timer_Typedef *st_user_timer_p)
	* @note	 	复位用户TIMER(秒表)
//  */
void	Reset_User_Timer(User_Timer_Typedef *st_user_timer_p)
{
	/*只有在timer使能时进行复位*/
	if(USER_TIMER_STOP != st_user_timer_p->en_timer_state)
	{
		st_user_timer_p->en_timer_state = USER_TIMER_RUN;
		st_user_timer_p->ul_timer_start_cnt = HAL_GetTick();
		st_user_timer_p->ul_timer_now_cnt = HAL_GetTick();
		st_user_timer_p->ul_timer_pause_cnt = 0;
		Update_User_Timer_Cnt(st_user_timer_p);
	}
}



/**
  * @brief 	void	Pause_User_Timer(User_Timer_Typedef *st_user_timer_p)
	* @note	 	暂停用户TIMER(秒表)
  */
void	Pause_User_Timer(User_Timer_Typedef *st_user_timer_p)
{
	if(USER_TIMER_RUN == st_user_timer_p->en_timer_state)
	{
		st_user_timer_p->en_timer_state = USER_TIMER_PAUSE;
		st_user_timer_p->ul_timer_pause_cnt += st_user_timer_p->ul_timer_once_cnt;
		Update_User_Timer_Cnt(st_user_timer_p);
	}
}


/**
  * @brief 	void	Resume_User_Timer(User_Timer_Typedef *st_user_timer_p)
	* @note	 	恢复用户TIMER(秒表)
  */
void	Resume_User_Timer(User_Timer_Typedef *st_user_timer_p)
{
	if(USER_TIMER_PAUSE == st_user_timer_p->en_timer_state)
	{
		st_user_timer_p->en_timer_state = USER_TIMER_RUN;
		st_user_timer_p->ul_timer_start_cnt = HAL_GetTick();
		st_user_timer_p->ul_timer_now_cnt = HAL_GetTick();
		Update_User_Timer_Cnt(st_user_timer_p);
	}
}

