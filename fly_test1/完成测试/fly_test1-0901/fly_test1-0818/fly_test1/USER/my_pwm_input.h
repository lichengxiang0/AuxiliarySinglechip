#ifndef __MY_PWM_INPUT_H__
#define __MY_PWM_INPUT_H__

#include "stm32f1xx_hal.h"

uint32_t cntTime = 0,Time3First=0,Time3Second=0,Time3Time=0;
uint8_t High_input,Low_input;
uint16_t AirOil = 0;
float pwm_test1 = 0;

extern uint8_t uc_Pack_And_Send_Buf[11];
	 /* 类型定义 ------------------------------------------------------------------*/
/************************** 输入捕获外部变量 ********************************/
typedef struct              //测量高电平脉宽
{   
	uint8_t   ucFinishFlag;
	uint8_t   ucStartFlag;
	uint16_t  usCtr;
	uint16_t  usPeriod;
}STRUCT_CAPTURE;

	 
	 
// 定义定时器预分频，定时器实际时钟频率为：72MHz/（GENERAL_TIMx_PRESCALER+1）
#define GENERAL_TIM_PRESCALER               71  // 实际时钟频率为：36kHz

// 定义定时器周期，当定时器开始计数到GENERAL_TIMx_PERIOD值是更新定时器并生成对应事件和中断
#define GENERAL_TIM_PERIOD                  0xFFFF

#define GENERAL_TIM_STRAT_ICPolarity        TIM_INPUTCHANNELPOLARITY_RISING          //测量的起始边沿
#define GENERAL_TIM_END_ICPolarity          TIM_INPUTCHANNELPOLARITY_FALLING         //测量的结束边沿



	





#endif



