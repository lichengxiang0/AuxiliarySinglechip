#ifndef __MY_PWM_INPUT_H__
#define __MY_PWM_INPUT_H__

#include "stm32f1xx_hal.h"

uint32_t cntTime = 0,Time3First=0,Time3Second=0,Time3Time=0;
uint8_t High_input,Low_input;
uint16_t AirOil = 0;
float pwm_test1 = 0;

extern uint8_t uc_Pack_And_Send_Buf[11];
	 /* ���Ͷ��� ------------------------------------------------------------------*/
/************************** ���벶���ⲿ���� ********************************/
typedef struct              //�����ߵ�ƽ����
{   
	uint8_t   ucFinishFlag;
	uint8_t   ucStartFlag;
	uint16_t  usCtr;
	uint16_t  usPeriod;
}STRUCT_CAPTURE;

	 
	 
// ���嶨ʱ��Ԥ��Ƶ����ʱ��ʵ��ʱ��Ƶ��Ϊ��72MHz/��GENERAL_TIMx_PRESCALER+1��
#define GENERAL_TIM_PRESCALER               71  // ʵ��ʱ��Ƶ��Ϊ��36kHz

// ���嶨ʱ�����ڣ�����ʱ����ʼ������GENERAL_TIMx_PERIODֵ�Ǹ��¶�ʱ�������ɶ�Ӧ�¼����ж�
#define GENERAL_TIM_PERIOD                  0xFFFF

#define GENERAL_TIM_STRAT_ICPolarity        TIM_INPUTCHANNELPOLARITY_RISING          //��������ʼ����
#define GENERAL_TIM_END_ICPolarity          TIM_INPUTCHANNELPOLARITY_FALLING         //�����Ľ�������



	





#endif



