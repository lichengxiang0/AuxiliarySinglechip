#include "my_adc.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "stm32_bsp_timer.h"
#include "system_control.h"
#include "protocol.h"

// ���ڱ���ת�������ĵ�ѹֵ	 
float ADC_ConvertedValueLocal;
uint16_t adcx;
// ADת�����ֵ
__IO uint16_t ADC_ConvertedValue;

extern uint8_t uc_Pack_And_Send_Buf[11];

uint16_t Get_Adc()
{
	HAL_ADC_Start(&hadc1);
	
	HAL_ADC_PollForConversion(&hadc1,50);
	
	if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1),HAL_ADC_STATE_REG_EOC))
	{
		ADC_ConvertedValue = HAL_ADC_GetValue(&hadc1);
	}
	return ADC_ConvertedValue;
}

uint16_t Get_Adc_Average(uint8_t times)
{
	uint32_t tem_val = 0;
	uint8_t t;
	for(t=0;t<times;t++)
	{
		tem_val += Get_Adc();
	}
	return tem_val/times;
}

/* ����������ADC�ɼ� */
void Start_ADC_Conv_Period()
{
//	static	User_Timer_Typedef	st_adc_conv_period_timer = USER_TIMER_INIT_VALUE;
		adcx = Get_Adc_Average(10);
		ADC_ConvertedValueLocal = (float)adcx*3.3/4096;
		uc_Pack_And_Send_Buf[6] = (uint8_t)ADC_ConvertedValueLocal;
	
//		printf("AD ת����ԭʼֵ = %X \n\r",adcx);
//		printf("val is = %f \n\r",ADC_ConvertedValueLocal);
}

