#ifndef __MY_ADC_H__
#define __MY_ADC_H__
#include "stdint.h"
#include "stm32f1xx_hal.h"

//// 用于保存转换计算后的电压值	 
//float ADC_ConvertedValueLocal;
//uint16_t adcx;
//// AD转换结果值
//__IO uint16_t ADC_ConvertedValue;




void Start_ADC_Conv_Period(void);
//uint16_t Get_Adc_Average(uint8_t times);
//uint16_t Get_Adc(void);


#endif



