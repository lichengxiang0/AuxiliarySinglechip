#ifndef _PID_REGULATOR_H
#define	_PID_REGULATOR_H

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
 extern "C" {
#endif
	 

/* Exported types ------------------------------------------------------------*/
typedef struct 
{  
  int16_t 	hKp_Gain;				//����ϵ��
  uint16_t 	hKp_Divisor;		//����ϵ������
  int16_t 	hKi_Gain;		    //����ϵ��
  uint16_t 	hKi_Divisor;  	//����ϵ������
	
  int32_t 	wIntegral;								//�����ۻ���
	int16_t		hLower_Threshold_Integral;
	int16_t		hUpper_Threshold_Integral;
	int32_t 	wLower_Limit_Integral;   //Lower Limit for Integral term limitation	����������
  int32_t 	wUpper_Limit_Integral;   //Lower Limit for Integral term limitation	����������
	
  int16_t 	hKd_Gain;			//΢��ϵ��
  uint16_t 	hKd_Divisor;		//΢��ϵ������
	int32_t 	wPreviousError;	//�ϴ����
	
	int16_t 	hLower_Limit_Output;     //Lower Limit for Output limitation			���������
  int16_t 	hUpper_Limit_Output;     //Lower Limit for Output limitation			���������
  
  int16_t 	hRef;
  int16_t 	hFdb;
  int16_t 	hOut;
} PID_Struct_Typedef;




/* Exported functions ------------------------------------------------------------*/
int16_t PI_Regulator(int16_t hReference, int16_t hPresentFeedback, PID_Struct_Typedef *PID_Struct);
int16_t PID_Regulator(int16_t hReference, int16_t hPresentFeedback, int16_t	hDifferentialFeedback, PID_Struct_Typedef *PID_Struct);
int16_t Fast_PI_Regulator(int16_t hReference, int16_t hPresentFeedback, PID_Struct_Typedef *PID_Struct);

#ifdef __cplusplus
}
#endif	 
//int16_t PID_Regulator_D(int16_t hReference, int16_t hPresentFeedback, PID_Struct_Typedef *PID_Struct,float spd);
#endif
