/**
  ******************************************************************************
  * File Name          :pid_regulator.c
  * Description        :PI��PID��ʵ��
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
#include "pid_regulator.h"


/* External function ---------------------------------------------------------*/

/**
  * @brief 	PI_Regulator
	* @note	 	
	* @param 	hReference    		:Ŀ��ֵ        
	* @param  hPresentFeedback  :����ֵ
	* @param	PID_Struct        :PID�ṹ��
  */
int16_t PI_Regulator(int16_t hReference, int16_t hPresentFeedback, PID_Struct_Typedef *PID_Struct)
{
  int32_t wError, wProportional_Term,wIntegral_Term, houtput_32;
  int64_t dwAux; 

 
  // error computation
  wError= (int32_t)(hReference - hPresentFeedback);		  //ȡ����Ҫ�����	delta_e
  PID_Struct->hRef = hReference;
  PID_Struct->hFdb = hPresentFeedback;
  // Proportional term computation
  wProportional_Term = PID_Struct->hKp_Gain * wError;	 // wP = Kp * delta_e
														 // wP Ϊ�����ܵ�����
  // Integral term computation
  if (PID_Struct->hKi_Gain == 0)
  {
    PID_Struct->wIntegral = 0;
  }
  else
  { 
    if( (wError > PID_Struct->hUpper_Threshold_Integral)||(wError < PID_Struct->hLower_Threshold_Integral) )
		{
			wIntegral_Term = 0;
			PID_Struct->wIntegral = 0;
			dwAux = 0;
		}
		else
		{
			wIntegral_Term = PID_Struct->hKi_Gain * wError;		   // wI = Ki * delta_e	�����λ�����
			dwAux = PID_Struct->wIntegral + (int64_t)(wIntegral_Term);	// �����ۻ��ĵ����� = ��ǰ�Ļ����ۻ��� + ���εĻ�����
			
			if (dwAux > PID_Struct->wUpper_Limit_Integral)		   //dwAuxΪ��ǰ�����ۻ��������Ի��ֱ��Ͷ�
			{
				PID_Struct->wIntegral = PID_Struct->wUpper_Limit_Integral;	// ������
			}
			else if (dwAux < PID_Struct->wLower_Limit_Integral)				//������
			{ 
				PID_Struct->wIntegral = PID_Struct->wLower_Limit_Integral;
			}
			else
			{
			 PID_Struct->wIntegral = (int32_t)(dwAux);		  //������, ���»����ۻ���ΪdwAux
			}
		}
  }
	
  houtput_32 = (wProportional_Term/PID_Struct->hKp_Divisor+ 	
                PID_Struct->wIntegral/PID_Struct->hKi_Divisor);
  

  PID_Struct->hOut = (int16_t)(houtput_32);
  if (houtput_32 >= PID_Struct->hUpper_Limit_Output)	   //��������Ƿ񱥺ͣ�������
  {
      return(PID_Struct->hUpper_Limit_Output);		  			 	
  }
  else if (houtput_32 < PID_Struct->hLower_Limit_Output) //������
	{
		return(PID_Struct->hLower_Limit_Output);
	}
	else 
	{
		return((int16_t)(houtput_32)); 						   //�����ޡ������� houtput_32
	}
}



/**
  * @brief 	PID_Regulator
	* @note	 	΢�ֵķ�������ֱ�����룬Ҳ����������΢�ֵõ�
	* @param 	hReference    		:Ŀ��ֵ        
	* @param  hPresentFeedback  :����ֵ
	* @param	PID_Struct        :PID�ṹ��
  */
int16_t PID_Regulator(int16_t hReference, int16_t hPresentFeedback, int16_t	hDifferentialFeedback, PID_Struct_Typedef *PID_Struct)
{
  int32_t wError, wProportional_Term,wIntegral_Term, houtput_32;
  int64_t dwAux; 
 
 
  // error computation
  wError= (int32_t)(hReference - hPresentFeedback);		  //ȡ����Ҫ�����	delta_e
  PID_Struct->hRef = hReference;
  PID_Struct->hFdb = hPresentFeedback;
  // Proportional term computation
  wProportional_Term = PID_Struct->hKp_Gain * wError;	 // wP = Kp * delta_e
														 // wP Ϊ�����ܵ�����
  // Integral term computation
  if (PID_Struct->hKi_Gain == 0)
  {
    PID_Struct->wIntegral = 0;
  }
  else
  { 
    if( (wError > PID_Struct->hUpper_Threshold_Integral)||(wError < PID_Struct->hLower_Threshold_Integral) )
		{
			wIntegral_Term = 0;
			PID_Struct->wIntegral = 0;
			dwAux = 0;
		}
		else
		{
			wIntegral_Term = PID_Struct->hKi_Gain * wError;		   // wI = Ki * delta_e	�����λ�����
			dwAux = PID_Struct->wIntegral + (int64_t)(wIntegral_Term);	// �����ۻ��ĵ����� = ��ǰ�Ļ����ۻ��� + ���εĻ�����
			
			if (dwAux > PID_Struct->wUpper_Limit_Integral)		   //dwAuxΪ��ǰ�����ۻ��������Ի��ֱ��Ͷ�
			{
				PID_Struct->wIntegral = PID_Struct->wUpper_Limit_Integral;	// ������
			}
			else if (dwAux < PID_Struct->wLower_Limit_Integral)				//������
			{ 
				PID_Struct->wIntegral = PID_Struct->wLower_Limit_Integral;
			}
			else
			{
			 PID_Struct->wIntegral = (int32_t)(dwAux);		  //������, ���»����ۻ���ΪdwAux
			}
		}

  }
	
	
  // Differential term computation
#ifdef DIFFERENTIAL_TERM_ENABLED						  //ʹ��΢�ֵ�����
  {
		int32_t wtemp,wDifferential_Term;
		
		wtemp = wError - PID_Struct->wPreviousError;			  //ȡ���ϴκ���������֮��
		wDifferential_Term = PID_Struct->hKd_Gain * wtemp;	  //wD = Kd * delta_d
		PID_Struct->wPreviousError = wError;    				  // �����ϴ����	
		
		houtput_32 = (wProportional_Term/PID_Struct->hKp_Divisor+     //����ܵĵ����� = ����������/�������� +
                PID_Struct->wIntegral/PID_Struct->hKi_Divisor + //				 + ���ֵ�����/��������
                wDifferential_Term/PID_Struct->hKd_Divisor); 	//				 + ΢�ֵ�����/��������
  }
#else 
	{
		int32_t wDifferential_Term;
	
		wDifferential_Term = PID_Struct->hKd_Gain*hDifferentialFeedback;
	
		houtput_32 = (wProportional_Term/PID_Struct->hKp_Divisor+     //����ܵĵ����� = ����������/�������� +
                PID_Struct->wIntegral/PID_Struct->hKi_Divisor + //				 + ���ֵ�����/��������
                wDifferential_Term/PID_Struct->hKd_Divisor); 	//				 + ΢�ֵ�����/��������
	}
#endif
  	

  PID_Struct->hOut = (int16_t)(houtput_32);
  if (houtput_32 >= PID_Struct->hUpper_Limit_Output)	   //��������Ƿ񱥺ͣ�������
  {
      return(PID_Struct->hUpper_Limit_Output);		  			 	
  }
  else if (houtput_32 < PID_Struct->hLower_Limit_Output) //������
	{
		return(PID_Struct->hLower_Limit_Output);
	}
	else 
	{
		return((int16_t)(houtput_32)); 						   //�����ޡ������� houtput_32
	}
}



/**
  * @brief 	Fast_PI_Regulator
	* @note	 	���PID�Ǹ������PID���㣬Ϊ�˽�ʡ�ж���ִ��ʱ�䣬���Լ����˳������㣬��divider��Ϊ��λ
						����
	* @param 	hReference    		:Ŀ��ֵ        
	* @param  hPresentFeedback  :����ֵ
	* @param	PID_Struct        :PID�ṹ��
  */
int16_t Fast_PI_Regulator(int16_t hReference, int16_t hPresentFeedback, PID_Struct_Typedef *PID_Struct)
{
  int32_t wError, wProportional_Term,wIntegral_Term, houtput_32;
  int64_t dwAux; 

 
  // error computation
  wError= (int32_t)(hReference - hPresentFeedback);		  //ȡ����Ҫ�����	delta_e
  PID_Struct->hRef = hReference;
  PID_Struct->hFdb = hPresentFeedback;
	
  // Proportional term computation
  wProportional_Term = PID_Struct->hKp_Gain * wError;	 // wP = Kp * delta_e
														 
  // Integral term computation
  if (PID_Struct->hKi_Gain == 0)
  {
    PID_Struct->wIntegral = 0;
  }
  else
  {    
		if( (wError > PID_Struct->hUpper_Threshold_Integral)||(wError < PID_Struct->hLower_Threshold_Integral) )
		{
			wIntegral_Term = 0;
			PID_Struct->wIntegral = 0;
			dwAux = 0;
		}
		else
		{
			wIntegral_Term = PID_Struct->hKi_Gain * wError;		   // wI = Ki * delta_e	�����λ�����
			dwAux = PID_Struct->wIntegral + (int64_t)(wIntegral_Term);	// �����ۻ��ĵ����� = ��ǰ�Ļ����ۻ��� + ���εĻ�����
			
			if (dwAux > PID_Struct->wUpper_Limit_Integral)		   //dwAuxΪ��ǰ�����ۻ��������Ի��ֱ��Ͷ�
			{
				PID_Struct->wIntegral = PID_Struct->wUpper_Limit_Integral;	// ������
			}
			else if (dwAux < PID_Struct->wLower_Limit_Integral)				//������
			{ 
				PID_Struct->wIntegral = PID_Struct->wLower_Limit_Integral;
			}
			else
			{
			 PID_Struct->wIntegral = (int32_t)(dwAux);		  //������, ���»����ۻ���ΪdwAux
			}
		}
  }
	
	/*��λ����ɽ�ʡ50us��ʱ��*/
  houtput_32 = (wProportional_Term >> 8) + (PID_Struct->wIntegral >> 12); 

  PID_Struct->hOut = (int16_t)(houtput_32);
	
  if (houtput_32 >= PID_Struct->hUpper_Limit_Output)	   //��������Ƿ񱥺ͣ�������
  {
      return(PID_Struct->hUpper_Limit_Output);		  			 	
  }
  else if (houtput_32 < PID_Struct->hLower_Limit_Output) //������
	{
		return(PID_Struct->hLower_Limit_Output);
	}
	else 
	{
		return((int16_t)(houtput_32)); 						   //�����ޡ������� houtput_32
	}
}
