/**
  ******************************************************************************
  * File Name          :pid_regulator.c
  * Description        :PI和PID的实现
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
	* @param 	hReference    		:目标值        
	* @param  hPresentFeedback  :反馈值
	* @param	PID_Struct        :PID结构体
  */
int16_t PI_Regulator(int16_t hReference, int16_t hPresentFeedback, PID_Struct_Typedef *PID_Struct)
{
  int32_t wError, wProportional_Term,wIntegral_Term, houtput_32;
  int64_t dwAux; 

 
  // error computation
  wError= (int32_t)(hReference - hPresentFeedback);		  //取得需要误差量	delta_e
  PID_Struct->hRef = hReference;
  PID_Struct->hFdb = hPresentFeedback;
  // Proportional term computation
  wProportional_Term = PID_Struct->hKp_Gain * wError;	 // wP = Kp * delta_e
														 // wP 为比例总调节量
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
			wIntegral_Term = PID_Struct->hKi_Gain * wError;		   // wI = Ki * delta_e	，本次积分项
			dwAux = PID_Struct->wIntegral + (int64_t)(wIntegral_Term);	// 积分累积的调节量 = 以前的积分累积量 + 本次的积分项
			
			if (dwAux > PID_Struct->wUpper_Limit_Integral)		   //dwAux为当前积分累积项，下面测试积分饱和度
			{
				PID_Struct->wIntegral = PID_Struct->wUpper_Limit_Integral;	// 超上限
			}
			else if (dwAux < PID_Struct->wLower_Limit_Integral)				//超下限
			{ 
				PID_Struct->wIntegral = PID_Struct->wLower_Limit_Integral;
			}
			else
			{
			 PID_Struct->wIntegral = (int32_t)(dwAux);		  //不超限, 更新积分累积项为dwAux
			}
		}
  }
	
  houtput_32 = (wProportional_Term/PID_Struct->hKp_Divisor+ 	
                PID_Struct->wIntegral/PID_Struct->hKi_Divisor);
  

  PID_Struct->hOut = (int16_t)(houtput_32);
  if (houtput_32 >= PID_Struct->hUpper_Limit_Output)	   //测试输出是否饱和，超上限
  {
      return(PID_Struct->hUpper_Limit_Output);		  			 	
  }
  else if (houtput_32 < PID_Struct->hLower_Limit_Output) //超下限
	{
		return(PID_Struct->hLower_Limit_Output);
	}
	else 
	{
		return((int16_t)(houtput_32)); 						   //不超限。输出结果 houtput_32
	}
}



/**
  * @brief 	PID_Regulator
	* @note	 	微分的反馈可以直接输入，也可以由正常微分得到
	* @param 	hReference    		:目标值        
	* @param  hPresentFeedback  :反馈值
	* @param	PID_Struct        :PID结构体
  */
int16_t PID_Regulator(int16_t hReference, int16_t hPresentFeedback, int16_t	hDifferentialFeedback, PID_Struct_Typedef *PID_Struct)
{
  int32_t wError, wProportional_Term,wIntegral_Term, houtput_32;
  int64_t dwAux; 
 
 
  // error computation
  wError= (int32_t)(hReference - hPresentFeedback);		  //取得需要误差量	delta_e
  PID_Struct->hRef = hReference;
  PID_Struct->hFdb = hPresentFeedback;
  // Proportional term computation
  wProportional_Term = PID_Struct->hKp_Gain * wError;	 // wP = Kp * delta_e
														 // wP 为比例总调节量
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
			wIntegral_Term = PID_Struct->hKi_Gain * wError;		   // wI = Ki * delta_e	，本次积分项
			dwAux = PID_Struct->wIntegral + (int64_t)(wIntegral_Term);	// 积分累积的调节量 = 以前的积分累积量 + 本次的积分项
			
			if (dwAux > PID_Struct->wUpper_Limit_Integral)		   //dwAux为当前积分累积项，下面测试积分饱和度
			{
				PID_Struct->wIntegral = PID_Struct->wUpper_Limit_Integral;	// 超上限
			}
			else if (dwAux < PID_Struct->wLower_Limit_Integral)				//超下限
			{ 
				PID_Struct->wIntegral = PID_Struct->wLower_Limit_Integral;
			}
			else
			{
			 PID_Struct->wIntegral = (int32_t)(dwAux);		  //不超限, 更新积分累积项为dwAux
			}
		}

  }
	
	
  // Differential term computation
#ifdef DIFFERENTIAL_TERM_ENABLED						  //使用微分调节项
  {
		int32_t wtemp,wDifferential_Term;
		
		wtemp = wError - PID_Struct->wPreviousError;			  //取得上次和这个的误差之差
		wDifferential_Term = PID_Struct->hKd_Gain * wtemp;	  //wD = Kd * delta_d
		PID_Struct->wPreviousError = wError;    				  // 更新上次误差	
		
		houtput_32 = (wProportional_Term/PID_Struct->hKp_Divisor+     //输出总的调节量 = 比例调节量/分数因子 +
                PID_Struct->wIntegral/PID_Struct->hKi_Divisor + //				 + 积分调节量/分数因子
                wDifferential_Term/PID_Struct->hKd_Divisor); 	//				 + 微分调节量/分数因子
  }
#else 
	{
		int32_t wDifferential_Term;
	
		wDifferential_Term = PID_Struct->hKd_Gain*hDifferentialFeedback;
	
		houtput_32 = (wProportional_Term/PID_Struct->hKp_Divisor+     //输出总的调节量 = 比例调节量/分数因子 +
                PID_Struct->wIntegral/PID_Struct->hKi_Divisor + //				 + 积分调节量/分数因子
                wDifferential_Term/PID_Struct->hKd_Divisor); 	//				 + 微分调节量/分数因子
	}
#endif
  	

  PID_Struct->hOut = (int16_t)(houtput_32);
  if (houtput_32 >= PID_Struct->hUpper_Limit_Output)	   //测试输出是否饱和，超上限
  {
      return(PID_Struct->hUpper_Limit_Output);		  			 	
  }
  else if (houtput_32 < PID_Struct->hLower_Limit_Output) //超下限
	{
		return(PID_Struct->hLower_Limit_Output);
	}
	else 
	{
		return((int16_t)(houtput_32)); 						   //不超限。输出结果 houtput_32
	}
}



/**
  * @brief 	Fast_PI_Regulator
	* @note	 	这个PID是个特殊的PID运算，为了节省中断中执行时间，所以减少了除法运算，将divider设为移位
						运算
	* @param 	hReference    		:目标值        
	* @param  hPresentFeedback  :反馈值
	* @param	PID_Struct        :PID结构体
  */
int16_t Fast_PI_Regulator(int16_t hReference, int16_t hPresentFeedback, PID_Struct_Typedef *PID_Struct)
{
  int32_t wError, wProportional_Term,wIntegral_Term, houtput_32;
  int64_t dwAux; 

 
  // error computation
  wError= (int32_t)(hReference - hPresentFeedback);		  //取得需要误差量	delta_e
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
			wIntegral_Term = PID_Struct->hKi_Gain * wError;		   // wI = Ki * delta_e	，本次积分项
			dwAux = PID_Struct->wIntegral + (int64_t)(wIntegral_Term);	// 积分累积的调节量 = 以前的积分累积量 + 本次的积分项
			
			if (dwAux > PID_Struct->wUpper_Limit_Integral)		   //dwAux为当前积分累积项，下面测试积分饱和度
			{
				PID_Struct->wIntegral = PID_Struct->wUpper_Limit_Integral;	// 超上限
			}
			else if (dwAux < PID_Struct->wLower_Limit_Integral)				//超下限
			{ 
				PID_Struct->wIntegral = PID_Struct->wLower_Limit_Integral;
			}
			else
			{
			 PID_Struct->wIntegral = (int32_t)(dwAux);		  //不超限, 更新积分累积项为dwAux
			}
		}
  }
	
	/*移位运算可节省50us的时间*/
  houtput_32 = (wProportional_Term >> 8) + (PID_Struct->wIntegral >> 12); 

  PID_Struct->hOut = (int16_t)(houtput_32);
	
  if (houtput_32 >= PID_Struct->hUpper_Limit_Output)	   //测试输出是否饱和，超上限
  {
      return(PID_Struct->hUpper_Limit_Output);		  			 	
  }
  else if (houtput_32 < PID_Struct->hLower_Limit_Output) //超下限
	{
		return(PID_Struct->hLower_Limit_Output);
	}
	else 
	{
		return((int16_t)(houtput_32)); 						   //不超限。输出结果 houtput_32
	}
}
