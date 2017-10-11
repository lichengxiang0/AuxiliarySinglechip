#include "my_pwm_input.h"
#include "tim.h"
#include "protocol.h"


STRUCT_CAPTURE strCapture = { 0, 0, 0 };

/**
  * 函数功能: 定时器输入捕获中断回调函数
  * 输入参数: htim：定时器句柄
  * 返 回 值: 无
  * 说    明: 无
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{   
		
	
    // 获取定时器计数值
    strCapture .usCtr = HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_1);
    if ( cntTime == 0 )
		{		
			Time3First = strCapture .usCtr;
		}
		if( cntTime == 1 )
		{
			Time3Second = strCapture .usCtr;
			if(Time3Second<Time3First )
			{
				Time3Second = 65536-Time3First+Time3Second;
			}
			else
			{
				Time3Time = Time3Second - Time3First;
			}
			pwm_test1 = (float)Time3Time/1000000;
			
			AirOil = (uint16_t)(60/pwm_test1);
			Low_input = AirOil%100;  //低字节
			High_input = AirOil/100;   //高字节
			uc_Pack_And_Send_Buf[5] = Low_input;
			uc_Pack_And_Send_Buf[4] = High_input;
			
		}
		
		
    // 清除中断标志位
    __HAL_TIM_CLEAR_IT(htim, TIM_IT_CC1); 
    // 启动输入捕获并开启中断
    HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);    			
    strCapture .ucFinishFlag = 1;  		
		cntTime++;
		if ( cntTime >= 2 )
		{
			cntTime = 0;
		}
}

