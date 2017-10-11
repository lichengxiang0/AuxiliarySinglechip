/**
  ******************************************************************************
  * File Name          : protocol.c
  * Description        : 这个文件主要实现通信协议的解析和打包发送
	*55 AA 地址 长度	命令字1 命令字2 数据(长度为命令字2低4位) 校验 帧尾
	*55 AA 0x01 0x17	0x02 		0x0D     Data0-Data14            XOR  0xF0
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

#include "protocol.h"

#include "stm32_bsp_uart.h"
#include "stm32_bsp_timer.h"



void	Task_Anasyse_Protocol(USART_Buf_TypeDef * ptRxBuf,	void (*p_fun_p)(USART_Buf_TypeDef * ptRxBuf, uint8_t * data_array_p) )
{	
    uint8_t  ucHeadHighByte, ucHeadLowByte, ucTailByte, ucModuleAdd, ucCmdLength,ucData,ucCheckSum;
	uint8_t aircraftModel,abnormalInf,customerInf;
	uint8_t hourTime,minuteTime,secondTime;
	uint8_t gpsLatitude[4],gpsLongitude[4],gpsHeight[4],gpsTime[4];
	uint8_t i,uartCnt=0;
	uint8_t	uc_receivedata[RECEIVE_DATA_MAX_LENGTH];
	
	if(  ( ( *ptRxBuf ).ucBufCnt ) == 28 )
	{
		ucHeadHighByte = UART_ReadChar(ptRxBuf,0);
		ucHeadLowByte = UART_ReadChar(ptRxBuf,1);
		if((CMD_HEADING_HIGHBYTE == ucHeadHighByte)
    	    && (CMD_HEADING_LOWBYTE == ucHeadLowByte))  		//判断帧头是否正确
	      {
            ucModuleAdd = UART_ReadChar(ptRxBuf, 2);                                     
            ucCmdLength = UART_ReadChar(ptRxBuf, 3);
						// 判断地址是否正确，帧长度是否在可选范围内                                   
            if((MODULE_ADD_BYTE == ucModuleAdd)&&(ucCmdLength<=RECEIVE_DATA_MAX_LENGTH))		     
            {
							/* 分别保存飞机型号，异常信息，客户信息 */
							aircraftModel = UART_ReadChar(ptRxBuf, 4);
							abnormalInf = UART_ReadChar(ptRxBuf, 5);
							customerInf = UART_ReadChar(ptRxBuf, 6);
							/* 保存时：分：秒 */
							hourTime = UART_ReadChar(ptRxBuf, 7);
							minuteTime = UART_ReadChar(ptRxBuf, 8);
							secondTime = UART_ReadChar(ptRxBuf, 9);
							/* GPS 信息 */
							for ( i=0; i<3; i++ )
							{
								gpsLatitude[uartCnt++] = UART_ReadChar(ptRxBuf, 10+i);  //经度信息
							}
							uartCnt = 0;
							for ( i=0; i<3; i++ )
							{
								gpsLongitude[uartCnt++] = UART_ReadChar(ptRxBuf, 14+i);  //纬度信息
							}
							uartCnt = 0;
							for ( i=0; i<3; i++ )
							{
								gpsHeight[uartCnt++] = UART_ReadChar(ptRxBuf, 18+i);  //高度信息
							}
							uartCnt = 0;
							for ( i=0; i<3; i++ )
							{
								gpsTime[uartCnt++] = UART_ReadChar(ptRxBuf, 22+i);  //卫星受时
							}
							uartCnt = 0;
							
							if ( ( *ptRxBuf ).ucBufCnt>=( ucCmdLength+2+1 ) )  //计算异或校验和
							{
								for ( i=0; i<ucCmdLength+3;i++ )
								{
									ucData = UART_ReadChar(ptRxBuf, 2+i);
									ucCheckSum ^= ucData;
								}
								ucTailByte = UART_ReadChar(ptRxBuf,2+i);  //帧尾
								/* 判断校验和帧尾是否正确 */
								if ( ( 0x00 == ucCheckSum ) && CMD_ENDING_BYTE == ucTailByte )
								{
									UART_DelChar( ptRxBuf,4 );  //删除帧头帧尾，模块地址，长度
									
									for( i=0; i<ucCmdLength; i++ )
									{
										uc_receivedata[i] = UART_GetChar(ptRxBuf);  //读取字节
									}
									UART_DelChar(ptRxBuf,2+i);  //删除帧尾和校验
									
								}
								else
								{
									/* 校验和帧尾不满足，则删掉帧头和地址字节 */
									UART_DelChar( ptRxBuf,1 );
								}
								
							}
						
							
						}
	}

}

    
}// end void ProcUartCmd(UART_BUF * ptRxBuf)



#if 0
/* External functions ------------------------------------------------------------*/
/**
	* @brief：void	Anasyse_Protocol(USART_Buf_TypeDef * ptRxBuf,	void (*p_fun_p)(void) )
	* @note： 通信协议解析
	*	@param：USART_Buf_TypeDef * ptRxBuf：接收缓冲区
	* @param：void (*p_fun_p)(void)：解析后对命令的处理函数指针
	*/
void	Task_Anasyse_Protocol(USART_Buf_TypeDef * ptRxBuf,	void (*p_fun_p)(USART_Buf_TypeDef * ptRxBuf, uint8_t * data_array_p) )
{	
    uint8_t  ucHeadHighByte, ucHeadLowByte, ucTailByte, ucModuleAdd, ucCmdLength,ucData;
		uint8_t  ucNum;
    uint8_t  ucCheckSum=0,i;
	
		uint8_t	uc_recivedata[RECEIVE_DATA_MAX_LENGTH];

    if (((*ptRxBuf).ucBufCnt) >= 6)					 
    {	
        ucHeadHighByte = UART_ReadChar(ptRxBuf, 0);          
        ucHeadLowByte  = UART_ReadChar(ptRxBuf, 1);			 
        if((CMD_HEADING_HIGHBYTE == ucHeadHighByte)
    	    && (CMD_HEADING_LOWBYTE == ucHeadLowByte))  		//判断帧头是否正确
	      {
            ucModuleAdd = UART_ReadChar(ptRxBuf, 2);                                     
            ucCmdLength = UART_ReadChar(ptRxBuf, 3); 
            // 判断地址是否正确，帧长度是否在可选范围内                                   
            if((MODULE_ADD_BYTE == ucModuleAdd)&&(ucCmdLength<=RECEIVE_DATA_MAX_LENGTH))		     
            {
								Start_User_Timer(ptRxBuf->st_timer);
								Update_User_Timer_Cnt(ptRxBuf->st_timer);
                // 等待300ms，判断缓冲区是否充满设置的字长度  
                if(((*ptRxBuf).ucBufCnt>=(ucCmdLength+4+2))&&(ptRxBuf->st_timer->ul_timer_cnt<=RECEIVE_TIM_CNT))         
		      	    { 
			              Stop_User_Timer(ptRxBuf->st_timer);
			              for(ucNum=0;ucNum<ucCmdLength+3;ucNum++)				//计算异或校验和
			              {
			                  ucData = UART_ReadChar(ptRxBuf, 2+ucNum); 	 
			                  ucCheckSum ^= ucData;
			              } 
			              ucTailByte = UART_ReadChar(ptRxBuf, 2+ucNum);
			              // 判断校验和帧尾是否正确
			              if((0x00==ucCheckSum)&&(CMD_ENDING_BYTE==ucTailByte))                          
										{									
											UART_DelChar(ptRxBuf,4);   //删除帧头帧尾、模块地址、长度
											
											for(i=0;i<ucCmdLength;i++)
											{
												uc_recivedata[i] = UART_GetChar(ptRxBuf);  //读取并删除字节
											}
											UART_DelChar(ptRxBuf,2);   //删除帧尾和校验
											p_fun_p(ptRxBuf,uc_recivedata);							
			              }// end if((0x00==ucCheckSum)&&(CMD_ENDING_BYTE==ucTailByte))
			              else  // 校验和帧尾不不满足，则删掉帧头和地址字节                                                                        
			              {
			                  UART_DelChar(ptRxBuf,1);
		                }	
		            }	/*end if(((*ptRxBuf).ucBufCnt>=(ucDataLength+4+2))&&(uwGetNum<=RECEIVE_TIM_CNT))*/
		            // 如果到达300ms 数据还未充满缓冲区，则删掉帧头和地址字节															               
		            else if(((*ptRxBuf).ucBufCnt <= (ucCmdLength+4+2)) 
		        	    && (ptRxBuf->st_timer->ul_timer_cnt >= RECEIVE_TIM_CNT))
		            {
		                Stop_User_Timer(ptRxBuf->st_timer);	//	停止计时
		                UART_DelChar(ptRxBuf,1);			  // 删掉帧头和地址字节
		            }
		        }//end if((CMD_ADD_BYTE == ucMCUAdd) && (0 < ucDataLength)&&(ucDataLength <= 250))
		        else	
		        {				
		            UART_DelChar(ptRxBuf,1);	// 地址不对，删掉帧头
		        }
	      }//end  if((CMD_HEADING_HIGHBYTE == ucHeadHighByte)&& (CMD_HEADING_LOWBYTE == ucHeadLowByte))
	      else 
	      {
	          UART_DelChar(ptRxBuf,1);	// 帧头不对，删掉帧头高字节
		  }
    }//end if (((*ptRxBuf).ucBufCnt) >= 6)
    
}// end void ProcUartCmd(UART_BUF * ptRxBuf)

#endif

/**
	* @brief：void Pack_And_Send_Cmd(USART_Buf_TypeDef * stTxBuf,	uint8_t *uc_cmd_data_p,uint8_t uc_module_addr_p)
	* @note： 通信协议打包发送
	*	@param：USART_Buf_TypeDef * stTxBuf：发送使用的缓冲区
	* @param：uint8_t *uc_cmd_data_p：发送数据指针
	* @param：uint8_t uc_module_addr_p：发送模块地址
	*/
void Pack_And_Send_Cmd(USART_Buf_TypeDef * stTxBuf,	uint8_t *uc_cmd_data_p,uint8_t uc_module_addr_p)
{
	uint8_t uc_check_sum=0,i=0,cnt=0;
	uint8_t	uc_cmd_length;
	uint8_t	uc_send_cmd_data[RECEIVE_DATA_MAX_LENGTH];
	
	uc_cmd_length = (uc_cmd_data_p[1]&0x0F);

	uc_send_cmd_data[cnt++] = 0x55;
	uc_send_cmd_data[cnt++] = 0xAA;
	
	uc_send_cmd_data[cnt++] = uc_module_addr_p;
	uc_check_sum = uc_check_sum^uc_module_addr_p;
	
	uc_send_cmd_data[cnt++] = uc_cmd_length + 2;
	uc_check_sum = uc_check_sum^(uc_cmd_length + 2);
	
	uc_send_cmd_data[cnt++] = ( (uint8_t)(uc_cmd_data_p[0]) );
	uc_check_sum = uc_check_sum^( (uint8_t)(uc_cmd_data_p[0]) );
	
	uc_send_cmd_data[cnt++] = (uint8_t)uc_cmd_data_p[1];
	uc_check_sum = uc_check_sum^( (uint8_t)uc_cmd_data_p[1] );
	
	for(i=0;i<uc_cmd_length;i++)
	{
		uc_send_cmd_data[cnt++] = uc_cmd_data_p[i+2];
		uc_check_sum = uc_check_sum^uc_cmd_data_p[i+2];
	}
	
	uc_send_cmd_data[cnt++] = uc_check_sum;
	uc_send_cmd_data[cnt++] = 0xF0;
	
	for(i = 0;i < cnt;i++)
	{
		UART_PutChar(stTxBuf,uc_send_cmd_data[i]);
	}
}







