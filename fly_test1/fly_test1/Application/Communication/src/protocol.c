/**
  ******************************************************************************
  * File Name          : protocol.c
  * Description        : ����ļ���Ҫʵ��ͨ��Э��Ľ����ʹ������
	*55 AA ��ַ ����	������1 ������2 ����(����Ϊ������2��4λ) У�� ֡β
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
    	    && (CMD_HEADING_LOWBYTE == ucHeadLowByte))  		//�ж�֡ͷ�Ƿ���ȷ
	      {
            ucModuleAdd = UART_ReadChar(ptRxBuf, 2);                                     
            ucCmdLength = UART_ReadChar(ptRxBuf, 3);
						// �жϵ�ַ�Ƿ���ȷ��֡�����Ƿ��ڿ�ѡ��Χ��                                   
            if((MODULE_ADD_BYTE == ucModuleAdd)&&(ucCmdLength<=RECEIVE_DATA_MAX_LENGTH))		     
            {
							/* �ֱ𱣴�ɻ��ͺţ��쳣��Ϣ���ͻ���Ϣ */
							aircraftModel = UART_ReadChar(ptRxBuf, 4);
							abnormalInf = UART_ReadChar(ptRxBuf, 5);
							customerInf = UART_ReadChar(ptRxBuf, 6);
							/* ����ʱ���֣��� */
							hourTime = UART_ReadChar(ptRxBuf, 7);
							minuteTime = UART_ReadChar(ptRxBuf, 8);
							secondTime = UART_ReadChar(ptRxBuf, 9);
							/* GPS ��Ϣ */
							for ( i=0; i<3; i++ )
							{
								gpsLatitude[uartCnt++] = UART_ReadChar(ptRxBuf, 10+i);  //������Ϣ
							}
							uartCnt = 0;
							for ( i=0; i<3; i++ )
							{
								gpsLongitude[uartCnt++] = UART_ReadChar(ptRxBuf, 14+i);  //γ����Ϣ
							}
							uartCnt = 0;
							for ( i=0; i<3; i++ )
							{
								gpsHeight[uartCnt++] = UART_ReadChar(ptRxBuf, 18+i);  //�߶���Ϣ
							}
							uartCnt = 0;
							for ( i=0; i<3; i++ )
							{
								gpsTime[uartCnt++] = UART_ReadChar(ptRxBuf, 22+i);  //������ʱ
							}
							uartCnt = 0;
							
							if ( ( *ptRxBuf ).ucBufCnt>=( ucCmdLength+2+1 ) )  //�������У���
							{
								for ( i=0; i<ucCmdLength+3;i++ )
								{
									ucData = UART_ReadChar(ptRxBuf, 2+i);
									ucCheckSum ^= ucData;
								}
								ucTailByte = UART_ReadChar(ptRxBuf,2+i);  //֡β
								/* �ж�У���֡β�Ƿ���ȷ */
								if ( ( 0x00 == ucCheckSum ) && CMD_ENDING_BYTE == ucTailByte )
								{
									UART_DelChar( ptRxBuf,4 );  //ɾ��֡ͷ֡β��ģ���ַ������
									
									for( i=0; i<ucCmdLength; i++ )
									{
										uc_receivedata[i] = UART_GetChar(ptRxBuf);  //��ȡ�ֽ�
									}
									UART_DelChar(ptRxBuf,2+i);  //ɾ��֡β��У��
									
								}
								else
								{
									/* У���֡β�����㣬��ɾ��֡ͷ�͵�ַ�ֽ� */
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
	* @brief��void	Anasyse_Protocol(USART_Buf_TypeDef * ptRxBuf,	void (*p_fun_p)(void) )
	* @note�� ͨ��Э�����
	*	@param��USART_Buf_TypeDef * ptRxBuf�����ջ�����
	* @param��void (*p_fun_p)(void)�������������Ĵ�����ָ��
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
    	    && (CMD_HEADING_LOWBYTE == ucHeadLowByte))  		//�ж�֡ͷ�Ƿ���ȷ
	      {
            ucModuleAdd = UART_ReadChar(ptRxBuf, 2);                                     
            ucCmdLength = UART_ReadChar(ptRxBuf, 3); 
            // �жϵ�ַ�Ƿ���ȷ��֡�����Ƿ��ڿ�ѡ��Χ��                                   
            if((MODULE_ADD_BYTE == ucModuleAdd)&&(ucCmdLength<=RECEIVE_DATA_MAX_LENGTH))		     
            {
								Start_User_Timer(ptRxBuf->st_timer);
								Update_User_Timer_Cnt(ptRxBuf->st_timer);
                // �ȴ�300ms���жϻ������Ƿ�������õ��ֳ���  
                if(((*ptRxBuf).ucBufCnt>=(ucCmdLength+4+2))&&(ptRxBuf->st_timer->ul_timer_cnt<=RECEIVE_TIM_CNT))         
		      	    { 
			              Stop_User_Timer(ptRxBuf->st_timer);
			              for(ucNum=0;ucNum<ucCmdLength+3;ucNum++)				//�������У���
			              {
			                  ucData = UART_ReadChar(ptRxBuf, 2+ucNum); 	 
			                  ucCheckSum ^= ucData;
			              } 
			              ucTailByte = UART_ReadChar(ptRxBuf, 2+ucNum);
			              // �ж�У���֡β�Ƿ���ȷ
			              if((0x00==ucCheckSum)&&(CMD_ENDING_BYTE==ucTailByte))                          
										{									
											UART_DelChar(ptRxBuf,4);   //ɾ��֡ͷ֡β��ģ���ַ������
											
											for(i=0;i<ucCmdLength;i++)
											{
												uc_recivedata[i] = UART_GetChar(ptRxBuf);  //��ȡ��ɾ���ֽ�
											}
											UART_DelChar(ptRxBuf,2);   //ɾ��֡β��У��
											p_fun_p(ptRxBuf,uc_recivedata);							
			              }// end if((0x00==ucCheckSum)&&(CMD_ENDING_BYTE==ucTailByte))
			              else  // У���֡β�������㣬��ɾ��֡ͷ�͵�ַ�ֽ�                                                                        
			              {
			                  UART_DelChar(ptRxBuf,1);
		                }	
		            }	/*end if(((*ptRxBuf).ucBufCnt>=(ucDataLength+4+2))&&(uwGetNum<=RECEIVE_TIM_CNT))*/
		            // �������300ms ���ݻ�δ��������������ɾ��֡ͷ�͵�ַ�ֽ�															               
		            else if(((*ptRxBuf).ucBufCnt <= (ucCmdLength+4+2)) 
		        	    && (ptRxBuf->st_timer->ul_timer_cnt >= RECEIVE_TIM_CNT))
		            {
		                Stop_User_Timer(ptRxBuf->st_timer);	//	ֹͣ��ʱ
		                UART_DelChar(ptRxBuf,1);			  // ɾ��֡ͷ�͵�ַ�ֽ�
		            }
		        }//end if((CMD_ADD_BYTE == ucMCUAdd) && (0 < ucDataLength)&&(ucDataLength <= 250))
		        else	
		        {				
		            UART_DelChar(ptRxBuf,1);	// ��ַ���ԣ�ɾ��֡ͷ
		        }
	      }//end  if((CMD_HEADING_HIGHBYTE == ucHeadHighByte)&& (CMD_HEADING_LOWBYTE == ucHeadLowByte))
	      else 
	      {
	          UART_DelChar(ptRxBuf,1);	// ֡ͷ���ԣ�ɾ��֡ͷ���ֽ�
		  }
    }//end if (((*ptRxBuf).ucBufCnt) >= 6)
    
}// end void ProcUartCmd(UART_BUF * ptRxBuf)

#endif

/**
	* @brief��void Pack_And_Send_Cmd(USART_Buf_TypeDef * stTxBuf,	uint8_t *uc_cmd_data_p,uint8_t uc_module_addr_p)
	* @note�� ͨ��Э��������
	*	@param��USART_Buf_TypeDef * stTxBuf������ʹ�õĻ�����
	* @param��uint8_t *uc_cmd_data_p����������ָ��
	* @param��uint8_t uc_module_addr_p������ģ���ַ
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







