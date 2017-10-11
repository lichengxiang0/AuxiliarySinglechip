#include "process.h"
#include "stm32f1xx_hal.h"
#include "usart.h"
#include "stdint.h"
#include "stm32_bsp_uart.h"

#include "stm32_bsp_timer.h"

#include "led.h"
#include "sys.h"
#include "tim.h"




/*****************************************************
*ͨ��Э��������
*��һ������������ʹ�õĻ�����
*�ڶ�����������������ָ��
*����������������ģ��ĵ�ַ
********************************************************/ 
void Pack_And_Send_Cmd(USART_Buf_TypeDef *stTxBuf,uint8_t *uc_cmd_data_p,uint8_t uc_module_addr_p)
{
	uint8_t uc_check_sum = 0,i=0,cnt = 0;
//	uint8_t uc_cmd_length;
	uint8_t uc_send_cmd_data[16];
	
//	uc_cmd_length = (uc_cmd_data_p[1]&0x0F);
	
	uc_send_cmd_data[cnt++] = 0x55;
	uc_send_cmd_data[cnt++] = 0xAA;
	
	uc_send_cmd_data[cnt++] = uc_module_addr_p;  /* ģ���ַ 0x01 */
	uc_check_sum = uc_check_sum^uc_module_addr_p;
	
	uc_send_cmd_data[cnt++] = 0x0A;	/* ���� */
	uc_check_sum = uc_check_sum^0x0A;
	
	/* �ɻ���Ȩ�� */
	uc_send_cmd_data[cnt++] = ( (uint8_t)(uc_cmd_data_p[0]) );  //��Ȩ������λ����һλ
	uc_check_sum = uc_check_sum^( (uint8_t)uc_cmd_data_p[0] );
	
	uc_send_cmd_data[cnt++] = ( (uint8_t)(uc_cmd_data_p[1]) );  //�ڶ�λ
 	uc_check_sum = uc_check_sum^( (uint8_t)uc_cmd_data_p[1] );
	
	uc_send_cmd_data[cnt++] = ( (uint8_t)(uc_cmd_data_p[2]) );  //����λ
 	uc_check_sum = uc_check_sum^( (uint8_t)uc_cmd_data_p[2] );
	
	/* �ͻ�״̬ */
	uc_send_cmd_data[cnt++] = ( (uint8_t)(uc_cmd_data_p[3]) );
	uc_check_sum = uc_check_sum^( (uint8_t)uc_cmd_data_p[3] );
	
	/* �ͻ�ת�ٻ��� */
	uc_send_cmd_data[cnt++] = ( (uint8_t)(uc_cmd_data_p[4]) );
	uc_check_sum = uc_check_sum^( (uint8_t)uc_cmd_data_p[4] );
	
	/* ���� */
	uc_send_cmd_data[cnt++] = ( (uint8_t)(uc_cmd_data_p[5]) );
	uc_check_sum = uc_check_sum^( (uint8_t)uc_cmd_data_p[5] );
	
	/* �ɻ��������� */
	uc_send_cmd_data[cnt++] = ( (uint8_t)(uc_cmd_data_p[6]) );
	uc_check_sum = uc_check_sum^( (uint8_t)uc_cmd_data_p[6] );
	
	/* ��ɡ״̬ */
	uc_send_cmd_data[cnt++] = ( (uint8_t)(uc_cmd_data_p[7]) );
	uc_check_sum = uc_check_sum^( (uint8_t)uc_cmd_data_p[7] );
	
	/* ����״̬ */
	uc_send_cmd_data[cnt++] = ( (uint8_t)(uc_cmd_data_p[8]) );
	uc_check_sum = uc_check_sum^( (uint8_t)uc_cmd_data_p[8] );
	
	/* ���״̬ */
	uc_send_cmd_data[cnt++] = ( (uint8_t)(uc_cmd_data_p[9]) );
	uc_check_sum = uc_check_sum^( (uint8_t)uc_cmd_data_p[9] );
	
	/* У�飬֡β */
	uc_send_cmd_data[cnt++] = uc_check_sum;
	uc_send_cmd_data[cnt++] = 0xF0;
	
	for(i=0;i<cnt;i++)
	{
		UART_PutChar(stTxBuf,uc_send_cmd_data[i]);
	}
	
}


/* ***********************************************************
*ͨ��Э�����
*��һ�����������ջ�����
*�ڶ��������������������Ĵ�����ָ��
*/
void Task_Anasyse_Protocol(USART_Buf_TypeDef * ptRxBuf,void (*p_fun_p)(USART_Buf_TypeDef * ptRxBuf,uint8_t * data_array_p))
 {	 
	 /* ���������� */
		uint8_t  ucHeadHighByte1, ucHeadLowByte1, ucTailByte1, ucModuleAdd1, ucCmdLength1,ucData1;
		uint8_t  ucNum1;
    uint8_t  ucCheckSum1=0,j;
		uint8_t	uc_recivedata1[RECEIVE_DATA_MAX_LENGTH];
	 
		uint8_t  ucHeadHighByte2, ucHeadLowByte2, ucTailByte2, ucModuleAdd2, ucCmdLength2,ucData2;
		uint8_t  ucNum2;
    uint8_t  ucCheckSum2=0,i;
		uint8_t	uc_recivedata2[RECEIVE_DATA_MAX_LENGTH];
	
		uint8_t AircraftModule1;
	 
		uint8_t OPenUmbrella,AirLight,AirEngineOrder,AirEnginerSpeed,ChargeControl;
//		uint8_t ucNumCnt = (*ptRxBuf).ucBufCnt;
	 


	 
	 /* 28����Ϣ�ֽڴ��� */
	 if ( (*ptRxBuf).ucBufCnt == 28 )
		{
			ucHeadHighByte1 = UART_ReadChar(ptRxBuf,0);
			ucHeadLowByte1 = UART_ReadChar(ptRxBuf,1);
			
			/* �ж�֡ͷ�Ƿ���ȷ */
			if ( ( CMD_HEADING_HIGHBYTE == ucHeadHighByte1 )  && ( CMD_HEADING_LOWBYTE == ucHeadLowByte1 ) )
			{
				ucModuleAdd1 = UART_ReadChar(ptRxBuf,2);
				ucCmdLength1 = UART_ReadChar(ptRxBuf,3);
				
				/* �жϵ�ַ�Ƿ���ȷ��֡�����Ƿ��ڿ�ѡ�ķ�Χ�� */
				if ( ( MODULE_ADD_BYTE == ucModuleAdd1 ) && ( ucCmdLength1 <= 28 ) )
				{
						Start_User_Timer(ptRxBuf->st_timer);
						Update_User_Timer_Cnt(ptRxBuf->st_timer);
					
						//�ȴ�300ms���жϻ������Ƿ�������õ��ֽ�
						if( (*ptRxBuf).ucBufCnt>=(ucCmdLength1+4+2) && ( ptRxBuf->st_timer->ul_timer_cnt<=RECEIVE_TIM_CNT ) ) 
						{								
							
							AircraftModule1 = UART_ReadChar(ptRxBuf,4);
							
							Stop_User_Timer(ptRxBuf->st_timer);
								
							for(ucNum1=0;ucNum1<ucCmdLength1+3;ucNum1++)  /* �������У�� */
							{
								ucData1 = UART_ReadChar(ptRxBuf,2+ucNum1);
								ucCheckSum1 ^= ucData1;
							}
							ucTailByte1 = UART_ReadChar(ptRxBuf,2+ucNum1);
							/* �ж�У��ͺ�֡β�Ƿ���ȷ */
							if( (0x00==ucCheckSum1) && (CMD_ENDING_BYTE==ucTailByte1) )
							{
									UART_DelChar(ptRxBuf,4);  /* ɾ��֡ͷ��֡β��ģ���ַ�ͳ��� */
											
								for(j=0;j<ucCmdLength1;j++)
								{
									uc_recivedata1[j] = UART_GetChar(ptRxBuf);  //��ȡ��ɾ���ֽ�
								}
								UART_DelChar(ptRxBuf,2);  //ɾ��֡ͷ��֡β
								p_fun_p(ptRxBuf,uc_recivedata1);
								
							}
							else  /* У���֡β�����㣬��ɾ��֡ͷ����β�ֽ� */
							{
								UART_ReadChar(ptRxBuf,1);
							}
							
							/* ��������Ӵ��� */
							
							uc_Pack_And_Send_Buf[0] = AircraftModule1;
							
						}
						/* �����300ms��δ��������������ɾ��֡ͷ�͵�ַ�ֽ� */
					else if((*ptRxBuf).ucBufCnt<=(ucCmdLength2+4+2) && ( ptRxBuf->st_timer->ul_timer_cnt>= RECEIVE_TIM_CNT ) )
					{
						Stop_User_Timer(ptRxBuf->st_timer);
						UART_DelChar(ptRxBuf,1);
					}
				}
				else
				{
					UART_ReadChar(ptRxBuf,1);  /* ��ַ���ԣ�ɾ��֡ͷ */
				}
				
			}
			else
			{
				UART_DelChar(ptRxBuf,1);  //֡ͷ���ԣ�ɾ��֡ͷ���ֽ�
			}
			
		}	

	 /* 12����Ϣ�ֽڴ��� */
		if ((*ptRxBuf).ucBufCnt==12)
		{
			ucHeadHighByte2 = UART_ReadChar(ptRxBuf,0);
			ucHeadLowByte2 = UART_ReadChar(ptRxBuf,1);
								
			/* �ж�֡ͷ�Ƿ���ȷ */
			if( ( CMD_HEADING_HIGHBYTE == ucHeadHighByte2 ) && ( CMD_HEADING_LOWBYTE == ucHeadLowByte2 ) )  
			{
					ucModuleAdd2 = UART_ReadChar(ptRxBuf,2);
					ucCmdLength2 = UART_ReadChar(ptRxBuf,3);
								
					/* �жϵ�ַ�Ƿ���ȷ��֡�����Ƿ��ڿ�ѡ�ķ�Χ�� */
					if(  ( MODULE_ADD_BYTE == ucModuleAdd2 ) && ( ucCmdLength2<=RECEIVE_DATA_MAX_LENGTH ) )
					{						
						Start_User_Timer(ptRxBuf->st_timer);
						Update_User_Timer_Cnt(ptRxBuf->st_timer);
						
					//�ȴ�300ms���жϻ������Ƿ�������õ��ֽ�
						if( (*ptRxBuf).ucBufCnt>=(ucCmdLength2+4+2)&& ( ptRxBuf->st_timer->ul_timer_cnt<=RECEIVE_TIM_CNT ) ) 

						{
								OPenUmbrella = UART_ReadChar(ptRxBuf,4);  //���ƿ�ɡ
								AirLight = UART_ReadChar(ptRxBuf,5);  		//���Ƶƹ�						
								AirEngineOrder = UART_ReadChar(ptRxBuf,6);  //�ͻ�����
								AirEnginerSpeed = UART_ReadChar(ptRxBuf,7);  //�ͻ������ٶ�
								ChargeControl = UART_ReadChar(ptRxBuf,8);		//������
							
								Stop_User_Timer(ptRxBuf->st_timer);
								
							for(ucNum2=0;ucNum2<ucCmdLength2+3;ucNum2++)  /* �������У�� */
							{
								ucData2 = UART_ReadChar(ptRxBuf,2+ucNum2);
								ucCheckSum2 ^= ucData2;
							}
							ucTailByte2 = UART_ReadChar(ptRxBuf,2+ucNum2);
							/* �ж�У��ͺ�֡β�Ƿ���ȷ */
							if( (0x00==ucCheckSum2) && (CMD_ENDING_BYTE==ucTailByte2) )
							{
									UART_DelChar(ptRxBuf,4);  /* ɾ��֡ͷ��֡β��ģ���ַ�ͳ��� */
								
								for(i=0;i<ucCmdLength2;i++)
								{
									uc_recivedata2[i] = UART_GetChar(ptRxBuf);  //��ȡ��ɾ���ֽ�
								}
								UART_DelChar(ptRxBuf,2);  //ɾ�� У���֡β
								p_fun_p(ptRxBuf,uc_recivedata2);
								
							}
							else  /* У���֡β�����㣬��ɾ��֡ͷ����β�ֽ� */
							{
								UART_ReadChar(ptRxBuf,1);
							}
						
							
							switch(OPenUmbrella)  //��ɡ
							{
								case 0x01:
											__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1000);  //45��
													
										uc_Pack_And_Send_Buf[7] = 0x01;  //��ɡ
										break;
								case 0x02:
										__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,2000);  //135��
						
										uc_Pack_And_Send_Buf[7] = 0x02;  //��ɡ
										break;
								default:
											__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,500);  //0��
										uc_Pack_And_Send_Buf[7] = 0x0;
										break;
							}
				
							switch(AirLight)  //�ƹ�
							{
								case 0x01:
										uc_Pack_And_Send_Buf[8] = 0x01;  /* �ɷ��� */
										break;
								case 0x02:
										uc_Pack_And_Send_Buf[8] = 0x02;
										break;
								case 0x03:
										uc_Pack_And_Send_Buf[8] = 0x03;
										break;
								default:
										uc_Pack_And_Send_Buf[8] = 0x00;
										break;
							}
							
							switch(AirEngineOrder)  //�ͻ�����
							{
								case 0x01:  //����
										
										uc_Pack_And_Send_Buf[3] = 0x01;
										break;
								case 0x02:  //Ϩ��
										uc_Pack_And_Send_Buf[3] = 0x02;
										break;
								case 0x03:  //������
										uc_Pack_And_Send_Buf[3] = 0x10;
										break;
								default: 
										uc_Pack_And_Send_Buf[3] = 0x00;
										break;
							}
							
							switch(ChargeControl)  //������
							{
								case 0x01:
										uc_Pack_And_Send_Buf[9] = 0x01;
										break;
								case 0x0:
										uc_Pack_And_Send_Buf[9] = 0x0;
										break;
								default:uc_Pack_And_Send_Buf[9] = 0x0;
										break;								
							}						

						}  //end if( (*ptRxBuf).ucBufCnt>=(ucCmdLength+4+2)&& ( ptRxBuf->st_timer->ul_timer_cnt<=RECEIVE_TIM_CNT ) ) 
						/* �����300ms��δ��������������ɾ��֡ͷ�͵�ַ�ֽ� */
						else if((*ptRxBuf).ucBufCnt<=(ucCmdLength2+4+2) && ( ptRxBuf->st_timer->ul_timer_cnt>= RECEIVE_TIM_CNT ) )
						{
							Stop_User_Timer(ptRxBuf->st_timer);
							UART_DelChar(ptRxBuf,1);
						}
						
					}  //end if(  ( MODULE_ADD_BYTE == ucModuleAdd ) && ( ucCmdLength<=RECEIVE_DATA_MAX_LENGTH ) )
					else
					{
						UART_ReadChar(ptRxBuf,1);  /* ��ַ���ԣ�ɾ��֡ͷ */						
					}
			}  //end if( ( CMD_HEADING_HIGHBYTE == ucHeadHighByte ) && ( CMD_HEADING_LOWBYTE == ucHeadLowByte ) )  
			else
			{
				UART_DelChar(ptRxBuf,1);  //֡ͷ���ԣ�ɾ��֡ͷ���ֽ�			
			}
			
		}  //end if ((*ptRxBuf).ucBufCnt>=12)
		
}//end void Task_Anasyse_Protocol()




