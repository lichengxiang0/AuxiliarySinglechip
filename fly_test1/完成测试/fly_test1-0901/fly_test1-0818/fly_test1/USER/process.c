#include "process.h"
#include "stm32f1xx_hal.h"
#include "usart.h"
#include "stdint.h"
#include "stm32_bsp_uart.h"

#include "stm32_bsp_timer.h"

#include "led.h"
#include "sys.h"
#include "tim.h"
#include "24cxx.h"



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
	uint8_t uc_send_cmd_data[17];
	
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
	uc_send_cmd_data[cnt++] = ( (uint8_t)(uc_cmd_data_p[4]) );   //���ֽ�
	uc_check_sum = uc_check_sum^( (uint8_t)uc_cmd_data_p[4] );
	uc_send_cmd_data[cnt++] = ( (uint8_t)(uc_cmd_data_p[5]) );   //���ֽ�
	uc_check_sum = uc_check_sum^( (uint8_t)uc_cmd_data_p[5] );
	
	/* ���� */
	uc_send_cmd_data[cnt++] = ( (uint8_t)(uc_cmd_data_p[6]) );
	uc_check_sum = uc_check_sum^( (uint8_t)uc_cmd_data_p[6] );
	
	/* �ɻ��������� */
	uc_send_cmd_data[cnt++] = ( (uint8_t)(uc_cmd_data_p[7]) );
	uc_check_sum = uc_check_sum^( (uint8_t)uc_cmd_data_p[7] );
	
	/* ��ɡ״̬ */
	uc_send_cmd_data[cnt++] = ( (uint8_t)(uc_cmd_data_p[8]) );
	uc_check_sum = uc_check_sum^( (uint8_t)uc_cmd_data_p[8] );
	
	/* ����״̬ */
	uc_send_cmd_data[cnt++] = ( (uint8_t)(uc_cmd_data_p[9]) );
	uc_check_sum = uc_check_sum^( (uint8_t)uc_cmd_data_p[9] );
	
	/* ���״̬ */
	uc_send_cmd_data[cnt++] = ( (uint8_t)(uc_cmd_data_p[10]) );
	uc_check_sum = uc_check_sum^( (uint8_t)uc_cmd_data_p[10] );
	
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
		uint8_t	uc_recivedata1[28];
	 
		uint8_t  ucHeadHighByte2, ucHeadLowByte2, ucTailByte2, ucModuleAdd2, ucCmdLength2,ucData2;
		uint8_t  ucNum2;
    uint8_t  ucCheckSum2=0,i;
		uint8_t	uc_recivedata2[17];
	
//	  uint8_t AircraftModule1,AirExceptionInf,AirCustomerInf;
//	  uint8_t AirLongitude[4],AirLatitude[4],AirHight[4],AirSatelliteTime[4];
	 
		uint8_t OPenUmbrella,AirLight,AirEngineOrder,AirEnginerSpeedHigh,AirEnginerSpeedLow,ChargeControl;

	 /* ����28���ֽڵĴ������������� */

	 
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

							AircraftModule1 = UART_ReadChar(ptRxBuf,4);  //�ɻ��ͺ�
							AirExceptionInf = UART_ReadChar(ptRxBuf,5);  //�쳣��Ϣ
							AirCustomerInf = UART_ReadChar(ptRxBuf,6);   //�ͻ���Ϣ
														
							AirLongitude[0] = UART_ReadChar(ptRxBuf,10);  //����
							AirLongitude[1] = UART_ReadChar(ptRxBuf,11);
							AirLongitude[2] = UART_ReadChar(ptRxBuf,12);
							AirLongitude[3] = UART_ReadChar(ptRxBuf,13);							
							
							AirLatitude[0] = UART_ReadChar(ptRxBuf,14);  //γ��
							AirLatitude[1] = UART_ReadChar(ptRxBuf,15);
							AirLatitude[2] = UART_ReadChar(ptRxBuf,16);
							AirLatitude[3] = UART_ReadChar(ptRxBuf,17);
													
							AirHight[0] = UART_ReadChar(ptRxBuf,18);  //�߶�
							AirHight[1] = UART_ReadChar(ptRxBuf,19);
							AirHight[2] = UART_ReadChar(ptRxBuf,20);
							AirHight[3] = UART_ReadChar(ptRxBuf,21);
														
							AirSatelliteTime[0] = UART_ReadChar(ptRxBuf,22);  //������ʱ
							AirSatelliteTime[1] = UART_ReadChar(ptRxBuf,23);
							AirSatelliteTime[2] = UART_ReadChar(ptRxBuf,24);
							AirSatelliteTime[3] = UART_ReadChar(ptRxBuf,25);
																					
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
							
							/* ����������µĴ��빦�� */
							myUart1_4g_module_data[0] = AircraftModule1;  //���Ƹ���Ҫ���͵�����
							myUart1_4g_module_data[1] = AirExceptionInf;
							myUart1_4g_module_data[2] = AirCustomerInf;
							
							myUart1_4g_module_data[3] = AirLongitude[0];  //���Ƶ�4Gģ�鷢�͵�����
							myUart1_4g_module_data[4] = AirLongitude[1];
							myUart1_4g_module_data[5] = AirLongitude[2];
							myUart1_4g_module_data[6] = AirLongitude[3];
							
							myUart1_4g_module_data[7] = AirLatitude[0];
							myUart1_4g_module_data[8] = AirLatitude[1];
							myUart1_4g_module_data[9] = AirLatitude[2];
							myUart1_4g_module_data[10] = AirLatitude[3];
							
							myUart1_4g_module_data[11] = AirHight[0];
							myUart1_4g_module_data[12] = AirHight[1];
							myUart1_4g_module_data[13] = AirHight[2];
							myUart1_4g_module_data[14] = AirHight[3];
							
							myUart1_4g_module_data[15] = AirSatelliteTime[0];
							myUart1_4g_module_data[16] = AirSatelliteTime[1];
							myUart1_4g_module_data[17] = AirSatelliteTime[2];
							myUart1_4g_module_data[18] = AirSatelliteTime[3];
							
							uc_Pack_And_Send_Buf[0] = AircraftModule1;  //�ɻ��ͺ�
													
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

	 /* 13����Ϣ�ֽڴ�����չ���ͻ����ٶ�     */
		if ((*ptRxBuf).ucBufCnt==13)
		{
			ucHeadHighByte2 = UART_ReadChar(ptRxBuf,0);
			ucHeadLowByte2 = UART_ReadChar(ptRxBuf,1);
								
			/* �ж�֡ͷ�Ƿ���ȷ */
			if( ( CMD_HEADING_HIGHBYTE == ucHeadHighByte2 ) && ( CMD_HEADING_LOWBYTE == ucHeadLowByte2 ) )  
			{
					ucModuleAdd2 = UART_ReadChar(ptRxBuf,2);
					ucCmdLength2 = UART_ReadChar(ptRxBuf,3);
								
					/* �жϵ�ַ�Ƿ���ȷ��֡�����Ƿ��ڿ�ѡ�ķ�Χ�� */
					if(  ( MODULE_ADD_BYTE == ucModuleAdd2 ) && ( ucCmdLength2<=13 ) )   //RECEIVE_DATA_MAX_LENGTH
					{						
						Start_User_Timer(ptRxBuf->st_timer);
						Update_User_Timer_Cnt(ptRxBuf->st_timer);
						
					//�ȴ�300ms���жϻ������Ƿ�������õ��ֽ�
						if( (*ptRxBuf).ucBufCnt>=(ucCmdLength2+4+2)&& ( ptRxBuf->st_timer->ul_timer_cnt<=RECEIVE_TIM_CNT ) ) 
						{
								OPenUmbrella = UART_ReadChar(ptRxBuf,4);  //���ƿ�ɡ
								AirLight = UART_ReadChar(ptRxBuf,5);  		//���Ƶƹ�						
								AirEngineOrder = UART_ReadChar(ptRxBuf,6);  //�ͻ�����
								AirEnginerSpeedHigh = UART_ReadChar(ptRxBuf,7);  //�ͻ������ٶ� ���ֽ�
							  AirEnginerSpeedLow = UART_ReadChar(ptRxBuf,8);  //�ͻ������ٶ�  ���ֽ�
								ChargeControl = UART_ReadChar(ptRxBuf,9);		//������
							
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
													
										uc_Pack_And_Send_Buf[8] = 0x01;  //��ɡ
										break;
								case 0x02:
										__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,2000);  //135��
						
										uc_Pack_And_Send_Buf[8] = 0x02;  //��ɡ
										break;
								default:
											__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,500);  //0��
										uc_Pack_And_Send_Buf[8] = 0x0;
										break;
							}
				
							switch(AirLight)  //�ƹ�
							{
								case 0x01:
									TCA62724FMG_Write_OFF(0xF,0x0,0x0);
										uc_Pack_And_Send_Buf[9] = 0x01;  /* �ɷ��� */
										
										break;
								case 0x02:
									TCA62724FMG_Write_OFF(0x0,0xF,0x0);
										uc_Pack_And_Send_Buf[9] = 0x02;  /* ���ɷ��� */
										
										break;
								case 0x03:
									TCA62724FMG_Write_OFF(0x0,0x0,0xF);
										uc_Pack_And_Send_Buf[9] = 0x03;  /* ���ڷ��� */
										
										break;
								default:
										uc_Pack_And_Send_Buf[9] = 0x00;  /* Ĭ�Ϸ���0 */
										break;
							}
							
							switch(AirEngineOrder)  //�ͻ�����
							{
								case 0x01:  //����
									
											static	User_Timer_Typedef	AirEngineOrder_time_delay = USER_TIMER_INIT_VALUE;
								
											OilEngineStop = 0;
								
											Start_User_Timer(&AirEngineOrder_time_delay);
											Update_User_Timer_Cnt(&AirEngineOrder_time_delay);
											if( AirEngineOrder_time_delay.ul_timer_cnt==500 )  //500ms����
											{
												Reset_User_Timer(&AirEngineOrder_time_delay);
												OilEngineStart = 1;
											}
											
//										OilEngineStop = 0;
//										HAL_Delay(500);
//										OilEngineStart = 1;
										uc_Pack_And_Send_Buf[3] = 0x01;
										break;
								case 0x02:  //Ϩ��
										OilEngineStop = 0;
										uc_Pack_And_Send_Buf[3] = 0x02;
										break;
								case 0x03:  //������
										uc_Pack_And_Send_Buf[3] = 0x10;
										break;
								default: 
										OilEngineStart = 0;
										OilEngineStop = 1;
										uc_Pack_And_Send_Buf[3] = 0x00;
										break;
							}
							
							switch(ChargeControl)  //������
							{
								case 0x01:  //���
										CHARGE = 1;
										uc_Pack_And_Send_Buf[10] = 0x01;
										break;
								case 0x0:  //ֹͣ���
										CHARGE = 0;
										uc_Pack_And_Send_Buf[10] = 0x0;
										break;
								default:
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





