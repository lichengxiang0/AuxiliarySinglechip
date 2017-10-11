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
*通信协议打包发送
*第一个参数：发送使用的缓冲区
*第二个参数：发送数据指针
*第三个参数：发送模块的地址
********************************************************/ 
void Pack_And_Send_Cmd(USART_Buf_TypeDef *stTxBuf,uint8_t *uc_cmd_data_p,uint8_t uc_module_addr_p)
{
	uint8_t uc_check_sum = 0,i=0,cnt = 0;
//	uint8_t uc_cmd_length;
	uint8_t uc_send_cmd_data[16];
	
//	uc_cmd_length = (uc_cmd_data_p[1]&0x0F);
	
	uc_send_cmd_data[cnt++] = 0x55;
	uc_send_cmd_data[cnt++] = 0xAA;
	
	uc_send_cmd_data[cnt++] = uc_module_addr_p;  /* 模块地址 0x01 */
	uc_check_sum = uc_check_sum^uc_module_addr_p;
	
	uc_send_cmd_data[cnt++] = 0x0A;	/* 数长 */
	uc_check_sum = uc_check_sum^0x0A;
	
	/* 飞机授权码 */
	uc_send_cmd_data[cnt++] = ( (uint8_t)(uc_cmd_data_p[0]) );  //授权码有三位，第一位
	uc_check_sum = uc_check_sum^( (uint8_t)uc_cmd_data_p[0] );
	
	uc_send_cmd_data[cnt++] = ( (uint8_t)(uc_cmd_data_p[1]) );  //第二位
 	uc_check_sum = uc_check_sum^( (uint8_t)uc_cmd_data_p[1] );
	
	uc_send_cmd_data[cnt++] = ( (uint8_t)(uc_cmd_data_p[2]) );  //第三位
 	uc_check_sum = uc_check_sum^( (uint8_t)uc_cmd_data_p[2] );
	
	/* 油机状态 */
	uc_send_cmd_data[cnt++] = ( (uint8_t)(uc_cmd_data_p[3]) );
	uc_check_sum = uc_check_sum^( (uint8_t)uc_cmd_data_p[3] );
	
	/* 油机转速回旋 */
	uc_send_cmd_data[cnt++] = ( (uint8_t)(uc_cmd_data_p[4]) );
	uc_check_sum = uc_check_sum^( (uint8_t)uc_cmd_data_p[4] );
	
	/* 油量 */
	uc_send_cmd_data[cnt++] = ( (uint8_t)(uc_cmd_data_p[5]) );
	uc_check_sum = uc_check_sum^( (uint8_t)uc_cmd_data_p[5] );
	
	/* 飞机紧急控制 */
	uc_send_cmd_data[cnt++] = ( (uint8_t)(uc_cmd_data_p[6]) );
	uc_check_sum = uc_check_sum^( (uint8_t)uc_cmd_data_p[6] );
	
	/* 开伞状态 */
	uc_send_cmd_data[cnt++] = ( (uint8_t)(uc_cmd_data_p[7]) );
	uc_check_sum = uc_check_sum^( (uint8_t)uc_cmd_data_p[7] );
	
	/* 飞行状态 */
	uc_send_cmd_data[cnt++] = ( (uint8_t)(uc_cmd_data_p[8]) );
	uc_check_sum = uc_check_sum^( (uint8_t)uc_cmd_data_p[8] );
	
	/* 充电状态 */
	uc_send_cmd_data[cnt++] = ( (uint8_t)(uc_cmd_data_p[9]) );
	uc_check_sum = uc_check_sum^( (uint8_t)uc_cmd_data_p[9] );
	
	/* 校验，帧尾 */
	uc_send_cmd_data[cnt++] = uc_check_sum;
	uc_send_cmd_data[cnt++] = 0xF0;
	
	for(i=0;i<cnt;i++)
	{
		UART_PutChar(stTxBuf,uc_send_cmd_data[i]);
	}
	
}


/* ***********************************************************
*通信协议解析
*第一个参数：接收缓冲区
*第二个参数：解析后对命令的处理函数指针
*/
void Task_Anasyse_Protocol(USART_Buf_TypeDef * ptRxBuf,void (*p_fun_p)(USART_Buf_TypeDef * ptRxBuf,uint8_t * data_array_p))
 {	 
	 /* 变量定义区 */
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
	 


	 
	 /* 28个消息字节处理 */
	 if ( (*ptRxBuf).ucBufCnt == 28 )
		{
			ucHeadHighByte1 = UART_ReadChar(ptRxBuf,0);
			ucHeadLowByte1 = UART_ReadChar(ptRxBuf,1);
			
			/* 判断帧头是否正确 */
			if ( ( CMD_HEADING_HIGHBYTE == ucHeadHighByte1 )  && ( CMD_HEADING_LOWBYTE == ucHeadLowByte1 ) )
			{
				ucModuleAdd1 = UART_ReadChar(ptRxBuf,2);
				ucCmdLength1 = UART_ReadChar(ptRxBuf,3);
				
				/* 判断地址是否正确，帧长度是否在可选的范围内 */
				if ( ( MODULE_ADD_BYTE == ucModuleAdd1 ) && ( ucCmdLength1 <= 28 ) )
				{
						Start_User_Timer(ptRxBuf->st_timer);
						Update_User_Timer_Cnt(ptRxBuf->st_timer);
					
						//等待300ms，判断缓冲区是否充满设置的字节
						if( (*ptRxBuf).ucBufCnt>=(ucCmdLength1+4+2) && ( ptRxBuf->st_timer->ul_timer_cnt<=RECEIVE_TIM_CNT ) ) 
						{								
							
							AircraftModule1 = UART_ReadChar(ptRxBuf,4);
							
							Stop_User_Timer(ptRxBuf->st_timer);
								
							for(ucNum1=0;ucNum1<ucCmdLength1+3;ucNum1++)  /* 计算异或校验 */
							{
								ucData1 = UART_ReadChar(ptRxBuf,2+ucNum1);
								ucCheckSum1 ^= ucData1;
							}
							ucTailByte1 = UART_ReadChar(ptRxBuf,2+ucNum1);
							/* 判断校验和和帧尾是否正确 */
							if( (0x00==ucCheckSum1) && (CMD_ENDING_BYTE==ucTailByte1) )
							{
									UART_DelChar(ptRxBuf,4);  /* 删除帧头，帧尾，模块地址和长度 */
											
								for(j=0;j<ucCmdLength1;j++)
								{
									uc_recivedata1[j] = UART_GetChar(ptRxBuf);  //读取并删除字节
								}
								UART_DelChar(ptRxBuf,2);  //删除帧头和帧尾
								p_fun_p(ptRxBuf,uc_recivedata1);
								
							}
							else  /* 校验和帧尾不满足，则删掉帧头和字尾字节 */
							{
								UART_ReadChar(ptRxBuf,1);
							}
							
							/* 在下面添加代码 */
							
							uc_Pack_And_Send_Buf[0] = AircraftModule1;
							
						}
						/* 如果到300ms还未充满缓冲区，则删掉帧头和地址字节 */
					else if((*ptRxBuf).ucBufCnt<=(ucCmdLength2+4+2) && ( ptRxBuf->st_timer->ul_timer_cnt>= RECEIVE_TIM_CNT ) )
					{
						Stop_User_Timer(ptRxBuf->st_timer);
						UART_DelChar(ptRxBuf,1);
					}
				}
				else
				{
					UART_ReadChar(ptRxBuf,1);  /* 地址不对，删掉帧头 */
				}
				
			}
			else
			{
				UART_DelChar(ptRxBuf,1);  //帧头不对，删掉帧头高字节
			}
			
		}	

	 /* 12个消息字节处理 */
		if ((*ptRxBuf).ucBufCnt==12)
		{
			ucHeadHighByte2 = UART_ReadChar(ptRxBuf,0);
			ucHeadLowByte2 = UART_ReadChar(ptRxBuf,1);
								
			/* 判断帧头是否正确 */
			if( ( CMD_HEADING_HIGHBYTE == ucHeadHighByte2 ) && ( CMD_HEADING_LOWBYTE == ucHeadLowByte2 ) )  
			{
					ucModuleAdd2 = UART_ReadChar(ptRxBuf,2);
					ucCmdLength2 = UART_ReadChar(ptRxBuf,3);
								
					/* 判断地址是否正确，帧长度是否在可选的范围内 */
					if(  ( MODULE_ADD_BYTE == ucModuleAdd2 ) && ( ucCmdLength2<=RECEIVE_DATA_MAX_LENGTH ) )
					{						
						Start_User_Timer(ptRxBuf->st_timer);
						Update_User_Timer_Cnt(ptRxBuf->st_timer);
						
					//等待300ms，判断缓冲区是否充满设置的字节
						if( (*ptRxBuf).ucBufCnt>=(ucCmdLength2+4+2)&& ( ptRxBuf->st_timer->ul_timer_cnt<=RECEIVE_TIM_CNT ) ) 

						{
								OPenUmbrella = UART_ReadChar(ptRxBuf,4);  //控制开伞
								AirLight = UART_ReadChar(ptRxBuf,5);  		//控制灯光						
								AirEngineOrder = UART_ReadChar(ptRxBuf,6);  //油机命令
								AirEnginerSpeed = UART_ReadChar(ptRxBuf,7);  //油机控制速度
								ChargeControl = UART_ReadChar(ptRxBuf,8);		//充电控制
							
								Stop_User_Timer(ptRxBuf->st_timer);
								
							for(ucNum2=0;ucNum2<ucCmdLength2+3;ucNum2++)  /* 计算异或校验 */
							{
								ucData2 = UART_ReadChar(ptRxBuf,2+ucNum2);
								ucCheckSum2 ^= ucData2;
							}
							ucTailByte2 = UART_ReadChar(ptRxBuf,2+ucNum2);
							/* 判断校验和和帧尾是否正确 */
							if( (0x00==ucCheckSum2) && (CMD_ENDING_BYTE==ucTailByte2) )
							{
									UART_DelChar(ptRxBuf,4);  /* 删除帧头，帧尾，模块地址和长度 */
								
								for(i=0;i<ucCmdLength2;i++)
								{
									uc_recivedata2[i] = UART_GetChar(ptRxBuf);  //读取并删除字节
								}
								UART_DelChar(ptRxBuf,2);  //删除 校验和帧尾
								p_fun_p(ptRxBuf,uc_recivedata2);
								
							}
							else  /* 校验和帧尾不满足，则删掉帧头和字尾字节 */
							{
								UART_ReadChar(ptRxBuf,1);
							}
						
							
							switch(OPenUmbrella)  //开伞
							{
								case 0x01:
											__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1000);  //45°
													
										uc_Pack_And_Send_Buf[7] = 0x01;  //开伞
										break;
								case 0x02:
										__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,2000);  //135°
						
										uc_Pack_And_Send_Buf[7] = 0x02;  //关伞
										break;
								default:
											__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,500);  //0°
										uc_Pack_And_Send_Buf[7] = 0x0;
										break;
							}
				
							switch(AirLight)  //灯光
							{
								case 0x01:
										uc_Pack_And_Send_Buf[8] = 0x01;  /* 可飞行 */
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
							
							switch(AirEngineOrder)  //油机命令
							{
								case 0x01:  //启动
										
										uc_Pack_And_Send_Buf[3] = 0x01;
										break;
								case 0x02:  //熄火
										uc_Pack_And_Send_Buf[3] = 0x02;
										break;
								case 0x03:  //运行中
										uc_Pack_And_Send_Buf[3] = 0x10;
										break;
								default: 
										uc_Pack_And_Send_Buf[3] = 0x00;
										break;
							}
							
							switch(ChargeControl)  //充电控制
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
						/* 如果到300ms还未充满缓冲区，则删掉帧头和地址字节 */
						else if((*ptRxBuf).ucBufCnt<=(ucCmdLength2+4+2) && ( ptRxBuf->st_timer->ul_timer_cnt>= RECEIVE_TIM_CNT ) )
						{
							Stop_User_Timer(ptRxBuf->st_timer);
							UART_DelChar(ptRxBuf,1);
						}
						
					}  //end if(  ( MODULE_ADD_BYTE == ucModuleAdd ) && ( ucCmdLength<=RECEIVE_DATA_MAX_LENGTH ) )
					else
					{
						UART_ReadChar(ptRxBuf,1);  /* 地址不对，删掉帧头 */						
					}
			}  //end if( ( CMD_HEADING_HIGHBYTE == ucHeadHighByte ) && ( CMD_HEADING_LOWBYTE == ucHeadLowByte ) )  
			else
			{
				UART_DelChar(ptRxBuf,1);  //帧头不对，删掉帧头高字节			
			}
			
		}  //end if ((*ptRxBuf).ucBufCnt>=12)
		
}//end void Task_Anasyse_Protocol()




