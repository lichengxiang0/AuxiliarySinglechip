#ifndef __PROCESS_H__
#define __PROCESS_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h" 
#include "stm32_bsp_uart.h"


/*--------------------���ʹ���Զ��建����---------------------*/
extern uint8_t uc_Pack_And_Send_Buf[10];


/* Exported define ------------------------------------------------------------*/
/*-----------------ͨ��Э�鴮������-------------------*/
//�����д��ڶ���
#define	FEEDBACK_CMD_TXBUF		((USART_Buf_TypeDef *)&TxBuf_Struct1)
#define	SET_CMD_RXBUF					((USART_Buf_TypeDef *)&RxBuf_Struct1)	 


/*-----------------ͨ��Э��֡��ʽ-------------------*/	 
//ͨѶЭ���е��ֽڶ���
#define CMD_HEADING_HIGHBYTE     (uint8_t)0x55	 	  //֡ͷ���ֽ�
#define CMD_HEADING_LOWBYTE      (uint8_t)0xAA	  	//֡ͷ���ֽ�
#define MODULE_ADD_BYTE          (uint8_t)0xFE      //MCUָ����ַ
#define CMD_ENDING_BYTE          (uint8_t)0xF0      //֡β



//���ݽ��ճ�ʱ����ֵ
//��ʱʱ��Ϊ: RECEIVE_TIM_CNT*��ʱ��ʱ������
#define	RECEIVE_TIM_CNT          (uint16_t)10000  
#define RECEIVE_DATA_MAX_LENGTH  ((uint8_t)10) 	    //(2+8+2)
void	Task_Anasyse_Protocol(USART_Buf_TypeDef * ptRxBuf,	void (*p_fun_p)(USART_Buf_TypeDef * ptRxBuf, uint8_t * data_array_p));

#endif


