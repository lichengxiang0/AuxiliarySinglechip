#ifndef __PROCESS_H__
#define __PROCESS_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h" 
#include "stm32_bsp_uart.h"


/*--------------------发送打包自定义缓冲区---------------------*/
extern uint8_t uc_Pack_And_Send_Buf[10];


/* Exported define ------------------------------------------------------------*/
/*-----------------通信协议串口配置-------------------*/
//上下行串口定义
#define	FEEDBACK_CMD_TXBUF		((USART_Buf_TypeDef *)&TxBuf_Struct1)
#define	SET_CMD_RXBUF					((USART_Buf_TypeDef *)&RxBuf_Struct1)	 


/*-----------------通信协议帧格式-------------------*/	 
//通讯协议中的字节定义
#define CMD_HEADING_HIGHBYTE     (uint8_t)0x55	 	  //帧头高字节
#define CMD_HEADING_LOWBYTE      (uint8_t)0xAA	  	//帧头低字节
#define MODULE_ADD_BYTE          (uint8_t)0xFE      //MCU指定地址
#define CMD_ENDING_BYTE          (uint8_t)0xF0      //帧尾



//数据接收超时计数值
//超时时间为: RECEIVE_TIM_CNT*超时定时器周期
#define	RECEIVE_TIM_CNT          (uint16_t)10000  
#define RECEIVE_DATA_MAX_LENGTH  ((uint8_t)10) 	    //(2+8+2)
void	Task_Anasyse_Protocol(USART_Buf_TypeDef * ptRxBuf,	void (*p_fun_p)(USART_Buf_TypeDef * ptRxBuf, uint8_t * data_array_p));

#endif


