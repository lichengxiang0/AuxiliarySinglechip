/**
  ******************************************************************************
  * File Name          : stm32_bsp_uart.h
  * Description        : 串口缓冲区相关函数，包含缓冲区的定义和读取
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef	_STM32_BSP_UART_H
#define	_STM32_BSP_UART_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
	 
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32_bsp_timer.h"
//#include "stm32_bsp_peripheral_init_export.h"



typedef struct								    //缓冲区各变量，用于相关函数的参数
{	
		User_Timer_Typedef *st_timer;
		uint8_t  	ucBufSize;							//缓冲区数组：缓冲区大小
		volatile	uint8_t  ucBufWrInde;						//缓冲区数组：写指针
		volatile	uint8_t  ucBufRdInde;						//缓冲区数组：读指针
		volatile	uint8_t  ucBufCnt;							//缓冲区数组：数据计数
		volatile	uint8_t  ucBufOvf;							//缓冲区数组：溢出标志，注意清零
		uint8_t  * pcBufAddr;						//缓冲区数组：首地址指针变量
		UART_HandleTypeDef	*ptUartHandel;
} USART_Buf_TypeDef;


typedef	struct {
	   uint16_t *pSendData;
		uint16_t DataLength;			 // 数据长度
}TransData;


#define RxBufSize1			30 	// USART1接收缓冲区，大小200，不使用设为0
#define TxBufSize1			30 	// USART1发送缓冲区，大小250，不使用设为0

#define RxBufSize2			30 	// USART2接收缓冲区，大小200，不使用设为0
#define TxBufSize2			30 	// USART2发送缓冲区，大小250，不使用设为0



#if RxBufSize1 != 0
	extern uint8_t RxBuf1[RxBufSize1];
	extern USART_Buf_TypeDef RxBuf_Struct1;
#endif

#if TxBufSize1 != 0
	extern uint8_t TxBuf1[TxBufSize1];
	extern USART_Buf_TypeDef TxBuf_Struct1;
#endif

#if RxBufSize2 != 0
	extern uint8_t RxBuf2[RxBufSize2];
	extern USART_Buf_TypeDef RxBuf_Struct2;
#endif

#if TxBufSize2 != 0
	extern uint8_t TxBuf2[TxBufSize2];
	extern USART_Buf_TypeDef TxBuf_Struct2;
#endif



//函数声明
uint8_t UART_GetChar(USART_Buf_TypeDef * ptRx);
uint8_t UART_ReadChar(USART_Buf_TypeDef * ptRx, uint8_t ucNum);
void UART_DelChar(USART_Buf_TypeDef * ptRx, uint8_t ucNum);
void UART_PutChar(USART_Buf_TypeDef * ptTx, uint8_t ucData);
void UART_PutChar_DMA(USART_Buf_TypeDef * ptTx, uint8_t *ucData ,uint16_t	uw_data_len);


void UART_Tx_ISR(USART_Buf_TypeDef * ptTx);
void UART_Rx_ISR(USART_Buf_TypeDef * ptRx);
void	UART_TxRx_ISR(UART_HandleTypeDef	*huart);


#ifdef __cplusplus
}
#endif


#endif
/*--------------------------------结束------------------------------------------*/
