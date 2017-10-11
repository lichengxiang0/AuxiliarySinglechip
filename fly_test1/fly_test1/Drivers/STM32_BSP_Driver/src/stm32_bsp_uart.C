/**
  ******************************************************************************
  * File Name          : stm32_bsp_uart.c
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
	
#include "stm32f1xx_hal.h"
#include "stm32_bsp_uart.h"

#include "usart.h"
/**/
#if RxBufSize1 != 0
	uint8_t RxBuf1[RxBufSize1];
	User_Timer_Typedef    st_usart1_rx_com_timer = USER_TIMER_INIT_VALUE;    //超时记数结构体
//	USART_Buf_TypeDef 		RxBuf_Struct1={&st_usart1_rx_com_timer,RxBufSize1,0,0,0,0,RxBuf1,&huart1};
	USART_Buf_TypeDef 		RxBuf_Struct1={&st_usart1_rx_com_timer,RxBufSize1,0,0,0,0,RxBuf1,&huart3};


#endif

#if TxBufSize1 != 0
	uint8_t TxBuf1[TxBufSize1];
	User_Timer_Typedef    st_usart1_tx_com_timer = USER_TIMER_INIT_VALUE;    //超时记数结构体
//	USART_Buf_TypeDef 		TxBuf_Struct1={&st_usart1_tx_com_timer,TxBufSize1,0,0,0,0,TxBuf1,&huart1};
	USART_Buf_TypeDef 		TxBuf_Struct1={&st_usart1_tx_com_timer,TxBufSize1,0,0,0,0,TxBuf1,&huart3};
#endif

//#if RxBufSize2 != 0
//	uint8_t RxBuf2[RxBufSize2];
//	User_Timer_Typedef    st_usart2_rx_com_timer = USER_TIMER_INIT_VALUE;    //超时记数结构体
//	USART_Buf_TypeDef 		RxBuf_Struct2={&st_usart2_rx_com_timer,RxBufSize2,0,0,0,0,RxBuf2,&huart2};
//#endif

//#if TxBufSize2 != 0
//	uint8_t TxBuf2[TxBufSize2];
//	User_Timer_Typedef    st_usart2_tx_com_timer = USER_TIMER_INIT_VALUE;    //超时记数结构体
//	USART_Buf_TypeDef 		TxBuf_Struct2={&st_usart2_tx_com_timer,TxBufSize2,0,0,0,0,TxBuf2,&huart2};
//#endif


/*******************************************************************************
* 函数名  :  	  uart_GetChar
* 函数描述:       从接收缓冲区读一字节，并清除此字节，需确保缓冲区内有数据。
* 输入参数:       - UART_BUF * ptRx: 接收缓冲区结构体的首地址
*
* 输出参数:       无
* 返 回 值:       从接收缓冲区读取那个字节的数据
* 其它说明： 
* 修改日期    	  版本号     修改人	     修改内容
* -----------------------------------------------
* 2010/06/11      V1.0	     00574zsm	       XXXX
*******************************************************************************/

uint8_t UART_GetChar(USART_Buf_TypeDef * ptRx)
{	
	uint8_t ucData;
	ucData = *((*ptRx).pcBufAddr + (*ptRx).ucBufRdInde);
	if ((*ptRx).ucBufCnt != 0)
	{	
		if (++(*ptRx).ucBufRdInde == (*ptRx).ucBufSize)
		{
		   (*ptRx).ucBufRdInde = 0;
		}
		__HAL_UART_DISABLE_IT(ptRx->ptUartHandel,UART_IT_RXNE);//关接收中断

		--(*ptRx).ucBufCnt;
		__HAL_UART_ENABLE_IT(ptRx->ptUartHandel,UART_IT_RXNE);//开接收中断
	}
	
	return ucData;
}
/*******************************************************************************
* 函数名  : 			 uart_ReadChar
* 函数功能:        从接收缓冲区读取指定位置的一字节，但不清除此字节，需确保缓冲区内有数据。
* 输入参数:        - UART_BUF * stRx： 接收缓冲区结构体的地址
*									 - ucNum：指定缓冲区中要读取字节的位置
* 输出参数:        无
* 返 回 值: 			 从接收缓冲区读取那个字节的数据
* 其它说明： 
* 修改日期    	  版本号     修改人	     修改内容
* -----------------------------------------------
* 2010/06/11      V1.0	     00574zsm	       XXXX
*******************************************************************************/
uint8_t UART_ReadChar(USART_Buf_TypeDef * ptRx, uint8_t ucNum)	 
{	
	uint8_t ucData;
	uint16_t  i;
	i = ucNum;
	if ((*ptRx).ucBufCnt >= ucNum)
	 {
		i += (*ptRx).ucBufRdInde;
		if (i >= (*ptRx).ucBufSize)
		{	
			i -=((*ptRx).ucBufSize);
		}
	 }
	else
	 {
	  i=0;
	 }
	ucData = *((*ptRx).pcBufAddr + i);
	return ucData;
}

/*******************************************************************************
* 函数名  :			   uart_DelChar
* 函数功能:        从接收缓冲区清除指定长度的数据，
* 输入参数:        - UART_BUF * ptRx: 接收缓冲区结构体的地址
*									 - ucNum：指定缓冲区中要删除字节的长度									
* 输出参数:        无
* 返 回 值:        无
* 其它说明： 
* 修改日期    	  版本号     修改人	     修改内容
* -----------------------------------------------
* 2010/06/11      V1.0	     00574zsm	       XXXX
*******************************************************************************/
void UART_DelChar(USART_Buf_TypeDef * ptRx, uint8_t ucNum)	 
{	
	uint16_t i;
	if ((*ptRx).ucBufCnt >= ucNum)	
	{	
		__HAL_UART_DISABLE_IT(ptRx->ptUartHandel,UART_IT_RXNE);//关接收中断
		(*ptRx).ucBufCnt -= ucNum;	
		__HAL_UART_ENABLE_IT(ptRx->ptUartHandel,UART_IT_RXNE);//开接收中断
		i = ucNum;
		i += (*ptRx).ucBufRdInde;
		if (i >= (*ptRx).ucBufSize)
		{
		  i -= (*ptRx).ucBufSize;
		}
		(*ptRx).ucBufRdInde = i;
	}
	else
	{  
		__HAL_UART_DISABLE_IT(ptRx->ptUartHandel,UART_IT_RXNE);//关接收中断
   	    i = (*ptRx).ucBufCnt;
        (*ptRx).ucBufCnt = 0;
		__HAL_UART_ENABLE_IT(ptRx->ptUartHandel,UART_IT_RXNE);//开接收中断
        i += (*ptRx).ucBufRdInde;
		if (i >= (*ptRx).ucBufSize)
		{
		  i -= (*ptRx).ucBufSize;
		}
		(*ptRx).ucBufRdInde = i;

	 }
}




/*******************************************************************************
* 函数名  :			   uart_PutChar
* 函数功能:        向发送缓冲区写入一个字节数据
* 输入参数:        - UART_BUF * ptTx: 发送缓冲区结构体的首地址
*									 - cData：写入发送缓冲区中的数据									
* 输出参数:        无
* 返 回 值:        无
* 其它说明： 
* 修改日期    	  版本号     修改人	     修改内容
* -----------------------------------------------
* 2010/06/11      V1.0	     00574zsm	       XXXX
*******************************************************************************/

void UART_PutChar(USART_Buf_TypeDef * ptTx, uint8_t ucData)
{	
	while ((*ptTx).ucBufCnt == (*ptTx).ucBufSize);			 //发送缓冲满，等待。若有此情况建议增大缓冲区
	__HAL_UART_DISABLE_IT(ptTx->ptUartHandel,UART_IT_TXE);//关接收中断
	if((*ptTx).ucBufCnt == 0)
	{
		if(__HAL_UART_GET_FLAG(ptTx->ptUartHandel, UART_FLAG_TXE)== SET) //check if transmit data register full or not
		{
			ptTx->ptUartHandel->Instance->DR = ucData;
			__HAL_UART_ENABLE_IT(ptTx->ptUartHandel,UART_IT_TXE);//开发送中断
			return;
		}
	}	
	*((*ptTx).pcBufAddr + (*ptTx).ucBufWrInde) = ucData;
	if(++(*ptTx).ucBufWrInde == (*ptTx).ucBufSize)
	{
	  (*ptTx).ucBufWrInde = 0;
	}
	++(*ptTx).ucBufCnt;
	__HAL_UART_ENABLE_IT(ptTx->ptUartHandel,UART_IT_TXE);//开发送中断
}




/*******************************************************************************
* 函数名  :        USARTx_Tx_ISR
* 函数功能:        此函数为串口发送完成中断的服务函数
* 输入    :        - UART_BUF * ptTx: 发送缓冲区结构体的首地址
* 输出    :        无
* 返 回 值:        无
* 其它说明： 
* 修改日期    	  版本号     修改人	     修改内容
* -----------------------------------------------
* 2010/06/11      V1.0	     00574zsm	       XXXX
*******************************************************************************/
void UART_Tx_ISR(USART_Buf_TypeDef * ptTx)
{
	 if((__HAL_UART_GET_FLAG(ptTx->ptUartHandel, UART_FLAG_TXE) != RESET) && (__HAL_UART_GET_IT_SOURCE(ptTx->ptUartHandel, UART_IT_TXE) != RESET))
   {    
		 __HAL_UART_CLEAR_FLAG(ptTx->ptUartHandel,UART_FLAG_TXE);
	  	if((* ptTx).ucBufCnt)
	  	{
		 		--(* ptTx).ucBufCnt;
				ptTx->ptUartHandel->Instance->DR = *(ptTx->pcBufAddr + ptTx->ucBufRdInde);		
		 		if(++(* ptTx).ucBufRdInde >= (* ptTx).ucBufSize)
				{
		   		  (* ptTx).ucBufRdInde = 0;
				}
	  	}
			else
			{
				/* Disable the UART Transmit Data Register Empty Interrupt */
				__HAL_UART_DISABLE_IT(ptTx->ptUartHandel, UART_IT_TXE);
			}
   } 
}

/*******************************************************************************
* 函数名  :        USARTx_Rx_ISR
* 函数功能:        此函数为串口接收中断的服务函数
* 输入参数:        - UART_BUF * ptRx: 接收缓冲区结构体的地址
* 输出参数:				 无
* 返 回 值:        无
* 其它说明： 
* 修改日期    	   版本号     修改人	     修改内容
* -----------------------------------------------

*******************************************************************************/

void UART_Rx_ISR(USART_Buf_TypeDef * ptRx)
{
 	uint8_t ucData;
	
//	 __HAL_UART_CLEAR_OREFLAG(ptRx->ptUartHandel);	
	if((__HAL_UART_GET_FLAG(ptRx->ptUartHandel, UART_FLAG_RXNE) != RESET) && (__HAL_UART_GET_IT_SOURCE(ptRx->ptUartHandel, UART_IT_RXNE) != RESET))
  {
			ucData = (uint8_t)(ptRx->ptUartHandel->Instance->DR&0x01FF);
			
	  	if(((ptRx->ptUartHandel->Instance->SR)& 0x0000) == 0)//无错误
	  	{
				/*buf满了则清除缓冲区*/
				if(++(* ptRx).ucBufCnt > (* ptRx).ucBufSize)  
				{
//		   			(* ptRx).ucBufCnt = (* ptRx).ucBufSize;   //test
						(* ptRx).ucBufCnt = 0;  //test
		   			(* ptRx).ucBufOvf = 1;
				}
				else
				{
					*((* ptRx).pcBufAddr + (* ptRx).ucBufWrInde) = ucData;         
					if(++(* ptRx).ucBufWrInde >= (* ptRx).ucBufSize)
					{
							(* ptRx).ucBufWrInde = 0; 
					}	
				}
	   }
  }
} 


void	UART_TxRx_ISR(UART_HandleTypeDef	*huart)
{
	uint32_t tmp_flag = 0, tmp_it_source = 0;

  tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_PE);
  tmp_it_source = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_PE);  
  /* UART parity error interrupt occurred ------------------------------------*/
  if((tmp_flag != RESET) && (tmp_it_source != RESET))
  { 
    __HAL_UART_CLEAR_PEFLAG(huart);
    
    huart->ErrorCode |= HAL_UART_ERROR_PE;
  }
  
	
  tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_FE);
  /* UART frame error interrupt occurred -------------------------------------*/
  if((tmp_flag != RESET))
  { 
    __HAL_UART_CLEAR_FEFLAG(huart);
    
    huart->ErrorCode |= HAL_UART_ERROR_FE;
  }
  
  tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_NE);
  /* UART noise error interrupt occurred -------------------------------------*/
  if((tmp_flag != RESET))
  { 
    __HAL_UART_CLEAR_NEFLAG(huart);
    
    huart->ErrorCode |= HAL_UART_ERROR_NE;
  }
  
  tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_ORE);
  /* UART Over-Run interrupt occurred ----------------------------------------*/
  if((tmp_flag != RESET))
  { 
    __HAL_UART_CLEAR_OREFLAG(huart);
    
    huart->ErrorCode |= HAL_UART_ERROR_ORE;
  }
  


  if(huart->ErrorCode != HAL_UART_ERROR_NONE)
  {
    /* Set the UART state ready to be able to start again the process */
    huart->State = HAL_UART_STATE_READY;
    
    HAL_UART_ErrorCallback(huart);
  }  
}
/*--------------------------------结束------------------------------------------*/
