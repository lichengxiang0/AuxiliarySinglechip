/**
  ******************************************************************************
  * File Name          : protocol.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef	_PROTOCOL_H
#define	_PROTOCOL_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h" 
#include "stm32_bsp_uart.h"
	 
/* Exported define ------------------------------------------------------------*/
/*-----------------通信协议串口配置-------------------*/
//上下行串口定义
#define	FEEDBACK_CMD_TXBUF		((USART_Buf_TypeDef *)&TxBuf_Struct1)
#define	SET_CMD_RXBUF					((USART_Buf_TypeDef *)&RxBuf_Struct1)	 
	 
/*-----------------通信协议帧格式-------------------*/	 
//通讯协议中的字节定义
#define CMD_HEADING_HIGHBYTE     (uint8_t)0x55	 	  //帧头高字节
#define CMD_HEADING_LOWBYTE      (uint8_t)0xAA	  	//帧头低字节
#define MODULE_ADD_BYTE          (uint8_t)0x01      //MCU指定地址
#define CMD_ENDING_BYTE          (uint8_t)0xF0      //帧尾

	 
//数据接收超时计数值
//超时时间为: RECEIVE_TIM_CNT*超时定时器周期
#define	RECEIVE_TIM_CNT          (uint16_t)10000  
//#define RECEIVE_DATA_MAX_LENGTH  ((uint8_t)25) 	    //(15+4+2+2)
#define RECEIVE_DATA_MAX_LENGTH  ((uint8_t)12) 	    //(2+8+2)


/*-----------------通信协议总体命令字-------------------*/
/*上行设置电机力矩命令和下行传送控制命令*/
#define CMD_SET_TORQUE_REF					((uint16_t)0x0108)
#define CMD_SEND_POS_CTRL          	((uint16_t)0x020F)	

/*升级跳转命令*/
#define	CMD_FIRMWARE_UPDATA					((uint16_t)0x5000)
#define CMD_JUMP_TO_IAP							((uint16_t)0x5010)

/*查询/反馈固件版本命令，查询/反馈配置信息命令*/
#define	CMD_QUERY_FIRMWARE_INFO				((uint16_t)0x5021)
#define	CMD_RELAY_QUERY_FIRMWARE_INFO	((uint16_t)0x503F)

#define	CMD_QUERY_CONFIG_INFO					((uint16_t)0x5042)
#define	CMD_RELAY_QUERY_CONFIG_INFO		((uint16_t)0x505F)

#define CMD_SET_ZERO_FRAME_ANGLE					((uint16_t)0x5800)
#define	CMD_REPLY_SET_ZERO_FRAME_ANGLE		((uint16_t)0x5810)

/*-----------------------姿态板命令段---------------------*/
#define AHRS_CMD_ENTER_DEBUG_MODE					((uint16_t)0x5641)
#define AHRS_CMD_CAL_GYRO_OFFSET					((uint16_t)0x5710)
#define AHRS_CMD_CAL_ACC_OFFSET						((uint16_t)0x5601)
#define	AHRS_CMD_QUERY_AHRS_CAL_STATUS		((uint16_t)0x5640)
#define AHRS_CMD_SAVE_ROLL_REF						((uint16_t)0x5670)
#define	AHRS_CMD_SET_SPD_RATIO						((uint16_t)0x56A3)

#define	AHRS_CMD_REPLY_CAL_GYRO_OFFSET				((uint16_t)0x5720)
#define	AHRS_CMD_REPLY_CAL_ACC_OFFSET					((uint16_t)0x5611)
#define	AHRS_CMD_REPLY_AHRS_CAL_STATUS_QUERY	((uint16_t)0x565C)
#define	AHRS_CMD_REPLY_SAVE_ROLL_REF					((uint16_t)0x5680)
#define	AHRS_CMD_REPLY_SET_SPD_RATIO					((uint16_t)0x56B1)


#define	AHRS_CMD_FEEDBACK_INFO0								((uint16_t)0x562C)
#define	AHRS_CMD_FEEDBACK_INFO1								((uint16_t)0x563A)

/*-----------------------电机板命令段----------------------------*/
#define MOTOR_CMD_ENTER_DEBUG_MODE						((uint16_t)0x5351)
#define MOTOR_CMD_SET_TORQUE									((uint16_t)0x5303)
#define MOTOR_CMD_SET_ZERO_ELECTRICAL_ANGLE		((uint16_t)0x5413)
#define MOTOR_CMD_SAVE_ZERO_ELECTRICAL_ANGLE	((uint16_t)0x5421)
#define	MOTOR_CMD_SET_ELECTRICAL_ANGLE_DIR		((uint16_t)0x5432)
#define MOTOR_CMD_SET_TORQUE_DIR							((uint16_t)0x5442)
#define	MOTOR_CMD_QUERY_ZERO_ELEC_ANGLE				((uint16_t)0x5451)

#define	MOTOR_CMD_REPLY_ZERO_ELEC_ANGLE_QUERY	((uint16_t)0x5463)

/*-----------------------摇杆板命令段----------------------------*/
#define JOYSTICK_CMD_ENTER_DEBUG_MODE						((uint16_t)0x5551)
#define JOYSTICK_CMD_CAL_JOY         						((uint16_t)0x5501) // 摇杆校准命令字
#define	JOYSTICK_CMD_RESET_KEY_VALUE						((uint16_t)0x5530)
#define	JOYSTICK_CMD_QUERY_JOY_CAL_STATUS				((uint16_t)0x5540)

#define	JOYSTICK_CMD_REPLY_CAL_JOY							((uint16_t)0x5511)
#define	JOYSTICK_CMD_REPLY_JOY_CAL_STATUS_QUERY	((uint16_t)0x554C)

#define	JOYSTICK_CMD_FEEDBACK_INFO0							((uint16_t)0x5538)
	 
/* Exported functions ------------------------------------------------------------*/
void	Task_Anasyse_Protocol(USART_Buf_TypeDef * ptRxBuf,	void (*p_fun_p)(USART_Buf_TypeDef * ptRxBuf, uint8_t * data_array_p));
void 	Pack_And_Send_Cmd(USART_Buf_TypeDef * stTxBuf,	uint8_t *uc_cmd_data_p,uint8_t uc_module_addr_p);

#ifdef __cplusplus
}
#endif


#endif

