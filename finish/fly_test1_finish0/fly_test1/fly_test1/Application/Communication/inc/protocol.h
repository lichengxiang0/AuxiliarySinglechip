/**
  ******************************************************************************
  * File Name          : protocol.h
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
/*-----------------ͨ��Э�鴮������-------------------*/
//�����д��ڶ���
#define	FEEDBACK_CMD_TXBUF		((USART_Buf_TypeDef *)&TxBuf_Struct1)
#define	SET_CMD_RXBUF					((USART_Buf_TypeDef *)&RxBuf_Struct1)	 
	 
/*-----------------ͨ��Э��֡��ʽ-------------------*/	 
//ͨѶЭ���е��ֽڶ���
#define CMD_HEADING_HIGHBYTE     (uint8_t)0x55	 	  //֡ͷ���ֽ�
#define CMD_HEADING_LOWBYTE      (uint8_t)0xAA	  	//֡ͷ���ֽ�
#define MODULE_ADD_BYTE          (uint8_t)0x01      //MCUָ����ַ
#define CMD_ENDING_BYTE          (uint8_t)0xF0      //֡β

	 
//���ݽ��ճ�ʱ����ֵ
//��ʱʱ��Ϊ: RECEIVE_TIM_CNT*��ʱ��ʱ������
#define	RECEIVE_TIM_CNT          (uint16_t)10000  
//#define RECEIVE_DATA_MAX_LENGTH  ((uint8_t)25) 	    //(15+4+2+2)
#define RECEIVE_DATA_MAX_LENGTH  ((uint8_t)12) 	    //(2+8+2)


/*-----------------ͨ��Э������������-------------------*/
/*�������õ��������������д��Ϳ�������*/
#define CMD_SET_TORQUE_REF					((uint16_t)0x0108)
#define CMD_SEND_POS_CTRL          	((uint16_t)0x020F)	

/*������ת����*/
#define	CMD_FIRMWARE_UPDATA					((uint16_t)0x5000)
#define CMD_JUMP_TO_IAP							((uint16_t)0x5010)

/*��ѯ/�����̼��汾�����ѯ/����������Ϣ����*/
#define	CMD_QUERY_FIRMWARE_INFO				((uint16_t)0x5021)
#define	CMD_RELAY_QUERY_FIRMWARE_INFO	((uint16_t)0x503F)

#define	CMD_QUERY_CONFIG_INFO					((uint16_t)0x5042)
#define	CMD_RELAY_QUERY_CONFIG_INFO		((uint16_t)0x505F)

#define CMD_SET_ZERO_FRAME_ANGLE					((uint16_t)0x5800)
#define	CMD_REPLY_SET_ZERO_FRAME_ANGLE		((uint16_t)0x5810)

/*-----------------------��̬�������---------------------*/
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

/*-----------------------����������----------------------------*/
#define MOTOR_CMD_ENTER_DEBUG_MODE						((uint16_t)0x5351)
#define MOTOR_CMD_SET_TORQUE									((uint16_t)0x5303)
#define MOTOR_CMD_SET_ZERO_ELECTRICAL_ANGLE		((uint16_t)0x5413)
#define MOTOR_CMD_SAVE_ZERO_ELECTRICAL_ANGLE	((uint16_t)0x5421)
#define	MOTOR_CMD_SET_ELECTRICAL_ANGLE_DIR		((uint16_t)0x5432)
#define MOTOR_CMD_SET_TORQUE_DIR							((uint16_t)0x5442)
#define	MOTOR_CMD_QUERY_ZERO_ELEC_ANGLE				((uint16_t)0x5451)

#define	MOTOR_CMD_REPLY_ZERO_ELEC_ANGLE_QUERY	((uint16_t)0x5463)

/*-----------------------ҡ�˰������----------------------------*/
#define JOYSTICK_CMD_ENTER_DEBUG_MODE						((uint16_t)0x5551)
#define JOYSTICK_CMD_CAL_JOY         						((uint16_t)0x5501) // ҡ��У׼������
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

