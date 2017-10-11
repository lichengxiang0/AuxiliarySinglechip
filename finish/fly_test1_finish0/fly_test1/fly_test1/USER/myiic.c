#include "myiic.h"
#include "stm32f1xx_hal.h"
#include "mynop.h"

//#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//Mini STM32������
//IIC ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2010/6/10 
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ����ԭ�� 2009-2019
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	  

/*************************************************************************************
����ԭ�ͣ�void I2C_Delay(unsigned int dly)
�������ܣ���ʱ����
���������unsigned int dly����ʾ��ʱ���ٸ�ʱ������
�����������
�汾��A1.0
**************************************************************************************/
//void I2C_Delay(unsigned int dly)               
//{


//	while(--dly);	//dly=100: 8.75us; dly=100: 85.58 us (SYSCLK=72MHz)
//}


//��ʼ��IIC
void IIC_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	//RCC->APB2ENR|=1<<4;//��ʹ������IO PORTCʱ�� 
//	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOC, ENABLE );	
	__HAL_RCC_GPIOC_CLK_ENABLE();
	   
	GPIO_InitStructure.Pin = GPIO_PIN_12|GPIO_PIN_11;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP ;   //�������
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
 
	IIC_SCL=1;
	IIC_SDA=1;

}
//����IIC��ʼ�ź�
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA=1;	  	  
	IIC_SCL=1;
//	delay_us(4);
	nop_delay_2us();
	nop_delay_2us();

 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
//	delay_us(4);
		nop_delay_2us();
	nop_delay_2us();
	IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
// 	delay_us(4);
		nop_delay_2us();
	nop_delay_2us();
	IIC_SCL=1; 
	IIC_SDA=1;//����I2C���߽����ź�
//	delay_us(4);		
	nop_delay_2us();
	nop_delay_2us();	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
//	IIC_SDA=1;delay_us(1);	   
//	IIC_SCL=1;delay_us(1);	 
		IIC_SDA=1;nop_delay_1us();	   
		IIC_SCL=1;nop_delay_1us();
	
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	nop_delay_2us();
	IIC_SCL=1;
	nop_delay_2us();
	IIC_SCL=0;
}
//������ACKӦ��		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	nop_delay_2us();
	IIC_SCL=1;
	nop_delay_2us();
	IIC_SCL=0;
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		nop_delay_2us();   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL=1;
		nop_delay_2us();
		IIC_SCL=0;	
		nop_delay_2us();
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        nop_delay_2us();
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		nop_delay_1us();
    }					 
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}



























