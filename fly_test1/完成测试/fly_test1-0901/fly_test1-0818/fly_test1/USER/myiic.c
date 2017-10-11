#include "myiic.h"
#include "stm32f1xx_hal.h"
#include "mynop.h"

//#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//Mini STM32开发板
//IIC 驱动函数	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2010/6/10 
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 正点原子 2009-2019
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	  

/*************************************************************************************
函数原型：void I2C_Delay(unsigned int dly)
函数功能：延时函数
输入参数：unsigned int dly：表示延时多少个时钟周期
输出参数：无
版本：A1.0
**************************************************************************************/



//初始化IIC
void IIC_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	//RCC->APB2ENR|=1<<4;//先使能外设IO PORTC时钟 
//	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOC, ENABLE );	
	__HAL_RCC_GPIOB_CLK_ENABLE();
	   
	GPIO_InitStructure.Pin = GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP ;   //推挽输出
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
 
	IIC_SCL=1;
	IIC_SDA=1;

}
//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA=1;	  	  
	IIC_SCL=1;
//	delay_us(4);
	Delay_nop_4us();

 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
//	delay_us(4);
	Delay_nop_4us();
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
// 	delay_us(4);
	Delay_nop_4us();
	IIC_SCL=1; 
	IIC_SDA=1;//发送I2C总线结束信号
//	delay_us(4);		
		Delay_nop_4us();
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入   
		IIC_SDA=1;Delay_nop_1us();	   
		IIC_SCL=1;Delay_nop_1us();
	
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	Delay_nop_2us();
	IIC_SCL=1;
	Delay_nop_2us();
	IIC_SCL=0;
}
//不产生ACK应答		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	Delay_nop_2us();
	IIC_SCL=1;
	Delay_nop_2us();
	IIC_SCL=0;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		Delay_nop_2us();   //对TEA5767这三个延时都是必须的
		IIC_SCL=1;
		Delay_nop_2us();
		IIC_SCL=0;	
		Delay_nop_2us();
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        Delay_nop_2us();
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		Delay_nop_1us();
    }					 
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}



























