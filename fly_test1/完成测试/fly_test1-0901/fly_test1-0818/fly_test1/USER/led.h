#ifndef __LED_H__
#define __LED_H__
#include "sys.h"


#define LED0 PAout(8)	// PA8
#define LED1 PDout(2)	// PD2	

#define CHARGE PCout(6)  //PC6

//�ͻ�������IO�ڶ���
//PB5 ��Ϊ�ͻ�������
#define OilEngineStart PBout(5)  //PB5

//PB4 ��Ϊ�ͻ�Ϩ��
#define OilEngineStop PBout(4)  //PB4


#endif



