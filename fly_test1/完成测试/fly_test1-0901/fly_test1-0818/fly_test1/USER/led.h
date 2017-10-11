#ifndef __LED_H__
#define __LED_H__
#include "sys.h"


#define LED0 PAout(8)	// PA8
#define LED1 PDout(2)	// PD2	

#define CHARGE PCout(6)  //PC6

//油机启动的IO口定义
//PB5 作为油机的启动
#define OilEngineStart PBout(5)  //PB5

//PB4 作为油机熄火
#define OilEngineStop PBout(4)  //PB4


#endif



