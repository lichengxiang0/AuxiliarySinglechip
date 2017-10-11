#ifndef _MC_FOC_DRIVE_H
#define	_MC_FOC_DRIVE_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "MC_type.h"
#include "MC_Clarke_Park.h"
#include "MC_Motor_Pos.h"
#include "stm32f0x_svpwm_3shunt.h"	 
	 
#include "pid_regulator.h"
	 

/* default values for Torque control loop */	 
#define TORQUE_PID_REFERENCE   (s16)3000   //(N.b: that's the reference init  
																						//value in both torque and speed control)
#define TORQUE_PID_KP_DEFAULT  (s16)5000//1000//1500//500   //250    
#define TORQUE_PID_KI_DEFAULT  (s16)30000//8000//8000      //1800  (30000->2048)
#define TORQUE_PID_KD_DEFAULT  (s16)0//100

/* default values for Flux control loop */
#define FLUX_PID_REFERENCE   (s16)0

#define FLUX_PID_KP_DEFAULT  (s16)5000//157 
#define FLUX_PID_KI_DEFAULT  (s16)30000//67
#define FLUX_PID_KD_DEFAULT  (s16)0


/* default values for Torque Flux(TF) PID 的放大倍数 */
#define TF_KPDIV ((u16)(256))			//  						//只能设置为256  Fast_PI_Regulator
#define TF_KIDIV ((u16)(4096))		//							//只能设置为24096		Fast_PI_Regulator
#define TF_KDDIV ((u16)(4096))		//							//只能设置为24096		Fast_PI_Regulator


/* External variables ---------------------------------------------------------*/
extern	Curr_Components Stat_Curr_q_d_ref; //速度环的输出，如果没有弱磁模块，直接作为电流环给定值。
extern	Curr_Components Stat_Curr_q_d;





void	Motor_Torque_Flux_Pid_Init(void);

void 	FOC_Model(void);
void 	Motor_Fix_On_Elec_Zero_Pos(void);


#ifdef __cplusplus
}
#endif	 

#endif
