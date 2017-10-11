/**
  ******************************************************************************
  * File Name          : gimbal_ahrs.c
  * Description        : This file provides code for gimbal attitude .
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
	
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "gimbal_ahrs.h"
#include <math.h>

/* Private define --------------------------------------------------------*/

#define		GYRO_FLT_BUF_LENGTH  ( (uint8_t)1 ) 


#define	DEG_TO_RADIAN		((float)0.01744)
#define	RADIAN_TO_DEG		((float)57.3)
#define	PI							((float)3.14)
	
#define ToRad(x)				(x*DEG_TO_RADIAN)


/* External variables --------------------------------------------------------*/
float	f_sensor_gyro_raw_data_array[3],	f_sensor_gyro_offset_data_array[3] = {1.00,0.70,0.30};
//f_sensor_gyro_offset_data_array[3] = {0.84,0.11,1.35};
float	f_sensor_gyro_cal_data_array[3],	f_sensor_gyro_flt_data_array[3];

float	f_sensor_acc_raw_data_array[3],		f_sensor_acc_offset_data_array[3] = {-0.02,0,0.01};	
//f_sensor_acc_offset_data_array[3] = {-0.05,0,0.1};		
float	f_sensor_acc_cal_data_array[3],		f_sensor_acc_flt_data_array[3];
float	f_sensor_acc_scale_data_array[3]= {1,1,1};

float	f_gimbal_gyro_data_array[3];		
float	f_gimbal_acc_data_array[3];

float	f_quat_x_right_array[4] = {1,0,0,0},	f_quat_x_front_array[4] = {1,0,0,0},	f_quat_array[4];
float	f_euler_x_right_array[3],							f_euler_x_front_array[3],							f_euler_array[3];

FlagStatus	en_ahrs_init_flag = RESET;

/* Private variables --------------------------------------------------------*/
float	f_quad_detection_array[4] = {1,0,0,0};

/*X_Right坐标系和X_Front坐标系的旋转矩阵
float rot_matrix_x_right[9];
float	rot_matrix_x_front[9];
*/

float	f_rot_matrix_detection_array[9];


float	quad_kp = 0.2,		quad_ki = 0.004;

float	acc_correction_gain = 1,	acc_correction_gain_thr = 0.4;  //0.8-0.05g  0.2-0.2g 


/* Private functions ---------------------------------------------------------*/
float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long  i = *(long*)&y;
	
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}


/**
  * @brief 	void	Ahrs_Init(Acc_Data_Typedef st_acc_p,	Quat_Typedef*	st_quat_p)
	* @note	 	初始化quat
	* @param 	st_acc_p : 	acc data  
	* @param 	st_quat_p：	四元数
  */
void Ahrs_Init(float ax,	float ay,	float az,	float*	p_f_quat_p)
{
	float initialRoll, initialPitch;
	float cosRoll, sinRoll, cosPitch, sinPitch;
	float initialHdg, cosHeading, sinHeading;
//	float q0q0 ,q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;		//计算转换矩阵时可以节省乘法的运算次数
	

	initialRoll = -atan2(ax, az);
	initialPitch = atan2(ay, az);
	initialHdg = 0;

	cosRoll = cosf(initialRoll * 0.5f);
	sinRoll = sinf(initialRoll * 0.5f);

	cosPitch = cosf(initialPitch * 0.5f);
	sinPitch = sinf(initialPitch * 0.5f);

	cosHeading = cosf(initialHdg * 0.5);
	sinHeading = sinf(initialHdg * 0.5);

	p_f_quat_p[0] = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
	p_f_quat_p[1] = cosRoll * sinPitch * cosHeading - sinRoll * cosPitch * sinHeading;	
	p_f_quat_p[2] = sinRoll * cosPitch * cosHeading + cosRoll * sinPitch * sinHeading;	
	p_f_quat_p[3] = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;
}

/**
  * @brief 	void Ahrs_Update_X_Right(	float gx,	float gy,	float gz,	float ax,	float ay,	float az,	float	dt)																			
	* @note	 	X_RIGHT坐标系四元数更新
	* @param 	float gx gy gz：陀螺数据
	* @param 	float ax ay az：加速度数据
	* @param 	float	dt：	更新时间
  */
float 	ex_int_x_right = 0,	ey_int_x_right =0 ,	ez_int_x_right=0; 		//test
float		err_int_x_right[3],	err_int_x_right_flt[3];										//test
FlagStatus		err_int_flt_flag = RESET;																//test
uint8_t				uc_err_int_flt_index = 0,	uc_err_int_flt_num = 200;			//test

void Ahrs_Update_X_Right(	float gx,	float gy,	float gz,	float ax,	float ay,	float az,	float	dt)
{	
	float norm;
	float vx, vy, vz;
	float ex, ey, ez; 	
	
	float q0q0,q0q1,q0q2,q0q3,q1q1,q1q2,q1q3,q2q2,q2q3,q3q3;
	
	float		ex_kp = 0,	ey_kp = 0,	ez_kp = 0;
	static	float 	ex_int = 0,	ey_int =0 ,	ez_int=0;
	
	q0q0 = f_quat_x_right_array[0] * f_quat_x_right_array[0];								
	q0q1 = f_quat_x_right_array[0] * f_quat_x_right_array[1];
	q0q2 = f_quat_x_right_array[0] * f_quat_x_right_array[2];
	q0q3 = f_quat_x_right_array[0] * f_quat_x_right_array[3];

	q1q1 = f_quat_x_right_array[1] * f_quat_x_right_array[1];
	q1q2 = f_quat_x_right_array[1] * f_quat_x_right_array[2];
	q1q3 = f_quat_x_right_array[1] * f_quat_x_right_array[3];
	
	q2q2 = f_quat_x_right_array[2] * f_quat_x_right_array[2];
	q2q3 = f_quat_x_right_array[2] * f_quat_x_right_array[3];

	q3q3 = f_quat_x_right_array[3] * f_quat_x_right_array[3];	
	
	/*加速度归一化*/
	norm = invSqrt(ax*ax + ay* ay +  az*az);  
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm; 
	
	/*g在机体坐标系三个轴的值*/
	vx = 2.f * (q1q3 - q0q2);												//????xyz???
  vy = 2.f * (q2q3 + q0q1);	
  vz = q0q0 - q1q1 - q2q2 + q3q3;
	
	//角速度误差计算,only acc
	ex = (ay*vz - az*vy);
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);
	
	ex_kp = acc_correction_gain*quad_kp*ex;
	ey_kp = acc_correction_gain*quad_kp*ey;
	ez_kp = acc_correction_gain*quad_kp*ez;
	
	if(acc_correction_gain > acc_correction_gain_thr)    //加速度正常时才加积分值，否则将积分值清零
	{
		ex_int += acc_correction_gain*acc_correction_gain*quad_ki*ex*dt;
		ey_int += acc_correction_gain*acc_correction_gain*quad_ki*ey*dt;
		ez_int += acc_correction_gain*acc_correction_gain*quad_ki*ez*dt;
		
	}
	
	gx += (ex_kp + ex_int);
	gy += (ey_kp + ey_int);
	gz += (ez_kp + ez_int);
	
	/*test
	ex_int_x_right = ex_kp + ex_int;
	ey_int_x_right = ey_kp + ey_int;
	ez_int_x_right = ez_kp + ez_int;

	err_int_x_right[0] = ex_int_x_right;
	err_int_x_right[1] = ey_int_x_right;
	err_int_x_right[2] = ez_int_x_right;
	
	if(SET == err_int_flt_flag)
	{
		Average_Filter_Float(err_int_x_right,	err_int_x_right_flt,	&uc_err_int_flt_index,	uc_err_int_flt_num);
	}*/
	
	//四元数更新
	f_quat_x_right_array[0] = f_quat_x_right_array[0] + (-f_quat_x_right_array[1]*gx - f_quat_x_right_array[2]*gy - f_quat_x_right_array[3]*gz)*dt*0.5;
	f_quat_x_right_array[1] = f_quat_x_right_array[1] + ( f_quat_x_right_array[0]*gx + f_quat_x_right_array[2]*gz - f_quat_x_right_array[3]*gy)*dt*0.5;
	f_quat_x_right_array[2] = f_quat_x_right_array[2] + ( f_quat_x_right_array[0]*gy - f_quat_x_right_array[1]*gz + f_quat_x_right_array[3]*gx)*dt*0.5;
	f_quat_x_right_array[3] = f_quat_x_right_array[3] + ( f_quat_x_right_array[0]*gz + f_quat_x_right_array[1]*gy - f_quat_x_right_array[2]*gx)*dt*0.5;  	
	
	//四元数归一化处理
	norm = invSqrt(	f_quat_x_right_array[0]*f_quat_x_right_array[0] + f_quat_x_right_array[1]*f_quat_x_right_array[1] + 
									f_quat_x_right_array[2]*f_quat_x_right_array[2] + f_quat_x_right_array[3]*f_quat_x_right_array[3]);	
	
	f_quat_x_right_array[0] *= norm;
	f_quat_x_right_array[1] *= norm;
	f_quat_x_right_array[2] *= norm;
	f_quat_x_right_array[3] *= norm;
	
	f_euler_x_right_array[1] = -RADIAN_TO_DEG*asinf( 2.f * (q1q3 - q0q2) );
	f_euler_x_right_array[0] = RADIAN_TO_DEG*atan2f( 2.f * (q2q3 + q0q1), q0q0 - q1q1 - q2q2 + q3q3 );
	f_euler_x_right_array[2] = RADIAN_TO_DEG*atan2f( 2.f * (q1q2 + q0q3),	q0q0 + q1q1 - q2q2 - q3q3);
}

/**
  * @brief 	void Ahrs_Update_X_Front(	float gx,	float gy,	float gz,	float ax,	float ay,	float az,	float	dt)																			
	* @note	 	X_Front坐标系四元数更新
	* @param 	float gx gy gz：陀螺数据
	* @param 	float ax ay az：加速度数据
	* @param 	float	dt：	更新时间
  */
float 	ex_int_x_front = 0,	ey_int_x_front =0 ,	ez_int_x_front=0;				//test
void Ahrs_Update_X_Front(	float gx,	float gy,	float gz,	float ax,	float ay,	float az,	float	dt)
{	
	float norm;
	float vx, vy, vz;
	float ex, ey, ez; 	
	
	float q0q0,q0q1,q0q2,q0q3,q1q1,q1q2,q1q3,q2q2,q2q3,q3q3;
	
	float		ex_kp = 0,	ey_kp = 0,	ez_kp = 0;
	static	float 	ex_int = 0,	ey_int =0 ,	ez_int=0;
	
	q0q0 = f_quat_x_front_array[0] * f_quat_x_front_array[0];								
	q0q1 = f_quat_x_front_array[0] * f_quat_x_front_array[1];
	q0q2 = f_quat_x_front_array[0] * f_quat_x_front_array[2];
	q0q3 = f_quat_x_front_array[0] * f_quat_x_front_array[3];

	q1q1 = f_quat_x_front_array[1] * f_quat_x_front_array[1];
	q1q2 = f_quat_x_front_array[1] * f_quat_x_front_array[2];
	q1q3 = f_quat_x_front_array[1] * f_quat_x_front_array[3];
	
	q2q2 = f_quat_x_front_array[2] * f_quat_x_front_array[2];
	q2q3 = f_quat_x_front_array[2] * f_quat_x_front_array[3];

	q3q3 = f_quat_x_front_array[3] * f_quat_x_front_array[3];	
	
	/*加速度归一化*/
	norm = invSqrt(ax*ax + ay* ay +  az*az);  
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm; 
	
	/*g在机体坐标系三个轴的值*/
	vx = 2.f * (q1q3 - q0q2);												//????xyz???
  vy = 2.f * (q2q3 + q0q1);	
  vz = q0q0 - q1q1 - q2q2 + q3q3;
	
	//角速度误差计算,only acc
	ex = (ay*vz - az*vy) ;
	ey = (az*vx - ax*vz) ;
	ez = (ax*vy - ay*vx) ;
	
	ex_kp = acc_correction_gain*quad_kp*ex;
	ey_kp = acc_correction_gain*quad_kp*ey;
	ez_kp = acc_correction_gain*quad_kp*ez;
	
	if(acc_correction_gain > acc_correction_gain_thr)    //加速度正常时才加积分值，否则将积分值清零
	{
		ex_int += acc_correction_gain*acc_correction_gain*quad_ki*ex*dt;
		ey_int += acc_correction_gain*acc_correction_gain*quad_ki*ey*dt;
		ez_int += acc_correction_gain*acc_correction_gain*quad_ki*ez*dt;
	}
	
	
	gx += ex_kp + ex_int;
	gy += ey_kp + ey_int;
	gz += ez_kp + ez_int;
	
	/*test
	ex_int_x_front = ex_kp + ex_int;
	ey_int_x_front = ey_kp + ey_int;
	ez_int_x_front = ez_kp + ez_int;*/
	
	//四元数更新
	f_quat_x_front_array[0] = f_quat_x_front_array[0] + (-f_quat_x_front_array[1]*gx - f_quat_x_front_array[2]*gy - f_quat_x_front_array[3]*gz)*dt*0.5;
	f_quat_x_front_array[1] = f_quat_x_front_array[1] + ( f_quat_x_front_array[0]*gx + f_quat_x_front_array[2]*gz - f_quat_x_front_array[3]*gy)*dt*0.5;
	f_quat_x_front_array[2] = f_quat_x_front_array[2] + ( f_quat_x_front_array[0]*gy - f_quat_x_front_array[1]*gz + f_quat_x_front_array[3]*gx)*dt*0.5;
	f_quat_x_front_array[3] = f_quat_x_front_array[3] + ( f_quat_x_front_array[0]*gz + f_quat_x_front_array[1]*gy - f_quat_x_front_array[2]*gx)*dt*0.5;  	
	
	//四元数归一化处理
	norm = invSqrt(	f_quat_x_front_array[0]*f_quat_x_front_array[0] + f_quat_x_front_array[1]*f_quat_x_front_array[1] + 
									f_quat_x_front_array[2]*f_quat_x_front_array[2] + f_quat_x_front_array[3]*f_quat_x_front_array[3]);	
	
	f_quat_x_front_array[0] *= norm;
	f_quat_x_front_array[1] *= norm;
	f_quat_x_front_array[2] *= norm;
	f_quat_x_front_array[3] *= norm;
	
	f_euler_x_front_array[0] = RADIAN_TO_DEG*asinf( 2.f * (q1q3 - q0q2) );
	f_euler_x_front_array[1] = RADIAN_TO_DEG*atan2f( 2.f * (q2q3 + q0q1), q0q0 - q1q1 - q2q2 + q3q3 );
	f_euler_x_front_array[2] = RADIAN_TO_DEG*atan2f( 2.f * (q1q2 + q0q3),	q0q0 + q1q1 - q2q2 - q3q3);
}


/**
  * @brief 	void Ahrs_Update_Detection(	float gx,	float gy,	float gz,	float ax,	float ay,	float az,	float	dt)
	* @note	 	运动检测坐标系 的四元数更新
	* @param 	float gx gy gz：陀螺数据
	* @param 	float ax ay az：加速度数据
	* @param 	float	dt：	更新时间
  */
void Ahrs_Update_Detection(	float gx,	float gy,	float gz,	float ax,	float ay,	float az,	float	dt)
{	
	float norm;
	float vx, vy, vz;
	float ex, ey, ez; 	
	
	float q0q0,q0q1,q0q2,q0q3,q1q1,q1q2,q1q3,q2q2,q2q3,q3q3;
	
	float		ex_kp = 0,	ey_kp = 0,	ez_kp = 0;
	
	q0q0 = f_quad_detection_array[0] * f_quad_detection_array[0];								
	q0q1 = f_quad_detection_array[0] * f_quad_detection_array[1];
	q0q2 = f_quad_detection_array[0] * f_quad_detection_array[2];
	q0q3 = f_quad_detection_array[0] * f_quad_detection_array[3];

	q1q1 = f_quad_detection_array[1] * f_quad_detection_array[1];
	q1q2 = f_quad_detection_array[1] * f_quad_detection_array[2];
	q1q3 = f_quad_detection_array[1] * f_quad_detection_array[3];
	
	q2q2 = f_quad_detection_array[2] * f_quad_detection_array[2];
	q2q3 = f_quad_detection_array[2] * f_quad_detection_array[3];

	q3q3 = f_quad_detection_array[3] * f_quad_detection_array[3];

	f_rot_matrix_detection_array[0] = q0q0 + q1q1 - q2q2 - q3q3;		// 11
	f_rot_matrix_detection_array[1] = 2.f * (q1q2 + q0q3);			// 12
	f_rot_matrix_detection_array[2] = 2.f * (q1q3 - q0q2);			// 13
	f_rot_matrix_detection_array[3] = 2.f * (q1q2 - q0q3);			// 21
	f_rot_matrix_detection_array[4] = q0q0 - q1q1 + q2q2 - q3q3;		// 22
	f_rot_matrix_detection_array[5] = 2.f * (q2q3 + q0q1);			// 23
	f_rot_matrix_detection_array[6] = 2.f * (q1q3 + q0q2);			// 31
	f_rot_matrix_detection_array[7] = 2.f * (q2q3 - q0q1);			// 32
	f_rot_matrix_detection_array[8] = q0q0 - q1q1 - q2q2 + q3q3;		// 33	
	
	/*加速度归一化*/
	norm = invSqrt(ax*ax + ay* ay +  az*az);  
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm; 
	
	/*g在机体坐标系三个轴的值*/
	vx = 2.f * (q1q3 - q0q2);												//????xyz???
  vy = 2.f * (q2q3 + q0q1);	
  vz = q0q0 - q1q1 - q2q2 + q3q3;
	
	//角速度误差计算,only acc
	ex = (ay*vz - az*vy) ;
	ey = (az*vx - ax*vz) ;
	ez = (ax*vy - ay*vx) ;
	
	ex_kp = 1*ex;
	ey_kp = 1*ey;
	ez_kp = 1*ez;
	

	gx += ex_kp ;
	gy += ey_kp ;
	gz += ez_kp ;
	
	//四元数更新
	f_quad_detection_array[0] = f_quad_detection_array[0] + (-f_quad_detection_array[1]*gx - f_quad_detection_array[2]*gy - f_quad_detection_array[3]*gz)*dt*0.5;
	f_quad_detection_array[1] = f_quad_detection_array[1] + ( f_quad_detection_array[0]*gx + f_quad_detection_array[2]*gz - f_quad_detection_array[3]*gy)*dt*0.5;
	f_quad_detection_array[2] = f_quad_detection_array[2] + ( f_quad_detection_array[0]*gy - f_quad_detection_array[1]*gz + f_quad_detection_array[3]*gx)*dt*0.5;
	f_quad_detection_array[3] = f_quad_detection_array[3] + ( f_quad_detection_array[0]*gz + f_quad_detection_array[1]*gy - f_quad_detection_array[2]*gx)*dt*0.5;  	

	//四元数归一化处理
	norm = invSqrt(	f_quad_detection_array[0]*f_quad_detection_array[0] + f_quad_detection_array[1]*f_quad_detection_array[1] + 
									f_quad_detection_array[2]*f_quad_detection_array[2] + f_quad_detection_array[3]*f_quad_detection_array[3]);	
	
	f_quad_detection_array[0] *= norm;
	f_quad_detection_array[1] *= norm;
	f_quad_detection_array[2] *= norm;
	f_quad_detection_array[3] *= norm;
}
/**
  * @brief 	void	Gyro_Data_Fliter(float* p_f_gyro_in_p,	float* p_f_gyro_out_p,	FlagStatus	*en_first_run_flag_p)
	* @note	 	陀螺数据滤波
	* @param 	float* p_f_gyro_in_p：	原始数据
	* @param 	float* p_f_gyro_out_p：	滤波数据
  */
void	Gyro_Data_Fliter(float* p_f_gyro_in_p,	float* p_f_gyro_out_p,	FlagStatus	*en_first_run_flag_p)
{
	static	float			f_gyro_flt_buf_p[3][GYRO_FLT_BUF_LENGTH];
	static  uint8_t		gyro_flt_buf_index = 0;
	float							f_gyro_sum_p[3];
	
	/*当GYRO_FLT_BUF_LENGTH = 1时，非仿真模式下会出错,原因未知*/ 
	if(GYRO_FLT_BUF_LENGTH <= 1)
	{
		p_f_gyro_out_p[0] = p_f_gyro_in_p[0];
		p_f_gyro_out_p[1] = p_f_gyro_in_p[1];
		p_f_gyro_out_p[2] = p_f_gyro_in_p[2];
	}
	else
	{ 
		/*记录最近GYRO_FLT_BUF_LENGTH次陀螺数据*/	
		f_gyro_flt_buf_p[0][gyro_flt_buf_index] = p_f_gyro_in_p[0];
		f_gyro_flt_buf_p[1][gyro_flt_buf_index] = p_f_gyro_in_p[1];
		f_gyro_flt_buf_p[2][gyro_flt_buf_index] = p_f_gyro_in_p[2];
		
		gyro_flt_buf_index++;
		gyro_flt_buf_index = gyro_flt_buf_index%GYRO_FLT_BUF_LENGTH;
		
		/*判断滤波数据是否>GYRO_FLT_BUF_LENGTH*/
		if(gyro_flt_buf_index >= (GYRO_FLT_BUF_LENGTH -1) )
		{
			*en_first_run_flag_p = RESET;
		}

		/*第一个滤波周期，数据不足，避免得到错误的滤波数据*/
		if(SET == *en_first_run_flag_p)
		{
			p_f_gyro_out_p[0] = p_f_gyro_in_p[0];
			p_f_gyro_out_p[1] = p_f_gyro_in_p[1];
			p_f_gyro_out_p[2] = p_f_gyro_in_p[2];	
		}
		else
		{
			/*输出滤波后的值*/
			for(uint8_t i=0;i<GYRO_FLT_BUF_LENGTH;i++)
			{
				f_gyro_sum_p[0] += f_gyro_flt_buf_p[0][i];
				f_gyro_sum_p[1] += f_gyro_flt_buf_p[1][i];
				f_gyro_sum_p[2] += f_gyro_flt_buf_p[2][i];
			}
			
			p_f_gyro_out_p[0] = f_gyro_sum_p[0]/GYRO_FLT_BUF_LENGTH;
			p_f_gyro_out_p[1] = f_gyro_sum_p[1]/GYRO_FLT_BUF_LENGTH;
			p_f_gyro_out_p[2] = f_gyro_sum_p[2]/GYRO_FLT_BUF_LENGTH;
		}
	}
}

/**
  * @brief 	void	Acc_Data_Fliter(float* p_f_acc_in_p,	float* p_f_acc_out_p,	FlagStatus *en_first_run_flag_p)
	* @note	 	加速度数据滤波
	* @param 	float* p_f_acc_in_p：		原始数据
	* @param 	float* p_f_acc_out_p：	滤波数据
  */
void	Acc_Data_Fliter(float* p_f_acc_in_p,	float* p_f_acc_out_p,	FlagStatus *en_first_run_flag_p)
{
	/*filter*/
	if(SET == *en_first_run_flag_p)
	{
		p_f_acc_out_p[0] = p_f_acc_in_p[0];
		p_f_acc_out_p[1] = p_f_acc_in_p[1];
		p_f_acc_out_p[2] = p_f_acc_in_p[2];
		
		*en_first_run_flag_p = RESET;
	}
	else
	{
		p_f_acc_out_p[0] = 0.99*p_f_acc_out_p[0] + 0.01*p_f_acc_in_p[0];
		p_f_acc_out_p[1] = 0.99*p_f_acc_out_p[1] + 0.01*p_f_acc_in_p[1];
		p_f_acc_out_p[2] = 0.99*p_f_acc_out_p[2] + 0.01*p_f_acc_in_p[2];
	}
}


/**
  * @brief 	void	BondFloat(float *data,	float data_min,	float data_max)
	* @note	 	对Float类型的数据进行限幅
	* @param 	float *data：	数据
	* @param 	float data_min：下限
  * @param 	float data_max：上限
  */
void	BondFloat_Ahrs(float *data,	float data_min,	float data_max)
{
	if(*data < data_min)
	{
		*data = data_min;
	}
	else if(*data > data_max)
	{
		*data = data_max;
	}
}
/* External functions ---------------------------------------------------------*/

/**
  * @brief 	void	Cal_Filter_Sensor_Data(void)
	* @note	 	对加速度和陀螺数据进行滤波
  */
//FlagStatus	en_gyro_data_first_flt_flag = SET,	en_acc_data_first_flt_flag = SET;
void	Cal_Filter_Sensor_Data(void)
{
	static	FlagStatus	en_gyro_data_first_flt_flag = SET,	en_acc_data_first_flt_flag = SET;
	
	/*修正漂移*/
	f_sensor_gyro_cal_data_array[0] = f_sensor_gyro_raw_data_array[0] + f_sensor_gyro_offset_data_array[0];
	f_sensor_gyro_cal_data_array[1] = f_sensor_gyro_raw_data_array[1] + f_sensor_gyro_offset_data_array[1];
	f_sensor_gyro_cal_data_array[2] = f_sensor_gyro_raw_data_array[2] + f_sensor_gyro_offset_data_array[2];
	
	/*修正加速度偏置*/
	f_sensor_acc_cal_data_array[0] = f_sensor_acc_raw_data_array[0] + f_sensor_acc_offset_data_array[0];
	f_sensor_acc_cal_data_array[1] = f_sensor_acc_raw_data_array[1] + f_sensor_acc_offset_data_array[1];
	f_sensor_acc_cal_data_array[2] = f_sensor_acc_raw_data_array[2] + f_sensor_acc_offset_data_array[2];
	
	/*修正加速度量程*/
	f_sensor_acc_cal_data_array[0] *= f_sensor_acc_scale_data_array[0];
	f_sensor_acc_cal_data_array[1] *= f_sensor_acc_scale_data_array[1];
	f_sensor_acc_cal_data_array[2] *= f_sensor_acc_scale_data_array[2];
	
	Gyro_Data_Fliter(f_sensor_gyro_cal_data_array,	f_sensor_gyro_flt_data_array,	&en_gyro_data_first_flt_flag);  //速度滤波
	Acc_Data_Fliter(f_sensor_acc_cal_data_array,	f_sensor_acc_flt_data_array,	&en_acc_data_first_flt_flag);			//加速度滤波
}

void	Gimbal_Ahrs_Init(void)
{
	float	gx,gy,gz,ax,ay,az,mx,my,mz;
	
	f_gimbal_gyro_data_array[0] = f_sensor_gyro_cal_data_array[GIMBAL_GYRO_X_DATA_INDEX];
	f_gimbal_gyro_data_array[1] = f_sensor_gyro_cal_data_array[GIMBAL_GYRO_Y_DATA_INDEX];
	f_gimbal_gyro_data_array[2] = f_sensor_gyro_cal_data_array[GIMBAL_GYRO_Z_DATA_INDEX];
	
	f_gimbal_acc_data_array[0] = f_sensor_acc_cal_data_array[GIMBAL_ACC_X_DATA_INDEX];
	f_gimbal_acc_data_array[1] = f_sensor_acc_cal_data_array[GIMBAL_ACC_Y_DATA_INDEX];
	f_gimbal_acc_data_array[2] = f_sensor_acc_cal_data_array[GIMBAL_ACC_Z_DATA_INDEX];
	
	gx = f_gimbal_gyro_data_array[0]*DEG_TO_RADIAN;
	gy = f_gimbal_gyro_data_array[1]*DEG_TO_RADIAN;
	gz = f_gimbal_gyro_data_array[2]*DEG_TO_RADIAN;
	
	ax = f_gimbal_acc_data_array[0];
	ay = f_gimbal_acc_data_array[1];
	az = f_gimbal_acc_data_array[2];
	
	Ahrs_Init(ax,  ay, az, f_quat_x_right_array);
	Ahrs_Init(ay, -ax, az, f_quat_x_front_array);
	Ahrs_Init(ax,  ay, az, f_quad_detection_array);
}

uint32_t ul_gyro_err_cnt = 0, i=0;    	//test
void	Gimbal_Ahrs_Update(float	dt)
{
	float	gx,gy,gz,ax,ay,az,mx,my,mz;
	
	float	gravity_x,		gravity_y,	gravity_z;
	float gravity_hor,	gravity_ver;
	float acc_correction_gain_hor,	acc_correction_gain_ver;
	
	f_gimbal_gyro_data_array[0] = f_sensor_gyro_flt_data_array[GIMBAL_GYRO_X_DATA_INDEX];
	f_gimbal_gyro_data_array[1] = f_sensor_gyro_flt_data_array[GIMBAL_GYRO_Y_DATA_INDEX];
	f_gimbal_gyro_data_array[2] = f_sensor_gyro_flt_data_array[GIMBAL_GYRO_Z_DATA_INDEX];
	
	f_gimbal_acc_data_array[0] = f_sensor_acc_flt_data_array[GIMBAL_ACC_X_DATA_INDEX];
	f_gimbal_acc_data_array[1] = f_sensor_acc_flt_data_array[GIMBAL_ACC_Y_DATA_INDEX];
	f_gimbal_acc_data_array[2] = f_sensor_acc_flt_data_array[GIMBAL_ACC_Z_DATA_INDEX];
	
	
	/*test*/  /*求绝对值*/
	if( (fabs(f_gimbal_gyro_data_array[0]) > 100)||
			(fabs(f_gimbal_gyro_data_array[1]) > 100)||
			(fabs(f_gimbal_gyro_data_array[2]) > 100)
		)
	{		
		ul_gyro_err_cnt++;
		i++;
	}
	
	gx = f_gimbal_gyro_data_array[0]*DEG_TO_RADIAN;
	gy = f_gimbal_gyro_data_array[1]*DEG_TO_RADIAN;
	gz = f_gimbal_gyro_data_array[2]*DEG_TO_RADIAN;
	
	ax = f_gimbal_acc_data_array[0];
	ay = f_gimbal_acc_data_array[1];
	az = f_gimbal_acc_data_array[2];
	
	if(RESET == en_ahrs_init_flag)
	{
		quad_kp = 5; 
		quad_ki = 0.04;
//		quad_ki = 0;
	}
	else
	{
		quad_kp = 0.5;
		quad_ki = 0.004;
	
		/*计算运动加速度 修改加速度比重*/
		Ahrs_Update_Detection(gx, gy, gz,	ax,  ay, az, dt);
		
		gravity_x = ax*f_rot_matrix_detection_array[0] + ay*f_rot_matrix_detection_array[3] + az*f_rot_matrix_detection_array[6];
		gravity_y = ax*f_rot_matrix_detection_array[1] + ay*f_rot_matrix_detection_array[4] + az*f_rot_matrix_detection_array[7];
		gravity_z = ax*f_rot_matrix_detection_array[2] + ay*f_rot_matrix_detection_array[5] + az*f_rot_matrix_detection_array[8];
		
		gravity_hor	=	sqrt(gravity_x*gravity_x + gravity_y*gravity_y);
		gravity_ver = fabs( fabs(gravity_z) - 1 );
			
		acc_correction_gain_hor = 1.0 - 4*gravity_hor;  //0.4g=0.8->gravity_gain_hor = 0.2
		acc_correction_gain_ver = 1.0 - 4*gravity_ver;
		
		BondFloat_Ahrs(&acc_correction_gain_hor,0.1,1);
		BondFloat_Ahrs(&acc_correction_gain_ver,0.1,1);
						
		acc_correction_gain = acc_correction_gain_hor<acc_correction_gain_ver?acc_correction_gain_hor:acc_correction_gain_ver;
	}
	
	/*更新X_Right坐标系和X_Front坐标系的姿态*/
	Ahrs_Update_X_Right(gx,   gy, gz,	ax,  ay, az, dt);
	
	//Ahrs_Update_X_Front(gy,  -gx, gz,	ay, -ax, az, dt);
}



