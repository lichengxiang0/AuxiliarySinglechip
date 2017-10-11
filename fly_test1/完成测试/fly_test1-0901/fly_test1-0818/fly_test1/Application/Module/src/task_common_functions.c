/**
  ******************************************************************************
  * File Name          : task_common_functions.c
  * Description        : 这个文件主要是用来放置task层共用的函数
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
#include "task_common_functions.h"

/* External functions ------------------------------------------------------------*/

/**
  * @brief 	void	Bond_Int16_t(int16_t *sw_data_p,	int16_t sw_data_min_p,	int16_t sw_data_max_p)
	* @note	 	对int16_t类型的数据进行限幅
	* @param 	int16_t *sw_data_p：	数据
	* @param 	int16_t sw_data_min_p：下限
  * @param 	int16_t sw_data_max_p：上限
  */
void	Bond_Int16_t(int16_t *sw_data_p,	int16_t sw_data_min_p,	int16_t sw_data_max_p)
{
	if(*sw_data_p < sw_data_min_p)
	{
		*sw_data_p = sw_data_min_p;
	}
	else if(*sw_data_p > sw_data_max_p)
	{
		*sw_data_p = sw_data_max_p;
	}
}

/**
  * @brief 	void	BondFloat(float *data,	float data_min,	float data_max)
	* @note	 	对Float类型的数据进行限幅
	* @param 	float *data：	数据
	* @param 	float data_min：下限
  * @param 	float data_max：上限
  */
void	BondFloat(float *data,	float data_min,	float data_max)
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


/**
  * @brief 	void	BondTriangle(float *angle)
	* @note	 	对角度值进行限幅，限制在±180°的范围内
	* @param 	float *angle_p：	数据
  */
void	BondTriangle(float *angle_p)
{
	while(*angle_p > 180)
	{
		*angle_p -= 360;
	}
	
	while(*angle_p < -180)
	{
		*angle_p += 360;
	}
}



/**
  * @brief 	void	Average_Filter_Float(float* p_f_data_in_p,	float* p_f_data_out_p,uint8_t	uc_flt_data_num)
	* @note	 	对Float类型的数据进行均值滤波
	* @param 	float *p_f_data_in_p：	输入数据
	* @param 	float *p_f_data_in_p：	输出数据
  * @param 	uint8_t	uc_flt_data_num	滤波数据个数
*/ 
FlagStatus	Average_Filter_Float(float* p_f_data_in_p,	float* p_f_data_out_p,uint8_t	*uc_flt_cycle_index,uint8_t	uc_flt_data_num)
{
	static	float			f_data_in_sum_array[3] = {0,0,0};
	
	(*uc_flt_cycle_index)++;
	
	f_data_in_sum_array[0] += p_f_data_in_p[0];
	f_data_in_sum_array[1] += p_f_data_in_p[1];
	f_data_in_sum_array[2] += p_f_data_in_p[2];

	if(*uc_flt_cycle_index >= uc_flt_data_num)	
	{
		p_f_data_out_p[0] = f_data_in_sum_array[0]/uc_flt_data_num;
		p_f_data_out_p[1] = f_data_in_sum_array[1]/uc_flt_data_num;
		p_f_data_out_p[2] = f_data_in_sum_array[2]/uc_flt_data_num;
			
		f_data_in_sum_array[0] = 0;
		f_data_in_sum_array[1] = 0;
		f_data_in_sum_array[2] = 0;
		
		(*uc_flt_cycle_index) = 0;

		return(SET);
	}
	else
	{
		return(RESET);
	}
}


/**
  * @brief 	FlagStatus	Average_Filter_Uint16(uint16_t* p_uw_data_in_p,	uint16_t* p_uw_data_out_p,uint8_t	*uc_flt_cycle_index,uint8_t	uc_flt_data_num)
	* @note	 	对Float类型的数据进行均值滤波
	* @param 	float *p_uw_data_in_p：	输入数据
	* @param 	float *p_uw_data_out_p：	输出数据
  * @param 	uint8_t	uc_flt_data_num	滤波数据个数
*/ 
FlagStatus	Average_Filter_Uint16(uint16_t* p_uw_data_in_p,	uint16_t* p_uw_data_out_p,uint8_t	*uc_flt_cycle_index,uint8_t	uc_flt_data_num)
{
	static	uint32_t			ul_data_in_sum_array[2] = {0, 0};
	
	(*uc_flt_cycle_index)++;
	
	ul_data_in_sum_array[0] += p_uw_data_in_p[0];
	ul_data_in_sum_array[1] += p_uw_data_in_p[1];


	if(*uc_flt_cycle_index >= uc_flt_data_num)	
	{
		p_uw_data_out_p[0] = ul_data_in_sum_array[0]/uc_flt_data_num;
		p_uw_data_out_p[1] = ul_data_in_sum_array[1]/uc_flt_data_num;
			
		ul_data_in_sum_array[0] = 0;
		ul_data_in_sum_array[1] = 0;
		(*uc_flt_cycle_index) = 0;
		
		return(SET);
	}
	else
	{
		return(RESET);
	}
}


/**
  * @brief 	int8_t	Get_Float_Sign(float	f_float_data_p)
	* @note	 	获取Float类型的数据的符号
	* @param 	float f_float_data_p：	输入数据
*/ 
int8_t	Get_Float_Sign(float	f_float_data_p)
{
	if(f_float_data_p >= 0)
	{
		return 1;
	}
	else
	{
		return -1;
	}
}



