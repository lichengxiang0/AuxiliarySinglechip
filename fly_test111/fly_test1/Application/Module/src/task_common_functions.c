/**
  ******************************************************************************
  * File Name          : task_common_functions.c
  * Description        : ����ļ���Ҫ����������task�㹲�õĺ���
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
	* @note	 	��int16_t���͵����ݽ����޷�
	* @param 	int16_t *sw_data_p��	����
	* @param 	int16_t sw_data_min_p������
  * @param 	int16_t sw_data_max_p������
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
	* @note	 	��Float���͵����ݽ����޷�
	* @param 	float *data��	����
	* @param 	float data_min������
  * @param 	float data_max������
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
	* @note	 	�ԽǶ�ֵ�����޷��������ڡ�180��ķ�Χ��
	* @param 	float *angle_p��	����
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
	* @note	 	��Float���͵����ݽ��о�ֵ�˲�
	* @param 	float *p_f_data_in_p��	��������
	* @param 	float *p_f_data_in_p��	�������
  * @param 	uint8_t	uc_flt_data_num	�˲����ݸ���
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
	* @note	 	��Float���͵����ݽ��о�ֵ�˲�
	* @param 	float *p_uw_data_in_p��	��������
	* @param 	float *p_uw_data_out_p��	�������
  * @param 	uint8_t	uc_flt_data_num	�˲����ݸ���
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
	* @note	 	��ȡFloat���͵����ݵķ���
	* @param 	float f_float_data_p��	��������
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



