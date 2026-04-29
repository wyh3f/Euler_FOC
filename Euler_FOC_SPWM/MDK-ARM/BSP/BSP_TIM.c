/*
 * @Author: 王总
 * @Date: 2025-05-16 19:01:52
 * @LastEditTime: 2025-05-20 17:24:00
 * @LastEditors: 王总
 * @Description: 
 * @FilePath: \MDK-ARM\BSP\BSP_TIM.c
 * 兄弟，身体和灵魂总要有一个在路上
 */
#include "BSP_TIM.h"  
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim1;




/*
设置	CCR (占空比)	__HAL_TIM_SET_COMPARE(htim, channel, pulse)
读取	CCR (捕获值)	__HAL_TIM_GET_COMPARE(htim, channel)

设置	ARR (周期/频率)	__HAL_TIM_SET_AUTORELOAD(htim, autoreload)
读取	ARR (周期/频率)	__HAL_TIM_GET_AUTORELOAD(htim)

设置	CNT (计数器)	__HAL_TIM_SET_COUNTER(htim, counter)
读取	CNT (计数值)	__HAL_TIM_GET_COUNTER(htim)


*/



void BSP_TIM_init(void)
{
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim1);
	//开启PWM互补通道输出
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);		//PE8,PE9 开始产生PWM信号
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);  //在互补输出上开始PWM信号的产生
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_4);
	
	
//	
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
}


void TIM_1(TIM_HandleTypeDef *htim);
void TIM_6(TIM_HandleTypeDef *htim);


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
	if(htim == &htim6)
	{
		TIM_6(&htim6);
	}
	if(htim == &htim1)
	{
		TIM_1(&htim1);
	}
}

volatile uint16_t BSP_time_TIM6=0;
void TIM_6(TIM_HandleTypeDef *htim)
{
	
	if(++BSP_time_TIM6>=1000)
	{
		BSP_time_TIM6=0;
	}
	
	
}

volatile uint16_t BSP_time_TIM1=0;
void TIM_1(TIM_HandleTypeDef *htim)
{
	if(++BSP_time_TIM1>=100)BSP_time_TIM1=0;
//	TestCode_Main();
}

