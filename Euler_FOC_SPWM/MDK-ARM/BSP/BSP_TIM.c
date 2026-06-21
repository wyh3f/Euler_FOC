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
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,4190);
	//__HAL_TIM_GET_AUTORELOAD(&htim1)/2
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


#include "ALGORITHM_Clarke_Park.h"
#include "DRIVER_MT6816.h" 
#include "DRIVER_ThreePhase_Motor.h"
extern ThreePhase ABC;
extern ThreePhase PWM;
extern Clarke Alpha_Beta;
extern Park QP_thet;
extern ThreePhase Detection_ABC;
extern Clarke Detection_Alpha_Beta;
extern Park Detection_QP_thet;

volatile uint16_t BSP_time_TIM1=0;
void TIM_1(TIM_HandleTypeDef *htim)
{
	if(++BSP_time_TIM1>=100)BSP_time_TIM1=0;
	
//	DRIVER_Park_Limit(&QP_thet);
//	DRIVER_UpdateTheta_1(&QP_thet);
//	
//	ALGORITHM_Inverse_Park(&Alpha_Beta,&QP_thet);
//	ALGORITHM_Inverse_Clarke(&ABC,&Alpha_Beta);
//	
//	DRIVER_UpdateThreePhase_PWM_1(&ABC,&PWM);
//	
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,(uint16_t)PWM.A);
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,(uint16_t)PWM.B);
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,(uint16_t)PWM.C);	
	
	
//	TestCode_Main();
}

