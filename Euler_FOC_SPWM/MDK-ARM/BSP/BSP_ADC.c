//BSP_ADC
#include "BSP_ADC.h"

extern ADC_HandleTypeDef hadc1;



uint16_t ADC_IN11_Value;  //ADC值





void BSP_ADC_init(void)
{
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);    //AD校准
}


float BSP_GET_ADC(void)
{
	HAL_ADC_Start(&hadc1);     //启动ADC转换
	HAL_ADC_PollForConversion(&hadc1, 50);   //等待转换完成，时间为50ms
  ADC_IN11_Value = HAL_ADC_GetValue(&hadc1);
	
	return ADC_IN11_Value*0.0008058f;
}


