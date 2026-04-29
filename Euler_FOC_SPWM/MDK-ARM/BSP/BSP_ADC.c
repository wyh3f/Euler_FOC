//BSP_ADC
#include "BSP_ADC.h"


extern ADC_HandleTypeDef hadc1;

void BSP_ADC_init(void)
{
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);    //ADУ׼
}


