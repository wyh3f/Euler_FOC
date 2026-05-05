//BSP_ADC
#ifndef __BSP_ADC_H
#define __BSP_ADC_H
#include "adc.h"
#include "stdio.h"
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <math.h>
#include "stdbool.h"
#include <stdint.h>


void BSP_ADC_init(void);

float BSP_GET_ADC(void);

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc);

extern float ADC_IN_Value[4];

#endif





