#ifndef __BSP_TIM_H
#define __BSP_TIM_H
#include "tim.h"
#include "stdio.h"
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <math.h>
#include "stdbool.h"
#include <stdint.h>






void BSP_TIM_init(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);



extern volatile uint16_t BSP_time_TIM6;
extern volatile uint16_t BSP_time_TIM1;

#endif



