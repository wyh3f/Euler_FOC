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

void TIM_init(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);



extern volatile uint16_t time_TIM6;
extern volatile uint8_t time_TIM1;

#endif



