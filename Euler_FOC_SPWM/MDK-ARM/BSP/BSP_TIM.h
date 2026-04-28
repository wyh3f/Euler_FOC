#ifndef __BSP_TIM_H
#define __BSP_TIM_H
#include "main.h" 


void TIM_init(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void TIM_6(TIM_HandleTypeDef *htim);


extern uint16_t time_oled;
extern volatile uint8_t time_buf_1ms;

#endif



