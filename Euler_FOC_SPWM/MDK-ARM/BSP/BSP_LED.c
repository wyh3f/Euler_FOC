//BSP_LED
#include "BSP_LED.h"

void BSP_LED1_Write(uint8_t w)
{
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,w==LED_Value ? GPIO_PIN_RESET:GPIO_PIN_SET);
}

void BSP_LED2_Write(uint8_t w)
{
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,w==LED_Value ? GPIO_PIN_RESET:GPIO_PIN_SET);
}


void BSP_LED1_Flip(void)
{
	HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
}

void BSP_LED2_Flip(void)
{
	HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
}



