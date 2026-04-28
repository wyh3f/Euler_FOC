#include "BSP_KEY.h" 

uint8_t BSP_KEY1_Read(void)
{
    return HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == KEY_Value ? 1 : 0;
}

uint8_t BSP_KEY2_Read(void)
{
    return HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == KEY_Value ? 1 : 0;
}

uint8_t BSP_KEY3_Read(void)
{
    return HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) == KEY_Value ? 1 : 0;
}






