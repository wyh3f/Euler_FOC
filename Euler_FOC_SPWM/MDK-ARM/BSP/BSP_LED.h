//BSP_LED
#ifndef __BSP_LED_H
#define __BSP_LED_H
#include "gpio.h"
#include "stdio.h"
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <math.h>
#include "stdbool.h"
#include <stdint.h>

#define LED_Value 1

void BSP_LED1_Write(uint8_t w);
void BSP_LED2_Write(uint8_t w);

void BSP_LED1_Flip(void);
void BSP_LED2_Flip(void);
#endif


