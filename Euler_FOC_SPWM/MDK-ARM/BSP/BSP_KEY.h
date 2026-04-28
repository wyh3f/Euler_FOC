//BSP_KEY.c
#ifndef __BSP_KEY_H
#define __BSP_KEY_H
#include "gpio.h"
#include "stdio.h"
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <math.h>
#include "stdbool.h"
#include <stdint.h>


#define KEY_Value GPIO_PIN_SET

uint8_t BSP_KEY1_Read(void);
uint8_t BSP_KEY2_Read(void);
uint8_t BSP_KEY3_Read(void);


#endif








