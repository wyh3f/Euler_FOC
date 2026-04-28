//DRIVER_MT6816
#ifndef __DRIVER_MT6816_H
#define __DRIVER_MT6816_H
#include "spi.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdint.h>
#include "stdbool.h"
#include <math.h>

#include "BSP_SPI.h"  

#define MT6816_SPI_TransmitReceive(MT6816_t,MT6816_r,MT6816_s) BSP_SPI1_TransmitReceive(MT6816_t, MT6816_r, MT6816_s)

#define MT6816_CS_H HAL_GPIO_WritePin(MT6816_CS_GPIO_Port, MT6816_CS_Pin, GPIO_PIN_SET)
#define MT6816_CS_L HAL_GPIO_WritePin(MT6816_CS_GPIO_Port, MT6816_CS_Pin, GPIO_PIN_RESET)


/**
 * @brief 读取MT6816的原始14位角度值
 * @retval 14-bit原始角度数据 (0 - 16383)
 */
uint16_t DRIVER_MT6816_Read_RawAngle(void);



/**
 * @brief 将MT6816的14位原始值转换为角度（度）
 * @param raw_angle 14位原始角度数据 (0 - 16383)
 * @return 角度值，单位：度（浮点数）
 */
float DRIVER_MT6816_RawAngleToDegree(uint16_t raw_angle);


/**
 * @brief 将MT6816的14位原始值转换为角度（0.01度精度）
 * @param raw_angle 14位原始角度数据 (0 - 16383)
 * @return 角度值 = 实际角度 * 100，例如 9000 表示 90.00度
 */
uint16_t DRIVER_MT6816_RawAngleToCentidegree(uint16_t raw_angle);


#endif
