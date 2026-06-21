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

#define MT6816_MumberOfPolePairs 4U

#define MT6816_MECH_OFFSET 7.11f

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



/**
 * @brief 获取MT6816的电角度（浮点度数），基于机械角度转换
 * @note  电角度 = (极对数 × 机械角度 - 机械偏移) % 360°，结果归一化至 [0, 360)
 * @retval 电角度，单位：度（浮点数），范围 0.0 ~ 359.999...
 */
float DRIVER_MT6816_Get_ElectricalAngle_Degree(void);

/**
 * @brief 获取MT6816的电角度（浮点度数），直接从原始计数计算
 * @note  电角度 = (原始计数 × 极对数 × 360 / 16384 - 偏移等效计数) % 360°
 *        偏移等效计数 = 机械偏移 × 16384 / 360，提前计算可优化
 * @retval 电角度，单位：度（浮点数），范围 0.0 ~ 359.999...
 */
float DRIVER_MT6816_Get_ElectricalAngle_Direct(void);


#endif
