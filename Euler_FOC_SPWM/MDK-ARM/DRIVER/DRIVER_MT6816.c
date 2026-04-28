//DRIVER_MT6816
#include "DRIVER_MT6816.h" 
#include "spi.h"
#include <stdint.h>

#define MT6816_CS_H HAL_GPIO_WritePin(MT6816_CS_GPIO_Port, MT6816_CS_Pin, GPIO_PIN_SET)
#define MT6816_CS_L HAL_GPIO_WritePin(MT6816_CS_GPIO_Port, MT6816_CS_Pin, GPIO_PIN_RESET)

/* 快速指令定义 */
#define MT6816_READ_ANGLE_MSB  0x83
#define MT6816_READ_ANGLE_LSB  0x84


extern SPI_HandleTypeDef hspi1;




/**
 * @brief 读取MT6816的原始14位角度值
 * @retval 14-bit原始角度数据 (0 - 16383)
 */
uint16_t MT6816_Read_RawAngle(void) {
    uint8_t tx_buffer[2] = {0};
    uint8_t rx_buffer[2] = {0};
    uint8_t data_high = 0;
    uint8_t data_low = 0;

    /* 读取高字节寄存器 (0x03) */
    tx_buffer[0] = MT6816_READ_ANGLE_MSB;
    tx_buffer[1] = 0x00;
    MT6816_CS_L;
    if (HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 2, 100) != HAL_OK) {
        MT6816_CS_H;
        return 0;
    }
    MT6816_CS_H;
    data_high = rx_buffer[1]; /* 高字节有效数据 */

    /* 读取低字节寄存器 (0x04) */
    tx_buffer[0] = MT6816_READ_ANGLE_LSB;
    tx_buffer[1] = 0x00;
    MT6816_CS_L;
    if (HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 2, 100) != HAL_OK) {
        MT6816_CS_H;
        return 0;
    }
    MT6816_CS_H;
    data_low = rx_buffer[1]; /* 低字节有效数据 */

    /* 数据拼接：高6位+低8位，形成14位角度值 */
    uint16_t raw_angle = ((uint16_t)data_high << 6) | (data_low >> 2);
    return raw_angle;
}




/**
 * @brief 将MT6816的14位原始值转换为角度（度）
 * @param raw_angle 14位原始角度数据 (0 - 16383)
 * @return 角度值，单位：度（浮点数）
 */
float MT6816_RawAngleToDegree(uint16_t raw_angle)
{
    return (raw_angle * 360.0f) / 16384.0f;
}


/**
 * @brief 将MT6816的14位原始值转换为角度（0.01度精度）
 * @param raw_angle 14位原始角度数据 (0 - 16383)
 * @return 角度值 = 实际角度 * 100，例如 9000 表示 90.00度
 */
uint16_t MT6816_RawAngleToCentidegree(uint16_t raw_angle)
{
    // 使用 32 位中间变量避免溢出：raw_angle * 36000 / 16384
    return (uint16_t)(((uint32_t)raw_angle * 36000UL) / 16384UL);
}




