//DRIVER_MT6816
#include "DRIVER_MT6816.h" 
#include <math.h>



/* 快速指令定义 */
#define MT6816_READ_ANGLE_MSB  0x83
#define MT6816_READ_ANGLE_LSB  0x84

extern SPI_HandleTypeDef hspi1;




/**
 * @brief 读取MT6816的原始14位角度值
 * @retval 14-bit原始角度数据 (0 - 16383)
 */
uint16_t DRIVER_MT6816_Read_RawAngle(void) 
{
    uint8_t tx_buffer[2] = {0};
    uint8_t rx_buffer[2] = {0};
    uint8_t data_high = 0;
    uint8_t data_low = 0;

    /* 读取高字节寄存器 (0x03) */
    tx_buffer[0] = MT6816_READ_ANGLE_MSB;
    tx_buffer[1] = 0x00;
    MT6816_CS_L;
    if (MT6816_SPI_TransmitReceive(tx_buffer, rx_buffer, 2) != 0) {
        MT6816_CS_H;
        return 0;
    }
    MT6816_CS_H;
    data_high = rx_buffer[1]; /* 高字节有效数据 */

    /* 读取低字节寄存器 (0x04) */
    tx_buffer[0] = MT6816_READ_ANGLE_LSB;
    tx_buffer[1] = 0x00;
    MT6816_CS_L;
    if (MT6816_SPI_TransmitReceive(tx_buffer, rx_buffer, 2) != 0) {
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
float DRIVER_MT6816_RawAngleToDegree(uint16_t raw_angle)
{
    return (raw_angle * 360.0f) / 16384.0f;
}


/**
 * @brief 将MT6816的14位原始值转换为角度（0.01度精度）
 * @param raw_angle 14位原始角度数据 (0 - 16383)
 * @return 角度值 = 实际角度 * 100，例如 9000 表示 90.00度
 */
uint16_t DRIVER_MT6816_RawAngleToCentidegree(uint16_t raw_angle)
{
    // 使用 32 位中间变量避免溢出：raw_angle * 36000 / 16384
    return (uint16_t)(((uint32_t)raw_angle * 36000UL) / 16384UL);
}



/**
 * @brief 获取MT6816的电角度（浮点度数），基于机械角度转换
 * @note  电角度 = (极对数 × 机械角度 - 机械偏移) % 360°，结果归一化至 [0, 360)
 * @retval 电角度，单位：度（浮点数），范围 0.0 ~ 359.999...
 */
float DRIVER_MT6816_Get_ElectricalAngle_Degree(void)
{
    uint16_t raw = DRIVER_MT6816_Read_RawAngle();
    float mech_deg = DRIVER_MT6816_RawAngleToDegree(raw);  // 机械角度 0~359.978...
    
    // 计算电角度（未校准）
    float elec_deg = mech_deg * MT6816_MumberOfPolePairs;
    // 减去机械偏移（已换算至电角度空间）
    elec_deg -= MT6816_MECH_OFFSET * MT6816_MumberOfPolePairs;
    // 归一化至 [0, 360)
    elec_deg = fmodf(elec_deg, 360.0f);
    if (elec_deg < 0.0f) {
        elec_deg += 360.0f;
    }
    return elec_deg;
}

/**
 * @brief 获取MT6816的电角度（浮点度数），直接从原始计数计算
 * @note  电角度 = (原始计数 × 极对数 × 360 / 16384 - 偏移等效计数) % 360°
 *        偏移等效计数 = 机械偏移 × 16384 / 360，提前计算可优化
 * @retval 电角度，单位：度（浮点数），范围 0.0 ~ 359.999...
 */
float DRIVER_MT6816_Get_ElectricalAngle_Direct(void)
{
    uint16_t raw = DRIVER_MT6816_Read_RawAngle();
    
    // 将机械偏移转换为对应的原始计数偏移量（等效于在原始值上减去偏移）
    // 偏移量 = (MT6816_MECH_OFFSET / 360) * 16384
    const float OFFSET_COUNT = (MT6816_MECH_OFFSET * 16384.0f) / 360.0f;
    
    // 计算电角度：((raw - 偏移计数) * 极对数 * 360 / 16384) % 360
    float elec_deg = ((float)raw - OFFSET_COUNT) * MT6816_MumberOfPolePairs * 360.0f / 16384.0f;
    elec_deg = fmodf(elec_deg, 360.0f);
    if (elec_deg < 0.0f) {
        elec_deg += 360.0f;
    }
    return elec_deg;
}


