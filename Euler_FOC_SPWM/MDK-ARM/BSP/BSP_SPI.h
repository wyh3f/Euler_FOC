#ifndef __BSP_SPI_H
#define __BSP_SPI_H
#include "stdio.h"
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <math.h>
#include "stdbool.h"
#include <stdint.h>

/**
 * @brief       SPI1 同时发送和接收数据（阻塞模式）
 * @note        该函数基于 HAL_SPI_TransmitReceive 实现，超时时间固定为 100ms。
 *              若传输过程中出现超时、硬件错误或 SPI 总线忙，则返回错误。
 * @param[in]   tx   发送数据缓冲区指针（必须已分配，不能为 NULL）
 * @param[out]  rx   接收数据缓冲区指针（必须已分配，不能为 NULL）
 * @param[in]   size 要发送/接收的数据字节数（必须大于 0）
 * @retval      0    传输成功
 * @retval      1    传输失败（参数无效、超时、硬件错误或 SPI 忙）
 */
int BSP_SPI1_TransmitReceive(uint8_t *tx, uint8_t *rx, uint16_t size);

#endif






