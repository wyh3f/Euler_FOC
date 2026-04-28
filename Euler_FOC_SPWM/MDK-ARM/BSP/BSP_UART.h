//BSP_UART
#ifndef __BSP_UART_H
#define __BSP_UART_H
#include "main.h" 
#include "stdio.h"
void BSP_UART_Init(void);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size);
void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart);

void UART_Printf(char *p,...);


/* 3. 重写fputc函数 (与方式A相同) */
int fputc(int ch, FILE *f) ; 


// 函数功能：发送JustFloat协议数据
// 参数：huart -> 串口句柄，data -> 要发送的浮点数组，num -> 浮点数据个数
int Vofa_JustFloat_Send(float *data, uint16_t num);
#endif

