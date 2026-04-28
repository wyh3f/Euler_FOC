//BSP_UART
#include "BSP_UART.h"


extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;

#define Huart huart3
#define Hdma_usart_rx &hdma_usart3_rx
#define USARTx USART3

#define BUFF_SIZE	128
uint8_t rx_buff[BUFF_SIZE];

void BSP_UART_Init(void)
{
	 /* 需要在初始化时调用一次否则无法接收到内容 */
  HAL_UARTEx_ReceiveToIdle_DMA(&Huart, rx_buff, BUFF_SIZE);
  __HAL_DMA_DISABLE_IT(Hdma_usart_rx, DMA_IT_HT);		   // 手动关闭DMA_IT_HT中断	
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
    if(huart->Instance == USARTx)
    {
        if (Size <= BUFF_SIZE)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(&Huart, rx_buff, BUFF_SIZE); // 接收完毕后重启
            HAL_UART_Transmit(&Huart, rx_buff, Size, 0xffff);         // 将接收到的数据再发出
            __HAL_DMA_DISABLE_IT(Hdma_usart_rx, DMA_IT_HT);		   // 手动关闭DMA_IT_HT中断
            memset(rx_buff, 0, BUFF_SIZE);							   // 清除接收缓存
        }
        else  // 接收数据长度大于BUFF_SIZE，错误处理
        {
            
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
    if(huart->Instance == USARTx)
    {
			HAL_UARTEx_ReceiveToIdle_DMA(&Huart, rx_buff, BUFF_SIZE); // 接收发生错误后重启
			__HAL_DMA_DISABLE_IT(Hdma_usart_rx, DMA_IT_HT);		   // 手动关闭DMA_IT_HT中断
			memset(rx_buff, 0, BUFF_SIZE);							   // 清除接收缓存 
    }
}





// 增大缓冲区避免溢出 
#define FORMAT_BUF_SIZE 128  
void UART_Printf(char *format, ...) {
    static char formatBuf[FORMAT_BUF_SIZE] = {0};
    va_list args;
    va_start(args, format);
    // 使用 vsnprintf 限制写入长度 
    int len = vsnprintf(formatBuf, FORMAT_BUF_SIZE, format, args);
    va_end(args);
    if (len < 0 || len >= FORMAT_BUF_SIZE) {
        return;
    }
	HAL_UART_Transmit(&Huart, (uint8_t*)formatBuf, len, HAL_MAX_DELAY);    
}






/* 重写fputc函数*/
int fputc(int ch, FILE *f) 
{
	HAL_UART_Transmit(&Huart, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
  return ch;
}



  



// JustFloat协议帧尾
static const uint8_t JustFloat_Tail[4] = {0x00, 0x00, 0x80, 0x7f};

// 共用体：float <-> uint32_t
typedef union {
    float fdata;
    uint32_t udata;
} FloatData_t;

// 静态发送缓冲区（最大支持 256 个浮点数，可自行调整）
#define MAX_FLOAT_COUNT  256
static uint8_t dma_tx_buffer[MAX_FLOAT_COUNT * 4 + 4];  // 数据区 + 帧尾

/**
 * @brief 通过 DMA 发送 JustFloat 协议数据（非阻塞，若上次未发送完则丢弃本次）
 * @param data 浮点数组指针
 * @param num  浮点个数（不超过 MAX_FLOAT_COUNT）
 * @retval 0: 发送成功（或已启动DMA）; -1: 
 */
int Vofa_JustFloat_Send(float *data, uint16_t num) {
    // 上次 DMA 未完成则直接丢弃
    if (HAL_UART_GetState(&Huart) != HAL_UART_STATE_READY)
        return -1;

    uint32_t buf_size = num * 4 + 4;
    if (buf_size > sizeof(dma_tx_buffer))
        return -2;

    FloatData_t fd;
    for (uint16_t i = 0; i < num; i++) {
        fd.fdata = data[i];
        dma_tx_buffer[i*4]   = (uint8_t)fd.udata;
        dma_tx_buffer[i*4+1] = (uint8_t)(fd.udata >> 8);
        dma_tx_buffer[i*4+2] = (uint8_t)(fd.udata >> 16);
        dma_tx_buffer[i*4+3] = (uint8_t)(fd.udata >> 24);
    }
    memcpy(&dma_tx_buffer[num*4], JustFloat_Tail, 4);

    if (HAL_UART_Transmit_DMA(&Huart, dma_tx_buffer, buf_size) != HAL_OK)
        return -3;
    return 0;
}





