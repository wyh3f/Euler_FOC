//BSP_HALL
#ifndef BSP_HALL_H_
#define BSP_HALL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>




#define BSP_HALL_U() (uint8_t)gpio_input_data_bit_read(HALL_A_GPIO_PORT,HALL_A_PIN)
#define BSP_HALL_V() (uint8_t)gpio_input_data_bit_read(HALL_B_GPIO_PORT,HALL_B_PIN)
#define BSP_HALL_W() (uint8_t)gpio_input_data_bit_read(HALL_C_GPIO_PORT,HALL_C_PIN)

uint8_t BSP_HALL_Get_Value(void);
uint8_t BSP_HALL_Get_Value_Filtered(void);

extern uint8_t HALL_Detection_Veer_Flog;
extern float HALL_Value_Reality_Now;
extern uint32_t HALL_Interval_Time;
extern float HALL_AngularVelocityBuf;
extern float HALL_count;

void BSP_HALL_USER_IRQHandler(void);

float BSP_HALL_Speed(void);

/**
 * @brief 对BSP_HALL_Speed()进行一阶低通滤波
 * @param alpha 滤波系数 (0 < alpha <= 1)，越大响应越快
 * @return 滤波后的速度值
 */
float BSP_HALL_Speed_LPF(float alpha);


float BSP_HALL_Speed_Median(void);


#ifdef __cplusplus
}
#endif

#endif







