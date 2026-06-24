//BSP_HALL
#include "BSP_HALL.h"
#include "BSP_DWT.h"
#include "wk_adc.h"
#include "wk_op.h"
#include "wk_tmr.h"
#include "wk_usart.h"
#include "wk_dma.h"
#include "wk_gpio.h"
#include "wk_system.h"

/* 计数边界宏定义：计数器取值范围 [0, COUNT_BOUND-1] */
#define COUNT_BOUND    40000U   /* 可修改为任意不超过 2^32 的正整数 */

static uint32_t counter = 0;   /* 当前计数值 */

/**
 * @brief 获取当前计数值，并将计数器递增（循环）
 * @return 递增前的计数值
 */
uint32_t get_count(void)
{
    uint32_t current = counter;
    counter = (counter + 1) % COUNT_BOUND;   /* 到达边界后回绕到 0 */
    return current;
}

/**
 * @brief 计算两次计数值在模 COUNT_BOUND 意义下的差值 (second - first)
 * @param first  第一次获取的计数值
 * @param second 第二次获取的计数值
 * @return 正差值，范围 [0, COUNT_BOUND-1]
 */
uint32_t compute_diff(uint32_t first, uint32_t second)
{
    /* 处理循环边界：若 second >= first 则直接减法，否则表示已回绕 */
    if (second >= first) {
        return second - first;
    } else {
        return second + COUNT_BOUND - first;
    }
}


/* 静态变量：记录上一次调用时获取的计数值 */
static uint32_t last_value = 0;
static uint8_t first_call = 1;   /* 首次调用标志 */

/**
 * @brief 获取从上一次调用到本次调用之间的计数值差值
 * @return 差值，范围 [0, COUNT_BOUND-1]；首次调用返回 0
 */
uint32_t get_diff_since_last(void)
{
    uint32_t current = get_count();   /* 获取当前计数值（同时计数器自增） */
    
    if (first_call) {
        first_call = 0;
        last_value = current;
        return 0;                     /* 首次调用无差值 */
    }
    
    uint32_t diff = compute_diff(last_value, current);
    last_value = current;
    return diff;
}






uint8_t BSP_HALL_Get_Value(void)
{
	return ((BSP_HALL_U()<<2)|(BSP_HALL_V()<<1)|(BSP_HALL_W()));
}

uint8_t BSP_HALL_Get_Value_Filtered(void)
{
    static uint8_t last_valid = 0;
    uint8_t state = BSP_HALL_Get_Value();
    if (state >= 1 && state <= 6) {
        last_valid = state;
        return state;
    } else {
        return last_valid; // 保持上一次有效状态
    }
}


//uint8_t HALL_Value_buff[]={6,4,5,1,3,2};
//float HALL_Value_Reality_buff[]={15.42f,82.11f,135.42f,198.82f,258.86f,322.13f};
//float HALL_Value_Reality_Err_buff[]={53.29f,63.4f,66.69f,53.31f,63.4f,63.27f};


uint8_t HALL_Value_buff[]={3,2,6,4,5,1};
float HALL_Value_Reality_buff[]={2.47f,63.67f,122.47f,182.47f,245.87f,299.18f};
float HALL_Value_Reality_Err_buff[]={63.59f,61.4f,59.29f,60.71f,63.0f,54.01f};


uint8_t HALL_V_Now=0,HALL_V_Last=0;
uint8_t HALL_Detection_Veer_Flog=0;
float HALL_Value_Reality_Now=0;
uint32_t HALL_Interval_Time = 0;
float HALL_AngularVelocityBuf=0;

float HALL_count=0;
float HALL_count_speed=0;
float HALL_speed=0;

uint32_t time_buff=0;

void BSP_HALL_huidiao(void);

void BSP_HALL_USER_IRQHandler(void)
{
	HALL_V_Last=HALL_V_Now;
	HALL_V_Now=BSP_HALL_Get_Value_Filtered();
	get_count();
	
	
	if(HALL_V_Now==HALL_Value_buff[0])
	{

		if(HALL_V_Last==HALL_Value_buff[1])
		{
			HALL_count_speed=get_interval_sec();
			time_buff=interval_start();
			BSP_HALL_huidiao();
			HALL_AngularVelocityBuf=0.0f;
			HALL_Detection_Veer_Flog=1;
			HALL_Value_Reality_Now=360.0f;
			HALL_Interval_Time=get_diff_since_last();
			HALL_AngularVelocityBuf=HALL_Value_Reality_Err_buff[0]/(float)HALL_Interval_Time;
			HALL_AngularVelocityBuf*=-1.0f;
		}
		else if(HALL_V_Last==HALL_Value_buff[5])
		{
			BSP_HALL_huidiao();
			HALL_count_speed=get_interval_sec();
			time_buff=interval_start();
			HALL_AngularVelocityBuf=0.0f;
			HALL_Detection_Veer_Flog=2;
			HALL_Value_Reality_Now=0.0f;
			HALL_Interval_Time=get_diff_since_last();
			HALL_AngularVelocityBuf=HALL_Value_Reality_Err_buff[0]/(float)HALL_Interval_Time;
		}
		HALL_Value_Reality_Now+=HALL_AngularVelocityBuf;
		if(HALL_AngularVelocityBuf>0.0f&&HALL_Value_Reality_Now>HALL_Value_Reality_buff[1])HALL_Value_Reality_Now=HALL_Value_Reality_buff[1];
		if(HALL_AngularVelocityBuf<0.0f&&HALL_Value_Reality_Now<HALL_Value_Reality_buff[5])HALL_Value_Reality_Now=HALL_Value_Reality_buff[5];
	}
	
	if(HALL_V_Now==HALL_Value_buff[1])
	{
		
		if(HALL_V_Last==HALL_Value_buff[2])
		{
BSP_HALL_huidiao();
			HALL_AngularVelocityBuf=0.0f;
			HALL_Detection_Veer_Flog=1;
			HALL_Value_Reality_Now=HALL_Value_Reality_buff[1];
			HALL_Interval_Time=get_diff_since_last();
			HALL_AngularVelocityBuf=HALL_Value_Reality_Err_buff[1]/(float)HALL_Interval_Time;
			HALL_AngularVelocityBuf*=-1.0f;
		}
		else if(HALL_V_Last==HALL_Value_buff[0])
		{
BSP_HALL_huidiao();
			HALL_AngularVelocityBuf=0.0f;
			HALL_Detection_Veer_Flog=2;
			HALL_Value_Reality_Now=HALL_Value_Reality_buff[1];
			HALL_Interval_Time=get_diff_since_last();
			HALL_AngularVelocityBuf=HALL_Value_Reality_Err_buff[1]/(float)HALL_Interval_Time;
		}
		HALL_Value_Reality_Now+=HALL_AngularVelocityBuf;
		if(HALL_AngularVelocityBuf>0.0f&&HALL_Value_Reality_Now>HALL_Value_Reality_buff[2])HALL_Value_Reality_Now=HALL_Value_Reality_buff[2];
		if(HALL_AngularVelocityBuf<0.0f&&HALL_Value_Reality_Now<HALL_Value_Reality_buff[0])HALL_Value_Reality_Now=HALL_Value_Reality_buff[0];
	}
	
	if(HALL_V_Now==HALL_Value_buff[2])
	{
		if(HALL_V_Last==HALL_Value_buff[3])
		{
BSP_HALL_huidiao();
			HALL_AngularVelocityBuf=0.0f;
			HALL_Detection_Veer_Flog=1;
			HALL_Value_Reality_Now=HALL_Value_Reality_buff[2];
			HALL_Interval_Time=get_diff_since_last();
			HALL_AngularVelocityBuf=HALL_Value_Reality_Err_buff[2]/(float)HALL_Interval_Time;
			HALL_AngularVelocityBuf*=-1.0f;
		}
		else if(HALL_V_Last==HALL_Value_buff[1])
		{
BSP_HALL_huidiao();
			HALL_AngularVelocityBuf=0.0f;
			HALL_Detection_Veer_Flog=2;
			HALL_Value_Reality_Now=HALL_Value_Reality_buff[2];
			HALL_Interval_Time=get_diff_since_last();
			HALL_AngularVelocityBuf=HALL_Value_Reality_Err_buff[2]/(float)HALL_Interval_Time;
		}
		HALL_Value_Reality_Now+=HALL_AngularVelocityBuf;
		if(HALL_AngularVelocityBuf>0.0f&&HALL_Value_Reality_Now>HALL_Value_Reality_buff[3])HALL_Value_Reality_Now=HALL_Value_Reality_buff[3];
		if(HALL_AngularVelocityBuf<0.0f&&HALL_Value_Reality_Now<HALL_Value_Reality_buff[1])HALL_Value_Reality_Now=HALL_Value_Reality_buff[1];
	}
	
	if(HALL_V_Now==HALL_Value_buff[3])
	{

		if(HALL_V_Last==HALL_Value_buff[4])
		{
BSP_HALL_huidiao();
			HALL_AngularVelocityBuf=0.0f;
			HALL_Detection_Veer_Flog=1;
			HALL_Value_Reality_Now=HALL_Value_Reality_buff[3];
			HALL_Interval_Time=get_diff_since_last();
			HALL_AngularVelocityBuf=HALL_Value_Reality_Err_buff[3]/(float)HALL_Interval_Time;
			HALL_AngularVelocityBuf*=-1.0f;
		}
		else if(HALL_V_Last==HALL_Value_buff[2])
		{
BSP_HALL_huidiao();
			HALL_AngularVelocityBuf=0.0f;
			HALL_Detection_Veer_Flog=2;
			HALL_Value_Reality_Now=HALL_Value_Reality_buff[3];
			HALL_Interval_Time=get_diff_since_last();
			HALL_AngularVelocityBuf=HALL_Value_Reality_Err_buff[3]/(float)HALL_Interval_Time;
		}
		HALL_Value_Reality_Now+=HALL_AngularVelocityBuf;
		if(HALL_AngularVelocityBuf>0.0f&&HALL_Value_Reality_Now>HALL_Value_Reality_buff[4])HALL_Value_Reality_Now=HALL_Value_Reality_buff[4];
		if(HALL_AngularVelocityBuf<0.0f&&HALL_Value_Reality_Now<HALL_Value_Reality_buff[2])HALL_Value_Reality_Now=HALL_Value_Reality_buff[2];
	}
	
	if(HALL_V_Now==HALL_Value_buff[4])
	{
		
		if(HALL_V_Last==HALL_Value_buff[5])
		{
BSP_HALL_huidiao();
			HALL_AngularVelocityBuf=0.0f;
			HALL_Detection_Veer_Flog=1;
			HALL_Value_Reality_Now=HALL_Value_Reality_buff[4];
			HALL_Interval_Time=get_diff_since_last();
			HALL_AngularVelocityBuf=HALL_Value_Reality_Err_buff[4]/(float)HALL_Interval_Time;
			HALL_AngularVelocityBuf*=-1.0f;
		}
		else if(HALL_V_Last==HALL_Value_buff[3])
		{
BSP_HALL_huidiao();
			HALL_AngularVelocityBuf=0.0f;
			HALL_Detection_Veer_Flog=2;
			HALL_Value_Reality_Now=HALL_Value_Reality_buff[4];
			HALL_Interval_Time=get_diff_since_last();
			HALL_AngularVelocityBuf=HALL_Value_Reality_Err_buff[4]/(float)HALL_Interval_Time;
		}
		HALL_Value_Reality_Now+=HALL_AngularVelocityBuf;
		if(HALL_AngularVelocityBuf>0.0f&&HALL_Value_Reality_Now>HALL_Value_Reality_buff[5])HALL_Value_Reality_Now=HALL_Value_Reality_buff[5];
		if(HALL_AngularVelocityBuf<0.0f&&HALL_Value_Reality_Now<HALL_Value_Reality_buff[3])HALL_Value_Reality_Now=HALL_Value_Reality_buff[3];
	}
	
	if(HALL_V_Now==HALL_Value_buff[5])
	{
		
		if(HALL_V_Last==HALL_Value_buff[0])
		{
BSP_HALL_huidiao();
			HALL_AngularVelocityBuf=0.0f;
			HALL_Detection_Veer_Flog=1;
			HALL_Value_Reality_Now=HALL_Value_Reality_buff[5];
			HALL_Interval_Time=get_diff_since_last();
			HALL_AngularVelocityBuf=HALL_Value_Reality_Err_buff[5]/(float)HALL_Interval_Time;
			HALL_AngularVelocityBuf*=-1.0f;
		}
		else if(HALL_V_Last==HALL_Value_buff[4])
		{
BSP_HALL_huidiao();
			HALL_AngularVelocityBuf=0.0f;
			HALL_Detection_Veer_Flog=2;
			HALL_Value_Reality_Now=HALL_Value_Reality_buff[5];
			HALL_Interval_Time=get_diff_since_last();
			HALL_AngularVelocityBuf=HALL_Value_Reality_Err_buff[5]/(float)HALL_Interval_Time;
		}
		HALL_Value_Reality_Now+=HALL_AngularVelocityBuf;
		if(HALL_AngularVelocityBuf>0.0f&&HALL_Value_Reality_Now>360.0f)HALL_Value_Reality_Now=360.0f;
		if(HALL_AngularVelocityBuf<0.0f&&HALL_Value_Reality_Now<HALL_Value_Reality_buff[4])HALL_Value_Reality_Now=HALL_Value_Reality_buff[4];
	}

}


float BSP_HALL_Speed(void)
{
	
	float buf = interval_elapsed_sec(time_buff);
	if(buf <1.0f&&HALL_count_speed!=0.0f)
	{
		if(HALL_Detection_Veer_Flog==2)
		{
			return 1.0f/((float)HALL_count_speed*4.0f);
		}
		else
		{
			return -(1.0f/((float)HALL_count_speed*4.0f));
		}
	}
	HALL_count_speed = 0.0f;
	return 0.0f;
}

#include "arm_math.h"
#include <stddef.h>   // 如需使用NULL等

/* ======================== 一阶低通滤波 ======================== */
/**
 * @brief 对BSP_HALL_Speed()进行一阶低通滤波
 * @param alpha 滤波系数 (0 < alpha <= 1)，越大响应越快
 * @return 滤波后的速度值
 */
float BSP_HALL_Speed_LPF(float alpha)
{
    static float prev_out = 0.0f;
    static uint8_t init_flag = 0;

    float raw = BSP_HALL_Speed();          // 原始采样值
    float out;

    if (!init_flag) {
        prev_out = raw;                    // 首次直接赋值
        init_flag = 1;
        return raw;
    }

    // 使用arm_math加速单精度浮点运算：out = alpha * raw + (1-alpha) * prev_out
    float alpha_raw, beta_prev;
    float beta = 1.0f - alpha;

    // 等效于 arm_mult_f32(&alpha, &raw, &alpha_raw, 1);
    arm_mult_f32(&alpha, &raw, &alpha_raw, 1);
    arm_mult_f32(&beta, &prev_out, &beta_prev, 1);
    arm_add_f32(&alpha_raw, &beta_prev, &out, 1);

    prev_out = out;
    return out;
}

/* ======================== 窗口中值滤波 ======================== */
#define MEDIAN_WINDOW_SIZE  5   // 必须为奇数
static float med_buf[MEDIAN_WINDOW_SIZE];
static uint32_t med_idx = 0;
static uint32_t med_count = 0;

/* 手动插入排序，返回中值（适用于小窗口） */
static float manual_median(float *arr, uint32_t n)
{
    uint32_t i, j;
    float temp;
    for (i = 1; i < n; i++) {
        temp = arr[i];
        j = i;
        while (j > 0 && arr[j-1] > temp) {
            arr[j] = arr[j-1];
            j--;
        }
        arr[j] = temp;
    }
    return arr[n / 2];   // n为奇数，索引为 n/2 即为中值
}

float BSP_HALL_Speed_Median(void)
{
    float raw = BSP_HALL_Speed();

    // 存入循环缓冲区
    med_buf[med_idx] = raw;
    med_idx = (med_idx + 1) % MEDIAN_WINDOW_SIZE;
    if (med_count < MEDIAN_WINDOW_SIZE) {
        med_count++;
    }

    // 窗口未填满时返回原始值（或可改为平均值）
    if (med_count < MEDIAN_WINDOW_SIZE) {
        return raw;
    }

    // 复制当前窗口到临时数组（利用 arm_copy_f32 加速）
    float tmp[MEDIAN_WINDOW_SIZE];
    arm_copy_f32(med_buf, tmp, MEDIAN_WINDOW_SIZE);

    // 手动排序并返回中值
    return manual_median(tmp, MEDIAN_WINDOW_SIZE);
}


#include "BSP_TIM.h"
#include "BSP_UART.h"
#include "BSP_ADC.h"
#include "BSP_HALL.h"
#include "BSP_DWT.h"
#include "ALGORITHM_Clarke_Park.h"
#include "DRIVER_ThreePhase_Motor.h"
#include "dma_uart.h"
#include "PID.h"
#include "algorithms_filter.h"
extern PIDController pid;
extern PIDController pid_q;
extern PIDController pid_speed;
extern IncrementalPID I_pid_speed;
extern float Q;



uint32_t buf_hall_time=0;
uint32_t hall_time=0;
void BSP_HALL_huidiao(void)
{
	hall_time=interval_elapsed_us(buf_hall_time);
	buf_hall_time=interval_start();
}


