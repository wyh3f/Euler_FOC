//DRIVER_ThreePhase_Motor
#ifndef __DRIVER_ThreePhase_Motor_H
#define __DRIVER_ThreePhase_Motor_H

#include "ALGORITHM_Clarke_Park.h"
#include "DRIVER_MT6816.h" 

#include "stdio.h"
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <math.h>
#include "stdbool.h"
#include <stdint.h>

// 直流母线电压 24V
#define V_DC 24.0f          
// 最大相电压幅值 12V
#define V_MAX (V_DC / 2.0f) 

// 最大的PWM值
#define V_PWM_MAX  4195		
// 最小的PWM值
#define V_PWM_MIN  ((uint16_t)(V_PWM_MAX * 2 / 100))

// 限幅宏定义
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

// 电角度偏移量
#define MECH_OFFSET 75.83f

// 电机极对数
#define MumberOfPolePairs 4U

// 获得机械角度
#define MechanicalAngle DRIVER_MT6816_RawAngleToDegree(DRIVER_MT6816_Read_RawAngle())








//Park限幅
void DRIVER_Park_Limit(Park *qd_thet);

//Theta电角度更新方法1
//这个是根据当前的机械角度更新电角度的方法
void DRIVER_UpdateTheta_1(Park *qd_thet);

//用户自己跟新虚拟电角度
void DRIVER_UpdateTheta_User(Park *qd_thet,float add_Thet);


//更新PWM的CCR值
void DRIVER_UpdateThreePhase_PWM_1(ThreePhase *abc,ThreePhase *pwm);









#endif


