//ALGORITHM_Clarke_Park
#ifndef __ALGORITHM_Clarke_Park_H
#define __ALGORITHM_Clarke_Park_H

//,__CC_ARM,__TARGET_FPU_VFP, __FPU_PRESENT=1U, ARM_MATH_CM4
#include "arm_math.h"

#include "stdio.h"
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <math.h>
#include "stdbool.h"
#include <stdint.h>

typedef struct ThreePhase
{
	float A;
	float B;
	float C;
}ThreePhase;

typedef struct Clarke
{
	float Alpha;
	float Beta;
}Clarke;

typedef struct Park
{
	float Q;
	float D;
	float thet;
}Park;



/**
 * @brief 软件模拟三相正弦波（等幅值，三相对称）
 * @param abc         存放生成结果的结构体指针，依次为 A,B,C 相瞬时值
 * @param AngleStep   每调用一次函数增加的角度步长（度），决定输出频率
 * @param half_vdc    输出幅值系数（通常设为母线电压的一半，直接作为瞬时值）
 * @note  内部静态变量 angle 累计角度，超过 360° 后归零，保证角度正常范围
 *        使用 arm_sin_f32 计算正弦值，请确保已正确链接 CMSIS-DSP 库
 */
void ALGORITHM_Create_ThreePhase(ThreePhase *abc, float AngleStep, float half_vdc);


/**
 * @brief Clarke 变换（等幅值变换）: 三相静止 abc 坐标系 → 两相静止 αβ 坐标系
 * @param abc        输入三相值结构体（包含 A、B、C 三相采样值）
 * @param alpha_beta 输出 αβ 值结构体
 * @note  公式：Alpha = A
 *             Beta = (B - C) / √3
 *        该形式与原始公式 (A + 2B)/√3 在 A+B+C=0 时完全等效。
 *        优势：显式使用 C 相，可抵消 B、C 两相采样通道的共模直流偏置，
 *              有效抑制 Id/Iq 中的 1 次谐波（基频正弦波动）。
 *        缩放：等幅值变换，输出幅值与三相相电流峰值一致。
 */
void ALGORITHM_Clarke_(ThreePhase *abc, Clarke *alpha_beta);


/**
 * @brief Park 变换：αβ → dq
 * @param alpha_beta  输入 α, β 分量
 * @param theta_rad   输入电角度（弧度），来自转子位置/观测器
 * @param dq          输出 d, q 分量
 */
void ALGORITHM_Park_Transform(Clarke *alpha_beta, float theta, Park *dq);


/**
 * @brief 逆 Park 变换: 两相旋转 dq 坐标系 → 两相静止 αβ 坐标系
 * @param alpha_beta 输出 αβ 值结构体
 * @param qd_thet    输入 dq 值及电角度结构体，成员 thet 为电角度（度）
 * @note  变换公式：α = D·cosθ - Q·sinθ
 *                β = D·sinθ + Q·cosθ
 *        输入角度单位为度，内部转为弧度后再计算正余弦
 */
void ALGORITHM_Inverse_Park(Clarke *alpha_beta, Park *qd_thet);

/**
 * @brief 逆 Clarke 变换（等幅值）: 两相静止 αβ 坐标系 → 三相静止 abc 坐标系
 * @param abc        输出三相值结构体
 * @param alpha_beta 输入 αβ 值结构体
 * @note  公式：A = α
 *            B = ( -α + √3·β ) / 2
 *            C = ( -α - √3·β ) / 2
 *        假设三相平衡可由此重建
 */
void ALGORITHM_Inverse_Clarke(ThreePhase *abc, Clarke *alpha_beta);


/**
 * @brief SPWM 调制：由 αβ 参考电压生成三相 PWM 比较值（中心对齐，单极性调制）
 * @param pwm        输出三相 PWM 比较值结构体指针（成员 A/B/C 为比较计数值）
 * @param abc        经过克拉克变化后的三相电流，用于调试
 * @param alpha_beta 输入 αβ 参考电压结构体指针（通常来自逆 Park 变换的输出）
 * @param Udc        直流母线电压（单位：伏特），用于将 αβ 电压归一化到 [0,1] 区间
 * @param pwm_mum    PWM 定时器周期计数最大值（即自动重装载值），决定输出比较值的量程
 * @param pwm_max    PWM 比较值上限（用于限制输出，通常设为 pwm_mum）
 * @param pwm_min    PWM 比较值下限（通常设为 0）
 * @note  实现步骤：
 *        1. 调用逆 Clarke 变换将 αβ 转换为三相电压 abc（等幅值变换）；
 *        2. 将每相电压除以 Udc 并偏移 0.5，得到标幺化调制波（范围 0~1）；
 *        3. 乘以 pwm_mum 得到原始比较值；
 *        4. 通过 pwm_max/pwm_min 进行硬限幅，防止过调制或定时器溢出。
 *        该函数适用于常规 SPWM（正弦脉宽调制），输出为高有效电平（比较值越大，占空比越大）。
 *        若需低有效输出，调用者可在外部取反。
 *        当调制波幅值超过 Udc/2 时，输出将进入过调制区并限幅，可能产生谐波。
 */
void ALGORITHM_SPWM(ThreePhase *pwm,ThreePhase *abc, Clarke *alpha_beta,float Udc,float pwm_mum,float pwm_max,float pwm_min);



/**
 * @brief 标准七段式 SVPWM 算法（中心对齐，带输出幅值限制）
 * @param pwm        输出三相 PWM 比较值（钳位在 pwm_min ~ pwm_max 之间）
 * @param alpha_beta 输入 α-β 参考电压（等幅值 Clarke 变换）
 * @param Udc        直流母线电压（单位：伏特），必须大于 0
 * @param pwm_mum    PWM 周期计数值（定时器自动重装载值 ARR，即 Tpwm）
 * @param pwm_max    输出比较值的上限（PWM 最大占空比对应计数值）
 * @param pwm_min    输出比较值的下限（PWM 最小占空比对应计数值）
 * @note  采用等幅值 Clarke 变换，扇区由 Vref1=Vβ, Vref2=(√3Vα-Vβ)/2, Vref3=(-√3Vα-Vβ)/2 的符号决定。
 *        计算相邻基本矢量作用时间 t1、t2，零矢量时间 t0 = Tpwm - t1 - t2。
 *        过调制时按比例缩小 t1、t2 以保持电压矢量方向不变。
 *        Udc ≤ 0 时输出全下限（封锁 PWM）；参考电压为零时输出 50% 占空比（再钳位）。
 *        所有输出最终通过 pwm_min / pwm_max 进行限幅，确保安全。
 */
void ALGORITHM_SVPWM_(ThreePhase *pwm,Clarke *alpha_beta,float Udc, float pwm_mum, float pwm_max, float pwm_min);





/**
 * @brief SVPWM 调制（空间矢量脉宽调制）：由 αβ 参考电压生成三相 PWM 比较值（中心对齐，七段式）
 * @param pwm        输出三相 PWM 比较值结构体指针（成员 A/B/C 为比较计数值）
 * @param alpha_beta 输入 αβ 参考电压结构体指针（通常来自逆 Park 变换的输出）
 * @param Udc        直流母线电压（单位：伏特），用于计算基本矢量作用时间
 * @param Tpwm       PWM 周期计数最大值（即定时器自动重装载值），决定 PWM 载波周期
 * @param pwm_max    输出比较值的上限（PWM 最大占空比对应计数值）
 * @param pwm_min    输出比较值的下限（PWM 最小占空比对应计数值）
 * @note  实现采用等幅值 Clarke 变换，SVPWM 算法基于参考电压矢量所在扇区，
 *        计算相邻基本矢量的作用时间 t1、t2 以及零矢量作用时间 t0。
 *        采用七段式对称 PWM 波形，输出比较值按中心对齐方式分配。
 *        当 t1+t2 > Tpwm 时，自动进行过调制处理（等比例缩小 t1、t2，保持 t0=0），
 *        以维持输出电压矢量方向不变。
 *        函数内部包含扇区判断、矢量作用时间计算及三相比较值映射，
 *        输出结果可直接用于定时器通道的比较寄存器。
 *        注意：Udc 必须大于 0，否则函数直接返回并将 pwm 各相置 0。
 *        过调制处理时，t2 由 Tpwm - t1 计算得出，确保 t1+t2 精确等于 Tpwm。
 *        若参考电压矢量幅值为零，扇区判定结果为 0，此时所有比较值输出为 0（即 0% 占空比）。
 */
void ALGORITHM_SVPWM(ThreePhase *pwm, Clarke *alpha_beta,float Udc,float Tpwm ,float pwm_max, float pwm_min);

#endif


