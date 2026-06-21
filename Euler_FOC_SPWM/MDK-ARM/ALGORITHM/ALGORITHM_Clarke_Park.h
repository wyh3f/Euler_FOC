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
 * @param abc        输入三相值结构体
 * @param alpha_beta 输出 αβ 值结构体
 * @note  公式：Alpha = A
 *               Beta  = (A + 2B) / √3
 *        默认假设 A+B+C = 0（三相对称），未使用 C 相
 *        若系统不平衡，建议使用完整变换公式
 */
void ALGORITHM_Clarke(ThreePhase *abc, Clarke *alpha_beta);

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

///**
// * @brief Park 变换: 两相静止 αβ 坐标系 → 两相旋转 dq 坐标系（磁场定向）
// * @param alpha_beta 输入 αβ 值结构体
// * @param qd_thet    输出 dq 值及电角度结构体，成员 thet 存储电角度（度），Q/D 为旋转坐标系分量
// * @note  角度计算：θ = atan2(β, α)，然后通过正余弦进行旋转变换
// *        变换公式：D =  α·cosθ + β·sinθ
// *                Q = -α·sinθ + β·cosθ
// *        为处理原点（α=β=0）时 atan2 不定，将 θ 设为 0
// *        最终将弧度 θ 转换为度存入 qd_thet->thet
// */
//void ALGORITHM_Park(Clarke *alpha_beta, Park *qd_thet);

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






#endif


