//ALGORITHM_Clarke_Park
//,__CC_ARM,__TARGET_FPU_VFP, __FPU_PRESENT=1U, ARM_MATH_CM4
#include "ALGORITHM_Clarke_Park.h"

/*
*sin:arm_sin_f32,arm_sin_q31, arm_sin_q15
*cos:arm_cos_f32,arm_cos_q31, arm_cos_q15
*
*角度转弧度：DEG2RAD
*弧度转角度：RAD2DEG
*/
/* ********** 常用宏定义 ********** */
// 角度转弧度：输入角度值（度），输出弧度值
#define DEG2RAD(angleDegrees) ((angleDegrees) * 3.141592f / 180.0f)
// 弧度转角度：输入弧度值，输出角度值（度）
#define RAD2DEG(angleRadians) ((angleRadians) * 180.0f / 3.141592f)

// Clarke 变换中用到的常数：1/√3 ≈ 0.57735（等幅值变换）
#define square_root_3_divide_3 0.57735f
// √3 ≈ 1.73205，用于逆 Clarke 变换
#define square_root_3 1.73205f 

/**
 * @brief 软件模拟三相正弦波（等幅值，三相对称）
 * @param abc         存放生成结果的结构体指针，依次为 A,B,C 相瞬时值
 * @param AngleStep   每调用一次函数增加的角度步长（度），决定输出频率
 * @param half_vdc    输出幅值系数（通常设为母线电压的一半，直接作为瞬时值）
 * @note  内部静态变量 angle 累计角度，超过 360° 后归零，保证角度正常范围
 *        使用 arm_sin_f32 计算正弦值，请确保已正确链接 CMSIS-DSP 库
 */
void ALGORITHM_Create_ThreePhase(ThreePhase *abc, float AngleStep, float half_vdc)
{
    static float angle = 0.0f;          // 静态累积角度（度）
    angle += AngleStep;                 // 步进
    if (angle >= 360.0f) angle -= 360.0f; // 限制在 [0,360)

    // 三相分别超前 0°, 120°, 240°，乘以 half_vdc 得到最终瞬时值
    abc->A = half_vdc * arm_sin_f32(DEG2RAD(angle));
    abc->B = half_vdc * arm_sin_f32(DEG2RAD(angle + 120.0f));
    abc->C = half_vdc * arm_sin_f32(DEG2RAD(angle + 240.0f));
}

/**
 * @brief Clarke 变换（等幅值变换）: 三相静止 abc 坐标系 → 两相静止 αβ 坐标系
 * @param abc        输入三相值结构体
 * @param alpha_beta 输出 αβ 值结构体
 * @note  公式：Alpha = A
 *               Beta  = (A + 2B) / √3
 *        默认假设 A+B+C = 0（三相对称），未使用 C 相
 *        若系统不平衡，建议使用完整变换公式
 */
void ALGORITHM_Clarke(ThreePhase *abc, Clarke *alpha_beta)
{
    alpha_beta->Alpha = abc->A;   // α 直接等于 A 相
    // β = (A + 2B) / √3，其中 1/√3 已预定义为 square_root_3_divide_3
    alpha_beta->Beta = square_root_3_divide_3 * (2 * abc->B + abc->A);
}

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
void ALGORITHM_Clarke_(ThreePhase *abc, Clarke *alpha_beta)
{
    alpha_beta->Alpha = abc->A;   // α 直接等于 A 相
    // β = (B - C) / √3，使用预定义的 square_root_3_divide_3 (即 1/√3)
    alpha_beta->Beta = (abc->B - abc->C) * square_root_3_divide_3;
}

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
//void ALGORITHM_Park(Clarke *alpha_beta, Park *qd_thet)
//{
//    float thet_rad = 0.0f;   // 电角度（弧度）
//    float sin_thet = 0.0f, cos_thet = 0.0f;

//    // 避免 atan2f(0,0) 未定义
//    if (alpha_beta->Alpha == 0.0f && alpha_beta->Beta == 0.0f) {
//        thet_rad = 0.0f;
//    } else {
//        thet_rad = atan2f(alpha_beta->Beta, alpha_beta->Alpha);   // 返回弧度
//    }
//    // 使用 ARM DSP 函数计算正余弦（可替换为 sinf/cosf）
//    sin_thet = arm_sin_f32(thet_rad);
//    cos_thet = arm_cos_f32(thet_rad);

//    // 保存电角度（度），方便调试或后续使用
//    qd_thet->thet = RAD2DEG(thet_rad);
//    // Park 变换
//    qd_thet->Q = -alpha_beta->Alpha * sin_thet + alpha_beta->Beta * cos_thet;
//    qd_thet->D =  alpha_beta->Alpha * cos_thet + alpha_beta->Beta * sin_thet;
//}

/**
 * @brief Park 变换：αβ → dq
 * @param alpha_beta  输入 α, β 分量
 * @param theta_rad   输入电角度，来自转子位置/观测器
 * @param dq          输出 d, q 分量
 */
void ALGORITHM_Park_Transform(Clarke *alpha_beta, float theta, Park *dq)
{
    float cos_theta = arm_cos_f32(DEG2RAD(theta));  // 或 cosf
    float sin_theta = arm_sin_f32(DEG2RAD(theta));

    dq->D = alpha_beta->Alpha * cos_theta + alpha_beta->Beta * sin_theta;
    dq->Q = -alpha_beta->Alpha * sin_theta + alpha_beta->Beta * cos_theta;
		dq->thet=theta;
}

/**
 * @brief 逆 Park 变换: 两相旋转 dq 坐标系 → 两相静止 αβ 坐标系
 * @param alpha_beta 输出 αβ 值结构体
 * @param qd_thet    输入 dq 值及电角度结构体，成员 thet 为电角度（度）
 * @note  变换公式：α = D·cosθ - Q·sinθ
 *                β = D·sinθ + Q·cosθ
 *        输入角度单位为度，内部转为弧度后再计算正余弦
 */
void ALGORITHM_Inverse_Park(Clarke *alpha_beta, Park *qd_thet)
{
    float thet_rad = DEG2RAD(qd_thet->thet);   // 度 → 弧度
    float sin_thet = arm_sin_f32(thet_rad);
    float cos_thet = arm_cos_f32(thet_rad);

    // 逆 Park 旋转
    alpha_beta->Alpha = cos_thet * qd_thet->D - sin_thet * qd_thet->Q;
    alpha_beta->Beta  = sin_thet * qd_thet->D + cos_thet * qd_thet->Q;
}

/**
 * @brief 逆 Clarke 变换（等幅值）: 两相静止 αβ 坐标系 → 三相静止 abc 坐标系
 * @param abc        输出三相值结构体
 * @param alpha_beta 输入 αβ 值结构体
 * @note  公式：A = α
 *            B = ( -α + √3·β ) / 2
 *            C = ( -α - √3·β ) / 2
 *        假设三相平衡可由此重建
 */
void ALGORITHM_Inverse_Clarke(ThreePhase *abc, Clarke *alpha_beta)
{
    abc->A = alpha_beta->Alpha;
    // 计算 B 相: (-Alpha + √3·Beta)/2
    abc->B = (square_root_3 * alpha_beta->Beta - alpha_beta->Alpha) * 0.5f;
    // 计算 C 相: (-Alpha - √3·Beta)/2
    abc->C = (-square_root_3 * alpha_beta->Beta - alpha_beta->Alpha) * 0.5f;
}


















