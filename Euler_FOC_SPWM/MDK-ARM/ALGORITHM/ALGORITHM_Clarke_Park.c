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
// √3 /2 ≈ 0.86602
#define square_root_3_divide_2 0.86602f


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

// 限幅宏定义
#define ALGORITHM_SPWM_constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))




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
void ALGORITHM_SPWM(ThreePhase *pwm,ThreePhase *abc, Clarke *alpha_beta,float Udc,float pwm_mum,float pwm_max,float pwm_min)
{

	ALGORITHM_Inverse_Clarke(abc,alpha_beta);
	
	float a=abc->A/Udc+0.5f;
	float b=abc->B/Udc+0.5f;
	float c=abc->C/Udc+0.5f;
	
	pwm->A=a*pwm_mum;
	pwm->B=b*pwm_mum;
	pwm->C=c*pwm_mum;
	
	pwm->A=ALGORITHM_SPWM_constrain(pwm->A,pwm_min,pwm_max);
	pwm->B=ALGORITHM_SPWM_constrain(pwm->B,pwm_min,pwm_max);
	pwm->C=ALGORITHM_SPWM_constrain(pwm->C,pwm_min,pwm_max);
	
	
}



/**
 * @brief SVPWM 调制（空间矢量脉宽调制）：由 αβ 参考电压生成三相 PWM 比较值（中心对齐，七段式）
 * @param pwm        输出三相 PWM 比较值结构体指针（成员 A/B/C 为比较计数值）
 * @param alpha_beta 输入 αβ 参考电压结构体指针（通常来自逆 Park 变换的输出）
 * @param Udc        直流母线电压（单位：伏特），用于计算基本矢量作用时间
 * @param Tpwm       PWM 周期计数最大值（即定时器自动重装载值），决定 PWM 载波周期
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
void ALGORITHM_SVPWM(ThreePhase *pwm, Clarke *alpha_beta,float Udc,float Tpwm)
{
	float a=0.0f;
	float b=0.0f;
	float c=0.0f;
	
	a = alpha_beta->Beta;
	b = square_root_3*alpha_beta->Alpha - alpha_beta->Beta;
	c = -square_root_3*alpha_beta->Alpha - alpha_beta->Beta;
	
	uint8_t CodedValue=0;
	
	if(a>0.0f)CodedValue|=0x01;
	if(b>0.0f)CodedValue|=0x02;
	if(c>0.0f)CodedValue|=0x04;
	
	uint8_t sector=0;
	
	float s_buf= (square_root_3*Tpwm)/Udc;
	
	float x = s_buf*alpha_beta->Beta;
	float y = s_buf*(square_root_3_divide_2*alpha_beta->Alpha + alpha_beta->Beta);
	float z = s_buf*(-square_root_3_divide_2*alpha_beta->Alpha + alpha_beta->Beta);
	
	float t1=0;
	float t2=0;
	float t0=0;
	
	
	switch(CodedValue)
	{
		case 3:
			sector = 1;
			t1 = -z;
			t2 = x;
			break;
		case 1:
			sector = 2;
			t1 = z;
			t2 = y;
			break;
		case 5:
			sector = 3;
			t1 = x;
			t2 = -y;
			break;
		case 4:
			sector = 4;
			t1 = -x;
			t2 = z;
			break;
		case 6:
			sector = 5;
			t1 = -y;
			t2 = -z;
			break;
		case 2:
			sector = 6;
			t1 = y;
			t2 = -x;
			break;
	}
	
	if(sector==0) return ;
	
	if (t1 + t2 > Tpwm) {
			float sum = t1 + t2;
			t1 = t1 * Tpwm / sum;
			t2 = Tpwm - t1;   // 强制保证精确相等
	}
	
	
	t0 = Tpwm - (t1+t2);
	
	ThreePhase Tadc;
	
	Tadc.A = t0/4.0f;
	Tadc.B = Tadc.A + t1/2.0f;
	Tadc.C = Tadc.B + t2/2.0f; 
	
	switch(sector)
	{
		case 1:
			pwm->A = Tadc.A;
			pwm->B = Tadc.B;
			pwm->C = Tadc.C;
			break;
		case 2:
			pwm->A = Tadc.B;
			pwm->B = Tadc.A;
			pwm->C = Tadc.C;
			break;
		case 3:
			pwm->A = Tadc.C;
			pwm->B = Tadc.A;
			pwm->C = Tadc.B;
			break;
		case 4:
			pwm->A = Tadc.C;
			pwm->B = Tadc.B;
			pwm->C = Tadc.A;
			break;
		case 5:
			pwm->A = Tadc.B;
			pwm->B = Tadc.C;
			pwm->C = Tadc.A;
			break;
		case 6:
			pwm->A = Tadc.A;
			pwm->B = Tadc.C;
			pwm->C = Tadc.B;
			break;
	}
	
}











