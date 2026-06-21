//DRIVER_ThreePhase_Motor
#include "DRIVER_ThreePhase_Motor.h"

void DRIVER_Park_Limit(Park *qd_thet)
{
    const float v_max_sq = V_MAX * V_MAX;
    float Vs_sq = qd_thet->Q * qd_thet->Q + qd_thet->D * qd_thet->D;
    if (Vs_sq > v_max_sq)
    {
        float scale = V_MAX / sqrtf(Vs_sq);
        qd_thet->D *= scale;
        qd_thet->Q *= scale;
    }
}

void DRIVER_UpdateTheta_1(Park *qd_thet)
{
	qd_thet->thet=fmodf((MechanicalAngle-MECH_OFFSET)*MumberOfPolePairs,360.0f);
	if (qd_thet->thet < 0) qd_thet->thet += 360.0f;   // 转为 0~360°
}

void DRIVER_UpdateTheta_User(Park *qd_thet,float add_Thet)
{
	qd_thet->thet=fmodf(qd_thet->thet + add_Thet,360.0f);
	if (qd_thet->thet < 0) qd_thet->thet += 360.0f;   // 转为 0~360°
}

/*
对应电压矢量幅值 Vs = sqrt(V_d2 + V_q2) 必须满足：
Vs ≤ V_dc / 2

Duty_A = 0.5 + Va / V_dc
-V_dc/2 ≤ Va ≤ V_dc/2
Va：期望输出的正弦电压瞬时值（单位：V），通常来自电流环 PI 输出或直接给定的正弦波。
它的取值范围必须满足 -V_dc/2 ≤ Va ≤ V_dc/2，否则占空比会超出 0~1。
V_dc：直流母线电压（单位：V），即逆变器直流侧电源电压。
该值可以是固定常数（如 24V），也可以实时 ADC 采样（电池供电时）。
0.5：是偏置量，对应 PWM 占空比为 50% 时输出 V_dc/2 的直流电平。在单极性调制的 H 桥中，这个偏置会被抵消，最终负载上只得到交流 Va。
*/

void DRIVER_UpdateThreePhase_PWM_1(ThreePhase *abc,ThreePhase *pwm)
{
	
	
	float a=abc->A/V_DC+0.5f;
	float b=abc->B/V_DC+0.5f;
	float c=abc->C/V_DC+0.5f;
	
	pwm->A=a*V_PWM_MAX;
	pwm->B=b*V_PWM_MAX;
	pwm->C=c*V_PWM_MAX;
	
	pwm->A=_constrain(pwm->A,V_PWM_MIN,V_PWM_MAX-V_PWM_MIN);
	pwm->B=_constrain(pwm->B,V_PWM_MIN,V_PWM_MAX-V_PWM_MIN);
	pwm->C=_constrain(pwm->C,V_PWM_MIN,V_PWM_MAX-V_PWM_MIN);
}












