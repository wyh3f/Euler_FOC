//TestCode
#include "TestCode.h" 






void TestCode_INIT(void)
{
	HAL_Delay(2500);
	BSP_UART_Init();
	TIM_init();
}
/*
*sin:arm_sin_f32,arm_sin_q31, arm_sin_q15
*cos:arm_cos_f32,arm_cos_q31, arm_cos_q15
*
*角度转弧度：DEG2RAD
*弧度转角度：RAD2DEG
*/
#define DEG2RAD(angleDegrees) ((angleDegrees) * 3.141592f / 180.0f)
#define RAD2DEG(angleRadians) ((angleRadians) * 180.0f / 3.141592f)

#define square_root_3_divide_3 0.57735f
#define square_root_3 1.73205f 

#define MECH_OFFSET 75.83f

extern TIM_HandleTypeDef htim1;

float angle=0.0;
float radian_0=0,radian_120=0,radian_240=0;
float I_a,I_b,I_c;
float I_alpha =0.0f,I_beta=0.0f;
float I_q=3.0f,I_d=0.0f,thet=0.0f;
float sin_theta,cos_theta;
float pwm_a,pwm_b,pwm_c;

#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

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
#define V_DC 24.0f          // 直流母线电压 24V
#define V_MAX (V_DC / 2.0f) // 最大相电压幅值 12V

void TestCode_Main(void)
{
	
	float buf[8];
//	if(time_buf_1ms>=1)
	{
		time_buf_1ms=0;
//克拉克、帕克变化
//		radian_0=DEG2RAD(angle);
//		radian_120=DEG2RAD(angle+120);
//		radian_240=DEG2RAD(angle+240);
//		
//		angle=angle+0.5f;
//		if(angle>=360.0f)angle=0.0f;
//		I_a=buf[0]=RAD2DEG(arm_sin_f32(radian_0));
//		I_b=buf[1]=RAD2DEG(arm_sin_f32(radian_120));
//		I_c=buf[2]=RAD2DEG(arm_sin_f32(radian_240));
//		
//		I_alpha=I_a;
//		I_beta=square_root_3_divide_3*(2*I_b+I_a);
//		buf[3]=I_alpha+200;
//		buf[4]=I_beta+200;
//		
//		// 直接使用弧度，避免转换
//		float theta_rad = atan2f(I_beta, I_alpha);
//		float sin_theta = arm_sin_f32(theta_rad);
//		float cos_theta = arm_cos_f32(theta_rad);

//		// 标准 Park 变换（无缩放）
//		float I_q = -I_alpha * sin_theta + I_beta * cos_theta;
//		float I_d =  I_alpha * cos_theta + I_beta * sin_theta;

//		// 加偏移（如果需要）
//		buf[5] = RAD2DEG(theta_rad)+400;        // 如果外部需要角度值，可保留度
//		buf[6] = I_q + 400;
//		buf[7] = I_d + 400;
//		
//		Vofa_JustFloat_Send(buf,8);

		//克拉克、帕克逆变化
		float Vs = sqrtf(I_q * I_q + I_d * I_d);  // I_q, I_d 为原始 PI 输出
		if (Vs > V_MAX) 
			{
				float scale = V_MAX / Vs;
				I_q *= scale;
				I_d *= scale;
			}
		float theta_rad;		
		//fmodf((MT6816_RawAngleToDegree(MT6816_Read_RawAngle())-MECH_OFFSET) * 4.0f, 360.0f)
		//机械角度*极对数=电角度
//		thet=thet+I_q*0.25f;
//		thet = fmodf((MT6816_RawAngleToDegree(MT6816_Read_RawAngle())-MECH_OFFSET) * 4.0f, 360.0f) + I_q * 0.25f;
//		if(thet>=360.0f)thet=0.0f;
		
		thet = fmodf((MT6816_RawAngleToDegree(MT6816_Read_RawAngle())-MECH_OFFSET) * 4.0f, 360.0f);
		if (thet < 0) thet += 360.0f;   // 转为 0~360°
		theta_rad=DEG2RAD(thet);
		buf[0]=thet;
		buf[1]=I_q;
		buf[2]=I_d;
		
		sin_theta=arm_sin_f32(theta_rad);
		cos_theta=arm_cos_f32(theta_rad);
		
		I_alpha=cos_theta*I_d-sin_theta*I_q;
		I_beta=sin_theta*I_d+cos_theta*I_q;
		buf[3]=I_alpha+0;
		buf[4]=I_beta+0;
		
		I_a=I_alpha;
		I_b=(square_root_3*I_beta-I_alpha)*0.5f;
		I_c= (-square_root_3*I_beta-I_alpha)*0.5f;
		buf[5]=I_a+0;
		buf[6]=I_b+0;
		buf[7]=I_c+0;
		
		
		
		pwm_a=I_a/V_DC+0.5f;
		pwm_b=I_b/V_DC+0.5f;
		pwm_c=I_c/V_DC+0.5f;
		
		
		pwm_a=pwm_a*200.0f;
		pwm_b=pwm_b*200.0f;
		pwm_c=pwm_c*200.0f;
		
		
//		pwm_a=_constrain(pwm_a,4,196);
//		pwm_b=_constrain(pwm_b,4,196);
//		pwm_c=_constrain(pwm_c,4,196);
		
		buf[0]=pwm_a;
		buf[1]=pwm_b;
		buf[2]=pwm_c;
		
		
//		Vofa_JustFloat_Send(buf,8);

		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,pwm_a);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,pwm_b);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,pwm_c);
	}

}



