//TestCode
#include "TestCode.h" 

#include "tim.h"

#include "BSP_UART.h"
#include "BSP_TIM.h" 
#include "BSP_LED.h"
#include "BSP_KEY.h" 
#include "BSP_ADC.h"

#include "DRIVER_MT6816.h" 
#include "DRIVER_ThreePhase_Motor.h"

#include "ALGORITHM_Clarke_Park.h"
#include "PID.h"


void TestCode_INIT(void)
{
	
	BSP_UART_Init();
	BSP_TIM_init();
	BSP_ADC_init();
	BSP_LED1_Write(1);
	BSP_LED2_Write(1);

	
	PIDController_Init(&pid);
	PIDController_Init(&pid_speed);
	
	pid.Kp = 2.2f;
	pid.Ki = 2.1f;
	pid.Kd = 0.0f;
	pid.limMax = 6.0f;
	pid.limMin = -1.0f;
	pid.limMaxInt = 15.0f;
	pid.limMinInt = -15.0f;
	
	pid_speed.Kp = 0.7f;
	pid_speed.Ki = 0.002f;
	pid_speed.Kd = 0.0f;
	pid_speed.limMax = 2.4f;
	pid_speed.limMin = 0.0f;
	pid_speed.limMaxInt = 1500.0f;
	pid_speed.limMinInt = 100.0f;
	
	
}


extern TIM_HandleTypeDef htim1;

ThreePhase ABC;
ThreePhase PWM;
Clarke Alpha_Beta;
Park QP_thet={
	.Q=3.5,
	.D=0.0,
	.thet=0
};

ThreePhase Detection_ABC;
Clarke Detection_Alpha_Beta;
Park Detection_QP_thet;

PIDController pid;
PIDController pid_speed;

float buf[6];
void TestCode_Main(void)
{
//	float buf[6];
//	if(BSP_time_TIM6>=10)
//	{
//		BSP_time_TIM6=0;
//		
//		
//		
//	}
//	

	
	if(BSP_time_TIM1>=4)
	{
		

		
//		Vofa_JustFloat_Send(ADC_IN_Value,4);
		
		
//	DRIVER_Park_Limit(&QP_thet);
//	QP_thet.thet=0.0f;
	DRIVER_UpdateTheta_1(&QP_thet);
	
	ALGORITHM_Inverse_Park(&Alpha_Beta,&QP_thet);
	ALGORITHM_Inverse_Clarke(&ABC,&Alpha_Beta);
	
	DRIVER_UpdateThreePhase_PWM_1(&ABC,&PWM);
	
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,(uint16_t)PWM.A);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,(uint16_t)PWM.B);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,(uint16_t)PWM.C);	
	
		
		buf[0] = Detection_QP_thet.D;//MechanicalAngle;
		buf[1] = Detection_QP_thet.Q;//MechanicalAngle;
//		buf[0] = Detection_Alpha_Beta.Alpha;//MechanicalAngle;
//		buf[1] = Detection_Alpha_Beta.Beta;//MechanicalAngle;
		buf[2] = QP_thet.Q;
		buf[3] = out_;
		Vofa_JustFloat_Send(buf,4);
	
	}
	

}



