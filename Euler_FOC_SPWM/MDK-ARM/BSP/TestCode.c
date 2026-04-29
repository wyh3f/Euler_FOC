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



void TestCode_INIT(void)
{
	HAL_Delay(2500);
	BSP_UART_Init();
	BSP_TIM_init();
	BSP_ADC_init();
	BSP_LED1_Write(0);
	BSP_LED2_Write(0);
}


extern TIM_HandleTypeDef htim1;

ThreePhase ABC;
ThreePhase PWM;
Clarke Alpha_Beta;
Park QP_thet={
	.Q=4.0,
	.D=0,
	.thet=0
};


void TestCode_Main(void)
{
	float buf[6];
	if(BSP_time_TIM6>=10)
	{
		BSP_time_TIM6=0;
		buf[0] = (float)BSP_GET_ADC();
		Vofa_JustFloat_Send(buf,1);
		
		
	}
	if(BSP_time_TIM1>=4)
	{
		BSP_time_TIM1=0;
		
		
		
		
	
	
	

	
	DRIVER_Park_Limit(&QP_thet);
	DRIVER_UpdateTheta_1(&QP_thet);
	
	ALGORITHM_Inverse_Park(&Alpha_Beta,&QP_thet);
	ALGORITHM_Inverse_Clarke(&ABC,&Alpha_Beta);
	
	DRIVER_UpdateThreePhase_PWM_1(&ABC,&PWM);
	
//	float buf[6];
//	
//	buf[0]=ABC.A;
//	buf[1]=ABC.B;
//	buf[2]=ABC.C;
//	
//	buf[3]=PWM.A;
//	buf[4]=PWM.B;
//	buf[5]=PWM.C;

//	Vofa_JustFloat_Send(buf,6);

		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,(uint16_t)PWM.A);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,(uint16_t)PWM.B);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,(uint16_t)PWM.C);
	
	}
}



