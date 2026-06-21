//BSP_ADC
#include "BSP_ADC.h"
#include "BSP_LED.h"
#include "BSP_TIM.h"  
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern OPAMP_HandleTypeDef hopamp1;
extern OPAMP_HandleTypeDef hopamp2;
extern OPAMP_HandleTypeDef hopamp3;

float ADC_IN_Value[4];  //ADC值

float ADC_IN_Median_Buf[3]={0};
float ADC_Median=0;


void BSP_ADC_init(void)
{
	//开启运放
	HAL_OPAMP_Start(&hopamp1);
	HAL_OPAMP_Start(&hopamp2);
	HAL_OPAMP_Start(&hopamp3);
	
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);    //AD校准
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);    //AD校准
	


	// 2. 启动 ADC1 注入组 外部触发 + 中断
	HAL_ADCEx_InjectedStart_IT(&hadc1);
	// 3. 启动 ADC2 注入组 外部触发 + 中断
	HAL_ADCEx_InjectedStart_IT(&hadc2);
	
	
}


float BSP_GET_ADC(void)
{
	HAL_ADC_Start(&hadc1);     //启动ADC转换
  ADC_IN_Value[0] = (float)HAL_ADC_GetValue(&hadc1)*0.0008058f;
	return ADC_IN_Value[0];
}



#include "BSP_UART.h"

// 转换系数
#define ADC_VREF 3.3f
#define ADC_MAX  4095.0f
#define ADC1_Paranoia 1.64f
#define ADC2_Paranoia 1.64f
#define ADC3_Paranoia 1.64f
#define OP_Magnify 16.5f
#define Resistance 0.005f



#include "ALGORITHM_Clarke_Park.h"
#include "PID.h"


extern ThreePhase ABC;
extern ThreePhase PWM;
extern Clarke Alpha_Beta;
extern Park QP_thet;
extern ThreePhase Detection_ABC;
extern Clarke Detection_Alpha_Beta;
extern Park Detection_QP_thet;


extern PIDController pid;
extern PIDController pid_speed;

float out_=0.13f;

uint8_t adc1_buf=0;
uint8_t adc2_buf=0;
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  if (hadc == &hadc1)
  {
		

		ADC_IN_Value[1]=(float)(HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1));
		ADC_IN_Value[2]=(float)(HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2));


		ADC_IN_Value[1]=((ADC_IN_Value[1]*ADC_VREF)/ADC_MAX-ADC1_Paranoia)/OP_Magnify/Resistance-0.04f;
		ADC_IN_Value[2]=((ADC_IN_Value[2]*ADC_VREF)/ADC_MAX-ADC2_Paranoia)/OP_Magnify/Resistance-0.1f;
		
		ADC_IN_Value[3]=-(ADC_IN_Value[1]+ADC_IN_Value[2]);
		
		Detection_ABC.A=ADC_IN_Value[1];
		Detection_ABC.B=ADC_IN_Value[2];
		Detection_ABC.C=ADC_IN_Value[3];
		ALGORITHM_Clarke_(&Detection_ABC,&Detection_Alpha_Beta);
		ALGORITHM_Park_Transform(&Detection_Alpha_Beta,-QP_thet.thet,&Detection_QP_thet);
		
		
//		QP_thet.D = PIDController_Update(&pid,0.0f,Detection_QP_thet.D);
		QP_thet.Q = PIDController_Update(&pid,out_,Detection_QP_thet.Q);
//		QP_thet.D = out_;
		
		adc1_buf=1;
//		Detection_ABC.C=-(ADC_IN_Value[1]+ADC_IN_Value[2]);
		
  }
	
	if (hadc == &hadc2)
  {

		ADC_IN_Value[3]=(float)(HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1));
		
		ADC_IN_Value[3]=((ADC_IN_Value[3]*ADC_VREF)/ADC_MAX-ADC3_Paranoia)/OP_Magnify/Resistance+0.035f;

//		Detection_ABC.C=ADC_IN_Value[3];
		
		adc2_buf=1;
//		BSP_LED2_Write(0);
  }
	
	if((adc1_buf==1)&&(adc2_buf==1))
	{
		adc1_buf=adc2_buf=0;
		
//		Detection_ABC.A=ADC_IN_Value[1];
//		Detection_ABC.B=ADC_IN_Value[2];
//		Detection_ABC.C=ADC_IN_Value[3];
//		ALGORITHM_Clarke_(&Detection_ABC,&Detection_Alpha_Beta);
//		ALGORITHM_Park_Transform(&Detection_Alpha_Beta,QP_thet.thet,&Detection_QP_thet);
		
		
		
	}
	
  /* NOTE : This function should not be modified. When the callback is needed,
            function HAL_ADCEx_InjectedConvCpltCallback must be implemented in the user file.
  */
}









//// main.c 的初始化部分
//HAL_ADCEx_InjectedStart_IT(&hadc1); // 启动ADC注入组并开启中断
//HAL_TIM_Base_Start(&htim1);         // 启动定时器，开始产生触发信号
//// 在 stm32g4xx_it.c 或 main.c 文件末尾添加
//volatile uint32_t injected_adc_values[3]; // 用于存储3个通道的转换结果

//void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//    if(hadc->Instance == ADC1) // 判断是否为ADC1
//    {
//        // 读取注入通道的转换结果
//        for (int i = 0; i < 3; i++) {
//            injected_adc_values[i] = HAL_ADCEx_InjectedGetValue(&hadc1, i + 1);
//        }
//        // 在这里处理转换结果，例如执行FOC计算等
//    
//        /* 如果需要，这里可以添加一个回调或设置一个标志，
//         * 通知主循环（main loop）有新的数据已准备好。 */
//    }
//}





