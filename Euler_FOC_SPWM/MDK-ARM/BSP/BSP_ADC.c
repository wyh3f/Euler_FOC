//BSP_ADC
#include "BSP_ADC.h"

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
//	HAL_ADCEx_InjectedStart_IT(&hadc1);
//	HAL_ADCEx_InjectedStart_IT(&hadc2);

  ADC_IN_Value[0] = (float)HAL_ADC_GetValue(&hadc1)*0.0008058f;
	
	return ADC_IN_Value[0];
}



#include "BSP_UART.h"


void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  if (hadc == &hadc1)
  {
		
//		if(ADC_IN_Median_Buf[0]==0&&ADC_IN_Median_Buf[1]==0)
//		{
//			ADC_IN_Median_Buf[0]=ADC_IN_Value[1]=HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
//			ADC_IN_Median_Buf[1]=ADC_IN_Value[2]=HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
//		}
		
//	if(ADC_Median!=0)
//	{
		ADC_IN_Value[1]=((HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1)-ADC_IN_Median_Buf[0])-ADC_Median)*0.0008058f;
		ADC_IN_Value[2]=((HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2)-ADC_IN_Median_Buf[1])-ADC_Median)*0.0008058f;
//		ADC_IN_Value[3]=((HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1)-ADC_IN_Median_Buf[2])-ADC_Median)*1.181391f;
//	}
//		

//		ADC_IN_Value[1]=(ADC_IN_Value[1]-0)*0.0008058f;//0008058f;

		
  }
	
	if (hadc == &hadc2)
  {
//		if(ADC_IN_Median_Buf[2]==0)
//		{
//			ADC_IN_Median_Buf[2]=ADC_IN_Value[3]=HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
//		}

//	if(ADC_Median!=0)
//	{
//		ADC_IN_Value[1]=((HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1)-ADC_IN_Median_Buf[0])-ADC_Median)*1.181391f;
//		ADC_IN_Value[2]=((HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2)-ADC_IN_Median_Buf[1])-ADC_Median)*1.181391f;
		ADC_IN_Value[3]=((HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1)-ADC_IN_Median_Buf[2])-ADC_Median)*0.0008058f;
//	}
		
  }
	
//	if((ADC_IN_Median_Buf[0]!=0&&ADC_IN_Median_Buf[1]!=0&&ADC_IN_Median_Buf[2]!=0)&&ADC_Median==0)
//	{
//		ADC_Median=(ADC_IN_Median_Buf[0]+ADC_IN_Median_Buf[1]+ADC_IN_Median_Buf[2])/3.0f;
//		
//		ADC_IN_Median_Buf[0]=HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1)-ADC_Median;
//		ADC_IN_Median_Buf[1]=HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2)-ADC_Median;
//		ADC_IN_Median_Buf[2]=HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1)-ADC_Median;
//		
//	}
	
//1.181391f
//0.0008058f;
	ADC_IN_Value[0]=ADC_IN_Value[0]+0.0001f;
	if(ADC_IN_Value[0]>=1.0f)ADC_IN_Value[0]=0.0f;
	
	Vofa_JustFloat_Send(ADC_IN_Value,4);
	
	
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





