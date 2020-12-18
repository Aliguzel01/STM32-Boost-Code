/*
 * main.c
 *
 *  Created on: Dec 8, 2020
 *      Author: HP
 */
/*
 * main.c
 *
 *  Created on: 7 Ara 2020
 *      Author: HP
 */
#include "main.h"
#include<string.h>
#include<stdio.h>
#include<stdint.h>

void SystemclockConfig_HSE(void);
void ADC_Init(void);
void TIM1_Init(void);
void TIM3_Init(void);


 void MPPT_PerturbandObserve(void);
 uint16_t readADC(uint8_t PinNum,uint8_t ranknum);
void change_float_DutyPin(uint8_t PinNum,float floatduty);
void Dutyload_CCR_Register(uint8_t PinNum,uint16_t duty);
void ADC_Setpin(uint8_t PinNum,uint8_t ranknum);

ADC_HandleTypeDef  adc1;
TIM_HandleTypeDef  tim1;
TIM_HandleTypeDef  tim3;

uint32_t period=3599;
uint16_t voltage,current;
uint32_t power,previouspower;
uint16_t voltagesetpoint=0;
int8_t deltavoltage=2;


void Error_handler(void);



int main(void)
{

  HAL_Init();

  SystemclockConfig_HSE();

  ADC_Init();

  TIM1_Init();

  TIM3_Init();

  if (HAL_TIM_Base_Start_IT(&tim3) != HAL_OK )
  {
	  Error_handler();
  }

  if (HAL_TIM_PWM_Start(&tim1, TIM_CHANNEL_1) != HAL_OK )
  {
	  Error_handler();
  }

  if (HAL_TIM_PWM_Start(&tim1, TIM_CHANNEL_4) != HAL_OK )
   {
 	  Error_handler();
   }




  while(1);


	return 0;
}

void SystemclockConfig_HSE(void)
{
  RCC_OscInitTypeDef  OscInit;
  RCC_ClkInitTypeDef  ClkInit;

  //Ana dahili çıkış regulatörü yaoılandırması
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

memset(&OscInit,0,sizeof(OscInit));
  OscInit.OscillatorType=RCC_OSCILLATORTYPE_HSE;
  OscInit.HSEState=RCC_HSE_BYPASS;
  OscInit.PLL.PLLState=RCC_PLL_ON;
  OscInit.PLL.PLLSource=RCC_PLLSOURCE_HSE;
  OscInit.PLL.PLLM=8;
  OscInit.PLL.PLLN=144;
  OscInit.PLL.PLLP=2;
  OscInit.PLL.PLLQ=2;

  if( HAL_RCC_OscConfig(&OscInit) != HAL_OK )
  {
	  Error_handler();
  }
  memset(&ClkInit,0,sizeof(ClkInit));
  ClkInit.ClockType= RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
		             |RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;

  ClkInit.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK;
  ClkInit.AHBCLKDivider=RCC_SYSCLK_DIV1;
  ClkInit.APB1CLKDivider=RCC_HCLK_DIV2;
  ClkInit.APB2CLKDivider=RCC_HCLK_DIV1;

  if( HAL_RCC_ClockConfig(&ClkInit,FLASH_ACR_LATENCY_2WS) !=HAL_OK )
  {
	  Error_handler();
  }

	/* Sistem zamanlayıcı kesinti frekansını yapılandırın (her 1 ms için) */
	uint32_t hclk_freq = HAL_RCC_GetHCLKFreq ();
	HAL_SYSTICK_Config (hclk_freq / 1000 );

	/* * Systick'i Yapılandırın
	*/
	HAL_SYSTICK_CLKSourceConfig (SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn kesinti yapılandırması */
	HAL_NVIC_SetPriority (SysTick_IRQn, 0 , 0 );


}

void ADC_Init(void)
{
  ADC_ChannelConfTypeDef   adcconfig;

  adc1.Instance=ADC1;
  adc1.Init.ClockPrescaler=ADC_CLOCK_SYNC_PCLK_DIV6;
  adc1.Init.ContinuousConvMode=DISABLE;
  adc1.Init.DMAContinuousRequests=DISABLE;
  adc1.Init.DataAlign=ADC_DATAALIGN_RIGHT;
  adc1.Init.DiscontinuousConvMode=DISABLE;
  adc1.Init.EOCSelection=ADC_EOC_SEQ_CONV;
  adc1.Init.ExternalTrigConv=ADC_SOFTWARE_START;
  adc1.Init.ExternalTrigConvEdge=ADC_EXTERNALTRIGCONVEDGE_NONE;
  adc1.Init.NbrOfConversion=2;
  adc1.Init.Resolution=ADC_RESOLUTION_12B;
  adc1.Init.ScanConvMode=DISABLE;

  if( HAL_ADC_Init(&adc1) != HAL_OK )
  {
	  Error_handler();

  }

  memset(&adcconfig,0,sizeof(adcconfig));

  adcconfig.Channel=ADC_CHANNEL_0;
  adcconfig.Rank=1;
  adcconfig.SamplingTime=ADC_SAMPLETIME_3CYCLES;

  if( HAL_ADC_ConfigChannel(&adc1,&adcconfig) != HAL_OK )
  {
	  Error_handler();
  }

	adcconfig.Channel = ADC_CHANNEL_1;
	adcconfig.Rank = 2;
	adcconfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

	if (HAL_ADC_ConfigChannel(&adc1, &adcconfig) != HAL_OK)
	{
		Error_handler();
	}

}



void TIM1_Init(void)
{
   TIM_MasterConfigTypeDef  masterConfig;
   TIM_OC_InitTypeDef       ocinit;
   TIM_BreakDeadTimeConfigTypeDef  breakdeadTimeConfig;

   tim1.Instance=TIM1;
   tim1.Init.Prescaler=0;
   tim1.Init.CounterMode=TIM_COUNTERMODE_UP;
   tim1.Init.Period=period;
   tim1.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
   tim1.Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_DISABLE;
   tim1.Init.RepetitionCounter=0;

   if( HAL_TIM_PWM_Init(&tim1) != HAL_OK )
   {
	   Error_handler();
   }

   memset(&masterConfig,0,sizeof(masterConfig));
   masterConfig.MasterOutputTrigger=TIM_TRGO_RESET;
   masterConfig.MasterSlaveMode=TIM_MASTERSLAVEMODE_DISABLE;
   if( HAL_TIMEx_MasterConfigSynchronization(&tim1,&masterConfig) != HAL_OK )
   {
	   Error_handler();
   }

   memset(&ocinit,0,sizeof(ocinit));

   ocinit.OCMode=TIM_OCMODE_PWM1;
   ocinit.Pulse=0;
   ocinit.OCFastMode=TIM_OCFAST_DISABLE;
   ocinit.OCIdleState=TIM_OCIDLESTATE_RESET;
   ocinit.OCNIdleState=TIM_OCNIDLESTATE_RESET;
   ocinit.OCNPolarity=TIM_OCNPOLARITY_HIGH;
   ocinit.OCPolarity=TIM_OCPOLARITY_HIGH;

   if( HAL_TIM_PWM_ConfigChannel(&tim1,&ocinit,TIM_CHANNEL_1 ) != HAL_OK )
   {
	   Error_handler();
   }

   ocinit.OCMode=TIM_OCMODE_TIMING;
   if( HAL_TIM_PWM_ConfigChannel(&tim1,&ocinit,TIM_CHANNEL_4 ) != HAL_OK )
   {
	   Error_handler();
   }

   memset(&breakdeadTimeConfig,0,sizeof(breakdeadTimeConfig));

   breakdeadTimeConfig.AutomaticOutput=TIM_AUTOMATICOUTPUT_DISABLE;
   breakdeadTimeConfig.BreakFilter=0x0;
   breakdeadTimeConfig.BreakPolarity=TIM_BREAKPOLARITY_HIGH;
   breakdeadTimeConfig.BreakState=TIM_BREAK_DISABLE;
   breakdeadTimeConfig.DeadTime=0x00;
   breakdeadTimeConfig.LockLevel=TIM_LOCKLEVEL_OFF;
   breakdeadTimeConfig.OffStateIDLEMode=TIM_OSSI_DISABLE;
   breakdeadTimeConfig.OffStateRunMode=TIM_OSSR_DISABLE;

   if( HAL_TIMEx_ConfigBreakDeadTime(&tim1,&breakdeadTimeConfig) != HAL_OK )
   {
	   Error_handler();
   }


}

void TIM3_Init(void)
{
	TIM_ClockConfigTypeDef clockConfig;
	TIM_MasterConfigTypeDef  masterConfig;

   tim3.Instance=TIM3;
   tim3.Init.Prescaler=0;
   tim3.Init.CounterMode=TIM_COUNTERMODE_UP;
   tim3.Init.Period=period;
   tim3.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
   tim3.Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_DISABLE;
   tim3.Init.RepetitionCounter=0;
   if( HAL_TIM_Base_Init(&tim3) != HAL_OK  )
   {
	   Error_handler();
   }

   memset(&clockConfig,0,sizeof(clockConfig));

   clockConfig.ClockSource=TIM_CLOCKSOURCE_INTERNAL;
   if( HAL_TIM_ConfigClockSource(&tim3,&clockConfig) != HAL_OK )
   {
	   Error_handler();
   }

   memset(&masterConfig,0,sizeof(masterConfig));

   masterConfig.MasterOutputTrigger=TIM_TRGO_RESET;
   masterConfig.MasterSlaveMode=TIM_MASTERSLAVEMODE_DISABLE;
   if( HAL_TIMEx_MasterConfigSynchronization(&tim3,&masterConfig) != HAL_OK )
   {
	   Error_handler();
   }

}





void MPPT_PerturbandObserve(void)
{
	voltage=readADC(VOLTAGE_PIN,1);
	current=readADC(CURRENT_PIN,2);

	if( power < previouspower )
	{
		deltavoltage = -deltavoltage;
	}

	voltagesetpoint += deltavoltage;
	change_float_DutyPin(MOSFET_PIN,voltagesetpoint);

	previouspower=power;

}

uint16_t readADC(uint8_t PinNum,uint8_t ranknum)
{
	ADC_Setpin(PinNum,ranknum);

	if(HAL_ADC_Start(&adc1) != HAL_OK )
	{
	Error_handler();
	}

	if(  HAL_ADC_PollForConversion(&adc1,1000000) != HAL_OK )
	{
	Error_handler();
	}

	return HAL_ADC_GetValue(&adc1);

}

void ADC_Setpin(uint8_t PinNum,uint8_t ranknum)
{
	ADC_ChannelConfTypeDef sConfig;
	if(ranknum == 1)
	{
		sConfig.Channel=PinNum;
		sConfig.Rank=ranknum;
		sConfig.SamplingTime=ADC_SAMPLETIME_3CYCLES;
		if(HAL_ADC_ConfigChannel(&adc1, &sConfig) != HAL_OK )
		{
			Error_handler();
		}

	}
	else
	{

		sConfig.Channel=PinNum;
		sConfig.Rank=ranknum;
		sConfig.SamplingTime=ADC_SAMPLETIME_3CYCLES;
		if(HAL_ADC_ConfigChannel(&adc1, &sConfig) != HAL_OK )
		{
			Error_handler();
		}
	}
}




 void change_float_DutyPin(uint8_t PinNum,float floatduty)
{
	uint16_t duty=floatduty * period;
	Dutyload_CCR_Register(PinNum,duty);

}



void Dutyload_CCR_Register(uint8_t PinNum,uint16_t duty)
{
	if(PinNum == 9)
	{
		TIM1->CCR1 =duty;
	}
	else if(PinNum == 14)
	{
		TIM1->CCR4 =duty;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	MPPT_PerturbandObserve();
}


 void Error_handler(void)
{
	while(1);
}



