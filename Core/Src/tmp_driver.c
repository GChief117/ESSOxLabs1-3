/*
 * tmp_driver.c
 *
 *  Created on: 2 Feb 2022
 *      Author: zhuangzhuangdai
 */

#include "stm32f4xx.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_it.h"
#include "tmp_driver.h"

ADC_HandleTypeDef hadc1;

void tmp_sensor_init(void)
{
	//ADC_HandleTypeDef hadc1;
	ADC_ChannelConfTypeDef sConfig;
	hadc1.Instance = ADC1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	HAL_ADC_Init(&hadc1);
	// Enable clocks to ADC1
	__HAL_RCC_ADC1_CLK_ENABLE();
	// ADC1 Configuration, ADC_Channel_TempSensor is actual channel 16
	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

void get_tmp_value(uint32_t * temp_value)
{
	//uint32_t temp_value;
	HAL_ADC_Start(&hadc1);//Start the conversion
	HAL_ADC_PollForConversion(&hadc1, 100);//Processing the conversion
	*temp_value = HAL_ADC_GetValue(&hadc1);//Return the converted data
	HAL_ADC_Stop(&hadc1);//Stop the conversion
}

