/*
 * ADC.c
 *
 *  Created on: May 28, 2023
 *      Author: tillkorsmeier
 */


#include "ADC.h"
#include <math.h>
#include <stdio.h>


HAL_StatusTypeDef READ_ADC (ADC_HandleTypeDef *hadc, uint32_t channel, int rank, uint16_t *data)
{
	// Set channel configuration
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = channel;
	sConfig.Rank = rank;

	if (HAL_ADC_ConfigChannel(hadc, &sConfig) == HAL_OK)	// Set configuration
	{
		HAL_ADC_Start(hadc);								// Start ADC
		HAL_ADC_PollForConversion(hadc, 100);				// Wait for end of conversion (Polling timeout in ms or HAL_MAX_DELAY)
		*data = HAL_ADC_GetValue(hadc);						// Get Value from register
		HAL_ADC_Stop(hadc);									// Stop ADC

		return HAL_OK;
	}

	else return HAL_ERROR;

}
