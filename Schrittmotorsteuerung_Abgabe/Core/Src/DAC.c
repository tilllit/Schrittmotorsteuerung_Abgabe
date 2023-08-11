/*
 * DAC.c
 *
 *  Created on: May 28, 2023
 *      Author: tillkorsmeier
 */


#include "DAC.h"

#include <math.h>
#include <stdio.h>

void Set_Voltage (DAC_HandleTypeDef *hdac, uint32_t Channel, float Voltage)
{
	uint32_t val = (uint32_t)roundf((float)4096 / (float)3.3 * Voltage);
	HAL_DAC_SetValue(hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, val);
}
