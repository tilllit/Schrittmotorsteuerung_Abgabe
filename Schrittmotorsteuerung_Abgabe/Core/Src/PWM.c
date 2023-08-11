/*
 * PWM.c
 *
 *  Created on: May 28, 2023
 *      Author: tillkorsmeier
 */


#include "PWM.h"
#include <math.h>
#include <stdio.h>


void PWM_on (struct Coil* coil)
{
	adjust_PWM_DC(coil->HS_timer, coil->HS_channel, 20.0);	// High side
	adjust_PWM_DC(coil->LS_timer, coil->LS_channel, 100.0);	// Low side
}


void PWM_off (struct Coil* coil)
{
	adjust_PWM_DC(coil->HS_timer, coil->HS_channel, 0.0);	// High side
	adjust_PWM_DC(coil->LS_timer, coil->LS_channel, 0.0);	// Low side
}

void set_PWM (struct Coil* coil)
{
	float prescale = 100;

		float HS_percent = coil->DC * (prescale * 0.01);
		float LS_percent = 0;

		if (coil->DC > 0)
		{
			LS_percent = 100;
		}

		adjust_PWM_DC(coil->HS_timer, coil->HS_channel, HS_percent);	// High side
		adjust_PWM_DC(coil->LS_timer, coil->LS_channel, LS_percent);	// Low side
}


/**
 * Adaptive adjustment of PWM duty cycle
 */
void adjust_PWM_DC(TIM_HandleTypeDef* const pwmHandle, int Channel, const float DC)
{
	// Check parameters
	if ((pwmHandle != NULL) && ((DC >= 0.0) && (DC <= 100.0)))
	{
		// Capture compare value as percentage of ARR value (rounded to int)
		uint32_t newRegVal = (uint32_t)roundf((float)(pwmHandle->Instance->ARR) * (DC * 0.01));

		// Limit
		if(newRegVal > pwmHandle->Instance->ARR)
		{
			newRegVal = pwmHandle->Instance->ARR;
		}

		if (Channel == 1) pwmHandle->Instance->CCR1 = (uint32_t)(roundf(newRegVal)); // Channel 1 -> CCR1
		if (Channel == 2) pwmHandle->Instance->CCR2 = (uint32_t)(roundf(newRegVal));
		if (Channel == 3) pwmHandle->Instance->CCR3 = (uint32_t)(roundf(newRegVal));
		if (Channel == 4) pwmHandle->Instance->CCR4 = (uint32_t)(roundf(newRegVal));
	}
}
