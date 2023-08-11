/*
 * PWM.h
 *
 *  Created on: 28.05.2023
 *      Author: tillkorsmeier
 */

#include "main.h"
#include "I_Controller.h"

#ifndef INC_PWM_H_
#define INC_PWM_H_

struct Coil
{
	TIM_HandleTypeDef* HS_timer;	// High side
	int HS_channel;
	TIM_HandleTypeDef* LS_timer;	// Low side
	int LS_channel;
	int Shunt;						// Shunt (ADC)

	float e_sum;					// Error

	float DC;						// Duty Cycle
	float DC_old;					// Duty Cycle
	float w;

	long max_dif;					// Schrittverlust
	uint32_t I_t [2];				// Kalibrierung


};

void PWM_on (struct Coil* coil);
void PWM_off (struct Coil* coil);
void set_PWM (struct Coil* coil);
void adjust_PWM_DC(TIM_HandleTypeDef* const pwmHandle, int Channel, const float DC);


#endif /* INC_PWM_H_ */
