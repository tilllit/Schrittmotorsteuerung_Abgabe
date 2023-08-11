/*
 * I_Controller.c
 *
 *  Created on: Jul 11, 2023
 *      Author: tillkorsmeier
 */


#include "I_Controller.h"

extern uint32_t value[];
extern uint32_t val[4];
extern uint32_t ADC_VAL[4];

extern struct Status M_state;

float max_limit = 100;
float min_limit = 0;
float lim = 1;
float kp = 100000;
float ki = 10;
float kd = 0;

// Controller sampling time [s]
float Ts = 0.000001;	// PWM mit 10kHz

float e_old;
float I_max = 0.150 / 2;
float sum = 0;
float I_w = 0;
double I_y = 0;
float I_e = 0;
float DC = 0;
uint32_t ADC_val;


void CNTRL_I (struct Coil* coil)
{
	ADC_val = val[coil->Shunt];
	I_y = (((ADC_val / 4096) * 3.3f) / 50) / 0.050f;		// ADC:	Verstärkung	Widerstand -> [A]
	I_w = (coil->w * 0.01) * (M_state.I_max / 2);
	I_e = I_w - I_y;	// error

	// Anti-wind-up
	if ((coil->DC < max_limit) && (coil->DC > min_limit))
	{
		sum = coil->e_sum;
		sum = sum + I_e;
		coil->e_sum = sum;
	}

	// PI-Regler
	DC = (( kp * I_e ) + ( ki * coil->e_sum ));

	if (DC > (coil->DC + lim)) { DC = coil->DC + lim; }
	if (DC < (coil->DC - lim)) { DC = coil->DC - lim; }

	// Stellgrößenbegrenzung (u - Limit)
	if (DC > max_limit) { DC = max_limit; }
	if (DC < min_limit) { DC = min_limit; }

	if (coil->w == 0)
	{
		DC = 0;
	}

	coil->DC = DC;
	set_PWM(coil);
}
