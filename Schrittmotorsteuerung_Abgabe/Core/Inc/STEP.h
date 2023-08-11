/*
 * STEP.h
 *
 *  Created on: 05.07.2023
 *      Author: tillkorsmeier
 */

#ifndef INC_STEP_H_
#define INC_STEP_H_

#include "PWM.h"

struct Position
{
	int full;
	int half;
	int fourth;
	int eighth;
};

void fullstep_PID(int dir, int *pos);

void halfstep_PID(int dir, int *pos);

void microstep_PID(int dir, int *pos, int mstep);

void ALL_OFF ();

void HOLD();

void SetSpeed(uint16_t ARR, uint16_t PSC);

#endif /* INC_STEP_H_ */


