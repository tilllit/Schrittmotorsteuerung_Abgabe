/*
 * STEP.c
 *
 *  Created on: 05.07.2023
 *      Author: tillkorsmeier
 */

#include "stdio.h"
#include "PWM.h"
#include "STEP.h"
#include "I_Controller.h"

extern TIM_HandleTypeDef htim3;


extern struct Coil A1;
extern struct Coil A2;
extern struct Coil B1;
extern struct Coil B2;


int FULL[4][4] = {	{ 100, 000, 000, 100 },
					{ 100, 000, 100, 000 },
					{ 000, 100, 100, 000 },
					{ 000, 100, 000, 100 }	};

int HALF[8][4] = {	{ 100, 000, 000, 100 },
					{ 100, 000, 000, 000 },
					{ 100, 000, 100, 000 },
					{ 000, 000, 100, 000 },
					{ 000, 100, 100, 000 },
					{ 000, 100, 000, 000 },
					{ 000, 100, 000, 100 },
					{ 000, 000, 000, 100 }	};

int FOUR[16][4] = {	{ 100, 000, 000, 100 },
					{ 100, 000, 000, 050 },
					{ 100, 000, 000, 000 },
					{ 100, 000, 050, 000 },
					{ 100, 000, 100, 000 },
					{ 050, 000, 100, 000 },
					{ 000, 000, 100, 000 },
					{ 000, 050, 100, 000 },
					{ 000, 100, 100, 000 },
					{ 000, 100, 050, 000 },
					{ 000, 100, 000, 000 },
					{ 000, 100, 000, 050 },
					{ 000, 100, 000, 100 },
					{ 000, 050, 000, 100 },
					{ 000, 000, 000, 100 },
					{ 050, 000, 000, 100 }	};

int EIGHT[32][4] = {{ 100, 000, 000, 100 },
					{ 100, 000, 000, 075 },
					{ 100, 000, 000, 050 },
					{ 100, 000, 000, 025 },
					{ 100, 000, 000, 000 },
					{ 100, 000, 025, 000 },
					{ 100, 000, 050, 000 },
					{ 100, 000, 075, 000 },
					{ 100, 000, 100, 000 },
					{ 075, 000, 100, 000 },
					{ 050, 000, 100, 000 },
					{ 025, 000, 100, 000 },
					{ 000, 000, 100, 000 },
					{ 000, 025, 100, 000 },
					{ 000, 050, 100, 000 },
					{ 000, 075, 100, 000 },
					{ 000, 100, 100, 000 },
					{ 000, 100, 075, 000 },
					{ 000, 100, 050, 000 },
					{ 000, 100, 025, 000 },
					{ 000, 100, 000, 000 },
					{ 000, 100, 000, 025 },
					{ 000, 100, 000, 050 },
					{ 000, 100, 000, 075 },
					{ 000, 100, 000, 100 },
					{ 000, 075, 000, 100 },
					{ 000, 050, 000, 100 },
					{ 000, 025, 000, 100 },
					{ 000, 000, 000, 100 },
					{ 025, 000, 000, 100 },
					{ 050, 000, 000, 100 },
					{ 075, 000, 000, 100 }};

int SIXTEENTH[64][4] = {{ 100, 000, 000, 100 },
					{ 100, 000, 000, 88 },
					{ 100, 000, 000, 075 },
					{ 100, 000, 000, 62 },
					{ 100, 000, 000, 050 },
					{ 100, 000, 000, 38 },
					{ 100, 000, 000, 025 },
					{ 100, 000, 000, 012 },
					{ 100, 000, 000, 000 },
					{ 100, 000, 12, 000 },
					{ 100, 000, 025, 000 },
					{ 100, 000, 38, 000 },
					{ 100, 000, 050, 000 },
					{ 100, 000, 62, 000 },
					{ 100, 000, 075, 000 },
					{ 100, 000, 88, 000 },
					{ 100, 000, 100, 000 },
					{ 88, 000, 100, 000 },
					{ 075, 000, 100, 000 },
					{ 62, 000, 100, 000 },
					{ 050, 000, 100, 000 },
					{ 38, 000, 100, 000 },
					{ 025, 000, 100, 000 },
					{ 12, 000, 100, 000 },
					{ 000, 000, 100, 000 },
					{ 000, 12, 100, 000 },
					{ 000, 025, 100, 000 },
					{ 000, 38, 100, 000 },
					{ 000, 050, 100, 000 },
					{ 000, 68, 100, 000 },
					{ 000, 075, 100, 000 },
					{ 000, 88, 100, 000 },
					{ 000, 100, 100, 000 },
					{ 000, 100, 88, 000 },
					{ 000, 100, 075, 000 },
					{ 000, 100, 68, 000 },
					{ 000, 100, 050, 000 },
					{ 000, 100, 38, 000 },
					{ 000, 100, 025, 000 },
					{ 000, 100, 12, 000 },
					{ 000, 100, 000, 000 },
					{ 000, 100, 000, 12 },
					{ 000, 100, 000, 025 },
					{ 000, 100, 000, 38 },
					{ 000, 100, 000, 050 },
					{ 000, 100, 000, 68 },
					{ 000, 100, 000, 075 },
					{ 000, 100, 000, 88 },
					{ 000, 100, 000, 100 },
					{ 000, 88, 000, 100 },
					{ 000, 075, 000, 100 },
					{ 000, 68, 000, 100 },
					{ 000, 050, 000, 100 },
					{ 000, 38, 000, 100 },
					{ 000, 025, 000, 100 },
					{ 000, 12, 000, 100 },
					{ 000, 000, 000, 100 },
					{ 12, 000, 000, 100 },
					{ 025, 000, 000, 100 },
					{ 38, 000, 000, 100 },
					{ 050, 000, 000, 100 },
					{ 68, 000, 000, 100 },
					{ 075, 000, 000, 100 },
					{ 88, 000, 000, 100 }};



void SetSpeed(uint16_t ARR, uint16_t PSC)
{
	htim3.Instance->ARR = ARR;
	htim3.Instance->PSC = PSC;
}

void HOLD()
{
		A1.w = FULL[0][0];
		A2.w = FULL[0][1];
		B1.w = FULL[0][2];
		B2.w = FULL[0][3];
}

void fullstep_PID(int dir, int *pos)
{
	int length = 4;
		int position = *pos;

		if (dir == 1)
		{
			position++;
			if (position > length-1)
			{
				position = 0;
			}
		}
		if (dir == 0)
		{
			position--;
			if (position < 0)
			{
				position = length-1;
			}
		}

		*pos = position;

		A1.w = FULL[position][0];
		A2.w = FULL[position][1];
		B1.w = FULL[position][2];
		B2.w = FULL[position][3];
}

void halfstep_PID(int dir, int *pos)
{
	int length = 8;
	int position = *pos;

	if (dir == 1)
	{
		position++;
		if (position > length-1)
		{
			position = 0;
		}
	}
	if (dir == 0)
	{
		position--;
		if (position < 0)
		{
			position = length-1;
		}
	}

	*pos = position;

	A1.w = HALF[position][0];
	A2.w = HALF[position][1];
	B1.w = HALF[position][2];
	B2.w = HALF[position][3];
}

void microstep_PID(int dir, int *pos, int mstep)
{
	int length = 0;
	if (mstep == 4)
	{
		length = 16;
	}
	if (mstep == 8)
	{
		length = 32;
	}
	if (mstep == 16)
	{
		length = 64;
	}


	int position = *pos;

	if (dir == 1)
	{
		position++;
		if (position > length-1)
		{
			position = 0;
		}
	}
	if (dir == 0)
	{
		position--;
		if (position < 0)
		{
			position = length-1;
		}
	}
	*pos = position;


	if (mstep == 4)
	{
		A1.w = FOUR[position][0];
		A2.w = FOUR[position][1];
		B1.w = FOUR[position][2];
		B2.w = FOUR[position][3];
	}
	if (mstep == 8)
	{
		A1.w = EIGHT[position][0];
		A2.w = EIGHT[position][1];
		B1.w = EIGHT[position][2];
		B2.w = EIGHT[position][3];
	}
	if (mstep == 16)
	{
		A1.w = SIXTEENTH[position][0];
		A2.w = SIXTEENTH[position][1];
		B1.w = SIXTEENTH[position][2];
		B2.w = SIXTEENTH[position][3];
	}
}

void ALL_OFF ()
{
	PWM_off(&A1);
	PWM_off(&A2);
	PWM_off(&B1);
	PWM_off(&B2);
}




