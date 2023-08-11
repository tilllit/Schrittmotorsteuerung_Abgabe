/*
 * Calibration.c
 *
 *  Created on: 05.08.2023
 *      Author: tillkorsmeier
 */


#include "Calibration.h"
#include "STEP.h"
#include "PWM.h"
#include "I_Controller.h"
#include <stdio.h>

extern struct Coil A1;
extern struct Coil A2;
extern struct Coil B1;
extern struct Coil B2;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

extern int flag;
extern int tCnt;
extern uint32_t mBuff[30][4];
extern uint32_t value[4];
extern uint32_t val[4];

extern struct Status M_state;

long diff;
struct Coil coils[4];

int counter = 0;


void Calibrate()
{
	coils[0] = A1;
	coils[1] = A2;
	coils[2] = B1;
	coils[3] = B2;

	for (int i = 0; i < 4; i++)
	{
		coils[i].I_t[0] = 0;
		coils[i].I_t[1] = 0;
		coils[i].max_dif = 0;
	}

	uint32_t cnt = 0;
	HAL_TIM_Base_Start_IT(&htim3);

	while (cnt < 10000)
	{
		if (flag == 1)
		{
			for (int i = 0; i < 4; i++)
			{
				coils[i].I_t[1] = coils[i].I_t[0];
				coils[i].I_t[0] = value[i];

				diff = coils[i].I_t[0] - coils[i].I_t[1];
				if (diff > coils[i].max_dif && diff < 4000)
				{
					coils[i].max_dif = diff;
				}
			}

			val[0] = value[0];
			val[1] = value[1];
			val[2] = value[2];
			val[3] = value[3];

			CNTRL_I(&A1);
			CNTRL_I(&A2);
			CNTRL_I(&B1);
			CNTRL_I(&B2);

			cnt ++;
		}
	}

	// Add margin to maximum
	for (int i = 0; i < 4; i++)
	{
		coils[i].max_dif = coils[i].max_dif + coils[i].max_dif * 0.10;
	}

	printf("Kalibrierwerte: %li, %li, %li, %li \n", coils[0].max_dif, coils[1].max_dif, coils[2].max_dif, coils[3].max_dif);

	M_state.STEPS = 0;
	M_state.ON = 0;
}


void Continuous(int messungen , int Schrittverlust)
{
	// Checking interrupt flag
	if (flag == 1)
	{
		// Safe data to buffer
		mBuff[tCnt][0] = value[0];
		mBuff[tCnt][1] = value[1];
		mBuff[tCnt][2] = value[2];
		mBuff[tCnt][3] = value[3];

		// After X cycles
		if (!(tCnt < messungen))
		{
			val[0] = 0;
			val[1] = 0;
			val[2] = 0;
			val[3] = 0;
			for (int i = 0; i < messungen; i++)
			{
				// Average data
				val[0] += mBuff[i][0];
				val[1] += mBuff[i][1];
				val[2] += mBuff[i][2];
				val[3] += mBuff[i][3];
			}
			val[0] = val[0] / (messungen-1);
			val[1] = val[1] / (messungen-1);
			val[2] = val[2] / (messungen-1);
			val[3] = val[3] / (messungen-1);


			CNTRL_I(&A1);
			CNTRL_I(&A2);
			CNTRL_I(&B1);
			CNTRL_I(&B2);

			// End routine
			tCnt = 0;
		}

		if (Schrittverlust == 1)
		{
			for (int i = 0; i < 4; i++)
			{
				coils[i].I_t[1] = coils[i].I_t[0];
				coils[i].I_t[0] = value[i];

				diff = coils[i].I_t[0] - coils[i].I_t[1];
				if (diff > coils[i].max_dif && diff < 4000)
				{
					step_loss();
				}
			}
		}

		// Reset flag
		flag = 0;
	}
}

void step_loss()
{
	counter ++;
	if (counter > 5)
	{
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		M_state.active = 0;
		ALL_OFF();

		HAL_Delay(1000);

		ALL_OFF();
		HAL_TIM_Base_Stop_IT(&htim3);
		HAL_TIM_Base_Stop_IT(&htim1);
		HAL_TIM_Base_Stop_IT(&htim2);

		printf("Schrittverlust erkannt & Motor gestoppt \n");
	}
}
