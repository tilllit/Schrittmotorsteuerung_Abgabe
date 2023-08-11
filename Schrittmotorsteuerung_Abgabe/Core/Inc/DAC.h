/*
 * DAC.h
 *
 *  Created on: May 28, 2023
 *      Author: tillkorsmeier
 */

#include "main.h"

#ifndef INC_DAC_H_
#define INC_DAC_H_

void Set_Voltage (DAC_HandleTypeDef *hdac, uint32_t Channel, float Voltage);

#endif /* INC_DAC_H_ */
