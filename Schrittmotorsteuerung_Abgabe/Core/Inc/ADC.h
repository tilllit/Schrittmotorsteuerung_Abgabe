/*
 * ADC.h
 *
 *  Created on: May 28, 2023
 *      Author: tillkorsmeier
 */

#include "main.h"

#ifndef INC_ADC_H_
#define INC_ADC_H_

HAL_StatusTypeDef READ_ADC (ADC_HandleTypeDef *hadc, uint32_t channel, int rank, uint16_t *data);

#endif /* INC_ADC_H_ */
