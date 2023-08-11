/*
 * SSD1306I2C.cpp
 *
 *  Created on: Jun 2, 2023
 *      Author: tillkorsmeier
 */

#include "SSD1306I2C.hpp"

extern I2C_HandleTypeDef hi2c1;


// Constructor
SSD1306I2C::SSD1306I2C()
{
	SSD1306_I2C_Init();
}


//		FUNCTIONS HERE


void SSD1306I2C::SSD1306_I2C_Init()
{
	uint32_t p = 250000;
	while(p>0)
		p--;
}


void SSD1306I2C::SSD1306_I2C_Write(uint8_t address, uint8_t reg, uint8_t data)
{
	uint8_t dt[2];
	dt[0] = reg;
	dt[1] = data;
	HAL_I2C_Master_Transmit(&hi2c1, address, dt, 2, 10);
}


void SSD1306I2C::SSD1306_I2C_WriteMulti(uint8_t address, uint8_t reg, uint8_t* data, uint16_t count)
{
	uint8_t dt[256];
	dt[0] = reg;
	uint8_t i;
	for(i = 0; i < count; i++)
		dt[i+1] = data[i];

	HAL_I2C_Master_Transmit(&hi2c1, address, dt, count+1, 10);
}


