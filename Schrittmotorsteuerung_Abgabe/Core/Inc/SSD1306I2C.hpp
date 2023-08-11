/*
 * SSD1306I2C.hpp
 *
 *  Created on: Jul 11, 2023
 *      Author: tillkorsmeier
 */

#ifndef INC_SSD1306I2C_HPP_
#define INC_SSD1306I2C_HPP_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f4xx_hal.h"


class SSD1306I2C
{
	public:
		// Constructor
		SSD1306I2C();

	protected:

		/**
		 * @brief  Initializes SSD1306 LCD
		 * @param  None
		 * @retval Initialization status:
		 *           - 0: LCD was not detected on I2C port
		 *           - > 0: LCD initialized OK and ready to use
		 */
		void SSD1306_I2C_Init();



		/**
		 * @brief  Writes single byte to slave
		 * @param  *I2Cx: I2C used
		 * @param  address: 7 bit slave address, left aligned, bits 7:1 are used, LSB bit is not used
		 * @param  reg: register to write to
		 * @param  data: data to be written
		 * @retval None
		 */
		void SSD1306_I2C_Write(uint8_t address, uint8_t reg, uint8_t data);



		/**
		 * @brief  Writes multi bytes to slave
		 * @param  *I2Cx: I2C used
		 * @param  address: 7 bit slave address, left aligned, bits 7:1 are used, LSB bit is not used
		 * @param  reg: register to write to
		 * @param  *data: pointer to data array to write it to slave
		 * @param  count: how many bytes will be written
		 * @retval None
		 */
		void SSD1306_I2C_WriteMulti(uint8_t address, uint8_t reg, uint8_t *data, uint16_t count);

};

#ifdef __cplusplus
}
#endif

#endif /* INC_SSD1306I2C_HPP_ */
