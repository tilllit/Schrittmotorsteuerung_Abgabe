/*
 * SSD1306.hpp
 *
 *  Created on: Jul 11, 2023
 *      Author: tillkorsmeier
 */

#ifndef INC_SSD1306_HPP_
#define INC_SSD1306_HPP_


#include "stm32f4xx_hal.h"
#include <string.h>
#include <iostream>
using namespace std;
#include <stdbool.h>
#include "fonts.h"
#include "SSD1306I2C.hpp"


// SSD1306 settings
#define SSD1306_I2C_ADDR 0x78		// I2C address
#define SSD1306_WIDTH 128			// SSD1306 width in pixels
#define SSD1306_HEIGHT 64			// SSD1306 LCD height in pixels
#define SSD1306_COLOR_BLACK 0x00	// Black color, no pixel
#define SSD1306_COLOR_WHITE 0x01  	// Pixel is set. Color depends on LCD




class SSD1306: public SSD1306I2C
{
	public:
		// Constructor
		SSD1306();

		void SSD1306_String(string str, FontDef_t* Font, uint8_t color);

		/**
		 * @brief  Initializes SSD1306 LCD
		 * @param  None
		 * @retval Initialization status:
		 *           - 0: LCD was not detected on I2C port
		 *           - > 0: LCD initialized OK and ready to use
		 */
		uint8_t SSD1306_Init(void);



		/**
		 * @brief  Updates buffer from internal RAM to LCD
		 * @note   This function must be called each time you do some changes to LCD, to update buffer from RAM to LCD
		 * @param  None
		 * @retval None
		 */
		void SSD1306_UpdateScreen(void);



		/**
		 * @brief  Toggles pixels invertion inside internal RAM
		 * @note   @ref SSD1306_UpdateScreen() must be called after that in order to see updated LCD screen
		 * @param  None
		 * @retval None
		 */
		void SSD1306_ToggleInvert(void);



		/**
		 * @brief  Fills entire LCD with desired color
		 * @note   @ref SSD1306_UpdateScreen() must be called after that in order to see updated LCD screen
		 * @param  Color: Color to be used for screen fill. This parameter can be a value of @ref SSD1306_COLOR_t enumeration
		 * @retval None
		 */
		void SSD1306_Fill(uint8_t color);



		/**
		 * @brief  Draws pixel at desired location
		 * @note   @ref SSD1306_UpdateScreen() must called after that in order to see updated LCD screen
		 * @param  x: X location. This parameter can be a value between 0 and SSD1306_WIDTH - 1
		 * @param  y: Y location. This parameter can be a value between 0 and SSD1306_HEIGHT - 1
		 * @param  color: Color to be used for screen fill. This parameter can be a value of @ref SSD1306_COLOR_t enumeration
		 * @retval None
		 */
		void SSD1306_DrawPixel(uint16_t x, uint16_t y, uint8_t color);



		/**
		 * @brief  Sets cursor pointer to desired location for strings
		 * @param  x: X location. This parameter can be a value between 0 and SSD1306_WIDTH - 1
		 * @param  y: Y location. This parameter can be a value between 0 and SSD1306_HEIGHT - 1
		 * @retval None
		 */
		void SSD1306_GotoXY(uint16_t x, uint16_t y);



		/**
		 * @brief  Puts character to internal RAM
		 * @note   @ref SSD1306_UpdateScreen() must be called after that in order to see updated LCD screen
		 * @param  ch: Character to be written
		 * @param  *Font: Pointer to @ref FontDef_t structure with used font
		 * @param  color: Color used for drawing. This parameter can be a value of @ref SSD1306_COLOR_t enumeration
		 * @retval Character written
		 */
		char SSD1306_Putc(char ch, FontDef_t* Font, uint8_t color);



		/**
		 * @brief  Puts string to internal RAM
		 * @note   @ref SSD1306_UpdateScreen() must be called after that in order to see updated LCD screen
		 * @param  *str: String to be written
		 * @param  *Font: Pointer to @ref FontDef_t structure with used font
		 * @param  color: Color used for drawing. This parameter can be a value of @ref SSD1306_COLOR_t enumeration
		 * @retval Zero on success or character value when function failed
		 */
		char SSD1306_Puts(char* str, FontDef_t* Font, uint8_t color);



		/**
		 * @brief  Draws line on LCD
		 * @note   @ref SSD1306_UpdateScreen() must be called after that in order to see updated LCD screen
		 * @param  x0: Line X start point. Valid input is 0 to SSD1306_WIDTH - 1
		 * @param  y0: Line Y start point. Valid input is 0 to SSD1306_HEIGHT - 1
		 * @param  x1: Line X end point. Valid input is 0 to SSD1306_WIDTH - 1
		 * @param  y1: Line Y end point. Valid input is 0 to SSD1306_HEIGHT - 1
		 * @param  c: Color to be used. This parameter can be a value of @ref SSD1306_COLOR_t enumeration
		 * @retval None
		 */
		void SSD1306_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t color);



		/**
		 * @brief  Draws rectangle on LCD
		 * @note   @ref SSD1306_UpdateScreen() must be called after that in order to see updated LCD screen
		 * @param  x: Top left X start point. Valid input is 0 to SSD1306_WIDTH - 1
		 * @param  y: Top left Y start point. Valid input is 0 to SSD1306_HEIGHT - 1
		 * @param  w: Rectangle width in units of pixels
		 * @param  h: Rectangle height in units of pixels
		 * @param  c: Color to be used. This parameter can be a value of @ref SSD1306_COLOR_t enumeration
		 * @retval None
		 */
		void SSD1306_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t color);



		/**
		 * @brief  Draws the Bitmap
		 * @param  X:  X location to start the Drawing
		 * @param  Y:  Y location to start the Drawing
		 * @param  *bitmap : Pointer to the bitmap
		 * @param  W : width of the image
		 * @param  H : Height of the image
		 * @param  color : 1-> white/blue, 0-> black
		 */
		void SSD1306_DrawBitmap(int16_t x, int16_t y, const unsigned char* bitmap, int16_t w, int16_t h, uint8_t color);


		/**
		 * @brief  Clears all content
		 * @param  none
		 */
		void SSD1306_Clear (void);


	protected:

		// Class Variables
		uint16_t CurrentX;
		uint16_t CurrentY;
		uint8_t Initialized;
		uint8_t Inverted;
		uint8_t Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];
};

#endif /* INC_SSD1306_HPP_ */
