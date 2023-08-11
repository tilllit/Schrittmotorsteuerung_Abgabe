/*
 * SSD1306.cpp
 *
 *  Created on: Jun 2, 2023
 *      Author: tillkorsmeier
 */

#include "SSD1306.hpp"
#include <string.h>
#include <iostream>
using namespace std;

extern I2C_HandleTypeDef hi2c1;


// Constructor
SSD1306::SSD1306()
{
	// Set default values
	CurrentX = 0;
	CurrentY = 0;

	Initialized = 1;
}


uint8_t SSD1306::SSD1306_Init()
{

	// Check connection
	if (HAL_I2C_IsDeviceReady(&hi2c1, SSD1306_I2C_ADDR, 1, 20000) != HAL_OK)
		return 0;

	// Delay
	uint32_t p = 2500;
	while(p>0)
		p--;

	// INITIALIZATION
	SSD1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, 0xAE); //display off
	SSD1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, 0x20); //Set Memory Addressing Mode
	SSD1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, 0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	SSD1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, 0xB0); //Set Page Start Address for Page Addressing Mode,0-7
	SSD1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, 0xC8); //Set COM Output Scan Direction
	SSD1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, 0x00); //---set low column address
	SSD1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, 0x10); //---set high column address
	SSD1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, 0x40); //--set start line address
	SSD1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, 0x81); //--set contrast control register
	SSD1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, 0xFF);
	SSD1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, 0xA1); //--set segment re-map 0 to 127
	SSD1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, 0xA6); //--set normal display
	SSD1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, 0xA8); //--set multiplex ratio(1 to 64)
	SSD1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, 0x3F); //
	SSD1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, 0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	SSD1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, 0xD3); //-set display offset
	SSD1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, 0x00); //-not offset
	SSD1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, 0xD5); //--set display clock divide ratio/oscillator frequency
	SSD1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, 0xF0); //--set divide ratio
	SSD1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, 0xD9); //--set pre-charge period
	SSD1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, 0x22); //
	SSD1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, 0xDA); //--set com pins hardware configuration
	SSD1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, 0x12);
	SSD1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, 0xDB); //--set vcomh
	SSD1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, 0x20); //0x20,0.77xVcc
	SSD1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, 0x8D); //--set DC-DC enable
	SSD1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, 0x14); //
	SSD1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, 0xAF); //--turn on SSD1306 panel

	// Clear screen
	SSD1306_Fill(SSD1306_COLOR_BLACK);

	// Update screen
	SSD1306_UpdateScreen();

	return 1;
}


/**
 * @brief  Fills entire LCD with desired color
 * @note   @ref SSD1306_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  Color: Color to be used for screen fill.
 * @retval None
 */
void SSD1306::SSD1306_Fill(uint8_t color)
{
	// Set memory
	memset(Buffer, (color == SSD1306_COLOR_BLACK) ? 0x00 : 0xFF, sizeof(Buffer));
}



/**
 * @brief  Updates buffer from internal RAM to LCD
 * @note   This function must be called each time you do some changes to LCD, to update buffer from RAM to LCD
 * @param  None
 * @retval None
 */
void SSD1306::SSD1306_UpdateScreen(void)
{
	for (uint8_t m = 0; m < 8; m++)
	{
		SSD1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, 0xB0 + m);
		SSD1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, 0x00);
		SSD1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, 0x10);

		SSD1306_I2C_WriteMulti(SSD1306_I2C_ADDR, 0x40, &Buffer[SSD1306_WIDTH * m], SSD1306_WIDTH);
	}
}



/**
 * @brief  Sets cursor pointer to desired location for strings
 * @param  x: X location. This parameter can be a value between 0 and SSD1306_WIDTH - 1
 * @param  y: Y location. This parameter can be a value between 0 and SSD1306_HEIGHT - 1
 * @retval None
 */
void SSD1306::SSD1306_GotoXY(uint16_t x, uint16_t y)
{
	// Set write pointers
	CurrentX = x;
	CurrentY = y;
}



/**
 * @brief  Puts string to internal RAM
 * @note   @ref SSD1306_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  *str: String to be written
 * @param  *Font: Pointer to @ref FontDef_t structure with used font
 * @param  color: Color used for drawing.
 * @retval Zero on success or character value when function failed
 */
char SSD1306::SSD1306_Puts(char* str, FontDef_t* Font, uint8_t color)
{
	// Writing characters
	while (*str)
	{
		if (SSD1306_Putc(*str, Font, color) != *str)
		{
			return *str;	// Return error
		}
		str++;
	}

	// Everything OK
	return *str;
}

void SSD1306::SSD1306_String(string str, FontDef_t* Font, uint8_t color)
{
	for(char &c : str)
	{
		SSD1306_Putc(c, Font, color);
	}
}



/**
 * @brief  Puts character to internal RAM
 * @note   @ref SSD1306_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  ch: Character to be written
 * @param  *Font: Pointer to @ref FontDef_t structure with used font
 * @param  color: Color used for drawing.
 * @retval Character written
 */
char SSD1306::SSD1306_Putc(char ch, FontDef_t* Font, uint8_t color)
{
	uint32_t i, b, j;

	// Check for available space
	if (SSD1306_WIDTH <= (CurrentX + Font->FontWidth) || SSD1306_HEIGHT <= (CurrentY + Font->FontHeight))
		return 0;	// Error

	// Go through font
	for (i = 0; i < Font->FontHeight; i++)
	{
		b = Font->data[(ch - 32) * Font->FontHeight + i];
		for (j = 0; j < Font->FontWidth; j++)
		{
			if ((b << j) & 0x8000)
				SSD1306_DrawPixel(CurrentX + j, (CurrentY + i), color);
			else
				SSD1306_DrawPixel(CurrentX + j, (CurrentY + i), !color);
		}
	}

	// Increase pointer
	CurrentX += Font->FontWidth;

	// Return character written
	return ch;
}



/**
 * @brief  Draws pixel at desired location
 * @note   @ref SSD1306_UpdateScreen() must called after that in order to see updated LCD screen
 * @param  x: X location. This parameter can be a value between 0 and SSD1306_WIDTH - 1
 * @param  y: Y location. This parameter can be a value between 0 and SSD1306_HEIGHT - 1
 * @param  color: Color to be used for screen fill.
 * @retval None
 */
void SSD1306::SSD1306_DrawPixel(uint16_t x, uint16_t y, uint8_t color)
{
	if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT)
		return;		/* Error */

	/* Check if pixels are inverted */
	if (Inverted)
		color = !color;

	/* Set color */
	if (color == SSD1306_COLOR_WHITE)
		Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
	else
		Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
}


/**
 * @brief Clear the display
 */
void SSD1306::SSD1306_Clear(void)
{
	SSD1306_Fill(0);
    SSD1306_UpdateScreen();
}


/**
 * @brief  Draws the Bitmap
 * @param  X:  X location to start the Drawing
 * @param  Y:  Y location to start the Drawing
 * @param  *bitmap : Pointer to the bitmap
 * @param  W : width of the image
 * @param  H : Height of the image
 * @param  color : 1-> white/blue, 0-> black
 */
void SSD1306::SSD1306_DrawBitmap(int16_t x, int16_t y, const unsigned char* bitmap, int16_t w, int16_t h, uint8_t color)
{
    int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
    uint8_t byte = 0;

    for(int16_t j=0; j<h; j++, y++)
    {
        for(int16_t i=0; i<w; i++)
        {
            if(i & 7) {byte <<= 1;}
            else
            {
            	byte = (*(const unsigned char *)(&bitmap[j * byteWidth + i / 8]));
            }

            if(byte & 0x80) {SSD1306_DrawPixel(x+i, y, color);}
        }
    }
}



/**
 * @brief  Draws line on LCD
 * @note   @ref SSD1306_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  x0: Line X start point. Valid input is 0 to SSD1306_WIDTH - 1
 * @param  y0: Line Y start point. Valid input is 0 to SSD1306_HEIGHT - 1
 * @param  x1: Line X end point. Valid input is 0 to SSD1306_WIDTH - 1
 * @param  y1: Line Y end point. Valid input is 0 to SSD1306_HEIGHT - 1
 * @param  c: Color to be used.
 * @retval None
 */
void SSD1306::SSD1306_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t color)
{
	int16_t dx, dy, sx, sy, err, e2, i, tmp;

	/* Check for overflow */
	if (x0 >= SSD1306_WIDTH)
		x0 = SSD1306_WIDTH - 1;
	if (x1 >= SSD1306_WIDTH)
		x1 = SSD1306_WIDTH - 1;
	if (y0 >= SSD1306_HEIGHT)
		y0 = SSD1306_HEIGHT - 1;
	if (y1 >= SSD1306_HEIGHT)
		y1 = SSD1306_HEIGHT - 1;

	dx = (x0 < x1) ? (x1 - x0) : (x0 - x1);
	dy = (y0 < y1) ? (y1 - y0) : (y0 - y1);
	sx = (x0 < x1) ? 1 : -1;
	sy = (y0 < y1) ? 1 : -1;
	err = ((dx > dy) ? dx : -dy) / 2;

	if (dx == 0)
	{
		if (y1 < y0)
		{
			tmp = y1;
			y1 = y0;
			y0 = tmp;
		}

		if (x1 < x0)
		{
			tmp = x1;
			x1 = x0;
			x0 = tmp;
		}

		/* Vertical line */
		for (i = y0; i <= y1; i++)
		{
			SSD1306_DrawPixel(x0, i, color);
		}

		/* Return from function */
		return;
	}

	if (dy == 0)
	{
		if (y1 < y0)
		{
			tmp = y1;
			y1 = y0;
			y0 = tmp;
		}

		if (x1 < x0)
		{
			tmp = x1;
			x1 = x0;
			x0 = tmp;
		}

		/* Horizontal line */
		for (i = x0; i <= x1; i++)
		{
			SSD1306_DrawPixel(i, y0, color);
		}

		/* Return from function */
		return;
	}

	while(1)
	{
		SSD1306_DrawPixel(x0, y0, color);
		if (x0 == x1 && y0 == y1) {break;}
		e2 = err;
		if (e2 > -dx)
		{
			err -= dy;
			x0 += sx;
		}
		if (e2 < dy)
		{
			err += dx;
			y0 += sy;
		}
	}
}




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
void SSD1306::SSD1306_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t color)
{
	// Check input parameters
	if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) {return;}

	// Check width and height
	if ((x + w) >= SSD1306_WIDTH) {w = SSD1306_WIDTH - x;}
	if ((y + h) >= SSD1306_HEIGHT) {h = SSD1306_HEIGHT - y;}

	// Draw 4 lines
	SSD1306_DrawLine(x, y, x + w, y, color);         // Top line
	SSD1306_DrawLine(x, y + h, x + w, y + h, color); // Bottom line
	SSD1306_DrawLine(x, y, x, y + h, color);         // Left line
	SSD1306_DrawLine(x + w, y, x + w, y + h, color); // Right line
}

