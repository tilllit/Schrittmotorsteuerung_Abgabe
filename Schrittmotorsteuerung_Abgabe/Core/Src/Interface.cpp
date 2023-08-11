/*
 * Interface.cpp
 *
 *  Created on: Jul 11, 2023
 *      Author: tillkorsmeier
 */


#include "Interface.hpp"
#include "SSD1306.hpp"
#include "fonts.h"


// ###	Display object	###

SSD1306 Display;



// ###	C++ functions	###

void DisplayInitCpp()
{
	Display.SSD1306_Init();
	Display.SSD1306_Clear();
	Display.SSD1306_UpdateScreen();
}

void DisplayWriteCpp(char payload[])
{
	Display.SSD1306_Clear();
	Display.SSD1306_GotoXY(0, 5);
	Display.SSD1306_String(payload, &Font_7x10, SSD1306_COLOR_WHITE);
	Display.SSD1306_UpdateScreen();
}

void DisplayStatusCpp(int ON, int DIR, int SPEED, int STPMODE)
{
	Display.SSD1306_Clear();

	Display.SSD1306_GotoXY(0, 5);
	string payload = "ON/OFF: " + std::to_string(ON);
	Display.SSD1306_String(payload, &Font_7x10, SSD1306_COLOR_WHITE);

	Display.SSD1306_GotoXY(0, 20);
	payload = "Direction: " + std::to_string(DIR);
	Display.SSD1306_String(payload, &Font_7x10, SSD1306_COLOR_WHITE);

	Display.SSD1306_GotoXY(0, 35);
	payload = "STP MODE: " + std::to_string(STPMODE);
	Display.SSD1306_String(payload, &Font_7x10, SSD1306_COLOR_WHITE);

	Display.SSD1306_GotoXY(0, 50);
	payload = "SPEED: " + std::to_string(SPEED);
	Display.SSD1306_String(payload, &Font_7x10, SSD1306_COLOR_WHITE);

	Display.SSD1306_UpdateScreen();
}



// ###	C functions called in main.c	###

extern "C"
{
    void DisplayInit()
    {
    	DisplayInitCpp();
    }

    void DisplayWrite(char payload[])
    {
    	DisplayWriteCpp(payload);
    }

    void DisplayStatus(int ON, int DIR, int SPEED, int STPMODE)
    {
    	DisplayStatusCpp(ON, DIR, SPEED, STPMODE);
    }
}
