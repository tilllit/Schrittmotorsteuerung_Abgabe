/*
 * Interface.hpp
 *
 *  Created on: Jul 11, 2023
 *      Author: tillkorsmeier
 */

#ifndef INC_INTERFACE_HPP_
#define INC_INTERFACE_HPP_


#include <stdint.h>
#include <stdio.h>
#include <string.h>



// ###	C++ functions	###

void DisplayInitCpp();
void DisplayWriteCpp(char payload[]);
void DisplayStatusCpp(int ON, int DIR, int SPEED, int STPMODE);



// ###	C functions called in main.c	###

#ifdef __cplusplus
extern "C"
{
#endif

    void DisplayInit();
    void DisplayWrite(char payload[]);
    void DisplayStatus(int ON, int DIR, int SPEED, int STPMODE);

#ifdef __cplusplus
}
#endif


#endif /* INC_INTERFACE_HPP_ */
