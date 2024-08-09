/*
 * MSP432_Buttons.h
 *
 *  Created on: Jul 20, 2024
 *      Author: jchav
 */

#ifndef MSP432_BUTTONS_H_
#define MSP432_BUTTONS_H_
#include <stdbool.h>
#include<msp.h>
typedef struct{
    int buttonPort;
    int buttonPin;
    // constructors for controlling S1 and S2
    bool(*pressed)();
    void(*initialize)();
}buttons;
extern buttons s1;
extern buttons s2;

bool isS1Pressed();
bool isS2Pressed();
void initializeS1();
void initializeS2();

#endif /* MSP432_BUTTONS_H_ */
