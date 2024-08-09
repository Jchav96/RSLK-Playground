#include <msp.h>
#include <msp432p401r.h>
#include <stdbool.h>
#include "RSLK_Pins.h"
#ifndef MSP432_H_
#define MSP432_H_
// Function prototype to initialize Timer_A instances
void setupTimerA(Timer_A_Type *timerA, uint16_t period, uint16_t ccr1, uint16_t ccr2);
// Function prototype to set up LED GPIO Pins
void ledSetupGPIO();
bool redLED();
// Function prototype to send HIGH and LOW signals to the onboard LED (RGB)
void ledWrite(char color, bool state);
void setupClock(uint16_t freq);
void delay(uint32_t milliseconds);
#endif /* MSP432_H_ */
