/*
 * MSP432_Buttons.c
 *
 *  Created on: Jul 20, 2024
 *      Author: jchav
 */
#include<MSP432_Buttons.h>
#include "RSLK_Pins.h"
#include <stdbool.h>
#include<msp.h>
// Definitions for button instances
buttons s1;
buttons s2;

buttons s1 = {.initialize = initializeS1 , .pressed = isS1Pressed};
buttons s2 = {.initialize = initializeS2 , .pressed = isS2Pressed};


static void initializeS1(){
    // set s1 as an input
    pushbutton_Port->DIR &= ~s1_Pin;
    // enable internal resistor
    pushbutton_Port->REN |= s1_Pin;
    // set internal resistor
    pushbutton_Port->OUT |= s1_Pin;
    }

static void initializeS2(){
        // set s2 as an input
        pushbutton_Port->DIR &= ~s2_Pin;
        // enable internal resistor
        pushbutton_Port->REN |= s2_Pin;
        // set internal resistor
        pushbutton_Port->OUT |= s2_Pin;
}

static bool isS1Pressed(){
    while((pushbutton_Port->IN & s1_Pin)){
        delay(500);
        if((pushbutton_Port->IN & s1_Pin)){
            return false;
        }
        return(!(pushbutton_Port->IN & s1_Pin));
    }
    return false;
}

static bool isS2Pressed(){
    while((pushbutton_Port->IN & s2_Pin)){
        delay(500);
        if((pushbutton_Port->IN & s2_Pin)){
            return false;
        }
        return(!(pushbutton_Port->IN & s2_Pin));
    }
    return false;
}





