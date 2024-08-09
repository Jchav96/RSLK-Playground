/*
 * MSP432.c
 *
 *  Created on: Jul 4, 2024
 *      Author: jchav
 */
#include "MSP432.h"
#include <msp.h>
#include "RSLK_Pins.h"
#include <stdbool.h>
static uint32_t global_freq; // global variable to store clock frequency
void setupTimerA(Timer_A_Type *timerA, uint16_t period,uint16_t ccr1, uint16_t ccr2){
    // setting the period for timer A
    timerA->CCR[0] = period;
    // setting the capture / compare registers
    timerA->CCR[1] = ccr1;
    timerA->CCR[2] = ccr2;
        //setting the output modes for the capture / compare registers
    int i = 1;
    for(i; i < 3; i++){
        timerA->CCTL[i] = TIMER_A_CCTLN_OUTMOD_7;
    }
    // configuring the control settings for Timer A
    timerA->CTL = TIMER_A_CTL_SSEL__SMCLK   |  // setting the clock source as SMCLK
                   TIMER_A_CTL_MC__UP   |      // setting the timer to up mode
                   TIMER_A_CTL_CLR;            // Clear the TAR to start the timer
}

void setupClock(uint16_t freq){
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
    // Unlock the clock signal module for register access
    CS->KEY = CS_KEY_VAL;
    if(freq <= 1500000){
       CS->CTL0 = CS_CTL0_DCORSEL_0; // 1.5 MHz
    }
    else if(freq <= 3000000){
       CS->CTL0 = CS_CTL0_DCORSEL_1; // 3 MHz
    }
    else if(freq <= 6000000){
       CS->CTL0 = CS_CTL0_DCORSEL_2; // 6 MHz
    }
    else if(freq <= 12000000){
       CS->CTL0 = CS_CTL0_DCORSEL_3; // 12 MHz
    }
    else if(freq <= 24000000){
       CS->CTL0 = CS_CTL0_DCORSEL_4; // 24 MHz
    }
    else{
       CS->CTL0 = CS_CTL0_DCORSEL_5; // 48 MHz
    }
    // Select DCO as the clock source for MCLK, HSMCLK, and SMCLK
        CS->CTL1 = CS_CTL1_SELM__DCOCLK | CS_CTL1_DIVM__1 |
                   CS_CTL1_SELS__DCOCLK | CS_CTL1_DIVHS__1 |
                   CS_CTL1_SELS__DCOCLK | CS_CTL1_DIVS__1;
        // Lock CS registers
        CS->KEY = 0;
        global_freq = freq;
}

void delay(uint32_t milliseconds){
    uint16_t cycles = (global_freq/1000)*milliseconds;
    while(cycles > 0){
        __delay_cycles(1);
        cycles--;
    }
}

void ledWrite(char color, bool state){
    LED_Port->OUT |= Off; // Set the Color Pin to LOW
    // start by reseting the LED
    LED_Port->OUT |= (Red | Green | Blue| Off);
    while(1){
        if(state){
            LED_Port->OUT &= color; // Set the Color pin to HIGH
            break;
        }
        else{
            LED_Port->OUT &= Off; // Set the Color Pin to LOW
            break;
        }
    }
}

bool redLED(){
    if(0){
    RED_LED_PORT ->OUT |= RED_LED_Pin; // Set the RED LED to HIGH
    }
    else{
    RED_LED_PORT ->OUT &= ~RED_LED_Pin; // Set the RED LED to LOW
    }
}

void ledSetupGPIO(){
     RED_LED_PORT ->DIR |= RED_LED_Pin; // Set the RED LED Pin as an output (P1.0)
     // Set the variable LED (RGB) port and its respective pins as outputs
     LED_Port ->DIR |= (Off|   Red  |   Green   |   Blue) ;
     // Clear all onboard LED's
     LED_Port ->OUT &= ~(Off    |   Red  |   Green   |   Blue);
     // Clear all secondary function pins
     LED_Port->SEL0 &= ~(Off    |   Red  |   Green   |   Blue);
     LED_Port->SEL1 &= ~(Off    |   Red  |   Green   |   Blue);
     // Setting the RGB LED's to a default HIGH Signal (Off, active low LED's)
     LED_Port ->OUT |= (Off     |    Red  |   Green   |   Blue) ;
     ledWrite(Off,1);
     // Enable the internal resistors on the LED Pins
     //LED_Port ->REN |= ( Red  |   Green   |   Blue ) ;
     // Setting the pull down resistors for the LED Pins
     //LED_Port ->OUT &= ~(Red   |   Green   |   Blue);
}
/*
void initializeUART(){
    // Put eUSCI in reset mode for configuration
    EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST;
    // Configure UART control and setting
    EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST   |   EUSCI_A_CTLW0_SSEL__SMCLK   |   EUSCI_A_CTLW0_MODE_0;
    //EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SSEL__SMCLK;
    //EUSCI_A0->BRW = 26; // Baud rate 3*10^7 / 115200 = 26
    EUSCI_A0->BRW = 0; // Baud rate of 9600
    EUSCI_A0->MCTLW = 0;
    // Configure P1.2 and P1.3 for UART
    uartPort->SEL0 |= uartTransmitPin | uartReceivePin;
    uartPort->SEL1 &= ~(uartTransmitPin | uartReceivePin);
    // take UART out of reset mode
    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;
    EUSCI_A0->IE &= ~EUSCI_A_IE_RXIE;
}
*/
/*
void initializeUART(uint32_t baudRate){
    // Ensure the eUSCI_A0 module is in reset mode
    EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST;
    // Set the clock source to SMCLK, and UART mode
    EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SSEL__SMCLK    |   EUSCI_A_CTLW0_MODE_0;
    // Calculate and set Baud rate settings
    uint32_t clockFreq = CS->CTL0 & CS_CTL0_DCORSEL_MASK;
    uint32_t prescalar = clockFreq / baudRate;
    uint32_t brwValue = prescalar;
    EUSCI_A0->BRW = brwValue;
    // Disable Modulation
    EUSCI_A0->MCTLW = 0;
    // set P1.2 and P1.3 for UART functionality
    uartPort->SEL0 |= uartTransmitPin | uartReceivePin;
    uartPort->SEL1 &= ~(uartTransmitPin | uartReceivePin);
    // take UART out of reset mode
    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;
}
*/
void initializeUART(){
    EUSCI_A0->CTLW0 |= 1;
    EUSCI_A0->MCTLW = 0;
    EUSCI_A0->CTLW0 = 0x0081;
    EUSCI_A0->BRW = 321;
    P1->SEL0 |= 0x0C;
    P1->SEL1 &= ~0x0C;
    EUSCI_A0->CTLW0 &= ~1;
}
void serialWrite(const char *input){
    while(*input){
        // wait for the transmission buffer to be ready
        while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
        // Send the input character by character
        EUSCI_A0->TXBUF =*input++;
    }
}
void uartWrite(char data){
   while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));// Wait for transmit buffer to clear
   EUSCI_A0->TXBUF = data;
}

void uartPrint(char *message){
    while(*message){
        uartWrite(*message++);
    }
}
