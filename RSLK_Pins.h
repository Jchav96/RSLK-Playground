/*
 * RSLK_Pins.h
 *
 *  Created on: Jul 6, 2024
 *      Author: jchav
 */

#ifndef RSLK_PINS_H_
#define RSLK_PINS_H_
#include <msp432p401r.h>
//#include "msp432.h"
/* RSLK Left and Right Motor Pins between the MSP432P401R and
 * Polulu Motor Chasis
 */

/*
 * The following macro's are for the left motor
 */
// Left Motor Enabling
#define leftMotorEnablePort         P3
#define leftMotorEnablePin          BIT7
// left Motor Direction
#define leftMotorDirectionPort      P5
#define leftMotorDirectionPin       BIT4
// left Motor PWM
#define leftMotorPWM_Port           P2
#define leftMotorPWM_Pin            BIT7
/*
 *  The following macro's are for the right motor
 */
// right motor enabling
#define rightMotorEnablePort        P3
#define rightMotorEnablePin         BIT6
// right motor direction
#define rightMotorDirectionPort     P5
#define rightMotorDirectionPin      BIT5
// right motor PWM
#define rightMotorPWM_Port          P2
#define rightMotorPWM_Pin           BIT6

/*
 * The following macro's are for the onboard LED's on the
 * TI MSP-432P401R development board
 */
#define RED_LED_PORT                P1  // this LED can ONLY illuminate RED
#define RED_LED_Pin                 BIT0

#define LED_Port                    P2

#define Red                         BIT0
#define Green                       BIT1
#define Blue                        BIT2
#define Off                         0x00
/*
#define Off                         0x00
#define Red                         0x01
#define Green                       0x02
#define Blue                        0x04
*/
// MSP432P401R push buttons
#define pushbutton_Port             P1
#define s1_Pin                      BIT1
#define s2_Pin                      BIT4

// MSP432P401R UART Definitions
#define uartPort                    P1
#define uartReceivePin              BIT2
#define uartTransmitPin             BIT3

// Base address for eUSCI_A1 Peripheral
#define EUSCI_A1_BASE_ADDRESS       ((EUSCI_A_Type*)EUSCI_A1_BASE)
// Defining constants from memory map
#define EUSCI_A_CTLW0_MODE_0        ((uint16_t)0x0000)
#define EUSCI_A_CTLW0_MODE_1        ((uint16_t)0x0200)
#define EUSCI_A_CTLW0_MODE_2        ((uint26_t)0x0400)
#define EUSCI_A_CTLW0_MODE_3        ((uint26_t)0x0600)

#define BSL_UART_INTERFACE          ((uint16_t)0x0000C0000)
#define BSL_CONFIG_INTERFACE_UART   ((uint16_t)0x0000C000)
// Defining the base address
#define PERIPH_BASE                 ((uint32_t)0x40000000)
#define EUSCI_A0_BASE               (PERIPH_BASE + 0x00001000)

// Needed Timer definitions for UART Purposes

// Output mode register for timer A
#define TIMER_A_CCTLN_OUTMOD_7      (0x00E0)
//SMCLK register for timer A
#define TIMER_A_CTL_SSEL__SMCLK     (0x0200)
// Up mode register
#define TIMER_A_CTL_MC__UP          (0x0010)
// Clear TAR Register
#define TIMER_A_CTL_CLR             (0x0004)
// Access to CS registers
#define CS_KEY_VAL                  (0x695A)
// DCO range select: 1.5 MHz
#define CS_CTL0_DCORSEL_0           (0x0000)
// DCO range select: 3 MHz
#define CS_CTL0_DCORSEL_1           (0x1000)
// DCO range select: 6 MHz
#define CS_CTL0_DCORSEL_2           (0x2000)
// DCO range select: 12 MHz
#define CS_CTL0_DCORSEL_3           (0x3000)
// DCO range select: 24 MHz
#define CS_CTL0_DCORSEL_4           (0x4000)
// DCO range select: 48 MHz
#define CS_CTL0_DCORSEL_5           (0x5000)
// MCLK divider: /1
#define CS_CTL1_DIVM__1             (0x0000)
// DCO clock source for SMCLK
#define CS_CTL1_SELS__DCOCLK        (0x0006)
// HSMCLK divider: /1
#define CS_CTL1_DIVHS__1            (0x0000)
// SMCLK divider: /1
#define CS_CTL1_DIVS__1             (0x0000)
// DCO Clock source for MCLK
#define CS_CTL1_SELM__DCOCLK        (0x0006)
// DCO Clock source for SMCLK
#define CS_CTL1_SELS_DCOCLK         (0x0006)
#endif /* RSLK_PINS_H_ */
