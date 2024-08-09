#include "RSLK_MOTOR.h"
#include "RSLK_Pins.h"
#include<msp.h>
// Definitions for motor instances
Motor leftMotor;
Motor rightMotor;
Motor bothMotors;

Motor leftMotor = {.enable = enableLeftMotor, .disable = disableLeftMotor, .setDirection= setLeftMotorDirection, .setSpeed = setLeftMotorSpeed};
Motor rightMotor = {.enable = enableRightMotor, .disable = disableRightMotor, .setDirection = setRightMotorDirection, .setSpeed = setRightMotorSpeed};
Motor bothMotors = {.enable = enableBothMotors, .disable = disableBothMotors, .setDirection = setBothMotorsDirection, .setSpeed = setBothMotorsSpeed};
// Function implementations for motor methods
void enableLeftMotor() {
    leftMotorEnablePort->OUT |= leftMotorEnablePin; // set the pin to HIGH to enable the motor
}

void enableRightMotor(){
    rightMotorEnablePort->OUT |= rightMotorEnablePin; // set the pin to HIGH to enable the motor
}

void enableBothMotors(){
    leftMotorEnablePort->OUT |= (leftMotorEnablePin && rightMotorEnablePin);
}

void disableLeftMotor(){
    leftMotorEnablePort->OUT &= ~leftMotorEnablePin; // set the pin to LOW to disable the motor
}

void disableRightMotor(){
    rightMotorEnablePort->OUT &= ~rightMotorEnablePin; // set the pin to LOW to disable the motor
}

void disableBothMotors(){
    leftMotorEnablePort->OUT &= ~(leftMotorEnablePin && rightMotorEnablePin); // set the pins to LOW to disable the motors
}

void setLeftMotorDirection(char direction){
    if(direction == 'Forward'){
        leftMotorDirectionPort->OUT &= leftMotorDirectionPin; // set the direction pin to High to move forward
    }
    else if(direction == 'Backward'){
        leftMotorDirectionPort->OUT &= ~leftMotorDirectionPin; // set the direction pin to Low to move backward
    }
}

void setRightMotorDirection(char direction){
    if(direction == 'F'){
        leftMotorDirectionPort->OUT &= rightMotorDirectionPin; // set the direction pin to High to move forward
    }
    else if(direction == 'B'){
        leftMotorDirectionPort->OUT &= ~rightMotorDirectionPin; // set the direction pin to Low to move backward
    }
}

void setBothMotorsDirection(char direction){
    //leftMotorDirectionPort->OUT |= 0;
    if(direction == 'F'){
        leftMotorDirectionPort->OUT &= (leftMotorDirectionPin && rightMotorDirectionPin); // set the direction pins to High to move forward
    }
    else if(direction == 'B'){
        rightMotorDirectionPort->OUT |= ~(leftMotorDirectionPin && rightMotorDirectionPin); // set the direction pins to LOW to move backward
    }
}

void setLeftMotorSpeed(unsigned int speed){
    TIMER_A0->CCR[3] = (uint16_t)(speed * 255 / 100);
}

void setRightMotorSpeed(unsigned int speed){
    TIMER_A0->CCR[3] = (uint16_t)(speed * 255 / 100);
}

void setBothMotorsSpeed(unsigned int speed){
    TIMER_A0->CCR[3] = (uint16_t)(speed * 255 / 100);
    TIMER_A0->CCR[4] = (uint16_t)(speed * 255 / 100);
}

void setupMotorGPIO(){
    // set the left motor enable as an output
   leftMotorEnablePort ->DIR |= leftMotorEnablePin;
   // set the left motor direction as an output
   leftMotorDirectionPort ->DIR |= leftMotorDirectionPin;
   // set the left motor PWM as an output
   leftMotorPWM_Port ->DIR |= leftMotorPWM_Pin;

   // set the right motor enable as an output
   rightMotorEnablePort ->DIR |= rightMotorEnablePin;
   // set the right motor direction as an output
   rightMotorDirectionPort ->DIR |= rightMotorDirectionPin;
   // set the right motor PWM as an output
   rightMotorPWM_Port ->DIR |= rightMotorPWM_Pin;

   // set the PWM Pins for the secondary module function
   leftMotorPWM_Port->SEL0 |= leftMotorPWM_Pin;
   leftMotorPWM_Port->SEL1 &= ~leftMotorPWM_Pin;
   rightMotorPWM_Port->SEL0 |= rightMotorPWM_Pin;
   rightMotorPWM_Port->SEL1 &= ~rightMotorPWM_Pin;
   // configuting timer A0 for PWM
   TIMER_A0->CCR[0]=1000-1;
   TIMER_A0->CCTL[3]=TIMER_A_CCTLN_OUTMOD_7;
   TIMER_A0->CCTL[4]=TIMER_A_CCTLN_OUTMOD_7;
   TIMER_A0->CCR[3]=0;
   TIMER_A0->CCR[4]=0;
   TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_MC__UP | TIMER_A_CTL_CLR;
}


