/*
 * RSLK_Motor.h
 *
 *  Created on: Jul 4, 2024
 *      Author: jchav
 */
#include "msp.h"
#include "RSLK_Pins.h"
#ifndef RSLK_MOTOR_H_
#define RSLK_MOTOR_H_

typedef struct {
    int enablePort;
    int enablePin;
    int pwmPort;
    int pwmPin;
    int directionPort;
    int directionPin;
    int PWM_Channel;
    // Constructors for controlling the motors
    void(*enable)(void);
    void(*disable)(void);
    void(*setSpeed)(int speed);
    void(*setDirection)(char direction);
}Motor;
// Function prototype to initialize Motors
void initializeMotors(void);
// Function prototype to setup GPIO Pins for Motors
void setupMotorGPIO(void);
extern Motor leftMotor;
extern Motor rightMotor;
extern Motor bothMotors;

void enableLeftMotor();
void enableRightMotor();
void enableBothMotors();
void disableLeftMotor();
void disableRightMotor();
void disableBothMotors();
void setLeftMotorDirection(char direction);
void setRightMotorDirection(char direction);
void setBothMotorsDirection(char direction);
void setLeftMotorSpeed(unsigned int speed);
void setRightMotorSpeed(unsigned int speed);
void setBothMotorsSpeed(unsigned int speed);
#endif /* RSLK_MOTOR_H_ */
