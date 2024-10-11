#ifndef MSP432_I2C_H_
#define MSP432_I2C_H_

#include<msp432p401r.h>
#include<stdint.h>
#include <assert.h>
#include <stdio.h>
#include<stdbool.h>
#include <i2c.h>
#include <string.h>
#include"MSP432.h"



/* The following macro's are critical to the functionality of the
 * 1602 display with the attached I2C Module
 */

#define LCD_ADDR    0x27
#define i2c_Port    P1
#define scl_Bit     BIT7
#define sda_Bit     BIT6

#define clearDisplayLCD         0x01
#define returnHomeLCD           0x02
#define entryModeSetLCD         0x04
#define displayControlLCD       0x08
#define cursorShiftLCD          0x10
#define functionSetLCD          0x20
#define setCGRAM_Addr_LCD       0x40
#define setDDRAM_Addr_LCD       0x80
// Flags for display entry mode
#define entryRightLCD           0x00
#define entryLeftLCD            0x02
#define entryShiftIncrementLCD  0x01
#define entryShiftDecrementLCD  0x00
// Flags for display control
#define displayOnLCD            0x04
#define displayOffLCD           0x00
#define cursorOnLCD             0x02
#define cursorOffLCD            0x00
#define blinkOnLCD              0x01
#define blinkOffLCD             0x00
// FLags for function set
#define lcd8BitMode             0x10
#define lcd4BitMode             0x00
#define lcd2Line                0x08
#define lcd1Line                0x00
#define lcd5x10Dots             0x04
#define lcd5x8Dots              0x00
// Backlight Control
#define backlightLCD            0x08
#define nobacklightLCD          0x00
// I2C Recipient address
#define lcdAddress              0x27
#define lcdFirstRow             0x80
#define lcdSecondRow            0xC0
#define lcdMaxMessageSize       16

extern const eUSCI_I2C_MasterConfig configI2C;

void initializeI2C(uint32_t moduleInstance, const eUSCI_I2C_MasterConfig *config);
void startI2C();
void enableI2C();
void disableI2C();
void setDestinationAddressI2C(uint_fast16_t address);
void setModeI2C(uint_fast8_t mode);
void i2cWrite(uint8_t data);
int initializeLCD();
void lcdExpanderWrite(uint8_t data);
void lcdWrite4Bits(uint8_t value);
void lcdPulseEnable(uint8_t data);
void lcdSend(uint8_t value, uint8_t mode);
void lcdWrite(const char *message);

#endif /* MSP432_I2C_H_ */
