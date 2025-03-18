#include "MSP432_I2C.h"

static uint8_t displayMode;
static uint8_t backlightVal;



/* The follwing is from the MSP432 datasheet provided by TI
 * "Timer32 is an ARM dual 32-bit timer module. It contains two 32-bit timers, each of which can be
configured as two independent 16-bit timers. The two timers can generate independent events or a
combined event, which can be processed according to application requirements. Timer32 runs out of the
same clock as the Cortex-M4 CPU"
 * As such, timer 32 is clearly ideal for I2C communication on the MSP432P401R.
 */

// The following will contain the master configuration for I2C communication
const eUSCI_I2C_MasterConfig configI2C =
{
     EUSCI_B_I2C_CLOCKSOURCE_SMCLK,             // Use SMCLK as the Clock source
     3000000,                                   // Set the clock frequency accordingly (needed for prescalar calculation)
     EUSCI_B_I2C_SET_DATA_RATE_100KBPS,         // Set the I2C clock to the desired frequency
     0,                                         // Set the byte counter threshold as desired
     EUSCI_B_I2C_NO_AUTO_STOP                   // Set the autostop as desired (on or off (See declaration for more info))
};

void initializeI2C(uint32_t moduleInstance, const eUSCI_I2C_MasterConfig *config){
    uint32_t prescalar = (uint32_t)(config->i2cClk / config->dataRate);
    // Setting 1.6 and 1.7 as outputs
    i2c_Port->DIR |= (scl_Bit | sda_Bit);
    // Enabling the internal pull up resistors
    i2c_Port->REN |= (scl_Bit |sda_Bit);
    // Setting the internal resistors
    i2c_Port->OUT |= (scl_Bit | sda_Bit);
    i2c_Port->SEL0 |= (scl_Bit | sda_Bit);
    i2c_Port->SEL1 &= ~(scl_Bit | sda_Bit);
    // Disable the I2C during configuration
    //EUSCI_B0->CTLW0 |= UCSWRST;
    // Configure the I2C clock source and set the module to master mode
    EUSCI_B0->CTLW0 |= UCSWRST;
    // Configure the auto stop condition based on the configurarion
    EUSCI_B0->CTLW1 = (EUSCI_B0->CTLW1 & UCASTP_M) | config->autoSTOPGeneration;
    // Set the byte count threshold (optional, based on individual needs)
    EUSCI_B0->TBCNT = config->byteCounterThreshold;
    // Configure the I2C module as master
    EUSCI_B0->CTLW0 = (EUSCI_B0->CTLW0 & ~UCSSEL_M) |
                      (config->selectClockSource | UCMST | UCMODE_3 | UCSYNC | UCSWRST);
    // set the prescalar value in the baud rate register
    EUSCI_B0->BRW = prescalar;
    //EUSCI_B0->BRW = 30;
    // Ensure the interrupt flags are cleared before any communication
    EUSCI_B0->IFG &= ~(EUSCI_B_IFG_TXIFG0 | EUSCI_B_IFG_RXIFG0 | EUSCI_B_IFG_NACKIFG);
    // take EUSCI out of reset state
    EUSCI_B0->CTLW0 &= ~ UCSWRST;
    // Setting the i2c Address
    uartWrite("Setting i2c address \n");
    setDestinationAddressI2C(lcdAddress);
    uartWrite("i2c address set \n");
    // Setting the I2C in transmission mode
    uartWrite("setting i2c in transmit mode \n");
    setModeI2C(EUSCI_B_I2C_TRANSMIT_MODE);
    uartWrite("Transmit Mode Set! \n");
}


void startI2C(){
    uartWrite("Starting I2C \n");
    // Set the I2C in transmit mode
    EUSCI_B0->CTLW0 |= UCTR;  // Set UCTR
    EUSCI_B0->CTLW0 |= UCTXSTT;  // Set UCTXSTT
    uartWrite("Starting I2C...  \n");
    uartWrite("Press S1 to continue \n");
    while(!s1.pressed()){}
    uartWrite("I2C Started! \n");
}



void enableI2C(){
    // Clear the UCSWRST (bit0) of the control register
    EUSCI_B0->CTLW0 &= ~UCSWRST;
}

void disableI2C(){
    EUSCI_B0->CTLW0 |= UCSWRST;
}

void setDestinationAddressI2C(uint_fast16_t address){
    // set the address for I2C communication
    EUSCI_B0->I2CSA = address;
}

void setModeI2C(uint_fast8_t mode){
    EUSCI_B0->CTLW0 = (EUSCI_B0->CTLW0 & ~UCTR) | mode;
}

void i2cWrite(uint8_t data){
    // Load data to be transmitted
    EUSCI_B0->TXBUF = data;  
}

// The following code's are critical to proper LCD functionality

/* The following functions will return either 1's or 0's as flags
 * in order to indicate if the execution of said function succeeded or failed
 * which will be useful for error detection and correction
 */

int initializeLCD(){
    // Turn on the LCD backlight by default
    backlightVal = backlightLCD;
    // Initialize the display settings (4 bit, 2 line, 5x8 dots)
    uint8_t displaySettings = lcd4BitMode | lcd2Line | lcd5x8Dots;
    // As the LCD will take a min 40 ms timeframe to power up...
    delay(50);
    // write the backlight state
    uartWrite("Writing backlight state \n");
    lcdExpanderWrite(backlightVal);
    uartWrite("Backlight state written \n");
    uartWrite("setting the LCD into 4 bit mode \n");
    lcdWrite4Bits(0x03 << 4);
    delay(5);
    uartWrite("LCD set into 4 bit mode \n");
    // Set the 4 bit interface by sending 0x02
    uartWrite("setting the 4 bit interface \n");
    lcdWrite4Bits(0x02 << 4);
    uartWrite("4 bit interface set \n");
    // Now set up the function (number of lines and font size)
    uartWrite("setting up the number of lines and font size \n");
    lcdSend(functionSetLCD | displaySettings , 0);
    uartWrite("nunber of lines and font size set \n");
    // Turn the display on with no cursor or blinking
    uartWrite("turning on the display with no cursor or blinking \n");
    uint16_t displayControl = (displayOnLCD | cursorOffLCD | blinkOffLCD);
    uartWrite("display turned on with no cursor or blinking \n");
    lcdSend(displayControlLCD | displayControl, 0);
    // Clear the display
    lcdSend(clearDisplayLCD,0);
    delay(2);
    uartWrite("testing \n");
    // Set the text by default left to right
    displayMode = entryLeftLCD | entryShiftDecrementLCD;
    lcdSend(entryModeSetLCD | displayMode, 0);
    // set the lcd cursor to the first position
    lcdSend(0x80, 0);
    uartWrite("lcd initialized!");
    return 1;
}

// The following function writes the backlight state to the 1602 I2C expander
void lcdExpanderWrite(uint8_t data){
    i2cWrite(data | backlightVal);
}

// The following function writes  4 bit command to the LCD
void lcdWrite4Bits(uint8_t value){
    // Send the upper 4 bits
    lcdExpanderWrite(value);
    // Pulse enable pin to latch data
    lcdPulseEnable(value);
}

// The following function enables the transmission of serial data to the LCD display
void lcdPulseEnable(uint8_t data){
    // Enable HIGH
    lcdExpanderWrite(data | 0x04);
    delay(1);
    // enable LOW
    lcdExpanderWrite(data & ~0x04);
    delay(1);
}

void lcdSend(uint8_t value, uint8_t mode){
    // Upper 4 bits
    uint8_t highNibble = value & 0xF0;
    // Lower 4 bits
    uint8_t lowNibble = (value << 4) & 0xF0;
    // Send upper 4 bits with mode
    lcdWrite4Bits(highNibble | mode);
    // Send lower 4 bits with mode
    lcdWrite4Bits(lowNibble | mode);
}

void lcdWrite(const char *message){
    // variable to hold message currently on 1602
    static char previousMessage[lcdMaxMessageSize+1] = "";
    // if there was a previous message move it downward
    if(strlen(previousMessage)>0){
        // If there was any previous message first clear the display...
        lcdSend(clearDisplayLCD,0);
        lcdSend(lcdSecondRow,0);
        int i;
        for(i = 0; i < strlen(previousMessage) && i <= lcdMaxMessageSize;i++){
            lcdSend(previousMessage[i],1);
        }
    }
    // Now save the current message as the previous message
    strncpy(previousMessage,message,lcdMaxMessageSize);
    previousMessage[lcdMaxMessageSize] = '\0';
    lcdSend(0x80,0);
    int j;
    for(j = 0; j <= strlen(message) && j <= lcdMaxMessageSize; j++){
        lcdSend(message[j],1);
        if(j == strlen(message)){
            uartWrite("Message fully written!");
        }
    }
}
