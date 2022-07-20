/******************************************************************************
SparkFun_Alphanumeric_Display.cpp
SparkFun Alphanumeric Display Library Source File


Orignal Code is by Priyanka Makin @ SparkFun Electronics for Arduino. 
See README.md for details.

Current File is for Raspberry Pi Pico.

This file implements all functions of the HT16K33 class. Functions here range
from printing to one or more Alphanumeric Displays, changing the display settings, and writing/
reading the RAM of the HT16K33.

The Holtek HT16K33 seems to be susceptible to address changes intra-sketch. The ADR pins
are muxed with the ROW and COM drivers so as semgents are turned on/off that affect
the ADR1/ADR0 pins the address has been seen to change. The best way around this is
to do a isConnected check before updateRAM() is sent to the driver IC.

Development environment specifics:
	Hardware Platform: Raspberry Pi Pico
	Alphanumeric Display Breakout Version: 1.0.0


Distributed as-is; no warranty is given.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
******************************************************************************/

/* -- Includes -- */
#include "SparkFun_Alphanumeric_Display_Pico.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include <cstring>


/** @brief In case an error is detected blink the LED on the board.
 *  @return void input character
 */
void blinky(void) {
    #ifndef PICO_DEFAULT_LED_PIN
    #warning blink example requires a board with a regular LED
    #else
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    while (true) {
        gpio_put(LED_PIN, 1);
        sleep_ms(250);
        gpio_put(LED_PIN, 0);
        sleep_ms(250);
    }
    #endif
}




/*--------------------------- Character Map ----------------------------------*/
#define SFE_ALPHANUM_UNKNOWN_CHAR 95


//This is the lookup table of segments for various characters
static const uint16_t alphanumeric_segs[96] = {
	// nmlkjihgfedcba
	0b00000000000000, // ' ' (space)
	0b00001000001000, // '!'
	0b00001000000010, // '"'
 	0b01001101001110, // '#'
	0b01001101101101, // '$'
	0b10010000100100, // '%'
	0b00110011011001, // '&'
	0b00001000000000, // '''
	0b00000000111001, // '('
	0b00000000001111, // ')'
	0b11111010000000, // '*'
	0b01001101000000, // '+'
	0b10000000000000, // ','
	0b00000101000000, // '-'
	0b00000000000000, // '.'
	0b10010000000000, // '/'
	0b00000000111111, // '0'
	0b00010000000110, // '1'
	0b00000101011011, // '2'
	0b00000101001111, // '3'
	0b00000101100110, // '4'
	0b00000101101101, // '5'
	0b00000101111101, // '6'
	0b01010000000001, // '7'
	0b00000101111111, // '8'
	0b00000101100111, // '9'
	0b00000000000000, // ':'
	0b10001000000000, // ';'
	0b00110000000000, // '<'
	0b00000101001000, // '='
	0b01000010000000, // '>'
    0b01000100000011, // '?'
	0b00001100111011, // '@'
	0b00000101110111, // 'A'
	0b01001100001111, // 'B'
	0b00000000111001, // 'C'
	0b01001000001111, // 'D'
	0b00000101111001, // 'E'
	0b00000101110001, // 'F'
	0b00000100111101, // 'G'
	0b00000101110110, // 'H'
	0b01001000001001, // 'I'
	0b00000000011110, // 'J'
	0b00110001110000, // 'K'
	0b00000000111000, // 'L'
	0b00010010110110, // 'M'
	0b00100010110110, // 'N'
	0b00000000111111, // 'O'
	0b00000101110011, // 'P'
	0b00100000111111, // 'Q'
	0b00100101110011, // 'R'
	0b00000110001101, // 'S'
	0b01001000000001, // 'T'
	0b00000000111110, // 'U'
	0b10010000110000, // 'V'
	0b10100000110110, // 'W'
	0b10110010000000, // 'X'
	0b01010010000000, // 'Y'
	0b10010000001001, // 'Z'
	0b00000000111001, // '['
	0b00100010000000, // '\'
	0b00000000001111, // ']'
    0b10100000000000, // '^'
	0b00000000001000, // '_'
	0b00000010000000, // '`'
	0b00000101011111, // 'a'
	0b00100001111000, // 'b'
	0b00000101011000, // 'c'
	0b10000100001110, // 'd'
	0b00000001111001, // 'e'
	0b00000001110001, // 'f'
	0b00000110001111, // 'g'
	0b00000101110100, // 'h'
	0b01000000000000, // 'i'
	0b00000000001110, // 'j'
	0b01111000000000, // 'k'
	0b01001000000000, // 'l'
	0b01000101010100, // 'm'
	0b00100001010000, // 'n'
	0b00000101011100, // 'o'
	0b00010001110001, // 'p'
	0b00100101100011, // 'q'
	0b00000001010000, // 'r'
	0b00000110001101, // 's'
	0b00000001111000, // 't'
	0b00000000011100, // 'u'
	0b10000000010000, // 'v'
	0b10100000010100, // 'w'
	0b10110010000000, // 'x'
	0b00001100001110, // 'y'
	0b10010000001001, // 'z'
	0b10000011001001, // '{'
	0b01001000000000, // '|'
	0b00110100001001, // '}'
	0b00000101010010, // '~'
	0b11111111111111, // Unknown character (DEL or RUBOUT)
};



/*--------------------------- Device Status----------------------------------*/


// I2C reserves some addresses for special purposes. We exclude these from the scan.
// These are any addresses of the form 000 0xxx or 111 1xxx
 bool HT16K33::reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

// check if an I2c device is present
bool HT16K33::isI2CdevicePresent(int addr)
{
    bool devicePresent = false;
    int ret;
    uint8_t rxdata;
    
	if (reserved_addr(addr) == false)
    {
       // Perform a 1-byte dummy read from the probe address. If a slave
       // acknowledges this address, the function returns the number of bytes
       // transferred. If the address byte is ignored, the function returns
       // -1.

	   ret = i2c_read_blocking(this->i2c, addr, &rxdata, 1, false);
    
       devicePresent = (ret < 0) ? false : true; 
    }
    
	return devicePresent;
}

// Check that the display is responding on the I2C bus
// The Holtek IC sometimes fails to respond. This attempts multiple times before giving up.
bool HT16K33::isConnected(uint8_t displayNumber)
{
	uint8_t triesBeforeGiveup = 5;

	uint8_t deviceAddress = this->lookUpDisplayAddress(displayNumber);
	if(deviceAddress == DEFAULT_NOTHING_ATTACHED)
	{

#if ALPHA_DEBUG_ON_SERIAL
    printf("isConnected : nothign attached  %d n", displayNumber);
#endif // ALPHA_DEBUG_ON_SERIAL

		blinky();
		return false;
	}

	for (uint8_t x = 0; x < triesBeforeGiveup; x++)
	{
	
		if(this->isI2CdevicePresent(deviceAddress))
		{
			return true;
		}
		sleep_ms(100);
	}

#if ALPHA_DEBUG_ON_SERIAL
    printf("isConnected : display Not connected %d n", displayNumber);
#endif // ALPHA_DEBUG_ON_SERIAL

	blinky();

	return false;
}


bool HT16K33::begin(uint8_t addressDisplayOne, 
                    uint8_t addressDisplayTwo,
					uint8_t addressDisplayThree, 
					uint8_t addressDisplayFour, 
					i2c_inst_t* i2c
)
{
	_deviceAddressDisplayOne = addressDisplayOne;				// grab the address of the alphanumeric
	_deviceAddressDisplayTwo = addressDisplayTwo;   
	_deviceAddressDisplayThree = addressDisplayThree; 
	_deviceAddressDisplayFour = addressDisplayFour;				

	if (_deviceAddressDisplayFour != DEFAULT_NOTHING_ATTACHED)
		numberOfDisplays = 4;
	else if (_deviceAddressDisplayThree != DEFAULT_NOTHING_ATTACHED)
		numberOfDisplays = 3;
	else if (_deviceAddressDisplayTwo != DEFAULT_NOTHING_ATTACHED)
		numberOfDisplays = 2;
	else
		numberOfDisplays = 1;

    this->i2c = i2c;
	//_i2cPort = &wirePort; // Remember the user's setting

	for (uint8_t i = 1; i <= numberOfDisplays; i++)
	{
		
		if (isConnected(i) == false)
		{
			return false;
		}
		this->displayBrightNess[i] = 6; //  Not too bright
		// if (checkDeviceID(i) == false)
		// {
		// 	Serial.println(i);
		// 	Serial.println("Hello, I've failed checkDeviceID()");
		// 	return false;
		// }
		// Data sheet says wait for 100 milliseconds
		sleep_ms(100);
	}

	if (initialize() == false)
	{
		return false;
	}

	if (clear() == false) // Clear all displays
	{
		return false;
	}

	displayContent[4 * 4] = '\0'; // Terminate the array because we are doing direct prints

	return true;
}


// Returns displayI2C Address 
uint8_t HT16K33::lookUpDisplayAddress(uint8_t displayNumber)
{
    uint8_t addr = DEFAULT_NOTHING_ATTACHED;
    switch(displayNumber)
    {
        case 1 :
		{
	        addr = this->_deviceAddressDisplayOne;
		    break;
	    }
	    case 2 :
		{
		    addr = this->_deviceAddressDisplayTwo;
		    break;
	    }
	    case 3 :
		{
		    addr = this->_deviceAddressDisplayThree;
		    break;
	    }
	    case 4 :
		{
		    addr = this->_deviceAddressDisplayFour;
		    break;
	    }
	    default :
		    break;

   }
   return addr;

}

// Run through initialization procedure for each display connected on the bus
bool HT16K33::initialize()
{
	// Turn on system clock of all displays
	if (enableSystemClock() == false)
	{
		return false;
	}

#if ALPHA_DEBUG_ON_SERIAL
    printf("initialize : enableSystemClock done\n");
#endif // ALPHA_DEBUG_ON_SERIAL

	// Set brightness of all displays to full brightness
	if (setBrightness(15) == false)
	{
		return false;
	}

#if ALPHA_DEBUG_ON_SERIAL
    printf("initialize : setBrightness done\n");
#endif // ALPHA_DEBUG_ON_SERIAL

    

	// Turn blinking off for all displays
	if (setBlinkRate(ALPHA_BLINK_RATE_NOBLINK) == false)
	{
		return false;
	}


#if ALPHA_DEBUG_ON_SERIAL
    printf("initialize : setBlinkRate\n");
#endif // ALPHA_DEBUG_ON_SERIAL

    
	// Turn on all displays
	if (displayOn() == false)
	{
		return false;
	}

#if ALPHA_DEBUG_ON_SERIAL
    printf("initialize : displayOn\n");
#endif // ALPHA_DEBUG_ON_SERIAL

	
	return true;
}

// Turn a single alphanumeric display on
bool HT16K33::displayOnSingle(uint8_t displayNumber)
{
	return setDisplayOnOff(displayNumber, true);
}

// Turn a single alphanumeric display off
bool HT16K33::displayOffSingle(uint8_t displayNumber)
{
	return setDisplayOnOff(displayNumber, false);
}

// Set or clear the display on/off bit of a given display number
bool HT16K33::setDisplayOnOff(uint8_t displayNumber, bool turnOnDisplay)
{
	if (turnOnDisplay) {
		displayOnOff = ALPHA_DISPLAY_ON;
	}
	else {
		displayOnOff = ALPHA_DISPLAY_OFF;
	}

	uint8_t dataToWrite = ALPHA_CMD_DISPLAY_SETUP | (blinkRate << 1) | displayOnOff;
	return (writeCommand(lookUpDisplayAddress(displayNumber), dataToWrite));
}

// Turn on all displays on the I2C bus
bool HT16K33::displayOn()
{
	bool status = true;

	displayOnOff = ALPHA_DISPLAY_ON;

	for (uint8_t i = 1; i <= numberOfDisplays; i++)
	{
		if (displayOnSingle(i) == false)
			status = false;
	}

	return status;
}

// Turn off all displays on the I2C bus
bool HT16K33::displayOff()
{
	bool status = true;

	displayOnOff = ALPHA_DISPLAY_OFF;

	for (uint8_t i = 1; i <= numberOfDisplays; i++)
	{
		if (displayOffSingle(i) == false)
			status = false;
	}

	return status;
}

// Turn on the system oscillator for all displays on the I2C bus
bool HT16K33::enableSystemClock()
{
	bool status = true;
	for (uint8_t i = 1; i <= numberOfDisplays; i++)
	{
		if (enableSystemClockSingle(i) == false)
			status = false;
	}
	return status;
}

// Turn off the system oscillator for all displays on the bus
bool HT16K33::disableSystemClock()
{
	bool status = true;
	for (uint8_t i = 1; i <= numberOfDisplays; i++)
	{
		if (disableSystemClockSingle(i) == false)
			status = false;
	}
	return status;
}

// Turn on the system oscillator for normal operation mode
bool HT16K33::enableSystemClockSingle(uint8_t displayNumber)
{
	uint8_t dataToWrite = ALPHA_CMD_SYSTEM_SETUP | 1; // Enable system clock

	bool status = writeCommand(lookUpDisplayAddress(displayNumber), dataToWrite);
	sleep_ms(1); // Allow display to start
	return (status);
}

// Turn off the system oscillator for standby mode
bool HT16K33::disableSystemClockSingle(uint8_t displayNumber)
{
	uint8_t dataToWrite = ALPHA_CMD_SYSTEM_SETUP | 0; // Standby mode

	return (writeCommand(lookUpDisplayAddress(displayNumber), dataToWrite));
}

// This function sets the brightness of all displays on the bus.
// Duty cycle valid between 0 (off) and 15 (full brightness)
bool HT16K33::setBrightness(uint8_t duty)
{
	bool status = true;
	for (uint8_t i = 1; i <= numberOfDisplays; i++)
	{
		if (setBrightnessSingle(i, duty) == false)
			status = false;
	}
	return status;
}

// Set the brightness of a single display
// Duty cycle valid between 0 (off) and 15 (full brightness)
bool HT16K33::setBrightnessSingle(uint8_t displayNumber, uint8_t duty)
{
	if (duty > 15) // Error check
		duty = 15; 
	else if (duty < 0)
		duty = 0;

    this->displayBrightNess[displayNumber] = duty; 
	uint8_t dataToWrite = ALPHA_CMD_DIMMING_SETUP | duty;
	return (writeCommand(lookUpDisplayAddress(displayNumber), dataToWrite));
}


// Set the blink rate of all displays on the bus
// Parameter "rate" in Hz
// Valid options for "rate" are defined by datasheet: 2.0, 1.0, or 0.5 Hz
// Any other input to this function will result in steady alphanumeric display
bool HT16K33::setBlinkRate(float rate)
{
	bool status = true;
	for (uint8_t i = 1; i <= numberOfDisplays; i++)
	{
		if (setBlinkRateSingle(i, rate) == false)
			status = false;
	}
	return status;
}

// Set the blink rate of a single display on the bus
// Parameter "rate" is in Hz
// Valid options for "rate" are defined by datasheet: 2.0, 1.0, or 0.5 Hz
// Any other input to this function will result in steady alphanumeric display
bool HT16K33::setBlinkRateSingle(uint8_t displayNumber, float rate)
{
	if (rate == 2.0)
	{
		blinkRate = ALPHA_BLINK_RATE_2HZ;
	}
	else if (rate == 1.0)
	{
		blinkRate = ALPHA_BLINK_RATE_1HZ;
	}
	else if (rate == 0.5)
	{
		blinkRate = ALPHA_BLINK_RATE_0_5HZ;
	}
	//default to no blink
	else
	{
		blinkRate = ALPHA_BLINK_RATE_NOBLINK;
	}

	uint8_t dataToWrite = ALPHA_CMD_DISPLAY_SETUP | (blinkRate << 1) | displayOnOff;
	return (writeCommand(lookUpDisplayAddress(displayNumber), dataToWrite));
}

// Send a command to the display.
// The address of the data to write is contained in the first four bits of dataToWrite
bool HT16K33::writeCommand(uint8_t address, uint8_t reg)
{
   
   	
	int i2cWriteRetVal = i2c_write_blocking(this->i2c, 
	                                        address,
											(uint8_t*)(&reg),
											1, 
											false);

    if(i2cWriteRetVal == PICO_ERROR_GENERIC)
	{
		return false;
	}

	return true;
}

// Write the contents of the RAM array out to the Holtek IC
bool HT16K33::writeDataRam( uint8_t displayNumber )
{
    uint8_t buffer[17];
   
    buffer[0] = 0; // Address is 0;

    memcpy( (uint8_t *)(&buffer[1]), 
	       (uint8_t *)(displayRAM + ((displayNumber-1) * 16)),
	       16);
    size_t buffSize =  17;

    int i2cWriteRetVal =  PICO_ERROR_GENERIC;
    uint8_t address = lookUpDisplayAddress(displayNumber);

    if(this->isConnected(displayNumber) == false)
	{
		return false;
	}

    i2cWriteRetVal = i2c_write_blocking(this->i2c, 
                                        address,
									    buffer,
                                        buffSize ,
									    false);

    if(i2cWriteRetVal == PICO_ERROR_GENERIC)
	{
		return false;
	}

	return true;

}




// Push the contents of displayRAM out to the various displays in 16 byte chunks
bool HT16K33::updateDisplay()
{

	bool status = true;

	for (uint8_t i = 1; i <= numberOfDisplays; i++)
	{
		if(writeDataRam(i) == false)
		{
			status = false;
		}
	}

	return status;
}

// Turn off all segments of all displays connected to bus
bool HT16K33::clear()
{
	// Clear the displayRAM array
	for (uint8_t i = 0; i < 16 * numberOfDisplays; i++)
	{
		displayRAM[i] = 0;
	}

	digitPosition = 0;
 
	return (updateDisplay());
}




// Turn the decimal point on for a single display
bool HT16K33::decimalOnSingle(uint8_t displayNumber)
{
	return setDecimalOnOff(displayNumber, true);
}

// Turn the decimal point off for a single display
bool HT16K33::decimalOffSingle(uint8_t displayNumber)
{
	return setDecimalOnOff(displayNumber, false);
}

// Set or clear the decimal on/off bit
bool HT16K33::setDecimalOnOff(uint8_t displayNumber, bool turnOnDecimal)
{
	uint8_t adr = 0x03;
	uint8_t dat;

	if (turnOnDecimal == true)
	{
		decimalOnOff = ALPHA_DECIMAL_ON;
		dat = 0x01;
	}
	else
	{
		decimalOnOff = ALPHA_DECIMAL_OFF;
		dat = 0x00;
	}

	displayRAM[adr + (displayNumber - 1) * 16] = displayRAM[adr + (displayNumber - 1) * 16] | dat;
	return (updateDisplay());
}

// Turn the decimal on for all displays on bus
bool HT16K33::decimalOn()
{
	bool status = true;

	decimalOnOff = ALPHA_DECIMAL_ON;

	for (uint8_t i = 1; i <= numberOfDisplays; i++)
	{
		if (decimalOnSingle(i) == false)
			status = false;
	}

	return status;
}

// Turn the decimal point off for all displays on bus
bool HT16K33::decimalOff()
{
	bool status = true;

	decimalOnOff = ALPHA_DECIMAL_OFF;

	for (uint8_t i = 1; i <= numberOfDisplays; i++)
	{
		if (decimalOffSingle(i) == false)
			status = false;
	}
	return status;
}

// Turn the colon on for a single display
bool HT16K33::colonOnSingle(uint8_t displayNumber)
{
	return setColonOnOff(displayNumber, true);
}

// Turn the colon off for a single display
bool HT16K33::colonOffSingle(uint8_t displayNumber)
{
	return setColonOnOff(displayNumber, false);
}

// Set or clear the colon on/off bit
bool HT16K33::setColonOnOff(uint8_t displayNumber, bool turnOnColon)
{
	uint8_t adr = 0x01;
	uint8_t dat;

	if (turnOnColon == true)
	{
		colonOnOff = ALPHA_COLON_ON;
		dat = 0x01;
	}
	else
	{
		colonOnOff = ALPHA_COLON_OFF;
		dat = 0x00;
	}

	displayRAM[adr + (displayNumber - 1) * 16] = displayRAM[adr + (displayNumber - 1) * 16] | dat;
	return (updateDisplay());
}

// Turn the colon on for all displays on the bus
bool HT16K33::colonOn()
{
	bool status = true;

	colonOnOff = ALPHA_COLON_ON;

	for (uint8_t i = 1; i <= numberOfDisplays; i++)
	{
		if (colonOnSingle(i) == false)
			status = false;
	}
	return status;
}

// Turn the colon off for all displays on the bus
bool HT16K33::colonOff()
{
	bool status = true;

	colonOnOff = ALPHA_COLON_OFF;

	for (uint8_t i = 1; i <= numberOfDisplays; i++)
	{
		if (colonOffSingle(i) == false)
			status = false;
	}
	return status;
}

/*---------------------------- Light up functions ---------------------------------*/

// Given a segment and a digit, set the matching bit within the RAM of the Holtek RAM set
void HT16K33::illuminateSegment(char segment, uint8_t digit)
{
	uint8_t com;
	uint8_t row;

	com = (uint8_t)((char)(segment) - 'A'); // Convert the segment letter back to a number

	if (com > 6)
		com -= 7;
	if (segment == 'I')
		com = 0;
	if (segment == 'H')
		com = 1;

	row = digit % 4; // Convert digit (1 to 16) back to a relative position on a given digit on display
	if (segment > 'G')
		row += 4;

	uint8_t offset = ((uint8_t)(digit / 4)) * 16;
	uint8_t adr = com * 2 + offset;

	// Determine the address
	if (row > 7)
		adr++;

	// Determine the data bit
	if (row > 7)
		row -= 8;
	//uint8_t dat = 1 << row;
    uint8_t content = this->displayRAM[adr];

	content = content | (uint8_t)(0x1<<row);
	this->displayRAM[adr] = content;

}

// Clears a segment and a digit, 
// clear the matching bit within the RAM of the Holtek RAM set
void HT16K33::clearSegment(char segment, uint8_t digit)
{
	uint8_t com;
	uint8_t row;

	com = (uint8_t)((char)(segment) - 'A'); // Convert the segment letter back to a number

	if (com > 6)
		com -= 7;
	if (segment == 'I')
		com = 0;
	if (segment == 'H')
		com = 1;

	row = digit % 4; // Convert digit (1 to 16) back to a relative position on a given digit on display
	if (segment > 'G')
		row += 4;

	uint8_t offset = ((uint8_t)(digit / 4)) * 16;
	uint8_t adr = com * 2 + offset;

	// Determine the address
	if (row > 7)
		adr++;

	// Determine the data bit
	if (row > 7)
		row -= 8;
    
	uint8_t content = this->displayRAM[adr];

	content = content & (~(uint8_t)(0x1<<row));
	this->displayRAM[adr] = content;

}

// Given a binary set of segments and a digit, store this data into the RAM array
void HT16K33::illuminateChar(uint16_t segmentsToTurnOn, uint8_t digit)
{
	for (uint8_t i = 0; i < 14; i++) // There are 14 segments on this display
	{
		if ((segmentsToTurnOn >> i) & 0b1)
			illuminateSegment((char)('A' + i), digit); // Convert the segment number to a letter
	}
}

// Print a character, for a given digit, on display
void HT16K33::printChar(uint8_t displayChar, uint8_t digit)
{
	// moved alphanumeric_segs array to PROGMEM
	uint16_t characterPosition = 65532;

	// space
	if (displayChar == ' ')
		characterPosition = 0;
	// Printable Symbols -- Between first character ! and last character ~
	else if (displayChar >= '!' && displayChar <= '~')
	{
		characterPosition = displayChar - '!' + 1;
	}

	uint8_t dispNum = digitPosition / 4;

	// Take care of special characters by turning correct segment on
	if (characterPosition == 14) // '.'
		decimalOnSingle(dispNum+1);
	if (characterPosition == 26) // ':'
		colonOnSingle(dispNum+1);
	if (characterPosition == 65532) // unknown character
		characterPosition = SFE_ALPHANUM_UNKNOWN_CHAR;

	uint16_t segmentsToTurnOn = getSegmentsToTurnOn(characterPosition);
	illuminateChar(segmentsToTurnOn, digit);
}

// Get the character map from the definition list or default table
uint16_t HT16K33::getSegmentsToTurnOn(uint8_t charPos)
{
    uint16_t segments = alphanumeric_segs[charPos];
    return segments;
}

/*
 * Write a byte to the display.
 * Required for overloading the Print function.
 */
size_t HT16K33::write(uint8_t b)
{
	// If user wants to print '.' or ':', don't increment the digitPosition!
	if (b == '.' | b == ':')
		printChar(b, 0);
	else
	{
		printChar(b, digitPosition++);
		digitPosition %= (numberOfDisplays * 4); // Convert displays to number of digits
	}

	return (updateDisplay()); // Send RAM buffer over I2C bus
}

/*
 * Write a character buffer to the display.
 * Required for overloading the Print function.
 */
size_t HT16K33::write(const uint8_t *buffer, size_t size)
{
	size_t n = size;
	uint8_t buff;

	// Clear the displayRAM array
	for (uint8_t i = 0; i < 16 * numberOfDisplays; i++)
		displayRAM[i] = 0;

	digitPosition = 0;

	while (size--)
	{
		buff = *buffer++;
		// For special characters like '.' or ':', do not increment the digitPosition
		if (buff == '.')
			printChar('.', 0);
		else if (buff == ':')
			printChar(':', 0);
		else
		{
			printChar(buff, digitPosition);
			displayContent[digitPosition] = buff; // Record to internal array

			digitPosition++;
			digitPosition %= (numberOfDisplays * 4);
		}
	}

	updateDisplay(); // Send RAM buffer over I2C bus

	return n;
}



/*
 * Write a character buffer to the display.
 * Required for overloading the Print function.
 */
size_t HT16K33::writeLongBuffer(uint8_t *buffer, size_t size)
{
	size_t n = size;
	uint8_t count = 0;
	uint8_t *buff = buffer;
	uint8_t buffSize = 0;
	size_t displaySize = numberOfDisplays * 4;

	
	while(size)
	{
		buffSize = size > displaySize ? displaySize : size;
		this->write(buff, buffSize);
		buff++;
		size--;
		sleep_ms(250);
	}
    return 0;

}



/*
 * Write a string to the display.
 * Required for overloading the Print function.
 */
size_t HT16K33::write(const char *str)
{
	if (str == NULL)
		return 0;
	return write((const uint8_t *)str, strlen(str));
}



