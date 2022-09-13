/******************************************************************************
 * The orignal code was written by 
 * by Marshall Taylor @ SparkFun Electronics for Arduino.
 * 
 * This file has been ported to work with Raspberry Pi Pico board.
 * 
 * Development environment specifics:
 *     Raspberry Pico
 *
 * This code is released under the [MIT License](http://opensource.org/licenses/MIT).
 * 
 * Please review the LICENSE.md file included with directory for bm280 source code.
 * 
 * Distributed as-is; no warranty is given.
******************************************************************************/

#include <math.h>
#include "SparkFun_Bme280_Pico.h"
#include "pico/stdlib.h"


#define SPARK_FUN_BME_280_DEFAULT_ADDRESS 0x77 
#define BME280_DEBUG_ON_SERIAL 0

void BME280::print_uart(char* str)
{
	#if BME280_DEBUG_ON_SERIAL
       printf(str);   
	#endif   
}

void BME280::error_led_blink(char* str) {
    
	#ifndef PICO_DEFAULT_LED_PIN
    #warning blink example requires a board with a regular LED
    #else
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
	print_uart(str);
    while (true) {
        gpio_put(LED_PIN, 1);
        sleep_ms(250);
        gpio_put(LED_PIN, 0);
        sleep_ms(250);
    }
    #endif
}

/****************************************************************************
*
*  I2C related functions
*
*****************************************************************************/

void BME280::setI2C(i2c_inst_t* i2c)
{
	this->i2c = i2c;
}

// check if an I2c device is present
bool BME280::isI2CdevicePresent(void)
{
    bool devicePresent = false;
    int ret;
    uint8_t rxdata;
    
	// Perform a 1-byte dummy read from the probe address. If a slave
    // acknowledges this address, the function returns the number of bytes
    // transferred. If the address byte is ignored, the function returns
    // -1.

    if(this->i2c == NULL)
	{
		return false;
	}
	ret = i2c_read_blocking(this->i2c, settings.I2CAddress, &rxdata, 1, false);
    
    return ((ret < 0) ? false : true); 
}

// Check that the display is responding on the I2C bus
bool BME280::isConnectedI2C(void)
{
	uint8_t triesBeforeGiveup = 5;

	for (uint8_t x = 0; x < triesBeforeGiveup; x++)
	{
	
		if(this->isI2CdevicePresent())
		{
			return true;
		}
		sleep_ms(100);
	}

	return false;
}


bool BME280::isConnectedSPI(void)
{
	// Have not ported SPI mode Yet.
	// Sparkfun BME 280 supports QWICC mode which is based on I2C. 
	// For now using I2C bus on the QWICC board.
	return false;
}

uint8_t BME280::readRegister(uint8_t offset){
	uint8_t result = 0;
	uint8_t numBytes = 1;
	size_t len = 1;
	bool nonStop = true;

	switch (settings.commInterface) {

	    case I2C_MODE:
		{
	        if(i2c_write_blocking (this->i2c, (uint8_t)(settings.I2CAddress), (uint8_t *)(&offset), len, nonStop) == PICO_ERROR_GENERIC)
	        {
                error_led_blink("BME280::readRegister i2c write failed\n");
	        }
	  
	        if(i2c_read_blocking (this->i2c, (uint8_t)(settings.I2CAddress), (uint8_t *)(&result), len, nonStop) == PICO_ERROR_GENERIC)
	        {
                error_led_blink("BME280::readRegister i2c read failed\n");
	        }
	    }
		
	        break;

	    case SPI_MODE:
	    // Not Yet Supported 
		    break;

	    default:
		    break;
	} 
	return result;
}
	
void BME280::writeRegister(uint8_t offset, uint8_t dataToWrite)
{
	uint8_t result = 0;
	size_t len = 2;
	bool nonStop = true;
	uint8_t data[2];
	data[0] = offset;
	data[1] = dataToWrite;

	switch (settings.commInterface) {

	    case I2C_MODE:
		{
	        if(i2c_write_blocking (this->i2c, (uint8_t)(settings.I2CAddress), (uint8_t *)(data), len, nonStop) == PICO_ERROR_GENERIC)
	        {
                error_led_blink("BME280::writeRegister i2c write failed\n");
	        }
	    }
		
	        break;

	    case SPI_MODE:
	    // Not Yet Supported 
		    break;

	    default:
		    break;
	}
    
}

void BME280::readRegisterRegion(uint8_t *outputPointer , uint8_t offset, uint8_t length)
{
	//define pointer that will point to the external space
	uint8_t i = 0;
	char c = 0;
	bool nonStop = true;
	size_t len = 1;

	switch (settings.commInterface)
	{
        case I2C_MODE:
        {
		    if(i2c_write_blocking (this->i2c, (uint8_t)(settings.I2CAddress), (uint8_t *)(&offset), len, nonStop) == PICO_ERROR_GENERIC)
	        {
                error_led_blink("BME280::readRegisterRegion i2c write failed\n");
	        }
	  
	        if(i2c_read_blocking (this->i2c, (uint8_t)(settings.I2CAddress), (uint8_t *)(outputPointer), length, nonStop) == PICO_ERROR_GENERIC)
	        {
                error_led_blink("BME280::readRegisterRegion i2c read failed\n");
	        }
       }
	   break;

	   case SPI_MODE:
	       break;

	   default:
	       break;
	}

}
	
/****************************************************************************
*
*  Settings and configuration
*
*****************************************************************************/
//Constructor -- Specifies default configuration
BME280::BME280( void )
{
	//Construct with these default settings

	settings.commInterface = I2C_MODE; //Default to I2C

    //Default, jumper open is 0x77
	settings.I2CAddress = SPARK_FUN_BME_280_DEFAULT_ADDRESS;
	
	// When constructing the object, init this to NULL.
	i2c = NULL; 
	
	
	settings.chipSelectPin = 10; //Select CS pin for SPI
	
	//These are deprecated settings
	settings.runMode = 3; //Normal/Run
	settings.tStandby = 0; //0.5ms
	settings.filter = 0; //Filter off
	settings.tempOverSample = 1;
	settings.pressOverSample = 1;
	settings.humidOverSample = 1;
	settings.tempCorrection = 0.f; // correction of temperature - added to the result
}


//****************************************************************************//
//
//  Configuration section
//
//  This uses the stored BME280_SensorSettings to start the IMU
//  Use statements such as "mySensor.settings.commInterface = SPI_MODE;" to 
//  configure before calling .begin();
//
//****************************************************************************//

uint8_t BME280::init(void){
	
	//Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
    sleep_ms(2);
 

    // Init I2C Mode

    if(isConnectedI2C() == false) 
	{
        return (uint8_t)(false);
	}
		
	//Check communication with IC before anything else
	uint8_t chipID = readRegister(BME280_CHIP_ID_REG); //Should return 0x60 or 0x58
	// Is this BMP or BME?
	if(chipID != 0x58 && chipID != 0x60) 
	{
	   return(chipID); //This is not BMP nor BME!
	} 

	//Reading all compensation data, range 0x88:A1, 0xE1:E7
	calibration.dig_T1 = ((uint16_t)((readRegister(BME280_DIG_T1_MSB_REG) << 8) + readRegister(BME280_DIG_T1_LSB_REG)));
	calibration.dig_T2 = ((int16_t)((readRegister(BME280_DIG_T2_MSB_REG) << 8) + readRegister(BME280_DIG_T2_LSB_REG)));
	calibration.dig_T3 = ((int16_t)((readRegister(BME280_DIG_T3_MSB_REG) << 8) + readRegister(BME280_DIG_T3_LSB_REG)));

	calibration.dig_P1 = ((uint16_t)((readRegister(BME280_DIG_P1_MSB_REG) << 8) + readRegister(BME280_DIG_P1_LSB_REG)));
	calibration.dig_P2 = ((int16_t)((readRegister(BME280_DIG_P2_MSB_REG) << 8) + readRegister(BME280_DIG_P2_LSB_REG)));
	calibration.dig_P3 = ((int16_t)((readRegister(BME280_DIG_P3_MSB_REG) << 8) + readRegister(BME280_DIG_P3_LSB_REG)));
	calibration.dig_P4 = ((int16_t)((readRegister(BME280_DIG_P4_MSB_REG) << 8) + readRegister(BME280_DIG_P4_LSB_REG)));
	calibration.dig_P5 = ((int16_t)((readRegister(BME280_DIG_P5_MSB_REG) << 8) + readRegister(BME280_DIG_P5_LSB_REG)));
	calibration.dig_P6 = ((int16_t)((readRegister(BME280_DIG_P6_MSB_REG) << 8) + readRegister(BME280_DIG_P6_LSB_REG)));
	calibration.dig_P7 = ((int16_t)((readRegister(BME280_DIG_P7_MSB_REG) << 8) + readRegister(BME280_DIG_P7_LSB_REG)));
	calibration.dig_P8 = ((int16_t)((readRegister(BME280_DIG_P8_MSB_REG) << 8) + readRegister(BME280_DIG_P8_LSB_REG)));
	calibration.dig_P9 = ((int16_t)((readRegister(BME280_DIG_P9_MSB_REG) << 8) + readRegister(BME280_DIG_P9_LSB_REG)));

	calibration.dig_H1 = ((uint8_t)(readRegister(BME280_DIG_H1_REG)));
	calibration.dig_H2 = ((int16_t)((readRegister(BME280_DIG_H2_MSB_REG) << 8) + readRegister(BME280_DIG_H2_LSB_REG)));
	calibration.dig_H3 = ((uint8_t)(readRegister(BME280_DIG_H3_REG)));
	calibration.dig_H4 = ((int16_t)((readRegister(BME280_DIG_H4_MSB_REG) << 4) + (readRegister(BME280_DIG_H4_LSB_REG) & 0x0F)));
	calibration.dig_H5 = ((int16_t)((readRegister(BME280_DIG_H5_MSB_REG) << 4) + ((readRegister(BME280_DIG_H4_LSB_REG) >> 4) & 0x0F)));
	calibration.dig_H6 = ((int8_t)readRegister(BME280_DIG_H6_REG));

    
	//Most of the time the sensor will be init with default values
	//But in case user has old/deprecated code, use the settings.x values
	setStandbyTime(settings.tStandby);
	setFilter(settings.filter);
	setPressureOverSample(settings.pressOverSample); //Default of 1x oversample
	setHumidityOverSample(settings.humidOverSample); //Default of 1x oversample
	setTempOverSample(settings.tempOverSample); //Default of 1x oversample
	
	setMode(MODE_NORMAL); //Go!

	return(readRegister(BME280_CHIP_ID_REG)); //Should return 0x60
	
}


//Begin comm with BME280 over I2C
bool BME280::isBME280Ready(i2c_inst_t* i2c)
{
	settings.commInterface = I2C_MODE;
    this->setI2C(i2c);

	uint8_t chipID = this->init();

	if(chipID == 0x58) return(true); //Begin normal init with these settings. Should return chip ID of 0x58 for BMP
	if(chipID == 0x60) return(true); //Begin normal init with these settings. Should return chip ID of 0x60 for BME
	return(false);
}


//Set the mode bits in the ctrl_meas register
// Mode 00 = Sleep
// 01 and 10 = Forced
// 11 = Normal mode
void BME280::setMode(uint8_t mode)
{
	if(mode > 0b11) mode = 0; //Error check. Default to sleep mode
	
	uint8_t controlData = readRegister(BME280_CTRL_MEAS_REG);
	controlData &= ~( (1<<1) | (1<<0) ); //Clear the mode[1:0] bits
	controlData |= mode; //Set
	writeRegister(BME280_CTRL_MEAS_REG, controlData);
}

//Gets the current mode bits in the ctrl_meas register
//Mode 00 = Sleep
// 01 and 10 = Forced
// 11 = Normal mode
uint8_t BME280::getMode()
{
	uint8_t controlData = readRegister(BME280_CTRL_MEAS_REG);
	return(controlData & 0b00000011); //Clear bits 7 through 2
}

//Set the standby bits in the config register
//tStandby can be:
//  0, 0.5ms
//  1, 62.5ms
//  2, 125ms
//  3, 250ms
//  4, 500ms
//  5, 1000ms
//  6, 10ms
//  7, 20ms
void BME280::setStandbyTime(uint8_t timeSetting)
{
	if(timeSetting > 0b111) timeSetting = 0; //Error check. Default to 0.5ms
	
	uint8_t controlData = readRegister(BME280_CONFIG_REG);
	controlData &= ~( (1<<7) | (1<<6) | (1<<5) ); //Clear the 7/6/5 bits
	controlData |= (timeSetting << 5); //Align with bits 7/6/5
	writeRegister(BME280_CONFIG_REG, controlData);
}

//Set the filter bits in the config register
//filter can be off or number of FIR coefficients to use:
//  0, filter off
//  1, coefficients = 2
//  2, coefficients = 4
//  3, coefficients = 8
//  4, coefficients = 16
void BME280::setFilter(uint8_t filterSetting)
{
	if(filterSetting > 0b111) filterSetting = 0; //Error check. Default to filter off
	
	uint8_t controlData = readRegister(BME280_CONFIG_REG);
	controlData &= ~( (1<<4) | (1<<3) | (1<<2) ); //Clear the 4/3/2 bits
	controlData |= (filterSetting << 2); //Align with bits 4/3/2
	writeRegister(BME280_CONFIG_REG, controlData);
}

//Set the temperature oversample value
//0 turns off temp sensing
//1 to 16 are valid over sampling values
void BME280::setTempOverSample(uint8_t overSampleAmount)
{
	overSampleAmount = checkSampleValue(overSampleAmount); //Error check
	
	uint8_t originalMode = getMode(); //Get the current mode so we can go back to it at the end
	
	setMode(MODE_SLEEP); //Config will only be writeable in sleep mode, so first go to sleep mode

	//Set the osrs_t bits (7, 6, 5) to overSampleAmount
	uint8_t controlData = readRegister(BME280_CTRL_MEAS_REG);
	controlData &= ~( (1<<7) | (1<<6) | (1<<5) ); //Clear bits 765
	controlData |= overSampleAmount << 5; //Align overSampleAmount to bits 7/6/5
	writeRegister(BME280_CTRL_MEAS_REG, controlData);
	
	setMode(originalMode); //Return to the original user's choice
}

//Set the pressure oversample value
//0 turns off pressure sensing
//1 to 16 are valid over sampling values
void BME280::setPressureOverSample(uint8_t overSampleAmount)
{
	overSampleAmount = checkSampleValue(overSampleAmount); //Error check
	
	uint8_t originalMode = getMode(); //Get the current mode so we can go back to it at the end
	
	setMode(MODE_SLEEP); //Config will only be writeable in sleep mode, so first go to sleep mode

	//Set the osrs_p bits (4, 3, 2) to overSampleAmount
	uint8_t controlData = readRegister(BME280_CTRL_MEAS_REG);
	controlData &= ~( (1<<4) | (1<<3) | (1<<2) ); //Clear bits 432
	controlData |= overSampleAmount << 2; //Align overSampleAmount to bits 4/3/2
	writeRegister(BME280_CTRL_MEAS_REG, controlData);
	
	setMode(originalMode); //Return to the original user's choice
}

//Set the humidity oversample value
//0 turns off humidity sensing
//1 to 16 are valid over sampling values
void BME280::setHumidityOverSample(uint8_t overSampleAmount)
{
	overSampleAmount = checkSampleValue(overSampleAmount); //Error check
	
	uint8_t originalMode = getMode(); //Get the current mode so we can go back to it at the end
	
	setMode(MODE_SLEEP); //Config will only be writeable in sleep mode, so first go to sleep mode

	//Set the osrs_h bits (2, 1, 0) to overSampleAmount
	uint8_t controlData = readRegister(BME280_CTRL_HUMIDITY_REG);
	controlData &= ~( (1<<2) | (1<<1) | (1<<0) ); //Clear bits 2/1/0
	controlData |= overSampleAmount << 0; //Align overSampleAmount to bits 2/1/0
	writeRegister(BME280_CTRL_HUMIDITY_REG, controlData);

	setMode(originalMode); //Return to the original user's choice
}

//Validates an over sample value
//Allowed values are 0 to 16
//These are used in the humidty, pressure, and temp oversample functions
uint8_t BME280::checkSampleValue(uint8_t userValue)
{
	switch(userValue) 
	{
		case(0): 
			return 0;
			break; //Valid
		case(1): 
			return 1;
			break; //Valid
		case(2): 
			return 2;
			break; //Valid
		case(4): 
			return 3;
			break; //Valid
		case(8): 
			return 4;
			break; //Valid
		case(16): 
			return 5;
			break; //Valid
		default: 
			return 1; //Default to 1x
			break; //Good
	}
}

//Set the global setting for the I2C address we want to communicate with
//Default is 0x77
void BME280::setI2CAddress(uint8_t address)
{
	settings.I2CAddress = address; //Set the I2C address for this device
}

//Check the measuring bit and return true while device is taking measurement
bool BME280::isMeasuring(void)
{
	uint8_t stat = readRegister(BME280_STAT_REG);
	return(stat & (1<<3)); //If the measuring bit (3) is set, return true
}

//Strictly resets.  Init afterwards
void BME280::reset( void )
{
	writeRegister(BME280_RST_REG, 0xB6);
	
}

//****************************************************************************//
//
//  Burst Measurement Section
//
//****************************************************************************//

//Read all sensor registers as a burst. See BME280 Datasheet section 4. Data readout
//tempScale = 0 for Celsius scale (default setting)
//tempScale = 1 for Fahrenheit scale


void BME280::readAllMeasurements(BME280_SensorMeasurements *measurements, uint8_t tempScale){
	
	uint8_t dataBurst[8];
	readRegisterRegion(dataBurst, BME280_MEASUREMENTS_REG, 8);
	
	if(tempScale == 0){
		readTempCFromBurst(dataBurst, measurements);
	}else{
		readTempFFromBurst(dataBurst, measurements);
	}
	readFloatPressureFromBurst(dataBurst, measurements);
	readFloatHumidityFromBurst(dataBurst, measurements);
}

//****************************************************************************//
//
//  Pressure Section
//
//****************************************************************************//


float BME280::readFloatPressure( void )
{

	// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
	// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
    uint8_t buffer[3];
	readRegisterRegion(buffer, BME280_PRESSURE_MSB_REG, 3);
    int32_t adc_P = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);
	
	int64_t var1, var2, p_acc;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)calibration.dig_P6;
	var2 = var2 + ((var1 * (int64_t)calibration.dig_P5)<<17);
	var2 = var2 + (((int64_t)calibration.dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)calibration.dig_P3)>>8) + ((var1 * (int64_t)calibration.dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)calibration.dig_P1)>>33;
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p_acc = 1048576 - adc_P;
	p_acc = (((p_acc<<31) - var2)*3125)/var1;
	var1 = (((int64_t)calibration.dig_P9) * (p_acc>>13) * (p_acc>>13)) >> 25;
	var2 = (((int64_t)calibration.dig_P8) * p_acc) >> 19;
	p_acc = ((p_acc + var1 + var2) >> 8) + (((int64_t)calibration.dig_P7)<<4);
	
	return (float)p_acc / 256.0;
	
}

void BME280::readFloatPressureFromBurst(uint8_t buffer[], BME280_SensorMeasurements *measurements)
{

	// Set pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
	// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
  
  int32_t adc_P = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);
	
	int64_t var1, var2, p_acc;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)calibration.dig_P6;
	var2 = var2 + ((var1 * (int64_t)calibration.dig_P5)<<17);
	var2 = var2 + (((int64_t)calibration.dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)calibration.dig_P3)>>8) + ((var1 * (int64_t)calibration.dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)calibration.dig_P1)>>33;
	if (var1 == 0)
	{
		measurements->pressure = 0; // avoid exception caused by division by zero
	}
	else
	{
		p_acc = 1048576 - adc_P;
		p_acc = (((p_acc<<31) - var2)*3125)/var1;
		var1 = (((int64_t)calibration.dig_P9) * (p_acc>>13) * (p_acc>>13)) >> 25;
		var2 = (((int64_t)calibration.dig_P8) * p_acc) >> 19;
		p_acc = ((p_acc + var1 + var2) >> 8) + (((int64_t)calibration.dig_P7)<<4);
		
		measurements->pressure = (float)p_acc / 256.0;
	}
}

// Sets the internal variable _referencePressure so the altitude is calculated properly.
// This is also known as "sea level pressure" and is in Pascals. The value is probably
// within 10% of 101325. This varies based on the weather:
// https://en.wikipedia.org/wiki/Atmospheric_pressure#Mean_sea-level_pressure
//
// if you are concerned about accuracy or precision, make sure to pull the
// "sea level pressure"value from a trusted source like NOAA.



void BME280::setReferencePressure(float refPressure)
{
	_referencePressure = refPressure;
}

//Return the local reference pressure
float BME280::getReferencePressure()
{
	return(_referencePressure);
}

float BME280::readFloatAltitudeMeters( void )
{
	float heightOutput = 0;
	
  // Getting height from a pressure reading is called the "international barometric height formula".
  // The magic value of 44330.77 was adjusted in issue #30.
  // There's also some discussion of it here: https://www.sparkfun.com/tutorials/253
  // This calculation is NOT designed to work on non-Earthlike planets such as Mars or Venus;
  // see NRLMSISE-00. That's why it is the "international" formula, not "interplanetary".
  // Sparkfun is not liable for incorrect altitude calculations from this
  // code on those planets. Interplanetary selfies are welcome, however.
	heightOutput = ((float)-44330.77)*(pow(((float)readFloatPressure()/(float)_referencePressure), 0.190263) - (float)1); //Corrected, see issue 30
	return heightOutput;
	
}

float BME280::readFloatAltitudeFeet( void )
{
	float heightOutput = 0;
	
	heightOutput = readFloatAltitudeMeters() * 3.28084;
	return heightOutput;
	
}

//****************************************************************************//
//
//  Humidity Section
//
//****************************************************************************//


float BME280::readFloatHumidity( void )
{
	
	// Returns humidity in %RH as unsigned 32 bit integer in Q22. 10 format (22 integer and 10 fractional bits).
	// Output value of “47445” represents 47445/1024 = 46. 333 %RH
    uint8_t buffer[2];
	readRegisterRegion(buffer, BME280_HUMIDITY_MSB_REG, 2);
    int32_t adc_H = ((uint32_t)buffer[0] << 8) | ((uint32_t)buffer[1]);
	
	int32_t var1;
	var1 = (t_fine - ((int32_t)76800));
	var1 = (((((adc_H << 14) - (((int32_t)calibration.dig_H4) << 20) - (((int32_t)calibration.dig_H5) * var1)) +
	((int32_t)16384)) >> 15) * (((((((var1 * ((int32_t)calibration.dig_H6)) >> 10) * (((var1 * ((int32_t)calibration.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
	((int32_t)calibration.dig_H2) + 8192) >> 14));
	var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)calibration.dig_H1)) >> 4));
	var1 = (var1 < 0 ? 0 : var1);
	var1 = (var1 > 419430400 ? 419430400 : var1);

	return (float)(var1>>12) / 1024.0;
}


void BME280::readFloatHumidityFromBurst(uint8_t buffer[], BME280_SensorMeasurements *measurements)
{
	
	// Set humidity in %RH as unsigned 32 bit integer in Q22. 10 format (22 integer and 10 fractional bits).
	// Output value of “47445” represents 47445/1024 = 46. 333 %RH
  int32_t adc_H = ((uint32_t)buffer[6] << 8) | ((uint32_t)buffer[7]);
	
	int32_t var1;
	var1 = (t_fine - ((int32_t)76800));
	var1 = (((((adc_H << 14) - (((int32_t)calibration.dig_H4) << 20) - (((int32_t)calibration.dig_H5) * var1)) +
	((int32_t)16384)) >> 15) * (((((((var1 * ((int32_t)calibration.dig_H6)) >> 10) * (((var1 * ((int32_t)calibration.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
	((int32_t)calibration.dig_H2) + 8192) >> 14));
	var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)calibration.dig_H1)) >> 4));
	var1 = (var1 < 0 ? 0 : var1);
	var1 = (var1 > 419430400 ? 419430400 : var1);

	measurements->humidity = (float)(var1>>12) / 1024.0;
}


//****************************************************************************//
//
//  Temperature Section
//
//****************************************************************************//

void BME280::setTemperatureCorrection(float corr)
{
	settings.tempCorrection = corr;
}

float BME280::readTempC( void )
{
	// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
	// t_fine carries fine temperature as global value

	//get the reading (adc_T);
    uint8_t buffer[3];
	readRegisterRegion(buffer, BME280_TEMPERATURE_MSB_REG, 3);
    int32_t adc_T = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);

	//By datasheet, calibrate
	int64_t var1, var2;

	var1 = ((((adc_T>>3) - ((int32_t)calibration.dig_T1<<1))) * ((int32_t)calibration.dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)calibration.dig_T1)) * ((adc_T>>4) - ((int32_t)calibration.dig_T1))) >> 12) *
	((int32_t)calibration.dig_T3)) >> 14;
	t_fine = var1 + var2;
	float output = (t_fine * 5 + 128) >> 8;

	output = output / 100 + settings.tempCorrection;
	
	return output;
}

float BME280::readTempFromBurst(uint8_t buffer[])
{
  int32_t adc_T = ((uint32_t)buffer[3] << 12) | ((uint32_t)buffer[4] << 4) | ((buffer[5] >> 4) & 0x0F);

	//By datasheet, calibrate
	int64_t var1, var2;

	var1 = ((((adc_T>>3) - ((int32_t)calibration.dig_T1<<1))) * ((int32_t)calibration.dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)calibration.dig_T1)) * ((adc_T>>4) - ((int32_t)calibration.dig_T1))) >> 12) *
	((int32_t)calibration.dig_T3)) >> 14;
	t_fine = var1 + var2;
	float output = (t_fine * 5 + 128) >> 8;

	output = output / 100 + settings.tempCorrection;
	
 	return output;
}

void BME280::readTempCFromBurst(uint8_t buffer[], BME280_SensorMeasurements *measurements)
{
  measurements->temperature = readTempFromBurst(buffer);
}

float BME280::readTempF( void )
{
	float output = readTempC();
	output = (output * 9) / 5 + 32;

	return output;
}

void BME280::readTempFFromBurst(uint8_t buffer[], BME280_SensorMeasurements *measurements)
{
  float output = readTempFromBurst(buffer);
	output = (output * 9) / 5 + 32;

	measurements->temperature = output;
}

//****************************************************************************//
//
//  Dew point Section
//
//****************************************************************************//
// Returns Dew point in DegC


double BME280::dewPointC(void)
{
  double celsius = readTempC(); 
  double humidity = readFloatHumidity();
  // (1) Saturation Vapor Pressure = ESGG(T)
  double RATIO = 373.15 / (273.15 + celsius);
  double RHS = -7.90298 * (RATIO - 1);
  RHS += 5.02808 * log10(RATIO);
  RHS += -1.3816e-7 * (pow(10, (11.344 * (1 - 1/RATIO ))) - 1) ;
  RHS += 8.1328e-3 * (pow(10, (-3.49149 * (RATIO - 1))) - 1) ;
  RHS += log10(1013.246);
         // factor -3 is to adjust units - Vapor Pressure SVP * humidity
  double VP = pow(10, RHS - 3) * humidity;
         // (2) DEWPOINT = F(Vapor Pressure)
  double T = log(VP/0.61078);   // temp var
  return (241.88 * T) / (17.558 - T);
}

// Returns Dew point in DegF
double BME280::dewPointF(void)
{
	return(dewPointC() * 1.8 + 32); //Convert C to F
}
