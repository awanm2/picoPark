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

#ifndef __BME280_H__
#define __BME280_H__

#include <stdint.h>
#include "hardware/i2c.h"

#define BME280_CHIP_ID  0x60

#define I2C_MODE 0
#define SPI_MODE 1

#ifndef BME280_SPI_CLOCK
#ifdef ARDUINO_ARCH_ESP32
#define BME280_SPI_CLOCK 1000000
#else
#define BME280_SPI_CLOCK 500000
#endif
#endif

#ifndef BME280_SPI_MODE
#define BME280_SPI_MODE SPI_MODE0
#endif

#define NO_WIRE 0
#define HARD_WIRE 1
#define SOFT_WIRE 2

#define MODE_SLEEP 0b00
#define MODE_FORCED 0b01
#define MODE_NORMAL 0b11

//Register names:
#define BME280_DIG_T1_LSB_REG			0x88
#define BME280_DIG_T1_MSB_REG			0x89
#define BME280_DIG_T2_LSB_REG			0x8A
#define BME280_DIG_T2_MSB_REG			0x8B
#define BME280_DIG_T3_LSB_REG			0x8C
#define BME280_DIG_T3_MSB_REG			0x8D
#define BME280_DIG_P1_LSB_REG			0x8E
#define BME280_DIG_P1_MSB_REG			0x8F
#define BME280_DIG_P2_LSB_REG			0x90
#define BME280_DIG_P2_MSB_REG			0x91
#define BME280_DIG_P3_LSB_REG			0x92
#define BME280_DIG_P3_MSB_REG			0x93
#define BME280_DIG_P4_LSB_REG			0x94
#define BME280_DIG_P4_MSB_REG			0x95
#define BME280_DIG_P5_LSB_REG			0x96
#define BME280_DIG_P5_MSB_REG			0x97
#define BME280_DIG_P6_LSB_REG			0x98
#define BME280_DIG_P6_MSB_REG			0x99
#define BME280_DIG_P7_LSB_REG			0x9A
#define BME280_DIG_P7_MSB_REG			0x9B
#define BME280_DIG_P8_LSB_REG			0x9C
#define BME280_DIG_P8_MSB_REG			0x9D
#define BME280_DIG_P9_LSB_REG			0x9E
#define BME280_DIG_P9_MSB_REG			0x9F
#define BME280_DIG_H1_REG				0xA1
#define BME280_CHIP_ID_REG				0xD0 //Chip ID
#define BME280_RST_REG					0xE0 //Softreset Reg
#define BME280_DIG_H2_LSB_REG			0xE1
#define BME280_DIG_H2_MSB_REG			0xE2
#define BME280_DIG_H3_REG				0xE3
#define BME280_DIG_H4_MSB_REG			0xE4
#define BME280_DIG_H4_LSB_REG			0xE5
#define BME280_DIG_H5_MSB_REG			0xE6
#define BME280_DIG_H6_REG				0xE7
#define BME280_CTRL_HUMIDITY_REG		0xF2 //Ctrl Humidity Reg
#define BME280_STAT_REG					0xF3 //Status Reg
#define BME280_CTRL_MEAS_REG			0xF4 //Ctrl Measure Reg
#define BME280_CONFIG_REG				0xF5 //Configuration Reg
#define BME280_MEASUREMENTS_REG			0xF7 //Measurements register start
#define BME280_PRESSURE_MSB_REG			0xF7 //Pressure MSB
#define BME280_PRESSURE_LSB_REG			0xF8 //Pressure LSB
#define BME280_PRESSURE_XLSB_REG		0xF9 //Pressure XLSB
#define BME280_TEMPERATURE_MSB_REG		0xFA //Temperature MSB
#define BME280_TEMPERATURE_LSB_REG		0xFB //Temperature LSB
#define BME280_TEMPERATURE_XLSB_REG		0xFC //Temperature XLSB
#define BME280_HUMIDITY_MSB_REG			0xFD //Humidity MSB
#define BME280_HUMIDITY_LSB_REG			0xFE //Humidity LSB

//Class BME280_SensorSettings.  This object is used to hold settings data.  The application
//uses this classes' data directly.  The settings are adopted and sent to the sensor
//at special times, such as .begin.  Some are used for doing math.
//
//This is a kind of bloated way to do this.  The trade-off is that the user doesn't
//need to deal with #defines or enums with bizarre names.
//
//A power user would strip out BME280_SensorSettings entirely, and send specific read and
//write command directly to the IC. (ST #defines below)
//
struct BME280_SensorSettings
{
  public:
	
	//Main Interface and mode settings
    uint8_t commInterface;
    uint8_t I2CAddress;
    uint8_t chipSelectPin;

	//Deprecated settings
	uint8_t runMode;
	uint8_t tStandby;
	uint8_t filter;
	uint8_t tempOverSample;
	uint8_t pressOverSample;
	uint8_t humidOverSample;
    float tempCorrection; // correction of temperature - added to the result
};

//Used to hold the calibration constants.  These are used
//by the driver as measurements are being taking
struct SensorCalibration
{
  public:
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;
	
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;
	
	uint8_t dig_H1;
	int16_t dig_H2;
	uint8_t dig_H3;
	int16_t dig_H4;
	int16_t dig_H5;
	int8_t dig_H6;
	
};

struct BME280_SensorMeasurements
{
  public:
	float temperature;
	float pressure;
	float humidity;
};

//This is the main operational class of the driver.

class BME280
{
  private:
	i2c_inst_t* i2c;
	float _referencePressure = 101325.0; //Default but is changeable

    void setI2C(i2c_inst_t* i2c);
	uint8_t init(void); 
	void print_uart(char* str);
	uint8_t checkSampleValue(uint8_t userValue); //Checks for valid over sample values
    void readTempCFromBurst(uint8_t buffer[], BME280_SensorMeasurements *measurements);
	void readTempFFromBurst(uint8_t buffer[], BME280_SensorMeasurements *measurements);

  public:
    BME280( void );
    
	//settings
    
	
	BME280_SensorSettings settings;
	SensorCalibration calibration;
	int32_t t_fine;
    
	void error_led_blink(char* str);

	// I2C port 
	
    // I2C related functions
    bool isI2CdevicePresent(void);
    bool isConnectedI2C(void);


	// SPI: Not supported - will return false
	bool isConnectedSPI(void);

 /** @brief Illuminate the segement on a given digit.
  *  @param offset Register offset to be read.
  *  @param digit  Digit for current function.
  *  @return uin8_t Value read from the register.
 */   
    uint8_t readRegister(uint8_t offset);
	void writeRegister(uint8_t offset, uint8_t dataToWrite);
    void readRegisterRegion(uint8_t*, uint8_t, uint8_t );

	
    bool isBME280Ready(i2c_inst_t* i2c); // calls the init routines

	
	uint8_t getMode(void); //Get the current mode: sleep, forced, or normal
	void setMode(uint8_t mode); //Set the current mode

	void setTempOverSample(uint8_t overSampleAmount); //Set the temperature sample mode
	void setPressureOverSample(uint8_t overSampleAmount); //Set the pressure sample mode
	void setHumidityOverSample(uint8_t overSampleAmount); //Set the humidity sample mode
	void setStandbyTime(uint8_t timeSetting); //Set the standby time between measurements
	void setFilter(uint8_t filterSetting); //Set the filter
	
	void setI2CAddress(uint8_t i2caddress); //Set the address the library should use to communicate. Use if address jumper is closed (0x76).

	void setReferencePressure(float refPressure); //Allows user to set local sea level reference pressure
	float getReferencePressure();
	
	bool isMeasuring(void); //Returns true while the device is taking measurement
	
	//Software reset routine
	void reset( void );
	void readAllMeasurements(BME280_SensorMeasurements *measurements, uint8_t tempScale = 0);
	
    //Returns the values as floats.
    float readFloatPressure( void );
	float readFloatAltitudeMeters( void );
	float readFloatAltitudeFeet( void );
	void readFloatPressureFromBurst(uint8_t buffer[], BME280_SensorMeasurements *measurements);
	
	float readFloatHumidity( void );
	void readFloatHumidityFromBurst(uint8_t buffer[], BME280_SensorMeasurements *measurements);

    //Temperature related methods
    void setTemperatureCorrection(float corr);
    float readTempC( void );
    float readTempF( void );
	float readTempFromBurst(uint8_t buffer[]);

	//Dewpoint related methods
	//From Pavel-Sayekat: https://github.com/sparkfun/SparkFun_BME280_Breakout_Board/pull/6/files
    double dewPointC(void);
    double dewPointF(void);
	
};

#endif  // End of __BME280_H__ definition check
