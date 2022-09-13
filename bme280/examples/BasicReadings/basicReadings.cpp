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
 * 
 * Serial port : 
 *   Mini com can be run using
 *   sudo minicom -b 115200 -D /dev/ttyUSB0 

 * 
 * ******************************************************************************/


#include "SparkFun_Bme280_Pico.h"

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"

using namespace std;


int main(void)
{

    stdio_init_all();

    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    BME280 bme280;
   
    if(bme280.isBME280Ready(i2c_default) == false)
    {
        printf("BME 280 : DEVICE INIT FAILED\n ");
        sleep_ms(5000);
        while(1); // Freeeze
    }


    while(true)
    {
        while(bme280.isMeasuring()==true);// Wait for the device to take the measurement
        printf(">>>   Sensed Values   <<<\n");
        printf("Humidity: %06.3f \%RH\n", bme280.readFloatHumidity());
        // Pressure is read in Pa - divide by 100 to get hPa.
        printf("Pressure : %06.3f hPa\n",(bme280.readFloatPressure())/100);
        printf("Temperature : %06.3f F\n",bme280.readTempF());
        printf("Temperature : %06.3f C\n",bme280.readTempC());
        printf("---   Calculated Values   ---\n");
        printf("DewPoint : %06.3f F\n",bme280.dewPointF());
        printf("DewPoint : %06.3f C\n",bme280.dewPointC());
        printf("Altitude : %06.3f feet\n",bme280.readFloatAltitudeFeet());
        sleep_ms(250);
    
    }

    return 0; 
}