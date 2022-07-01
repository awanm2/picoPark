#include "SparkFun_Alphanumeric_Display_Pico.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include <cstring>


int main(void)
{
    HT16K33 display;

    uint8_t buffer[4];
    buffer[0]='M';
    buffer[1]='I';
    buffer[2]='L';
    buffer[3]='K';
    

    stdio_init_all();

    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));


    if(display.begin(ALPHANUMERIC_ADDRESS_71,DEFAULT_ADDRESS, 
        DEFAULT_NOTHING_ATTACHED , DEFAULT_NOTHING_ATTACHED, i2c_default) == false)
    {
       blinky();
    }
    //display.Animate_3();
    sleep_ms(500);

    static volatile bool runForever = true;
    
    /*
    while(runForever){
    //display.write(buffer,4);
       display.write("SOLIDIGM");
       sleep_ms(250);
       //display.clear();
       display.write("SPOTTED ");
       sleep_ms(250);
       //display.clear();
    }
*/
      //char* str = "   SOLIDIGM THE NEW PARADIGM OF SOLID-STATE STORAGE";
     char* str = "SOMETIMES YOUR JOY IS THE SOURCE OF YOUR SMILE, BUT SOMETIMES YOUR SIMLE CAN BE THE SOURCE OF YOUR JOY";
      while(runForever){

        display.writeLongBuffer( (uint8_t*)str,  strlen(str));
        sleep_ms(100);
      }


    

    //display.illuminateSegment('B', 0);
    return 0; 
}