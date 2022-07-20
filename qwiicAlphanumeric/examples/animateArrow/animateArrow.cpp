#include "SparkFun_Alphanumeric_Display_Animation_Pico.h"

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"

int main(void)
{
    SparkFun_Alphanumeric_Display_Animations display;

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

    if(display.setBrightness(3) == false)
    {
          blinky();
    }
    
    display.animate_Arrow();
    
    return 0; 
}