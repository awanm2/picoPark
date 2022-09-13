# Hardware
The library works with [SparkFun Atmospheric Sensor Breakout - BME280
](https://www.sparkfun.com/products/13676)
# Software
Software in this repository is  [SparkFun_BME280_Arduino_Library
](https://github.com/sparkfun/SparkFun_BME280_Arduino_Library) adapted to 
work with [Raspberry Pi Pico](https://www.raspberrypi.com/products/raspberry-pi-pico/) board.

# Supported boards
- [Raspberry Pi Pico](https://www.raspberrypi.com/products/raspberry-pi-pico/)

# Bus protocol supported
- I2C

# License
**Code, firmware, and software is released under the MIT License(http://opensource.org/licenses/MIT).**
**See the License file in the current repository**

# Building 
From main directroy 

`cmake -B build -S ./`

then 

`make -C build/examples`