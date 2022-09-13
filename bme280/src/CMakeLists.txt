# Fetch all files in current dir
file(GLOB FILES *.cpp *.h)
set(LIB__NAME  bme280)
add_library(${LIB__NAME} ${FILES})
# normal build, link with pico_stdlib
target_link_libraries(${LIB__NAME} 
                      pico_stdlib
                      hardware_i2c
                      )
# IMPORTANT define where to find the files when the lib is included from a different folder!
target_include_directories(${LIB__NAME} PUBLIC ../include)

# Enable usb output, disable uart output
pico_enable_stdio_usb(${LIB__NAME} 1)
pico_enable_stdio_uart(${LIB__NAME} 0)
