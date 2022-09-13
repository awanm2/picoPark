# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.12)
set(CMAKE_C_STANDARD 11)
# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Include build functions from Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Set name of project 
set(PROJ__NAME  bme280)

# Set C/C++ standards
project(${PROJ__NAME}
  LANGUAGES C CXX ASM)

  # Creates a pico-sdk subdirectory in our project for the libraries
pico_sdk_init()

#add_compile_options(-Wwrite-strings 
#                    -fpermissive)

add_subdirectory(./src)
add_subdirectory(./examples)