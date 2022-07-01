# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.12)
set(CMAKE_C_STANDARD 11)
# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Include build functions from Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Set name of project (as PROJECT_NAME) and C/C++ standards

project(bus_scan 
  #<PROJECT-NAME>_DESCRIPTION
  LANGUAGES C CXX ASM)
# Set C standard

#include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
#include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)
# Creates a pico-sdk subdirectory in our project for the libraries
pico_sdk_init()

add_compile_options(-Wwrite-strings -fpermissive)

add_subdirectory(./src)
add_subdirectory(./examples)


