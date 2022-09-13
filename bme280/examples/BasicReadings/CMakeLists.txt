
set(EXAMPLE_NAME  readBME)

add_executable(${EXAMPLE_NAME} basicReadings.cpp)

target_link_libraries(${EXAMPLE_NAME} 
   pico_stdlib 
   bme280
   )

pico_add_extra_outputs(${EXAMPLE_NAME})

add_compile_options(-Wwrite-strings 
                    -fpermissive)