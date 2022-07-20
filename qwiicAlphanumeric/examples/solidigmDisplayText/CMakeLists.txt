
set(EXAMPLE_NAME  solidigmDisplayText)

add_executable(${EXAMPLE_NAME} solidigmDisplayText.cpp)

add_compile_options(-Wwrite-strings 
                    -fpermissive
                    )

target_link_libraries(${EXAMPLE_NAME} 
   pico_stdlib 
   qwiicAlphanumeric
   )

pico_add_extra_outputs(${EXAMPLE_NAME})

