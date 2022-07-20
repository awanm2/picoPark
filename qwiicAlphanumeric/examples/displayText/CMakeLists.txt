
set(EXAMPLE_NAME  display_text)

add_executable(${EXAMPLE_NAME} displayText.cpp)

add_compile_options(-Wwrite-strings 
                    -fpermissive
                    )

target_link_libraries(${EXAMPLE_NAME} 
   pico_stdlib 
   qwiicAlphanumeric
   )

pico_add_extra_outputs(${EXAMPLE_NAME})

