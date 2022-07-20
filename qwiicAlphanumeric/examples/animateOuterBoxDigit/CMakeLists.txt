set(EXAMPLE_NAME  animateOuterBoxDigit)

add_executable(${EXAMPLE_NAME} animateOuterBoxDigit.cpp)

target_link_libraries(${EXAMPLE_NAME} 
   pico_stdlib 
   qwiicAlphanumeric
   )

pico_add_extra_outputs(${EXAMPLE_NAME})

