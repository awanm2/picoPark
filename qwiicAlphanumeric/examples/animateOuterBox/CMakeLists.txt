
set(EXAMPLE_NAME  animateOuterBox)

add_executable(${EXAMPLE_NAME} animateOuterBox.cpp)

target_link_libraries(${EXAMPLE_NAME} 
   pico_stdlib 
   qwiicAlphanumeric
   )

pico_add_extra_outputs(${EXAMPLE_NAME})

