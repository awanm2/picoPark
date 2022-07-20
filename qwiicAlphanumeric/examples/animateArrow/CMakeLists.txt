
set(EXAMPLE_NAME  animateArrow)

add_executable(${EXAMPLE_NAME} animateArrow.cpp)

target_link_libraries(${EXAMPLE_NAME} 
   pico_stdlib 
   qwiicAlphanumeric
   )

pico_add_extra_outputs(${EXAMPLE_NAME})

