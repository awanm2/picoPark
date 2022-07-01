# build the a concrete examples
add_executable(led_blink led_blink.cpp)
add_compile_options(-Wwrite-strings --fpermissive)

# define where to find the files for the lib <pico-shift-register>
target_link_libraries(led_blink 
   pico_stdlib 
   qwiicAlphanumeric)

pico_add_extra_outputs(led_blink)

