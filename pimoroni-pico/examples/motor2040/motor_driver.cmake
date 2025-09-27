set(OUTPUT_NAME motor_driver)
add_executable(${OUTPUT_NAME} motor_driver.cpp)

target_link_libraries(${OUTPUT_NAME}
        pico_stdlib
        motor2040
        button
        pid
        )

# enable usb output
pico_enable_stdio_usb(${OUTPUT_NAME} 1)

pico_add_extra_outputs(${OUTPUT_NAME})
add_compile_options(-Wno-error)