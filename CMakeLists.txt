cmake_minimum_required(VERSION 3.12)
include(pico_sdk_import.cmake)
project(BMP280 C CXX ASM)

pico_sdk_init()

add_executable(BMP280
        main.cpp
    )

pico_enable_stdio_usb(BMP280 1)
pico_enable_stdio_uart(BMP280 0)

target_link_libraries(BMP280 pico_stdlib hardware_i2c) #hardware_spi hardware_i2c
pico_add_extra_outputs(BMP280)