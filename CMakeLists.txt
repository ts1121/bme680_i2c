cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(bme680_i2c C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(bme680_i2c
	bme680_i2c.cpp
	main.cpp
)

pico_enable_stdio_usb(bme680_i2c 1)
pico_enable_stdio_uart(bme680_i2c 0)

pico_add_extra_outputs(bme680_i2c)

target_link_libraries(bme680_i2c pico_stdlib hardware_i2c)