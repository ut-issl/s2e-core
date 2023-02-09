project(SC_IO)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  ports/gpio_port.cpp
  ports/power_port.cpp
  ports/uart_port.cpp
  ports/i2c_port.cpp
  utility/ring_buffer.cpp
)

include(../../../common.cmake)
