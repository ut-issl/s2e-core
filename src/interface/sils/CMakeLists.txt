project(SC_IO)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  ports/gpio_port.cpp
  ports/PowerPort.cpp
  ports/SCIPort.cpp
  ports/i2c_port.cpp
  utility/RingBuffer.cpp
)

include(../../../common.cmake)
