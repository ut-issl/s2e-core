project(SC_IO)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  ports/GPIOPort.cpp
  ports/PowerPort.cpp
  ports/SCIPort.cpp
  ports/I2CPort.cpp
  Utils/RingBuffer.cpp
)

include(../../../common.cmake)
