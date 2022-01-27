project(SC_IO)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  Ports/GPIOPort.cpp
  Ports/PowerPort.cpp
  Ports/SCIPort.cpp
  Ports/I2CPort.cpp
  Utils/RingBuffer.cpp

## C++/CLI related codes
#  Utils/ITCTMChannel.h
#  TMTCDriver.cpp
#  TMTCDriver.h
)

include(../../../common.cmake)
