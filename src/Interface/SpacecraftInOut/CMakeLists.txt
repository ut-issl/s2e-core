cmake_minimum_required(VERSION 3.10)
project(SC_IO)
add_library(${PROJECT_NAME} STATIC
  Ports/GPIOPort.cpp
  Ports/GPIOPort.h
  Ports/PowerPort.cpp
  Ports/PowerPort.h
  Ports/SCIPort.cpp
  Ports/SCIPort.h
  Utils/RingBuffer.cpp
  Utils/RingBuffer.h
  GPIODriver.cpp
  GPIODriver.h
  PowerDriver.cpp
  PowerDriver.h
  SCIDriver.cpp
  SCIDriver.h
  SpacecraftInterface.cpp
  SpacecraftInterface.h

## C++/CLI related codes
#  Utils/ITCTMChannel.h
#  TMTCDriver.cpp
#  TMTCDriver.h
)
# Compile option
if(MSVC)
    target_compile_options(${PROJECT_NAME} PUBLIC "/W4")
	target_compile_options(${PROJECT_NAME} PUBLIC "/source-charset:utf-8")
else()
    target_compile_options(${PROJECT_NAME} PUBLIC "-Wall")
    set(CMAKE_CXX_FLAGS "-m32 -rdynamic -Wall -g")
    set(CMAKE_C_FLAGS "-m32 -rdynamic -Wall -g")
endif()
