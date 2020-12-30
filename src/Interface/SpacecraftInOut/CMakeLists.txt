cmake_minimum_required(VERSION 3.10)
project(SC_IO)
add_library(${PROJECT_NAME} STATIC
  Ports/GPIOPort.cpp
  Ports/PowerPort.cpp
  Ports/SCIPort.cpp
  Utils/RingBuffer.cpp
  GPIODriver.cpp
  PowerDriver.cpp
  #SCIDriver.cpp
  #SpacecraftInterface.cpp

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
