project(HILS_IO)
cmake_minimum_required(VERSION 3.13)

if(USE_HILS)
add_library(${PROJECT_NAME} STATIC
  HilsPortManager.cpp
  Ports/HilsUartPort.cpp
  Ports/HilsI2cTargetPort.cpp
  )
else()
add_library(${PROJECT_NAME} STATIC
  HilsPortManager.cpp
  )
endif()

include(../../../common.cmake)
