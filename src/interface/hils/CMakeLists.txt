project(HILS_IO)
cmake_minimum_required(VERSION 3.13)

if(USE_HILS)
add_library(${PROJECT_NAME} STATIC
  HilsPortManager.cpp
  ports/hils_uart_port.cpp
  ports/hils_i2c_target_port.cpp
  )
else()
add_library(${PROJECT_NAME} STATIC
  HilsPortManager.cpp
  )
endif()

include(../../../common.cmake)
