project(INTERFACE_) # INTERFACEだけだとCMakeの予約後に引っかかってしまうよう
cmake_minimum_required(VERSION 3.13)

set(SOURCE_FILES
  
  hils/hils_port_manager.cpp
)

if(USE_HILS)
  set(SOURCE_FILES
    ${SOURCE_FILES}
    hils/ports/hils_uart_port.cpp
    hils/ports/hils_i2c_target_port.cpp
  )
endif()

add_library(${PROJECT_NAME} STATIC
  ${SOURCE_FILES}
)

include(../../common.cmake)
