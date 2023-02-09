project(INTERFACE)
cmake_minimum_required(VERSION 3.13)

if(USE_HILS)
  add_library(${PROJECT_NAME} STATIC
    initialize_file_access.cpp
    
    hils/hils_port_manager.cpp
    hils/ports/hils_uart_port.cpp
    hils/ports/hils_i2c_target_port.cpp
 )
else()
  add_library(${PROJECT_NAME} STATIC
    initialize/initialize_file_access.cpp
    
    log_output/logger.cpp
    log_output/initialize_log.cpp

    hils/hils_port_manager.cpp
  )
endif()

include(../../common.cmake)
