project(INTERFACE_) # INTERFACEだけだとCMakeの予約後に引っかかってしまうよう
cmake_minimum_required(VERSION 3.13)

if(USE_HILS)
  add_library(${PROJECT_NAME} STATIC
    initialize_file_access.cpp

    log_output/logger.cpp
    log_output/initialize_log.cpp

    sils/ports/gpio_port.cpp
    sils/ports/power_port.cpp
    sils/ports/uart_port.cpp
    sils/ports/i2c_port.cpp
    sils/utility/ring_buffer.cpp

    hils/hils_port_manager.cpp
    hils/ports/hils_uart_port.cpp
    hils/ports/hils_i2c_target_port.cpp
 )
else()
  add_library(${PROJECT_NAME} STATIC
    initialize/initialize_file_access.cpp
    
    log_output/logger.cpp
    log_output/initialize_log.cpp

    sils/ports/gpio_port.cpp
    sils/ports/power_port.cpp
    sils/ports/uart_port.cpp
    sils/ports/i2c_port.cpp
    sils/utility/ring_buffer.cpp

    hils/hils_port_manager.cpp
  )
endif()

include(../../common.cmake)
