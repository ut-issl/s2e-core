project(INTERFACE_) # INTERFACEだけだとCMakeの予約後に引っかかってしまうよう
cmake_minimum_required(VERSION 3.13)

set(SOURCE_FILES
  
  hils/hils_port_manager.cpp
)

add_library(${PROJECT_NAME} STATIC
  ${SOURCE_FILES}
)

include(../../common.cmake)
