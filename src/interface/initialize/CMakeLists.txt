project(INI_ACC)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  initialize_file_access.cpp
)

include(../../../common.cmake)
