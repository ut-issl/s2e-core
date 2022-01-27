project(UTIL)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  endian.cpp
  slip.cpp
)

include(../../../common.cmake)
