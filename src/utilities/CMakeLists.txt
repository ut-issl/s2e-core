project(UTILITIES)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  endian.cpp
  slip.cpp
  quantization.cpp
  ring_buffer.cpp
)

include(../../common.cmake)
