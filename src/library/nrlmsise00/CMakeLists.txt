project(WRAPPER_NRLMSISE00)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  wrapper_nrlmsise00.cpp
  wrapper_nrlmsise00.hpp
)

include(../../../common.cmake)
