project(WRAPPER_NRLMSISE00)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  Wrapper_nrlmsise00.cpp
  Wrapper_nrlmsise00.h
)

include(../../../common.cmake)
