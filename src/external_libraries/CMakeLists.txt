project(EXTERNAL_LIBRARY)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC  
  inih/ini.c
  inih/cpp/INIReader.cpp
)

include(../../common.cmake)
