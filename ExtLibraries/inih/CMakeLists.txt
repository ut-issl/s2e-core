project(INIH)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC  
  ini.c
  cpp/INIReader.cpp
)

include(../../common.cmake)
