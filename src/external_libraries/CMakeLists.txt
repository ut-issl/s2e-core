project(EXTERNAL_LIBRARY)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  igrf/igrf.cpp
  
  inih/ini.c
  inih/cpp/INIReader.cpp
  
  nrlmsise00/wrapper_nrlmsise00.cpp
  
  sgp4/sgp4ext.cpp
  sgp4/sgp4io.cpp
  sgp4/sgp4unit.cpp
)

include(../../common.cmake)
