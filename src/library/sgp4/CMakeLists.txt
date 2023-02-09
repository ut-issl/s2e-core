project(SGP4)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  sgp4ext.cpp
  sgp4ext.h
  sgp4io.cpp
  sgp4io.h
  sgp4unit.cpp
  sgp4unit.h
)

include(../../../common.cmake)
