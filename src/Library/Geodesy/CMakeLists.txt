project(GEODESY)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  GeodeticPosition.cpp
)

include(../../../common.cmake)
