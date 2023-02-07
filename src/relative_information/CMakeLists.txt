project(RELATIVE_INFO)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  RelativeInformation.cpp
)

include(../../common.cmake)
