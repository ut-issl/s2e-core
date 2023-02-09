project(OPTICS)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  gaussian_beam_base.cpp
)

include(../../../common.cmake)
