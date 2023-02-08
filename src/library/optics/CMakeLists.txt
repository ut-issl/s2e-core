project(OPTICS)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  GaussianBeamBase.cpp
)

include(../../../common.cmake)
