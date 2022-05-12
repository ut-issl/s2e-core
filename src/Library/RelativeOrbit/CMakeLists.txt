project(RELATIVE_ORBIT_MODELS)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  RelativeOrbitModels.cpp
)

include(../../../common.cmake)
