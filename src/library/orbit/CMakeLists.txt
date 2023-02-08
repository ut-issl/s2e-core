project(ORBIT_MODELS)
cmake_minimum_required(VERSION 3.10)

add_library(${PROJECT_NAME} STATIC
  OrbitalElements.cpp
  kepler_orbit.cpp
)

include(../../../common.cmake)
