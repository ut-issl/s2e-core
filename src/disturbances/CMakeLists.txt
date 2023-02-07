project(DISTURBANCE)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  air_drag.cpp
  disturbances.cpp
  geopotential.cpp
  gravity_gradient.cpp
  MagDisturbance.cpp
  SolarRadiation.cpp
  SurfaceForce.cpp
  ThirdBodyGravity.cpp
  initialize_disturbances.cpp
)
include(../../common.cmake)
