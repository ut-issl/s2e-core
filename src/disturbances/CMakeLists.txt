project(DISTURBANCE)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  air_drag.cpp
  disturbances.cpp
  GeoPotential.cpp
  GravityGradient.cpp
  MagDisturbance.cpp
  SolarRadiation.cpp
  SurfaceForce.cpp
  ThirdBodyGravity.cpp
  InitDisturbance.cpp
)
include(../../common.cmake)
