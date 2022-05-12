project(DISTURBANCE)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  AirDrag.cpp
  Disturbances.cpp
  GeoPotential.cpp
  GGDist.cpp
  MagDisturbance.cpp
  SolarRadiation.cpp
  SurfaceForce.cpp
  ThirdBodyGravity.cpp
  InitDisturbance.cpp
)
include(../../common.cmake)
