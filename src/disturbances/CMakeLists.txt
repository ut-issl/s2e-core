project(DISTURBANCE)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  air_drag.cpp
  disturbances.cpp
  geopotential.cpp
  gravity_gradient.cpp
  magnetic_disturbance.cpp
  solar_radiation_pressure_disturbance.cpp
  surface_force.cpp
  third_body_gravity.cpp
  lunar_gravity_field.cpp
  initialize_disturbances.cpp
)
include(../../common.cmake)
