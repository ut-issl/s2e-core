project(LOCAL_ENVIRONMENT)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  atmosphere.cpp
  local_environment.cpp
  geomagnetic_field.cpp
  solar_radiation_pressure_environment.cpp
  local_celestial_information.cpp
  initialize_local_environment.cpp
)

include(../../../common.cmake)
