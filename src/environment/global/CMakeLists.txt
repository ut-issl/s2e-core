project(GLOBAL_ENVIRONMENT)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  global_environment.cpp
  celestial_information.cpp
  hipparcos_catalogue.cpp
  gnss_satellites.cpp
  simulation_time.cpp
  clock_generator.cpp
  earth_rotation.cpp
  moon_rotation.cpp
)

include(../../../common.cmake)
