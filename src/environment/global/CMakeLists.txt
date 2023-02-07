project(GLOBAL_ENVIRONMENT)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  global_environment.cpp
  celestial_information.cpp
  hipparcos_catalogue.cpp
  gnss_satellites.cpp
  SimTime.cpp
  clock_generator.cpp
  celestial_rotation.cpp
  initialize_global_environment.cpp
  initialize_gnss_satellites.cpp
)

include(../../../common.cmake)
