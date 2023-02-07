project(GLOBAL_ENVIRONMENT)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  GlobalEnvironment.cpp
  celestial_information.cpp
  HipparcosCatalogue.cpp
  GnssSatellites.cpp
  SimTime.cpp
  ClockGenerator.cpp
  CelestialRotation.cpp
  InitGlobalEnvironment.cpp
  InitGnssSatellites.cpp
)

include(../../../common.cmake)
