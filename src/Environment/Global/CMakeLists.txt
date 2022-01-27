project(GLOBAL_ENVIRONMENT)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  GlobalEnvironment.cpp
  CelestialInformation.cpp
  HipparcosCatalogue.cpp
  GnssSatellites.cpp
  SimTime.cpp
  ClockGenerator.cpp
  CelestialRotation.cpp
)

include(../../../common.cmake)
