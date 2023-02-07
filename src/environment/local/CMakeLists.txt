project(LOCAL_ENVIRONMENT)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  atmosphere.cpp
  LocalEnvironment.cpp
  MagEnvironment.cpp
  SRPEnvironment.cpp
  LocalCelestialInformation.cpp
  InitLocalEnvironment.cpp
)

include(../../../common.cmake)
