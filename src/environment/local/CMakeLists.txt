project(LOCAL_ENVIRONMENT)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  atmosphere.cpp
  local_environment.cpp
  geomagnetic_field.cpp
  SRPEnvironment.cpp
  local_celestial_information.cpp
  initialize_local_environment.cpp
)

include(../../../common.cmake)
