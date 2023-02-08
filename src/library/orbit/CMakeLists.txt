project(ORBIT_MODELS)
cmake_minimum_required(VERSION 3.10)

add_library(${PROJECT_NAME} STATIC
  orbital_elements.cpp
  kepler_orbit.cpp
  relative_orbit_models.cpp
)

include(../../../common.cmake)
