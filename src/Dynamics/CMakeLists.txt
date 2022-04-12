project(DYNAMICS)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  Orbit/Orbit.cpp
  Orbit/Sgp4OrbitPropagation.cpp
  Orbit/Rk4OrbitPropagation.cpp
  Orbit/RelativeOrbit.cpp
  Orbit/KeplerOrbitPropagation.cpp
  Orbit/EnckeOrbitPropagation.cpp
  Thermal/Node.cpp
  Thermal/Temperature.cpp
  Attitude/AttitudeRK4.cpp
  Attitude/ControlledAttitude.cpp
  Dynamics.cpp
  ../Interface/InitInput/Init_Attitude.cpp
  ../Interface/InitInput/Init_Orbit.cpp
)

include(../../common.cmake)
