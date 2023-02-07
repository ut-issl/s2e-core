project(DYNAMICS)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  orbit/Orbit.cpp
  orbit/Sgp4OrbitPropagation.cpp
  orbit/Rk4OrbitPropagation.cpp
  orbit/RelativeOrbit.cpp
  orbit/KeplerOrbitPropagation.cpp
  orbit/EnckeOrbitPropagation.cpp
  orbit/InitOrbit.cpp

  thermal/Node.cpp
  thermal/Temperature.cpp
  thermal/InitNode.cpp
  thermal/InitTemperature.cpp

  attitude/Attitude.cpp
  attitude/AttitudeRK4.cpp
  attitude/ControlledAttitude.cpp
  attitude/InitAttitude.cpp

  dynamics.cpp
)

include(../../common.cmake)
