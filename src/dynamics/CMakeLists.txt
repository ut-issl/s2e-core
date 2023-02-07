project(DYNAMICS)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  orbit/orbit.cpp
  orbit/Sgp4OrbitPropagation.cpp
  orbit/Rk4OrbitPropagation.cpp
  orbit/RelativeOrbit.cpp
  orbit/kepler_orbit_propagation.cpp
  orbit/encke_orbit_propagation.cpp
  orbit/initialize_orbit.cpp

  thermal/Node.cpp
  thermal/Temperature.cpp
  thermal/InitNode.cpp
  thermal/InitTemperature.cpp

  attitude/attitude.cpp
  attitude/attitude_rk4.cpp
  attitude/controlled_attitude.cpp
  attitude/initialize_attitude.cpp

  dynamics.cpp
)

include(../../common.cmake)
