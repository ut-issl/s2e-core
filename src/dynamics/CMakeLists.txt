project(DYNAMICS)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  orbit/orbit.cpp
  orbit/sgp4_orbit_propagation.cpp
  orbit/rk4_orbit_propagation.cpp
  orbit/relative_orbit.cpp
  orbit/kepler_orbit_propagation.cpp
  orbit/encke_orbit_propagation.cpp
  orbit/initialize_orbit.cpp

  thermal/node.cpp
  thermal/temperature.cpp
  thermal/heater.cpp
  thermal/heater_controller.cpp
  thermal/heatload.cpp
  thermal/initialize_temperature.cpp

  attitude/attitude.cpp
  attitude/attitude_rk4.cpp
  attitude/controlled_attitude.cpp
  attitude/initialize_attitude.cpp

  dynamics.cpp )

include(../../common.cmake)
