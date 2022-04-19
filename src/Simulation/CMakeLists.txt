project(SIMULATION)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  Case/SimulationCase.cpp
  MCSim/InitParameter.cpp
  MCSim/MCSimExecutor.cpp
  MCSim/SimulationObject.cpp
  Spacecraft/Spacecraft.cpp
  Spacecraft/InstalledComponents.cpp
  Spacecraft/Structure/Structure.cpp
  Spacecraft/Structure/KinematicsParams.cpp
  Spacecraft/Structure/RMMParams.cpp
  Spacecraft/Structure/Surface.cpp
  GroundStation/GroundStation.cpp
  InterSatComm/InterSatComm.cpp
)

include(../../common.cmake)
