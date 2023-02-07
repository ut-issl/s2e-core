project(SIMULATION)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  Case/SimulationCase.cpp
  
  MCSim/InitParameter.cpp
  MCSim/MCSimExecutor.cpp
  MCSim/SimulationObject.cpp
  MCSim/InitMcSim.cpp

  spacecraft/spacecraft.cpp
  spacecraft/installed_components.cpp
  spacecraft/Structure/Structure.cpp
  spacecraft/Structure/KinematicsParams.cpp
  spacecraft/Structure/RMMParams.cpp
  spacecraft/Structure/Surface.cpp
  spacecraft/Structure/InitStructure.cpp
  
  GroundStation/GroundStation.cpp
  
  InterSatComm/InterSatComm.cpp
)

include(../../common.cmake)
