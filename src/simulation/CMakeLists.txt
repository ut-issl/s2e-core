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
  spacecraft/structure/Structure.cpp
  spacecraft/structure/KinematicsParams.cpp
  spacecraft/structure/RMMParams.cpp
  spacecraft/structure/Surface.cpp
  spacecraft/structure/InitStructure.cpp
  
  GroundStation/GroundStation.cpp
  
  InterSatComm/InterSatComm.cpp
)

include(../../common.cmake)
