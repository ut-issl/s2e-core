cmake_minimum_required(VERSION 3.10)
project(SIMULATION)
add_library(${PROJECT_NAME} STATIC
  Case/SimulationCase.cpp
  MCSim/InitParameter.cpp
  MCSim/MCSimExecutor.cpp
  MCSim/SimulationObject.cpp
  Spacecraft/Spacecraft.cpp
  Spacecraft/Structure/Structure.cpp
  Spacecraft/Structure/KinematicsParams.cpp
  Spacecraft/Structure/RMMParams.cpp
  Spacecraft/Structure/Surface.cpp
  GroundStation/GroundStation.cpp
  InterSatComm/InterSatComm.cpp
  InterSatComm/Abstract/InterSatCommBase.cpp
)
# Compile option
if(MSVC)
    target_compile_options(${PROJECT_NAME} PUBLIC "/W4")
	target_compile_options(${PROJECT_NAME} PUBLIC "/source-charset:utf-8")
else()
    target_compile_options(${PROJECT_NAME} PUBLIC "-Wall")
    set(CMAKE_CXX_FLAGS "-m32 -rdynamic -Wall -g")
    set(CMAKE_C_FLAGS "-m32 -rdynamic -Wall -g")
endif()
