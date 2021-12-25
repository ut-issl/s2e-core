cmake_minimum_required(VERSION 3.10)
project(SIMULATION)
add_library(${PROJECT_NAME} STATIC
  Case/SimulationCase.h
  MCSim/InitParameter.cpp
  MCSim/InitParameter.h
  MCSim/MCSimExecutor.cpp
  MCSim/MCSimExecutor.h
  MCSim/SimulationObject.cpp
  MCSim/SimulationObject.h
  Spacecraft/Spacecraft.cpp
  Spacecraft/Spacecraft.h
  GroundStation/GroundStation.cpp
  GroundStation/GroundStation.h
  SimulationConfig.h
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
