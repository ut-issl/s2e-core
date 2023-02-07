project(SIMULATION)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  Case/SimulationCase.cpp
  
  monte_carlo_simulation/monte_carlo_simulation_executor.cpp
  monte_carlo_simulation/simulation_object.cpp
  monte_carlo_simulation/initialize_monte_carlo_parameters.cpp
  monte_carlo_simulation/initialize_monte_carlo_simulation.cpp

  spacecraft/spacecraft.cpp
  spacecraft/installed_components.cpp
  spacecraft/structure/structure.cpp
  spacecraft/structure/kinematics_parameters.cpp
  spacecraft/structure/residual_magnetic_moment.cpp
  spacecraft/structure/surface.cpp
  spacecraft/structure/initialize_structure.cpp
  
  GroundStation/GroundStation.cpp
  
  InterSatComm/InterSatComm.cpp
)

include(../../common.cmake)
