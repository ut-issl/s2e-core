cmake_minimum_required(VERSION 3.10)
project(DISTURBANCE)
add_library(${PROJECT_NAME} STATIC
  AirDrag.cpp
  AirDrag.h
  Disturbance.h
  Disturbances.cpp
  Disturbances.h
  GeoPotential.cpp
  GGDist.cpp
  GGDist.h
  MagDisturbance.cpp
  MagDisturbance.h
  SimpleDisturbance.h
  SolarRadiation.cpp
  SolarRadiation.h
  SurfaceForce.cpp
  SurfaceForce.h
  ThirdBodyGravity.cpp
  ThirdBodyGravity.h
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
