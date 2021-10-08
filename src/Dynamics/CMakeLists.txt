cmake_minimum_required(VERSION 3.13)
project(DYNAMICS)
add_library(${PROJECT_NAME} STATIC
  Orbit/EarthCenteredOrbit.cpp
  Orbit/SimpleCircularOrbit.cpp
  Orbit/RelativeOrbit.cpp
  Thermal/Node.cpp
  Thermal/Temperature.cpp
  Attitude/AttitudeRK4.cpp
  Attitude/ControlledAttitude.cpp
  Dynamics.cpp
  ../Interface/InitInput/Init_Attitude.cpp
  ../Interface/InitInput/Init_Orbit.cpp
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
