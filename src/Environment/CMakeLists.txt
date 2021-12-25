cmake_minimum_required(VERSION 3.10)
project(ENVIRONMENT)
add_library(${PROJECT_NAME} STATIC
  Atmosphere.cpp
  Atmosphere.h
  CelestialInformation.cpp
  CelestialInformation.h
  Environment.cpp
  Environment.h
  HipparcosCatalogue.cpp
  HipparcosCatalogue.h
  MagEnvironment.cpp
  MagEnvironment.h
  SRPEnvironment.cpp
  SRPEnvironment.h
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
