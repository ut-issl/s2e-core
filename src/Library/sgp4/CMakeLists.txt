cmake_minimum_required(VERSION 3.10)
project(SGP4)
add_library(${PROJECT_NAME} STATIC
  sgp4ext.cpp
  sgp4ext.h
  sgp4io.cpp
  sgp4io.h
  sgp4unit.cpp
  sgp4unit.h
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
