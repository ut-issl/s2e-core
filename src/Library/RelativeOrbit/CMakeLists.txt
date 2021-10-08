cmake_minimum_required(VERSION 3.13)
project(RELATIVE_ORBIT_MODELS)
add_library(${PROJECT_NAME} STATIC
  RelativeOrbitModels.cpp
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
