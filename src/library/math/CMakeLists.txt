project(MATH)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  global_randomization.cpp
  normal_randomization.cpp
  quantization.cpp
  quaternion.cpp
  randomize0.cpp
  randomize1.cpp
  vector.cpp
  s2e_math.cpp
)

include(../../../common.cmake)
