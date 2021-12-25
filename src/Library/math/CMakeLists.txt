cmake_minimum_required(VERSION 3.10)
project(MATH)
add_library(${PROJECT_NAME} STATIC
  GlobalRand.cpp
  GlobalRand.h
  Matrix.hpp
  Matrix_ifs.hpp
  Matrix_tfs.hpp
  MatVec.hpp
  MatVec_impl.hpp
  NormalRand.cpp
  NormalRand.hpp
  NormalRand_ifs.hpp
  ODE.hpp
  ODE_ifs.hpp
  ODE_impl.hpp
  ODE_tfs.hpp
  Quantization.cpp
  Quantization.h
  Quaternion.cpp
  Quaternion.hpp
  Quaternion_ifs.hpp
  Ran0.cpp
  Ran0.hpp
  Ran1.cpp
  Ran1.hpp
  RandomWalk.cpp
  RandomWalk.hpp
  Vector.cpp
  Vector.hpp
  Vector_ifs.hpp
  Vector_tfs.hpp
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
