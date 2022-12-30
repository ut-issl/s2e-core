project(ExtLibraries)

cmake_minimum_required(VERSION 3.18)

# build config
option(BUILD_64BIT "Build 64bit" OFF)

if(NOT DEFINED EXT_LIB_DIR)
  set(EXT_LIB_DIR "${CMAKE_CURRENT_LIST_DIR}/../../ExtLibraries/")
endif()

# windows path
string(REPLACE "\\" "/" EXT_LIB_DIR ${EXT_LIB_DIR})

message("ExtLibraries install dir: ${EXT_LIB_DIR}")

add_subdirectory(nrlmsise00)
add_subdirectory(cspice)
add_subdirectory(GeoPotential)
