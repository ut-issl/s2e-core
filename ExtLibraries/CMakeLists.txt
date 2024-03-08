project(ExtLibraries)

cmake_minimum_required(VERSION 3.18)

# build config
option(BUILD_64BIT "Build 64bit" OFF)

# Mac user setting
option(APPLE_SILICON "Build with Apple Silicon" OFF)
# Force build with 64bit for APPLE
if(APPLE)
  option(BUILD_64BIT "Build 64bit" ON)
endif()

if(NOT DEFINED EXT_LIB_DIR)
  set(EXT_LIB_DIR "${CMAKE_CURRENT_LIST_DIR}/../../ExtLibraries/")
endif()

# windows path
string(REPLACE "\\" "/" EXT_LIB_DIR ${EXT_LIB_DIR})

message("ExtLibraries install dir: ${EXT_LIB_DIR}")

add_subdirectory(nrlmsise00)
add_subdirectory(cspice)
add_subdirectory(lunar_gravity_field)
