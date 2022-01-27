project(ExtLibraries)

cmake_minimum_required(VERSION 3.13)

if(NOT DEFINED EXT_LIB_DIR)
  set(EXT_LIB_DIR "${CMAKE_CURRENT_LIST_DIR}/../../ExtLibraries/")
endif()

add_subdirectory(nrlmsise00)
add_subdirectory(cspice)
