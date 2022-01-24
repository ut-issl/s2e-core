cmake_minimum_required(VERSION 3.13)

set(EXT_LIB_DIR "${CMAKE_CURRENT_LIST_DIR}/../../ExtLibraries/")

add_subdirectory(nrlmsise00)
add_subdirectory(cspice)

install(TARGETS libnrlmsise00)
