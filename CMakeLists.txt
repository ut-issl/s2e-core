## General settings
cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 14)
project(S2E CXX)
# preprocessor
if(WIN32)
  add_definitions(-DWIN32)
  #add_definitions(-D_HAS_STD_BYTE=0) #when you chose C++ 17
endif()

## set directory path
set(CSPICE_DIR ../ExtLibraries/cspice)
set(NRLMSISE00_DIR ../ExtLibraries/nrlmsise00)
set(FLIGHT_SW_DIR ../FlightSW)
set(C2A_DIR ${FLIGHT_SW_DIR}/c2a_oss)

## options to use C2A
option(USE_C2A "Use C2A")
set(USE_C2A OFF)
if(USE_C2A)
  add_definitions(-DUSE_C2A)
  add_definitions(-DSILS_FW)
  #include_directories
  include_directories(${C2A_DIR}/SH7254R_C2A/src/Core/AnomalyLogger)
  include_directories(${C2A_DIR}/SH7254R_C2A/src/Core/ApplicationManager)
  include_directories(${C2A_DIR}/SH7254R_C2A/src/Applications)
  include_directories(${C2A_DIR}/SH7254R_C2A/src/Core/TimeManager)
  include_directories(${C2A_DIR}/SH7254R_C2A/src/CmdTlm)
  include_directories(${C2A_DIR}/SH7254R_C2A/src/Core/ModeManager)
  include_directories(${C2A_DIR}/SH7254R_C2A/src/Core/TaskManager)
  include_directories(${C2A_DIR}/SH7254R_C2A/src/Core/WatchdogTimer)
  include_directories(${S2E_CORE_DIR}/src/Interface/SpacecraftInOut)
  #add subdirectory
  add_subdirectory(${C2A_DIR} C2A)
endif()

## include directories
include_directories(${CSPICE_DIR}/include)
include_directories(${NRLMSISE00_DIR}/src)

## add_subdirectories
add_subdirectory(src/Component)
add_subdirectory(src/Disturbance)
add_subdirectory(src/Dynamics)
add_subdirectory(src/Environment)
#add_subdirectory(src/Interface/HilsInOut)
add_subdirectory(src/Interface/InitInput)
add_subdirectory(src/Interface/LogOutput)
add_subdirectory(src/Interface/SpacecraftInOut)
add_subdirectory(src/Library/igrf)
add_subdirectory(src/Library/inih)
add_subdirectory(src/Library/math)
add_subdirectory(src/Library/nrlmsise00)
add_subdirectory(src/Library/sgp4)
add_subdirectory(src/Library/utils)
add_subdirectory(src/Simulation)

set(SOURCE_FILES
  src/S2E.cpp
  src/Simulation/Case/SampleCase.cpp
  src/Simulation/Case/SampleCase.h
  src/Simulation/Spacecraft/SampleSpacecraft/Sample_PortConfig.h
  src/Simulation/Spacecraft/SampleSpacecraft/SampleComponents.cpp
  src/Simulation/Spacecraft/SampleSpacecraft/SampleComponents.h
  src/Simulation/Spacecraft/SampleSpacecraft/SampleSat.cpp
  src/Simulation/Spacecraft/SampleSpacecraft/SampleSat.h
  src/Simulation/GroundStation/SampleGroundStation/SampleGSComponents.cpp
  src/Simulation/GroundStation/SampleGroundStation/SampleGSComponents.h
  src/Simulation/GroundStation/SampleGroundStation/SampleGS.cpp
  src/Simulation/GroundStation/SampleGroundStation/SampleGS.h
)

## Create executable file
add_executable(${PROJECT_NAME} ${SOURCE_FILES})
# Compile option
if(MSVC)
    target_compile_options(${PROJECT_NAME} PUBLIC "/W4")
	target_compile_options(${PROJECT_NAME} PUBLIC "/source-charset:utf-8")
else()
    target_compile_options(${PROJECT_NAME} PUBLIC "${CMAKE_CXX_FLAGS}-Wall")
    set(CMAKE_CXX_FLAGS "-m32 -rdynamic -Wall -g")
    set(CMAKE_C_FLAGS "-m32 -rdynamic -Wall -g")
endif()

## Linking libraries
## components
target_link_libraries(${PROJECT_NAME} COMPONENT)
## dynamics, environment, disturbance
target_link_libraries(${PROJECT_NAME} DYNAMICS ENVIRONMENT DISTURBANCE)
## Interface
target_link_libraries(${PROJECT_NAME} INI_IN INI_ACC LOG_OUT SC_IO)

target_link_libraries(${PROJECT_NAME} COMPONENT)
target_link_libraries(${PROJECT_NAME} DYNAMICS ENVIRONMENT DISTURBANCE)
target_link_libraries(${PROJECT_NAME} INI_IN INI_ACC LOG_OUT SC_IO)

## HILS
if (WIN32)
#  target_link_libraries(${PROJECT_NAME} HILS_IO)
  ## winsock2
  #SET (CMAKE_FIND_LIBRARY_SUFFIXES ".lib")
  #find_library(WS2_32_LIB ws2_32.lib)
  #message("path for winsock2 is")
  #message(${WS2_32_LIB})
#  target_link_libraries(${PROJECT_NAME} ${WS2_32_LIB})
endif()
## Library
target_link_libraries(${PROJECT_NAME} IGRF WRAPPER_NRLMSISE00 INIH MATH SGP4 UTIL)
## simulation
target_link_libraries(${PROJECT_NAME} SIMULATION)

## cspice library
if(CYGWIN)
  SET (CMAKE_FIND_LIBRARY_SUFFIXES ".so" ".a")
  find_library(CSPICE_LIB
    NAMES cspice.a csupport.a
    PATHS ${CSPICE_DIR}/cspice_cygwin/lib)
elseif(UNIX)
  find_library(CSPICE_LIB
    NAMES cspice.a csupport.a
    PATHS ${CSPICE_DIR}/cspice_unix/lib)
elseif(WIN32)
  SET (CMAKE_FIND_LIBRARY_SUFFIXES ".lib")
  find_library(CSPICE_LIB
    NAMES cspice.lib csupport.lib
    PATHS ${CSPICE_DIR}/cspice_msvs/lib)
endif()
target_link_libraries(${PROJECT_NAME} ${CSPICE_LIB})

## nrlmsise00 library
if(CYGWIN)
  SET (CMAKE_FIND_LIBRARY_SUFFIXES ".a")
  find_library(NRLMSISE00_LIB 
    NAMES libnrlmsise00.a
    PATHS ${NRLMSISE00_DIR}/lib)
elseif(UNIX)
  find_library(NRLMSISE00_LIB 
    NAMES libnrlmsise00.a
    PATHS ${NRLMSISE00_DIR}/lib)
elseif(WIN32)
  SET (CMAKE_FIND_LIBRARY_SUFFIXES ".lib")
  find_library(NRLMSISE00_LIB 
    NAMES libnrlmsise00.lib
    PATHS ${NRLMSISE00_DIR}/lib)
endif()
target_link_libraries(${PROJECT_NAME} ${NRLMSISE00_LIB})

## C2A integration
if(USE_C2A)
  target_link_libraries(${PROJECT_NAME} C2A)
  target_link_libraries(${PROJECT_NAME} SC_IO)
endif()


## Cmake debug
message("Cspice_LIB:  "${CSPICE_LIB})
message("nrlmsise00_LIB:  "${NRLMSISE00_LIB})
