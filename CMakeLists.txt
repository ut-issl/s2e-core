cmake_policy(SET CMP0048 NEW)
cmake_minimum_required(VERSION 3.13)

project(S2E
  LANGUAGES CXX
  DESCRIPTION "S2E: Spacecraft Simulation Environment"
  VERSION 7.2.8
)

# build config
option(USE_HILS "Use HILS" OFF)
option(USE_C2A "Use C2A" OFF)
option(USE_C2A_COMMAND_SENDER "Use command sender to C2A" OFF)
option(BUILD_64BIT "Build 64bit" OFF)
option(GOOGLE_TEST "Execute GoogleTest" OFF)

# Mac user setting
option(APPLE_SILICON "Build with Apple Silicon" OFF)
# Force build with 64bit for APPLE
if(APPLE)
  option(BUILD_64BIT "Build 64bit" ON)
endif()

# preprocessor
if(WIN32)
  add_definitions(-DWIN32)
endif()

## set directory path
if(NOT DEFINED EXT_LIB_DIR)
  set(EXT_LIB_DIR ../ExtLibraries)
endif()
set(CSPICE_DIR ${EXT_LIB_DIR}/cspice)
set(NRLMSISE00_DIR ${EXT_LIB_DIR}/nrlmsise00)

if(NOT DEFINED FLIGHT_SW_DIR)
  set(FLIGHT_SW_DIR ../FlightSW)
endif()
if(NOT DEFINED C2A_NAME)
  set(C2A_NAME "c2a_oss")
endif()

## Directory path for ini files
if(NOT DEFINED INI_FILE_DIR_FROM_EXE)
  set(INI_FILE_DIR_FROM_EXE ../../settings)
endif()

if(NOT DEFINED EXT_LIB_DIR_FROM_EXE)
  set(EXT_LIB_DIR_FROM_EXE ../../${EXT_LIB_DIR})
endif()

if(NOT DEFINED CORE_DIR_FROM_EXE)
  set(CORE_DIR_FROM_EXE ../../../s2e-core)
endif()

## options to use C2A
if(USE_C2A)
  option(BUILD_64BIT OFF) # force to build with 32bit
  set(C2A_DIR ${FLIGHT_SW_DIR}/${C2A_NAME})
  message("C2A dir: ${C2A_DIR}")
  add_definitions(-DUSE_C2A)
  add_definitions(-DSILS_FW)
  #include_directories
  include_directories(${C2A_DIR}/src)
  #add subdirectory
  add_subdirectory(${C2A_DIR} C2A)
endif()

## options to use HILS
if(USE_HILS AND WIN32)
  add_definitions(-DUSE_HILS)
  ## winsock2
  SET (CMAKE_FIND_LIBRARY_SUFFIXES ".lib")
  find_library(WS2_32_LIB ws2_32.lib)
  message("path for winsock2 is")
  message(${WS2_32_LIB})
endif()

set(S2E_DIR ${CMAKE_CURRENT_SOURCE_DIR})

## include directories
include_directories(${CSPICE_DIR}/include)
include_directories(${NRLMSISE00_DIR}/src)

## add_subdirectories
add_subdirectory(src/simulation)
add_subdirectory(src/environment/global)
add_subdirectory(src/environment/local)
add_subdirectory(src/dynamics)
add_subdirectory(src/disturbances)
add_subdirectory(src/components)
add_subdirectory(src/math_physics)
add_subdirectory(src/setting_file_reader)
add_subdirectory(src/logger)
add_subdirectory(src/utilities)
add_subdirectory(ExtLibraries/inih)

set(SOURCE_FILES
  src/s2e.cpp
  src/simulation_sample/case/sample_case.cpp
  src/simulation_sample/spacecraft/sample_spacecraft.cpp
  src/simulation_sample/spacecraft/sample_components.cpp
  src/simulation_sample/ground_station/sample_ground_station_components.cpp
  src/simulation_sample/ground_station/sample_ground_station.cpp
)

## Create executable file
add_executable(${PROJECT_NAME} ${SOURCE_FILES})

## cspice library
if(APPLE)
  if(APPLE_SILICON)
    # APPLE Silicon
    set(CSPICE_LIB_DIR ${CSPICE_DIR}/cspice_apple_silicon64/lib)
  else()
    # APPLE Intel
    set(CSPICE_LIB_DIR ${CSPICE_DIR}/cspice_apple_intel64/lib)
  endif()
elseif(CYGWIN)
  SET (CMAKE_FIND_LIBRARY_SUFFIXES ".so" ".a")
  set(CSPICE_LIB_DIR ${CSPICE_DIR}/cspice_cygwin/lib)
elseif(UNIX)
  if(BUILD_64BIT)
    set(CSPICE_LIB_DIR ${CSPICE_DIR}/cspice_unix64/lib)
  else()
    set(CSPICE_LIB_DIR ${CSPICE_DIR}/cspice_unix/lib)
  endif()
endif()

if(WIN32)
  SET (CMAKE_FIND_LIBRARY_SUFFIXES ".lib")
  if(BUILD_64BIT)
    find_library(CSPICE_LIB
      NAMES cspice.lib csupport.lib
      PATHS ${CSPICE_DIR}/cspice_msvs64/lib)
  else()
    find_library(CSPICE_LIB
      NAMES cspice.lib csupport.lib
      PATHS ${CSPICE_DIR}/cspice_msvs/lib)
  endif()
else()
  find_library(CSPICE_LIB
    NAMES cspice.a csupport.a
    PATHS ${CSPICE_LIB_DIR})
endif()
if(NOT CSPICE_LIB)
  message(FATAL_ERROR "cspice not found in ${CSPICE_LIB_DIR}")
endif()
#target_link_libraries(${PROJECT_NAME} ${CSPICE_LIB})

## nrlmsise00 library
if(CYGWIN)
  SET (CMAKE_FIND_LIBRARY_SUFFIXES ".a")
  find_library(NRLMSISE00_LIB
    NAMES libnrlmsise00.a
    PATHS ${NRLMSISE00_DIR}/lib)
elseif(UNIX)
  if(BUILD_64BIT)
    find_library(NRLMSISE00_LIB
      NAMES libnrlmsise00.a
      PATHS ${NRLMSISE00_DIR}/lib64)
  else()
    find_library(NRLMSISE00_LIB
      NAMES libnrlmsise00.a
      PATHS ${NRLMSISE00_DIR}/lib)
  endif()
elseif(WIN32)
  SET (CMAKE_FIND_LIBRARY_SUFFIXES ".lib")
  if(BUILD_64BIT)
    find_library(NRLMSISE00_LIB
      NAMES libnrlmsise00.lib
      PATHS ${NRLMSISE00_DIR}/lib64)
  else()
    find_library(NRLMSISE00_LIB
      NAMES libnrlmsise00.lib
      PATHS ${NRLMSISE00_DIR}/lib)
  endif()
endif()
if(NOT NRLMSISE00_LIB)
  message(FATAL_ERROR "nrlmsise00 not found in ${EXT_LIB_DIR}")
endif()
#target_link_libraries(${PROJECT_NAME} ${NRLMSISE00_LIB})

# Initialize link
target_link_libraries(COMPONENT DYNAMICS GLOBAL_ENVIRONMENT LOCAL_ENVIRONMENT MATH_PHYSICS SETTING_FILE_READER LOGGER UTILITIES)
target_link_libraries(DYNAMICS GLOBAL_ENVIRONMENT LOCAL_ENVIRONMENT SIMULATION MATH_PHYSICS)
target_link_libraries(DISTURBANCE DYNAMICS GLOBAL_ENVIRONMENT LOCAL_ENVIRONMENT MATH_PHYSICS)
target_link_libraries(SIMULATION DYNAMICS GLOBAL_ENVIRONMENT LOCAL_ENVIRONMENT DISTURBANCE MATH_PHYSICS)
target_link_libraries(GLOBAL_ENVIRONMENT ${CSPICE_LIB} MATH_PHYSICS)
target_link_libraries(LOCAL_ENVIRONMENT GLOBAL_ENVIRONMENT ${CSPICE_LIB} MATH_PHYSICS)
target_link_libraries(MATH_PHYSICS ${NRLMSISE00_LIB})
target_link_libraries(SETTING_FILE_READER INIH)

target_link_libraries(${PROJECT_NAME} DYNAMICS)
target_link_libraries(${PROJECT_NAME} DISTURBANCE)
target_link_libraries(${PROJECT_NAME} SIMULATION)
target_link_libraries(${PROJECT_NAME} GLOBAL_ENVIRONMENT LOCAL_ENVIRONMENT)
target_link_libraries(${PROJECT_NAME} COMPONENT)

## C2A integration
if(USE_C2A)
  target_link_libraries(${PROJECT_NAME} C2A)
endif()

## HILS
if(USE_HILS)
  target_link_libraries(${PROJECT_NAME} ${WS2_32_LIB})
  set_target_properties(${PROJECT_NAME} PROPERTIES COMMON_LANGUAGE_RUNTIME "")
  set_target_properties(COMPONENT PROPERTIES COMMON_LANGUAGE_RUNTIME "")
  set_target_properties(DYNAMICS PROPERTIES COMMON_LANGUAGE_RUNTIME "")
  set_target_properties(DISTURBANCE PROPERTIES COMMON_LANGUAGE_RUNTIME "")
  set_target_properties(SIMULATION PROPERTIES COMMON_LANGUAGE_RUNTIME "")
  set_target_properties(GLOBAL_ENVIRONMENT PROPERTIES COMMON_LANGUAGE_RUNTIME "")
  set_target_properties(LOCAL_ENVIRONMENT PROPERTIES COMMON_LANGUAGE_RUNTIME "")
endif()

## GoogleTest settings
if (NOT BUILD_64BIT)
  option(GOOGLE_TEST OFF) # GoogleTest supports 64bit only
endif()
if(GOOGLE_TEST)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG release-1.12.1
  )
  if(WIN32)
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  endif()
  FetchContent_MakeAvailable(googletest)

  # Unit test
  set(TEST_PROJECT_NAME ${PROJECT_NAME}_TEST)

  # Add all test_*.cpp files as SOURCE_FILES
  file(GLOB_RECURSE TEST_FILES ${CMAKE_CURRENT_LIST_DIR}/src/test_*.cpp)
  # Uncomment the following line to exclude any files that match the REGEX from TEST_FILES
  # list(FILTER TEST_FILES EXCLUDE REGEX ${CMAKE_CURRENT_LIST_DIR}/src/test_example.cpp)

  add_executable(${TEST_PROJECT_NAME} ${TEST_FILES})
  target_link_libraries(${TEST_PROJECT_NAME} gtest gtest_main gmock)
  target_link_libraries(${TEST_PROJECT_NAME} MATH_PHYSICS)
  include_directories(${TEST_PROJECT_NAME})
  add_test(NAME s2e-test COMMAND ${TEST_PROJECT_NAME})
  enable_testing()

  # Settings
  set_target_properties(${TEST_PROJECT_NAME} PROPERTIES LANGUAGE CXX)
  set_target_properties(${TEST_PROJECT_NAME} PROPERTIES CXX_STANDARD 17)
  set_target_properties(${TEST_PROJECT_NAME} PROPERTIES CXX_EXTENSIONS FALSE)
  target_compile_definitions(${TEST_PROJECT_NAME} PRIVATE "CORE_DIR_FROM_EXE=\"${CORE_DIR_FROM_EXE}\"")

endif()


## Cmake debug
message("Cspice_LIB:  " ${CSPICE_LIB})
message("nrlmsise00_LIB:  " ${NRLMSISE00_LIB})

include(common.cmake)
