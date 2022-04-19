project(INI_IN)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  Initialize.h
  # Global Environment
  Init_SimTime.cpp
  Init_Log.cpp
  Init_CelesInfo.cpp
  Init_HipCatalogue.cpp
  Init_GnssSatellites.cpp
  # Structure
  Init_Structure.cpp

  Init_MCSim.cpp

  #InitComponent/InitGyro.cpp
  #InitComponent/InitANT.cpp
  #InitComponent/InitBAT.cpp
  #InitComponent/InitEMDS.cpp
  #InitComponent/InitGScalculator.cpp
  #InitComponent/InitMagSensor.cpp
  #InitComponent/InitMagTorquer.cpp
  #InitComponent/InitRWModel.cpp
  #InitComponent/InitSAP.cpp
  #InitComponent/InitSimpleThruster.cpp
  #InitComponent/InitSTT.cpp
  #InitComponent/InitSunSensor.cpp
  #InitComponent/initTelescope.cpp
  #InitComponent/InitUWBSensor.cpp
  #InitComponent/InitGNSSReceiver.cpp
  #InitPower/CsvScenarioInterface.cpp
  #  Init_COSMOSWrapper.cpp
#  Init_HardwareMessage.cpp
)

include(../../../common.cmake)

## ini access
### TODO: split CMakeLists.txt
if(WIN32)
  add_library(INI_ACC STATIC IniAccess.cpp)
else()
  add_library(INI_ACC STATIC IniAccessUnix.cpp)
endif()
# Compile option
target_include_directories(INI_ACC PUBLIC ${S2E_DIR}/src)
if(MSVC)
  target_compile_options(INI_ACC PUBLIC "/W4")
  target_compile_options(INI_ACC PUBLIC "/source-charset:utf-8")
else()
  target_compile_options(INI_ACC PUBLIC -Wall -g -m32 -rdynamic)
endif()
