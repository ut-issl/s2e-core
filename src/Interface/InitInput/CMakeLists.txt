cmake_minimum_required(VERSION 3.10)
project(INI_IN)
add_library(${PROJECT_NAME} STATIC
  InitComponent/InitANT.cpp
  InitComponent/InitBAT.cpp
  InitComponent/InitEMDS.cpp
  InitComponent/InitGScalculator.cpp
  InitComponent/InitGyro.cpp
  InitComponent/InitMagSensor.cpp
  InitComponent/InitMagTorquer.cpp
  InitComponent/InitRWModel.cpp
  InitComponent/InitSAP.cpp
  InitComponent/InitSimpleThruster.cpp
  InitComponent/InitSTT.cpp
  InitComponent/InitSunSensor.cpp
  InitComponent/initTelescope.cpp
  InitComponent/InitUWBSensor.cpp
  InitComponent/InitGNSSReceiver.cpp
  InitThermal/Init_Node.cpp
  InitThermal/Init_Temperature.cpp
  InitPower/CsvScenarioInterface.cpp
  IniAccess.h
  Init_Attitude.cpp
  Init_CelesInfo.cpp
  Init_Disturbance.cpp
  Init_Environment.cpp
  Init_HipCatalogue.cpp
  Init_MCSim.cpp
  Init_Orbit.cpp
  Init_SimTime.cpp
  Init_Log.cpp
  Initialize.h
#  Init_COSMOSWrapper.cpp
#  Init_HardwareMessage.cpp
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

## ini access
if(WIN32)
  add_library(INI_ACC STATIC IniAccess.cpp)
else()
  add_library(INI_ACC STATIC IniAccessUnix.cpp)
endif()
# Compile option
if(MSVC)
  target_compile_options(INI_ACC PUBLIC "/W4")
	target_compile_options(INI_ACC PUBLIC "/source-charset:utf-8")
else()
  target_compile_options(INI_ACC PUBLIC -Wall -g -m32)
endif()
