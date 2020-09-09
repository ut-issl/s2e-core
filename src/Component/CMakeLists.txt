cmake_minimum_required(VERSION 3.10)
project(COMPONENT)
add_library(${PROJECT_NAME} STATIC
  Abstract/ComponentBase.cpp
  Abstract/EXP.cpp
  AOCS/EMDS.cpp
  AOCS/Gyro.cpp
  AOCS/MagSensor.cpp
  AOCS/MagTorquer.cpp
  AOCS/rw_ode.cpp
  AOCS/RWModel.cpp
  #AOCS/RWModule.cpp
  AOCS/STT.cpp
  AOCS/SunSensor.cpp
  AOCS/UWBSensor.cpp
  AOCS/GNSSReceiver.cpp
  CommGS/ANT.cpp
  CommGS/GScalculator.cpp
  CDH/OBC.cpp
  Logic/LPF.cpp
  Logic/RVDController.cpp
  Logic/UWBEstimator.cpp
  Power/BAT.cpp
  Power/PCU_InitialStudy.cpp
  Mission/Telescope/Telescope.cpp
  Power/SAP.cpp
  Propulsion/SimpleThruster.cpp

    # initialize functions
  ../Interface/InitInput/InitComponent/InitGyro.cpp
  ../Interface/InitInput/InitComponent/InitANT.cpp
  ../Interface/InitInput/InitComponent/InitBAT.cpp
  ../Interface/InitInput/InitComponent/InitEMDS.cpp
  ../Interface/InitInput/InitComponent/InitGScalculator.cpp
  ../Interface/InitInput/InitComponent/InitMagSensor.cpp
  ../Interface/InitInput/InitComponent/InitMagTorquer.cpp
  ../Interface/InitInput/InitComponent/InitRWModel.cpp
  ../Interface/InitInput/InitComponent/InitSAP.cpp
  ../Interface/InitInput/InitComponent/InitSimpleThruster.cpp
  ../Interface/InitInput/InitComponent/InitSTT.cpp
  ../Interface/InitInput/InitComponent/InitSunSensor.cpp
  ../Interface/InitInput/InitComponent/initTelescope.cpp
  ../Interface/InitInput/InitComponent/InitUWBSensor.cpp
  ../Interface/InitInput/InitComponent/InitGNSSReceiver.cpp
  ../Interface/InitInput/InitPower/CsvScenarioInterface.cpp

## C++/CLI related codes
#  CDH/TMTCInterface.cpp
#  CDH/TMTCInterface.h
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
