project(COMPONENT)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  Abstract/ComponentBase.cpp
  Abstract/ObcCommunicationBase.cpp
  Abstract/ObcI2cCommunicationBase.cpp
  Abstract/ObcGpioBase.cpp
  Abstract/EXP.cpp
  Abstract/ExpHils.cpp
  AOCS/EMDS.cpp
  AOCS/Gyro.cpp
  AOCS/MagSensor.cpp
  AOCS/MagTorquer.cpp
  AOCS/rw_ode.cpp
  AOCS/RWModel.cpp
  AOCS/RWJitter.cpp
  AOCS/STT.cpp
  AOCS/SunSensor.cpp
  AOCS/UWBSensor.cpp
  AOCS/GNSSReceiver.cpp
  CDH/OBC.cpp
  CDH/OBC_C2A.cpp
  CommGS/ANT.cpp
  CommGS/GScalculator.cpp
  Logic/LPF.cpp
  Logic/RVDController.cpp
  Logic/UWBEstimator.cpp
  Mission/Telescope/Telescope.cpp
  Power/PCU.cpp
  Power/BAT.cpp
  Power/PCU_InitialStudy.cpp
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
)

include(../../common.cmake)
