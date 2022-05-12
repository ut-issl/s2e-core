project(COMPONENT)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  Abstract/ComponentBase.cpp
  Abstract/ObcCommunicationBase.cpp
  Abstract/I2cControllerCommunicationBase.cpp
  Abstract/ObcI2cTargetCommunicationBase.cpp
  Abstract/ObcGpioBase.cpp
  Abstract/EXP.cpp
  Abstract/ExpHils.cpp
  Abstract/ExpHilsI2cController.cpp
  Abstract/ExpHilsI2cTarget.cpp

  AOCS/EMDS.cpp
  AOCS/InitEmds.cpp
  AOCS/Gyro.cpp
  AOCS/InitGyro.cpp
  AOCS/MagSensor.cpp
  AOCS/InitMagSensor.cpp
  AOCS/MagTorquer.cpp
  AOCS/InitMagTorquer.cpp
  AOCS/rw_ode.cpp
  AOCS/RWModel.cpp
  AOCS/InitRwModel.cpp
  AOCS/RWJitter.cpp
  AOCS/STT.cpp
  AOCS/InitStt.cpp
  AOCS/SunSensor.cpp
  AOCS/InitSunSensor.cpp
  AOCS/UWBSensor.cpp
  AOCS/InitUwbSensor.cpp
  AOCS/GNSSReceiver.cpp
  AOCS/InitGnssReceiver.cpp

  CDH/OBC.cpp
  CDH/OBC_C2A.cpp

  CommGS/ANT.cpp
  CommGS/InitAnt.cpp
  CommGS/GScalculator.cpp
  CommGS/InitGsCalculator.cpp

  Logic/LPF.cpp
  Logic/RVDController.cpp
  Logic/UWBEstimator.cpp

  Mission/Telescope/Telescope.cpp
  Mission/Telescope/InitTelescope.cpp

  Power/PCU.cpp
  Power/BAT.cpp
  Power/InitBat.cpp
  Power/PCU_InitialStudy.cpp
  Power/SAP.cpp
  Power/InitSap.cpp
  Power/CsvScenarioInterface.cpp

  Propulsion/SimpleThruster.cpp
  Propulsion/InitSimpleThruster.cpp


## C++/CLI related codes
#  CDH/TMTCInterface.cpp
)

include(../../common.cmake)
