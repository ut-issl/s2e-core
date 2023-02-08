project(COMPONENT)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  Abstract/ComponentBase.cpp
  Abstract/ObcCommunicationBase.cpp
  Abstract/I2cControllerCommunicationBase.cpp
  Abstract/ObcI2cTargetCommunicationBase.cpp
  Abstract/ObcGpioBase.cpp

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
  AOCS/GNSSReceiver.cpp
  AOCS/InitGnssReceiver.cpp

  CDH/OBC.cpp
  CDH/OBC_C2A.cpp

  CommGS/Antenna.cpp
  CommGS/AntennaRadiationPattern.cpp
  CommGS/InitAntenna.cpp
  CommGS/GScalculator.cpp
  CommGS/InitGsCalculator.cpp

  examples/example_change_structure.cpp
  examples/example_serial_communication_with_obc.cpp
  examples/example_serial_communication_for_hils.cpp
  examples/example_i2c_controller_for_hils.cpp
  examples/example_i2c_target_for_hils.cpp

  IdealComponents/ForceGenerator.cpp
  IdealComponents/InitializeForceGenerator.cpp
  IdealComponents/TorqueGenerator.cpp
  IdealComponents/InitializeTorqueGenerator.cpp

  Mission/Telescope/Telescope.cpp
  Mission/Telescope/InitTelescope.cpp

  Power/PCU.cpp
  Power/BAT.cpp
  Power/InitBat.cpp
  Power/PCU_InitialStudy.cpp
  Power/InitPcu_InitialStudy.cpp
  Power/SAP.cpp
  Power/InitSap.cpp
  Power/CsvScenarioInterface.cpp

  Propulsion/SimpleThruster.cpp
  Propulsion/InitSimpleThruster.cpp


## C++/CLI related codes
#  CDH/TMTCInterface.cpp
)

include(../../common.cmake)
