project(COMPONENT)
cmake_minimum_required(VERSION 3.13)

add_library(${PROJECT_NAME} STATIC
  base_classes/ComponentBase.cpp
  base_classes/ObcCommunicationBase.cpp
  base_classes/I2cControllerCommunicationBase.cpp
  base_classes/ObcI2cTargetCommunicationBase.cpp
  base_classes/ObcGpioBase.cpp

  aocs/Gyro.cpp
  aocs/InitGyro.cpp
  aocs/MagSensor.cpp
  aocs/InitMagSensor.cpp
  aocs/MagTorquer.cpp
  aocs/InitMagTorquer.cpp
  aocs/rw_ode.cpp
  aocs/RWModel.cpp
  aocs/InitRwModel.cpp
  aocs/RWJitter.cpp
  aocs/STT.cpp
  aocs/InitStt.cpp
  aocs/SunSensor.cpp
  aocs/InitSunSensor.cpp
  aocs/GNSSReceiver.cpp
  aocs/InitGnssReceiver.cpp

  cdh/OBC.cpp
  cdh/OBC_C2A.cpp

  communication/Antenna.cpp
  communication/AntennaRadiationPattern.cpp
  communication/InitAntenna.cpp
  communication/GScalculator.cpp
  communication/InitGsCalculator.cpp

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

)

include(../../common.cmake)
