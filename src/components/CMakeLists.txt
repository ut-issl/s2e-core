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

  ideal_components/ForceGenerator.cpp
  ideal_components/InitializeForceGenerator.cpp
  ideal_components/TorqueGenerator.cpp
  ideal_components/InitializeTorqueGenerator.cpp

  mission/Telescope.cpp
  mission/InitTelescope.cpp

  power/PCU.cpp
  power/BAT.cpp
  power/InitBat.cpp
  power/PCU_InitialStudy.cpp
  power/InitPcu_InitialStudy.cpp
  power/SAP.cpp
  power/InitSap.cpp
  power/CsvScenarioInterface.cpp

  propulsion/SimpleThruster.cpp
  propulsion/InitSimpleThruster.cpp

)

include(../../common.cmake)
